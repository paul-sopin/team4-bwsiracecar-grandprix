"""
Team 4 - Grand Prix race script

Everything the car does in a run, in one racecar_core script. It sits at the line
until the light goes green, then follows the biggest open gap in the lidar scan,
then does the elevator at the end. Four other files ride along with it:

    ahrs.py            heading and turn rate, and the traction limiter off them
    ar_detector.py     the AR tag on the right wall before the elevator
    elevator_signs.py  the elevator's GO / STOP sign, on the Coral
    telemetry.py       records every frame and graphs it when we exit

The elevator is a small state machine on top of the gap follower:

    RACE      largest gap, normal racing, watching for the tag
    APPROACH  tag 0 seen, so pin to the leftmost gap and start the Coral
    HOLD      the sign said STOP, so sit HOLD_DIST_CM off the front wall
    ENTER     the sign said GO, so drive at the wall until ENTER_DIST_CM
    IN        we are in, park

Only the steering keeps coming from the gap follower once we are past APPROACH.
The speed comes off the front lidar from there on.

Run it with:

    racecar sim grand_prix/grand_prix_AHRS.py

Free the Coral first or the sign detector will not build. It stays claimed
across a reboot, and the race still runs without it, just deaf at the elevator:

    sudo kill $(sudo lsof -t /dev/apex_0)

Don't move the car for the first two seconds. Gyro bias gets measured while we
wait at the light, and if you bump it in there the heading is off all run.
"""

import os
import sys
import time

import racecar_core
import racecar_utils as rc_utils

# so ahrs.py, telemetry.py, ar_detector.py and elevator_signs.py get found when
# this is run from another directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ahrs import AHRS
from mag import Magnetometer
from telemetry import TelemetryLogger

# the two perception imports are the ones that can fall over on the day. the tag
# reader wants an OpenCV with aruco in it, and the sign detector wants
# tflite_runtime, libedgetpu, and /dev/apex_0 free. a car that won't start
# because of a perception file is worse than a car that races without one, so we
# try each, print, and drive either way
try:
    from ar_detector import ARTagGate
except Exception as ar_import_error:   # noqa: BLE001, anything here means no tags
    ARTagGate = None
    print("AR tag reader unavailable, racing without it:", ar_import_error)

try:
    from elevator_signs import ElevatorSigns, GO, STOP
except Exception as sign_import_error:   # noqa: BLE001, no Coral means no signs
    ElevatorSigns = None
    GO, STOP = "GO", "STOP"
    print("elevator sign reader unavailable, racing without it:",
          sign_import_error)

rc = racecar_core.create_racecar()

OPEN_THRESHOLD = 220.0   # cm, further open space (tighten if radius is small)
GAP_TURN_KP = 0.0085 # KP for turning towards the gap
SPEED_KP = 0.0012 * (4.0/4.5) # speed kp scaled to updated speed
MIN_SPEED = 0.533 # minimum speed, so we keep making progress
MAX_SPEED = 0.889 # maximum speed, so we do not crash
GREEN = ((39, 82, 53), (88, 255, 255))  # start detection color
START_AREA_THRESHOLD = 500 # start detection area
SKID_KD = 0.0022 # how hard we brake when the AHRS says we're spinning
SKID_DEADZONE = 25.0 # deg/s, anything under this is just normal cornering
SKID_TIME_MIN = 0.05 # s of skidding
STARTED_DISPLAY_TIME = 1.0 # how long GO stays up before the heading

# Which gap to follow. "largest" is normal racing. "leftmost"/"rightmost" pin the
# car to one side, for splits where we already know which way we want to go.
VALID_GAP_MODES = ("largest", "leftmost", "rightmost")
GAP_MODE = "largest" # what each run starts in

# the mode as a number, since telemetry graphs numbers. put it next to the
# steering angle on the graph and you can see if a tag fired where you thought
GAP_BIAS = {"largest": 0, "leftmost": -1, "rightmost": 1}

# min gap width in degrees, only for the side modes. without it leftmost will
# happily steer at a 1 degree sliver of noise at the edge of the scan.
MIN_GAP_WIDTH = 5

# The AR tag on the right wall before the elevator. See ar_detector.py.
AR_DICT = "DICT_6X6_250"  # the course tags are 6x6. the 50, 100 and 250
                          # dictionaries share their first markers, so this reads
                          # a tag printed from any of them
# Tag 0 is the elevator. 1 through 4 are elsewhere on the course and mean nothing
# to us, so they have to be filtered out here. Leave this as None and the first
# tag the car drives past sends it looking for an elevator that isn't there.
AR_IDS = (0,)
AR_MIN_SIZE = 0.06        # tag size (average edge over frame height) we will
                          # act on, which is really "how close to the elevator".
                          # aruco decodes from further out than we want to
                          # commit from, so this is what keeps the gate off
                          # until the last stretch. measure it with
                          # show_ar_tags.py, do not guess it. if the gate never
                          # fires on the course, this is the first thing to drop
AR_NEED = 3               # frames of evidence with the tag before we commit

# The elevator's GO / STOP sign, on the Coral. See elevator_signs.py.
SIGN_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "best_v5_edgetpu.tflite")
SIGN_CONF = 0.35          # score floor per box
SIGN_TRIGGER_H = 0.22     # box height over frame height worth a whole vote
SIGN_VOTE_N = 9           # frames in the window
SIGN_NEED = 2.0           # accumulated evidence before we act on a sign

# Neither reader skips frames. They both run on new camera frames only, about 30
# a second, and they are never both running: the gate only looks before the tag
# and the Coral only looks after it.

# What each sign gets us to do, in cm off the wall in front.
HOLD_DIST_CM = 76.2       # 30 inches, where we wait on STOP
ENTER_DIST_CM = 5.0       # where we stop on GO, which is inside the elevator
DIST_TOL_CM = 4.0         # close enough to either of those to call it parked

# Speed control for the last few metres. Slower than racing on purpose: the
# whole point of this bit is not hitting the back of the elevator.
ELEV_KP = 0.006           # speed per cm of error. 76 cm out gives about 0.45
ELEV_MIN_SPEED = 0.16     # under this the car does not move at all
ELEV_MAX_SPEED = 0.45     # cap on the approach
ELEV_BRAKE = -0.35        # a reverse pulse, because speed 0 is ESC neutral and
ELEV_BRAKE_S = 0.25       # coasts. this is how we actually stop
FINAL_STRAIGHT_CM = 60.0  # inside this we stop steering and go straight in. the
                          # gap follower has nothing useful left to say when the
                          # wall fills the scan
APPROACH_SLOW_CM = 250.0  # how close the wall has to be before APPROACH starts
                          # holding the throttle down. ELEV_MAX_SPEED is under
                          # MIN_SPEED, so without this a tag read early costs us
                          # racing speed for everything that is left

# The tag is on the right wall and the elevator is on the left, so once the tag
# fires we steer left harder than the gap follower would on its own. See
# left_bias(). Raise ELEV_LEFT_BIAS if the car does not commit, lower it if it
# reaches the left wall before it reaches the elevator.
ELEV_LEFT_BIAS = 0.30     # steering units added to the left, 1.0 is full lock
ELEV_LEFT_MIN_CM = 45.0   # stop pushing when the left wall is this close

# The lidar cannot see ENTER_DIST_CM. Its minimum range is somewhere around
# 15 cm and under that a sample comes back 0.0, which racecar_utils reads as no
# data. So the real arrival test is "the front went blank right after reading
# something short", and this is the number that decides what short means. If the
# car ends up stopping too early, lower it; too late, raise it.
LIDAR_BLIND_CM = 25.0
FRONT_WINDOW = 8.0        # width of the scan we average, centred straight ahead

# race states, see the top of the file
RACE, APPROACH, HOLD, ENTER, IN = "RACE", "APPROACH", "HOLD", "ENTER", "IN"
# as a number, since telemetry graphs numbers
STATE_CODE = {RACE: 0, APPROACH: 1, HOLD: 2, ENTER: 3, IN: 4}

speed = 0.0 # current speed
angle = 0.0 # current angle

left_dist = 0.0
right_dist = 0.0

gap_mode = GAP_MODE # current mode, changed by set_gap_mode()

target_angle = 0.0 # midpoint of the chosen gap, global so telemetry can see it
race_start_time = None # set when green is detected, times the GO message

race_started = False # whether the light has turned green

# the compass comes along to stop yaw drifting. if there is no /mag on the car,
# or no rclpy, it says so and the filter just runs on the gyro
imu = AHRS(mag=Magnetometer()) # heading and turn rate
logger = None # made in start()
gate = None # made in start(), None if the AR tag reader didn't import
signs = None # made in start(), None if the Coral sign reader didn't import

state = RACE # where we are in the run, see the top of the file
front_dist = 0.0 # cm to the wall straight ahead, 0 means nothing came back
last_front = 0.0 # newest front reading that wasn't 0, so we can tell "too close
                 # to measure" from "nothing out there"
brake_until = 0.0 # reverse pulse runs until this time

skid_time = 0.0 # how long the skid has lasted

def start():
    global logger, gate, signs, gap_mode, state
    global front_dist, last_front, brake_until

    rc.drive.set_max_speed(1.0)
    rc.drive.set_speed_angle(0, 0)

    # reset, so a mode switched last run doesn't carry into this one
    gap_mode = GAP_MODE
    state = RACE
    front_dist = last_front = brake_until = 0.0

    # not at import. building either detector touches hardware, OpenCV for one
    # and the Coral for the other, and if that goes wrong we want it printed and
    # the car racing, same as the imports above
    if gate is None and ARTagGate is not None:
        try:
            gate = ARTagGate(dictionary=AR_DICT, ids=AR_IDS,
                             min_size=AR_MIN_SIZE, need=AR_NEED)
        except Exception as error:   # noqa: BLE001
            print("AR tag reader failed to start, racing without it:", error)

    if signs is None and ElevatorSigns is not None:
        try:
            signs = ElevatorSigns(SIGN_MODEL, conf=SIGN_CONF,
                                  trigger_h=SIGN_TRIGGER_H, vote_n=SIGN_VOTE_N)
        except Exception as error:   # noqa: BLE001
            print("sign reader failed to start (is /dev/apex_0 free?),",
                  "racing without it:", error)

    # both get built once and reused, so anything they were holding onto at the
    # end of the last run has to go, or we start this one already at the elevator
    if gate is not None:
        gate.reset()
    if signs is not None:
        signs.clear()

    # not at import, the racecar isn't up yet when declare_variables gets called
    if logger is None:
        logger = TelemetryLogger(rc)

def start_detection():
    image = rc.camera.get_color_image() # gets the color image in the camera
    if image is None:
        return False
    # if the camera doesn't see the color, it doesn't start.
    h = image.shape[0]
    crop = image[h // 3 : 7 * h // 8, :]
    # crops to the bottom (crops out the sky)
    green_contours = rc_utils.find_contours(crop, GREEN[0], GREEN[1])
    green_c = rc_utils.get_largest_contour(green_contours)

    # if green is not detected, don't start.
    # has to be before get_contour_area. get_largest_contour returns None when
    # there's no green in frame, which is every frame until the light changes.
    if green_c is None:
        return False # not started

    green_a = rc_utils.get_contour_area(green_c)

    if green_a > START_AREA_THRESHOLD:
        return True # started

    return False # green is visible but still too far away


# switch which gap we aim at, mid run. elevator_update() is what calls this, once
# per run: the tag puts us in "leftmost" and we stay there to the end.
# bad mode gets ignored, a typo from perception shouldn't stop the car.
def set_gap_mode(mode):
    global gap_mode

    if mode not in VALID_GAP_MODES:
        print("ignoring unknown gap mode:", mode, "expected one of", VALID_GAP_MODES)
        return

    if mode != gap_mode:
        print("gap mode:", gap_mode, "->", mode)
        gap_mode = mode


# leftmost (0) or rightmost (-1) gap that's actually wide enough to fit through.
# if nothing clears MIN_GAP_WIDTH take the biggest gap instead, better than
# aiming at a sliver just because it's on the right side.
def pick_side_gap(runs, index):
    wide_enough = [run for run in runs if len(run) >= MIN_GAP_WIDTH]
    if wide_enough:
        return wide_enough[index]
    return max(runs, key=len)


# cm to whatever is straight ahead, or 0.0 for closer than the lidar can see.
#
# get_lidar_average_distance ignores 0.0 samples as no data and returns 0.0 when
# they all were. A wall too close to measure and nothing in range at all look
# the same in that number, so we go on what we read a moment ago. Blank right
# after something short means we drove into it. Blank after something far away
# means open track.
def front_distance(scan):
    global last_front

    if len(scan) != 0:
        reading = rc_utils.get_lidar_average_distance(scan, 0, FRONT_WINDOW)
        if reading > 0:
            last_front = reading
            return reading
        if 0 < last_front <= LIDAR_BLIND_CM:
            return 0.0                  # too close to measure, so we are there

    # nothing came back and nothing short came back before it. before the first
    # sweep there is no history to fall back on either, and calling that 0 would
    # read as "arrived" the moment we entered ENTER, so say far instead
    return last_front if last_front > 0 else OPEN_THRESHOLD


# how hard to lean on the steering to get us over to the elevator.
#
# "leftmost" alone does not do this. It picks the leftmost gap wide enough to fit
# through, so when the elevator door is the only opening in front of us, the
# leftmost gap and the largest gap are the same gap and the mode changes nothing.
# This is the part that actually moves the car across.
#
# It is a constant push, not a controller, so the one thing it must not do is
# push us into the wall it is aiming at. Once anything on the left is closer than
# ELEV_LEFT_MIN_CM the push comes off and the gap follower has the car back.
def left_bias(scan):
    if len(scan) == 0:
        return 0.0
    # angles run clockwise from straight ahead, so the left side is 270 to 360
    _angle, clearance = rc_utils.get_lidar_closest_point(scan, (270, 360))
    if 0 < clearance < ELEV_LEFT_MIN_CM:
        return 0.0
    return ELEV_LEFT_BIAS


# how fast to close on a wall we want to stop `target` cm from.
def approach_speed(front, target):
    global brake_until

    error = front - target
    if error > DIST_TOL_CM:
        brake_until = 0.0
        return rc_utils.clamp(ELEV_KP * error, ELEV_MIN_SPEED, ELEV_MAX_SPEED)

    # there or past it. speed 0 is ESC neutral and the car coasts through it, so
    # pull back for a moment first and then sit still
    now = time.time()
    if brake_until == 0.0:
        brake_until = now + ELEV_BRAKE_S
    return ELEV_BRAKE if now < brake_until else 0.0


# the elevator, end to end. runs every frame once the race is going.
#
# RACE watches for the tag. Everything after it is driving at a wall and the only
# question is how close we stop. We keep reading the sign the whole way in,
# because the elevator shows STOP first and GO later, and we have to catch that
# change while sitting in front of it.
def elevator_update():
    global state

    if state == RACE:
        if gate is not None and gate.poll(rc.camera.get_color_image()):
            state = APPROACH
            # the tag is taped to the RIGHT wall but the elevator is on the LEFT,
            # so the tag is a sign to look at, not a direction to drive at. we
            # want the leftmost gap from here on
            set_gap_mode("leftmost")
            print("[elevator] tag 0 seen -> leftmost gap, watching for the sign")
        return

    if state == IN:
        return

    if signs is None:
        return      # no Coral. we keep hugging the left and let the driver call it

    signs.poll(rc.camera.get_color_image())
    sign = signs.winner(SIGN_NEED)

    if sign == STOP and state != HOLD:
        # a STOP after we've already started in only counts while there is still
        # room to stop in. past that, braking mid doorway is the worse outcome
        if state == ENTER and front_dist <= HOLD_DIST_CM:
            return
        state = HOLD
        signs.clear()
        print("[elevator] STOP -> holding at", HOLD_DIST_CM, "cm")
    elif sign == GO and state != ENTER:
        state = ENTER
        signs.clear()
        print("[elevator] GO -> driving in to", ENTER_DIST_CM, "cm")


def gap_follow_update():
    global left_dist, right_dist, speed, angle, target_angle

    scan = rc.lidar.get_samples()
    if len(scan) != 0:
        # check -90 to 90 and collect every gap, not just the biggest, so
        # GAP_MODE can choose. negative is left, positive is right.
        runs, current_run = [], []
        for deg in range(-90, 91, 1):
            cur_scan = rc_utils.get_lidar_average_distance(scan, deg % 360, 0.5)
            if cur_scan > OPEN_THRESHOLD or cur_scan == 0:
                current_run.append(deg)
            else:
                if current_run:
                    runs.append(current_run)
                current_run = []
        # the last run never hits the else branch, so save it here
        if current_run:
            runs.append(current_run)

        # runs come out left to right already, so leftmost is first, rightmost last
        if not runs:
            best_run = []
        elif gap_mode == "leftmost":
            best_run = pick_side_gap(runs, 0)
        elif gap_mode == "rightmost":
            best_run = pick_side_gap(runs, -1)
        else:
            best_run = max(runs, key=len) # finds the biggest gap by comparing gap sizes

        if best_run:
            target_angle = sum(best_run) / len(best_run) # target angle is the midpoint of the gap
        else:
            target_angle = 0.0

        left_dist = 0
        for deg in range(-90, 0, 1):
            cur_scan = rc_utils.get_lidar_average_distance(scan, deg % 360, 0.5)
            if cur_scan > left_dist:
                left_dist = cur_scan

        right_dist = 0
        for deg in range(0, 91, 1):
            cur_scan = rc_utils.get_lidar_average_distance(scan, deg % 360, 0.5)
            if cur_scan > right_dist:
                right_dist = cur_scan

        avg_dist = (left_dist + right_dist) / 2

        angle = GAP_TURN_KP * target_angle
        angle = rc_utils.clamp(angle, -1.0, 1.0)
        speed = SPEED_KP * avg_dist
    else:
        speed = MAX_SPEED

def update():
    # just the names this function assigns. the rest we only read, and reading
    # a global doesn't need declaring
    global speed, angle, race_started, race_start_time, skid_time
    global front_dist, state

    # every frame, even before the race starts. the wait is free calibration time
    imu.update(rc)

    if not race_started:
        if start_detection():
            race_started = True
            race_start_time = time.time()
            rc.display.show_text("GO")
        else:
            rc.drive.set_speed_angle(0, 0)
            # so we can see the gyro finish calibrating before the race starts
            if imu.ready:
                rc.display.show_text("READY")
            else:
                rc.display.show_text("CALIB")
            return

    scan = rc.lidar.get_samples()
    front_dist = front_distance(scan)

    # goes before the follower, so a tag we read this frame steers this frame
    elevator_update()

    gap_follow_update()

    if state in (RACE, APPROACH):
        # turning way faster than a normal corner means we're probably sliding
        spin = abs(imu.turn_rate())
        if spin > SKID_DEADZONE:
            skid_time += rc.get_delta_time()
            if skid_time > SKID_TIME_MIN:
                speed -= SKID_KD * (spin - SKID_DEADZONE)
        else:
            skid_time = 0

        speed = rc_utils.clamp(speed, MIN_SPEED, MAX_SPEED)

        # past the tag, the wall gets a veto on the throttle even before a sign
        # has been read. MIN_SPEED is 0.53 and arriving at the elevator doing
        # that means reading STOP from too close to obey it. This also means a
        # run where the Coral never fires stops 30 inches out instead of into
        # the door.
        # only once the wall is actually near, though. the tag can be read from
        # further out than APPROACH_SLOW_CM, and there is no reason a tag seen
        # early should mean crawling the rest of the way to it
        if state == APPROACH and front_dist <= APPROACH_SLOW_CM:
            speed = min(speed, approach_speed(front_dist, HOLD_DIST_CM))
    else:
        # at the elevator the front wall sets the speed, and MIN_SPEED does not
        # apply: the whole job here is being allowed to stop
        skid_time = 0
        if state == HOLD:
            speed = approach_speed(front_dist, HOLD_DIST_CM)
        elif state == ENTER:
            speed = approach_speed(front_dist, ENTER_DIST_CM)
            if front_dist <= ENTER_DIST_CM + DIST_TOL_CM:
                state = IN
                print("[elevator] in, front reads", round(front_dist, 1), "cm")
        else:                                   # IN
            speed = 0.0

        # the follower has nothing useful left to say once the wall fills the
        # scan, and a late twitch here puts a corner into the doorway
        if front_dist <= FINAL_STRAIGHT_CM:
            angle = 0.0

    # everything from the tag to the doorway gets pushed left, because that is
    # where the elevator is. not in the final straight, where we are lined up and
    # committed, and not in RACE or IN, where it has no business steering at all
    if state in (APPROACH, HOLD, ENTER) and front_dist > FINAL_STRAIGHT_CM:
        angle = rc_utils.clamp(angle - left_bias(scan), -1.0, 1.0)

    rc.drive.set_speed_angle(speed, angle)

    # graphed on exit. this is where SKID_DEADZONE comes from.
    logger.log(
        target_angle=target_angle,
        left_dist=left_dist,
        right_dist=right_dist,
        angle=angle,
        speed=speed,
        heading=imu.heading(),
        turn_rate=imu.turn_rate(),
        gap_bias=GAP_BIAS.get(gap_mode, 0),
        front_dist=front_dist,
        state=STATE_CODE.get(state, 0),
    )

    # keep this SHORT. the matrix is 8x24 and scrolls anything longer at ~2
    # chars/sec, so a long readout every frame never finishes scrolling.
    if race_start_time is not None and time.time() - race_start_time < STARTED_DISPLAY_TIME:
        rc.display.show_text("GO")
    elif state == HOLD:
        rc.display.show_text("WAIT")
    elif state == ENTER:
        rc.display.show_text("IN")
    elif state == IN:
        rc.display.show_text("DONE")
    elif state == APPROACH:
        rc.display.show_text("ELV")   # tag seen, hugging the left
    else:
        rc.display.show_text(str(int(imu.heading())))

def update_slow():
    print("Speed:", speed, "Angle:", angle)
    print("Left:", left_dist, "Right:", right_dist)
    # where we are in the run, and what each detector is sitting on. new/dup at
    # the end of those is the repeat-frame filter. if dup keeps climbing and new
    # doesn't, the camera has stalled and it is not that there is nothing to see
    print("State:", state, "| Gap mode:", gap_mode,
          "| Front:", round(front_dist, 1), "cm")
    print("AR:", gate.summary() if gate is not None else "off")
    print("Signs:", signs.summary() if signs is not None else "off")
    # heading drifting while the car sits still = calibration didn't take
    print("AHRS ready?", imu.ready)
    print("Heading:", round(imu.heading(), 1), "Turn rate:", round(imu.turn_rate(), 1))
    # until this says LOCKED, yaw is still gyro-only and still drifting
    print("Compass:", imu.mag_status())
    print("Roll:", round(imu.roll, 3), "Pitch:", round(imu.pitch, 3))

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
