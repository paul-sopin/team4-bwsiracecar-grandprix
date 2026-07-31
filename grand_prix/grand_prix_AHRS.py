"""
Team 4 - Grand Prix race script

Everything the car does in a run, in one racecar_core script. It sits at the line
until the light goes green and then follows the biggest open gap in the lidar
scan. Three other files ride along with it:

    ahrs.py        heading and turn rate, and the traction limiter off them
    ar_support.py  AR tags at a split, which pick the left or right gap
    telemetry.py   records every frame and graphs it when we exit

Run it with:

    racecar sim grand_prix/grand_prix_AHRS.py

Don't move the car for the first two seconds. Gyro bias gets measured while we
wait at the light, and if you bump it in there the heading is off all run.
"""

import os
import sys
import time

import racecar_core
import racecar_utils as rc_utils

# so ahrs.py, telemetry.py and ar_support.py get found when this is run from
# another directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ahrs import AHRS
from mag import Magnetometer
from telemetry import TelemetryLogger

# the tag reader is the one import that can fall over on a car with the wrong
# OpenCV. a car that won't start because of a perception file is worse than a
# car that races without one, so we try it, print, and drive either way
try:
    from ar_support import ARWatcher, FLIPPED, UPRIGHT
except Exception as ar_import_error:   # noqa: BLE001, anything here means no tags
    ARWatcher = None
    UPRIGHT, FLIPPED = "UPRIGHT", "FLIPPED"
    print("AR tag reader unavailable, racing without it:", ar_import_error)

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

# AR tags. A tag at a split tells us which way to go by which way up it is.
# Upright is one side, turned 180 degrees is the other. That mapping is the
# whole point of the thing so we keep it up here where it is easy to swap after
# we've walked the course, instead of burying it in ar_support.py.
ORIENTATION_MODE = {UPRIGHT: "leftmost", FLIPPED: "rightmost"}

AR_DICT = "DICT_6X6_250"  # which dictionary the course tags are printed from
AR_IDS = None             # tag ids to act on, or None for any of them
AR_ANGLE_TOL = 50.0       # degrees off 0 or 180 we still read as that way up.
                          # sounds loose, but a tag is only ever one of two ways
                          # up, so what this really does is throw out a tag we
                          # are seeing edge on
AR_TRIGGER_SIZE = 0.10    # tag size (average edge over frame height) worth a
                          # whole vote
AR_EVIDENCE_NEED = 1.6    # how much we need before touching the gap mode
AR_VOTE_N = 7             # frames in the window
AR_DETECT_EVERY_N = 2     # detect every other new frame, so about 15 Hz. a tag
                          # we are driving at is in view for seconds
AR_HOLD_S = 4.0           # how long the side mode stays pinned once it fires
AR_CLEAR_S = 0.6          # extra time we give it while the tag is still there
AR_MAX_HOLD_S = 8.0       # cap on that. a tag we stop in front of never leaves
                          # view and can't be allowed to pin us all race
AR_COOLDOWN_S = 3.0       # after going back, ignore tags this long, so the one
                          # we just drove past can't fire again

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
watcher = None # made in start(), None if the AR tag reader didn't import

ar_facing = None # facing currently pinning the gap mode, None when free
ar_hold_until = 0.0 # when the pinned mode goes back to GAP_MODE
ar_hard_until = 0.0 # ceiling on the above, ignores whether the tag is still there
ar_cool_until = 0.0 # tags ignored until this time

def start():
    global logger, watcher, gap_mode
    global ar_facing, ar_hold_until, ar_hard_until, ar_cool_until

    rc.drive.set_max_speed(1.0)
    rc.drive.set_speed_angle(0, 0)

    # reset, so a mode switched last run doesn't carry into this one
    gap_mode = GAP_MODE
    ar_facing = None
    ar_hold_until = ar_hard_until = ar_cool_until = 0.0

    # not at import. building the detector touches OpenCV, and if that goes
    # wrong we want it printed and the car racing, same as the import above
    if watcher is None and ARWatcher is not None:
        try:
            watcher = ARWatcher(dictionary=AR_DICT, ids=AR_IDS,
                                angle_tol=AR_ANGLE_TOL,
                                trigger_size=AR_TRIGGER_SIZE,
                                vote_n=AR_VOTE_N, every_n=AR_DETECT_EVERY_N)
        except Exception as error:   # noqa: BLE001
            print("AR tag reader failed to start, racing without it:", error)

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


# switch which gap we aim at, mid run. ar_update() is what calls this: it takes
# a split by calling set_gap_mode("leftmost"), then "largest" once we're past.
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


# read the camera and let a tag pick which side of the split we take.
#
# There are two timers doing different jobs. hold_until is the normal one and it
# gets pushed back as long as the tag is still in the window, so the mode stays
# pinned the whole way through a split instead of running out halfway in.
# hard_until never moves. A tag we end up stopped in front of stays in view
# forever, and without a cap it would pin us to one side for the rest of the race.
def ar_update(now):
    global ar_facing, ar_hold_until, ar_hard_until, ar_cool_until

    if watcher is None:
        return

    watcher.poll(rc.camera.get_color_image())

    if ar_facing is not None:
        if watcher.count(ar_facing):     # still there, so keep the hold going
            ar_hold_until = max(ar_hold_until, now + AR_CLEAR_S)
        if now >= ar_hold_until or now >= ar_hard_until:
            ar_facing = None
            ar_cool_until = now + AR_COOLDOWN_S
            set_gap_mode(GAP_MODE)       # back to whatever the run started on
        return

    if now < ar_cool_until:
        return

    facing = watcher.winner(AR_EVIDENCE_NEED)   # enough to act on, or None
    if facing not in ORIENTATION_MODE:
        return

    ar_facing = facing
    ar_hold_until, ar_hard_until = now + AR_HOLD_S, now + AR_MAX_HOLD_S
    print("[ar]", facing, "->", ORIENTATION_MODE[facing])
    set_gap_mode(ORIENTATION_MODE[facing])
    # the votes that just fired are still sitting in the window, and they would
    # fire it again the moment the hold runs out
    watcher.clear()


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
    global speed, race_started, race_start_time

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

    # goes before the follower so a tag we read this frame steers this frame
    ar_update(time.time())

    gap_follow_update()

    # turning way faster than a normal corner means we're probably sliding
    spin = abs(imu.turn_rate())
    if spin > SKID_DEADZONE:
        speed -= SKID_KD * (spin - SKID_DEADZONE)

    speed = rc_utils.clamp(speed, MIN_SPEED, MAX_SPEED)
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
    )

    # keep this SHORT. the matrix is 8x24 and scrolls anything longer at ~2
    # chars/sec, so a long readout every frame never finishes scrolling.
    if race_start_time is not None and time.time() - race_start_time < STARTED_DISPLAY_TIME:
        rc.display.show_text("GO")
    elif gap_mode == "leftmost":
        rc.display.show_text("L")   # a tag is pinning us left
    elif gap_mode == "rightmost":
        rc.display.show_text("R")
    else:
        rc.display.show_text(str(int(imu.heading())))

def update_slow():
    print("Speed:", speed, "Angle:", angle)
    print("Left:", left_dist, "Right:", right_dist)
    # gap mode, and what the tag reader is sitting on. new/dup at the end is the
    # frame filter. if dup keeps climbing and new doesn't, the camera has
    # stalled and it is not that there are no tags
    print("Gap mode:", gap_mode, "| AR:",
          watcher.summary() if watcher is not None else "off")
    # heading drifting while the car sits still = calibration didn't take
    print("AHRS ready?", imu.ready)
    print("Heading:", round(imu.heading(), 1), "Turn rate:", round(imu.turn_rate(), 1))
    # until this says LOCKED, yaw is still gyro-only and still drifting
    print("Compass:", imu.mag_status())
    print("Roll:", round(imu.roll, 3), "Pitch:", round(imu.pitch, 3))

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
