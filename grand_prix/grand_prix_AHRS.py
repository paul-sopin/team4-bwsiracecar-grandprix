import os
import sys
import time

import racecar_core
import racecar_utils as rc_utils

# so ahrs.py and telemetry.py get found when this is run from another directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ahrs import AHRS
from telemetry import TelemetryLogger

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

# min gap width in degrees, only for the side modes. without it leftmost will
# happily steer at a 1 degree sliver of noise at the edge of the scan.
MIN_GAP_WIDTH = 5

speed = 0.0 # current speed
angle = 0.0 # current angle

left_dist = 0.0
right_dist = 0.0

left_angle = 0.0
right_angle = 0.0

gap_mode = GAP_MODE # current mode, changed by set_gap_mode()

target_angle = 0.0 # midpoint of the chosen gap, global so telemetry can see it
race_start_time = None # set when green is detected, times the GO message

race_started = False # whether the light has turned green

imu = AHRS() # heading and turn rate
logger = None # made in start()

def start():
    global logger, gap_mode

    rc.drive.set_max_speed(1.0)
    rc.drive.set_speed_angle(0, 0)

    # reset, so a mode switched last run doesn't carry into this one
    gap_mode = GAP_MODE

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


# switch which gap we aim at, mid run. nothing calls this yet. whatever ends up
# taking a split calls set_gap_mode("leftmost"), then "largest" once we're past.
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
        for deg in range (-90, 0, 1):
            cur_scan = rc_utils.get_lidar_average_distance(scan, deg % 360, 0.5)
            if cur_scan > left_dist:
                left_dist = cur_scan

        right_dist = 0
        for deg in range (0, 91, 1):
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
    global speed, angle, race_started
    global left_dist, right_dist
    global left_angle, right_angle
    global target_angle, race_start_time

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
    )

    # keep this SHORT. the matrix is 8x24 and scrolls anything longer at ~2
    # chars/sec, so a long readout every frame never finishes scrolling.
    if race_start_time is not None and time.time() - race_start_time < STARTED_DISPLAY_TIME:
        rc.display.show_text("GO")
    else:
        rc.display.show_text(str(int(imu.heading())))

def update_slow():
    print("Speed:", speed, "Angle:", angle)
    print("Left:", left_dist, "Right:", right_dist)
    print("Left Angle:", left_angle, "Right Angle:", right_angle)
    # heading drifting while the car sits still = calibration didn't take
    print("AHRS ready?", imu.ready)
    print("Heading:", round(imu.heading(), 1), "Turn rate:", round(imu.turn_rate(), 1))
    print("Roll:", round(imu.roll, 3), "Pitch:", round(imu.pitch, 3))

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
