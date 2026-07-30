import os
import sys
import time

import racecar_core
import racecar_utils as rc_utils

# Required so ahrs.py and telemetry.py are found when the script is run from
# another directory
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
SKID_KD = 0.0022 # how hard we brake when the AHRS reports excess rotation
SKID_DEADZONE = 25.0 # deg/s. Normal cornering stays below this and is ignored
STARTED_DISPLAY_TIME = 1.0 # seconds to hold GO on the display before the heading

# Which gap to follow. "largest" is the normal racing behavior and matches what
# the gap follower has always done. "leftmost" and "rightmost" force the car to
# hug one side, which is useful when the course has a split and we already know
# which way we want to go.
VALID_GAP_MODES = ("largest", "leftmost", "rightmost")
GAP_MODE = "largest" # the mode each run starts in

# Ignore gaps narrower than this many degrees when picking a side. Without it,
# leftmost and rightmost will happily steer at a one degree sliver of noise at
# the edge of the scan. This does not apply to "largest", so that mode behaves
# exactly as it did before this option existed.
MIN_GAP_WIDTH = 5

speed = 0.0 # current speed
angle = 0.0 # current angle

left_dist = 0.0
right_dist = 0.0

left_angle = 0.0
right_angle = 0.0

gap_mode = GAP_MODE # the mode in use right now, changed by set_gap_mode()

target_angle = 0.0 # midpoint of the chosen gap, kept global so it can be logged
race_start_time = None # set when green is detected, used to time the STARTED message

race_started = False # whether the light has turned green

imu = AHRS() # supplies heading and turn rate
logger = None # built in start(), see below

def start():
    global logger, gap_mode

    rc.drive.set_max_speed(1.0)
    rc.drive.set_speed_angle(0, 0)

    # Back to the default, so a mode switched during the last run does not carry
    # into this one
    gap_mode = GAP_MODE

    # Built here rather than at import, because the constructor calls
    # rc.telemetry.declare_variables and the racecar is not running yet at
    # import time. declare_variables only takes effect on the first call, so
    # pressing start again is harmless.
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
    # This check has to come before get_contour_area, because get_largest_contour
    # returns None whenever nothing green is in frame, which is every frame until
    # the light changes. The library does not document get_contour_area accepting
    # None, so it is not called until we know we have a real contour.
    if green_c is None:
        return False # not started

    green_a = rc_utils.get_contour_area(green_c)

    if green_a > START_AREA_THRESHOLD:
        return True # started

    return False # green is visible but still too far away


def set_gap_mode(mode):
    """
    Change which gap the follower aims at, while the car is running.

    Nothing calls this yet. It is here so that whatever ends up deciding to take
    a split (a sign detection, a heading from the AHRS, a lap counter) only has
    to call set_gap_mode("leftmost") and then set_gap_mode("largest") again once
    the split is behind us. gap_mode is read fresh every frame, so the change
    takes effect on the next one.

    A bad mode is ignored rather than raised, because this may end up being
    called from perception code mid race and a typo should not stop the car.
    """
    global gap_mode

    if mode not in VALID_GAP_MODES:
        print("ignoring unknown gap mode:", mode, "expected one of", VALID_GAP_MODES)
        return

    if mode != gap_mode:
        print("gap mode:", gap_mode, "->", mode)
        gap_mode = mode


def pick_side_gap(runs, index):
    """
    Take the leftmost (index 0) or rightmost (index -1) gap that is actually
    wide enough to drive through. If nothing clears MIN_GAP_WIDTH, fall back to
    the largest gap, because aiming at the widest opening available is safer
    than aiming at a sliver just because it is on the correct side.
    """
    wide_enough = [run for run in runs if len(run) >= MIN_GAP_WIDTH]
    if wide_enough:
        return wide_enough[index]
    return max(runs, key=len)


def gap_follow_update():
    global left_dist, right_dist, speed, angle, target_angle

    scan = rc.lidar.get_samples()
    if len(scan) != 0:
        # check -90 to 90 and collect every gap, not just the biggest one, so
        # GAP_MODE can pick between them. Negative is left, positive is right.
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

        # runs are already in order from left to right, so leftmost is the first
        # one and rightmost is the last
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

    # Run this EVERY frame, including before the race starts. That is the key
    # detail: the waiting state gives us free calibration time, because the car
    # is not permitted to move yet.
    imu.update(rc)

    if not race_started:
        if start_detection():
            race_started = True
            race_start_time = time.time()
            rc.display.show_text("GO")
        else:
            rc.drive.set_speed_angle(0, 0)
            # Shows whether the gyro has finished calibrating. Without this we
            # would have no way to confirm it before the race begins.
            if imu.ready:
                rc.display.show_text("READY")
            else:
                rc.display.show_text("CALIB")
            return

    gap_follow_update()

    # If the AHRS reports a yaw rate well above normal cornering, the car is
    # likely sliding, so reduce speed until it settles.
    spin = abs(imu.turn_rate())
    if spin > SKID_DEADZONE:
        speed -= SKID_KD * (spin - SKID_DEADZONE)

    speed = rc_utils.clamp(speed, MIN_SPEED, MAX_SPEED)
    rc.drive.set_speed_angle(speed, angle)

    # Record this frame. rc.telemetry graphs all of it when the program exits,
    # which is how SKID_DEADZONE gets set from a measured turn rate rather than
    # a guess.
    logger.log(
        target_angle=target_angle,
        left_dist=left_dist,
        right_dist=right_dist,
        angle=angle,
        speed=speed,
        heading=imu.heading(),
        turn_rate=imu.turn_rate(),
    )

    # One show_text per frame, and keep it short. The matrix is 8x24 and scrolls
    # anything that does not fit, at about 2 characters per second, so a long
    # readout written every frame would never finish scrolling and would be
    # unreadable. GO is held briefly, then the heading, which is at most four
    # characters. The full telemetry goes to the graph and to update_slow().
    if race_start_time is not None and time.time() - race_start_time < STARTED_DISPLAY_TIME:
        rc.display.show_text("GO")
    else:
        rc.display.show_text(str(int(imu.heading())))

def update_slow():
    print("Speed:", speed, "Angle:", angle)
    print("Left:", left_dist, "Right:", right_dist)
    print("Left Angle:", left_angle, "Right Angle:", right_angle)
    # AHRS output. Significant heading drift while the car is stationary means
    # the calibration did not settle correctly.
    print("AHRS ready?", imu.ready)
    print("Heading:", round(imu.heading(), 1), "Turn rate:", round(imu.turn_rate(), 1))
    print("Roll:", round(imu.roll, 3), "Pitch:", round(imu.pitch, 3))

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
