import os
import sys

import racecar_core
import racecar_utils as rc_utils

# Required so ahrs.py is found when the script is run from another directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ahrs import AHRS

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

speed = 0.0 # current speed
angle = 0.0 # current angle

left_dist = 0.0
right_dist = 0.0

left_angle = 0.0
right_angle = 0.0

race_started = False # whether the light has turned green

imu = AHRS() # supplies heading and turn rate

def start():
    rc.drive.set_max_speed(1.0)
    rc.drive.set_speed_angle(0, 0)

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
    green_a = rc_utils.get_contour_area(green_c)

    # if green is not detected, don't start
    if green_c is None:
        return False # not started
    
    if green_a > START_AREA_THRESHOLD:
        return True # started
        

def gap_follow_update():
    global left_dist, right_dist, speed, angle
    
    scan = rc.lidar.get_samples()
    if len(scan) != 0:
        # check -90 to 90, find largest gap
        best_run, current_run = [], []
        for deg in range(-90, 91, 1):
            cur_scan = rc_utils.get_lidar_average_distance(scan, deg % 360, 0.5)
            if cur_scan > OPEN_THRESHOLD or cur_scan == 0:
                current_run.append(deg)
            else:
                if len(current_run) > len(best_run):
                    best_run = current_run
                current_run = []
        if len(current_run) > len(best_run):
            best_run = current_run # finds the biggest gap by comparing gap sizes
    
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

    # Run this EVERY frame, including before the race starts. That is the key
    # detail: the waiting state gives us free calibration time, because the car
    # is not permitted to move yet.
    imu.update(rc)

    if not race_started:
        if start_detection():
            race_started = True
            rc.display.show_text("STARTED")
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

    # Heading on the dot matrix, so the IMU can be confirmed working from trackside
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
