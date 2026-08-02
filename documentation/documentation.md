# BWSI RACECAR 2026 - TEAM 4 DOCUMENTATION

# Wall Follower

Written by: Andrew Pan, Jason Ma, Jason Zeng

## Algorithms

- Midline Tracking

  - Tries to minimize the distance between the right and left walls through LIDAR scans

- Gap Follower

  - Finds the largest continuous gap from LIDAR scans and follows the midpoint of the gap

- Elastic Autonomous Tracking System (EATS)

  - Finds the angle of the largest distance on the left and right side through LIDAR scans, and follows the average of the angles weighted on left and right distances

## Selection

- Midline Tracking

  - Pros: The midline tracking algorithm is extremely easy to code and tune.

  - Cons: The midline tracking algorithm is subject to wall variability and doesn’t have the ability to look forwards, making it jerky at high speeds.

- Gap Follower

  - Pros: The gap follower algorithm is much more accurate than the midline tracking algorithm because of its ability to look ahead, allowing it to react to changes in the course earlier.

  - Cons: The gap follower algorithm is harder to tune and code and sometimes takes inefficient racing lines.

- Elastic Autonomous Tracking System (EATS)

  - Pros: The EATS algorithm is the most accurate algorithm for the RACECAR and follows optimal racing lines.

  - Cons: The EATS algorithm is very difficult to code and tune.

## Decision Matrix

Explaining the Decision Matrix:

We are utilising a decision matrix to find out which algorithm is the most optimal for us to use. We will rank all of them out of 10 on 4 aspects: Speed, Consistency, Ease to code, and efficiency. Afterwards, we add up the scores, and whichever one scores highest on our decision matrix will be the solution that we select.

Category Explanation

Speed: Can it maintain a high speed without lag?

Consistency: Out of 10 times, how many times can it complete the track without crashing?

Ease to code: How easy is the logic to understand and how easy is it to develop our code? Efficiency: Does it take the most efficient path (such as a racing line) in order to traverse the course?

| Solution | Speed | Consistency | Ease to code | Efficiency |
| --- | --- | --- | --- | --- |
| Midline Tracking | 4 | 5 | 10 | 2 |
| Gap Follower | 9 | 9 | 8 | 7 |
| EATS | 10 | 8 | 5 | 9 |

## Scores:

Midline Tracking: 4+5+10+2 = 21

Gap Follower: 9+9+8+7=33

EATS: 10+8+5+9=32

Because of this data, we have selected the Gap Follower to use for the Grand Prix as it is the best overall.

## Testing Solutions:

There are many ways to program a wall follower. To see which one works the best, we referred to our speed quest code and our previous lab codes:

VERSION 1: ORIGINAL GAP FOLLOWER (Pre Speed Quest)

```python
# grabs the 180 degree fov in front of the car, centered on heading
# reordered so it goes left (-90) to right (+90) instead of wrapping awkwardly
def _front_samples(scan):
    sample_count = len(scan)
    half_count = sample_count // 4  # quarter of the scan = 90 degrees
    # scan wraps around 0/360 and "front" straddles that wrap
    # left_side = last chunk of array (angles just left of front, coming from the back)
    # right_side = first chunk (angles just right of front)
    left_side = list(range(sample_count - half_count, sample_count))
    right_side = list(range(0, half_count + 1))
    ordered_indices = left_side + right_side  # now it's actually left to right, sane order
    front_samples = []
    sample_span = max(1, len(ordered_indices) - 1)  # just making sure we don't divide by 0
    for ordered_index, sample_index in enumerate(ordered_indices):
        # map position in the arc to an angle from -90 to +90
        sample_angle = -90.0 + 180.0 * ordered_index / sample_span
        front_samples.append((sample_angle, scan[sample_index]))
    return front_samples
# cleans up a single lidar reading. if it's None/nan/zero/negative, that
# means "nothing there" so just call it max range instead of treating it like
# there's a wall 0 inches away
def _clean_distance(distance):
    if distance is None or not math.isfinite(distance) or distance <= 0:
        return MAX_LIDAR_DISTANCE
    return distance
# finds whichever sample is closest to the angle we care about and gives back
# its (cleaned) distance. "What's the distance at roughly this bearing"
def _nearest_sample_distance(samples, target_angle):
    closest_angle, closest_distance = min(
        samples,
        key=lambda sample: abs(sample[0] - target_angle),
    )
    return _clean_distance(closest_distance)
# walks across the front samples looking for the longest stretch of "open" angles
# (distance > threshold), finding the biggest gap/hallway in front of us.
# returns the average angle/dist of that gap, how long it was, and whether we found
# anything open at all
def _largest_open_run(samples, threshold):
    best_run = []       # longest open run we've found so far
    current_run = []    # the run we're currently building
    found_open = False  # did ANY sample clear the threshold
    use_tracking = False # is there at least one sample that's REALLY open (not just barely)
    for sample_angle, sample_distance in samples:
        sample_distance = _clean_distance(sample_distance)
        # if something's very open, we trust the tracking more
        # (helps avoid chasing noise from a barely open gap)
        if sample_distance > USE_TRACKING_THRESHOLD:
            use_tracking = True
        if sample_distance > threshold:
            # open! keep building the run
            found_open = True
            current_run.append((sample_angle, sample_distance))
        else:
            # hit a wall/obstacle, run's over here
            # save it if it's the best one so far, then reset
            if len(current_run) > len(best_run):
                best_run = current_run
            current_run = []
    # need to check this in case the run goes all the way to the end of the array
    # (never hits the else branch to get saved otherwise)
    if len(current_run) > len(best_run):
        best_run = current_run
    # if len(best_run) == 0:
        #     # Fallback: no sample exceeded the threshold, so use the single farthest sample.
        #     best_index = max(range(len(samples)), key=lambda i: samples[i][1])
        #     best_run = [samples[best_index]]
    # only actually report a target if we found a run AND it's a "real" opening
    # (use_tracking), otherwise just default to straight ahead so we don't
    # steer based on garbage
    if len(best_run) != 0 and use_tracking:
        run_angles = [sample[0] for sample in best_run]
        run_distances = [sample[1] for sample in best_run]
        average_angle = sum(run_angles) / len(run_angles)
        average_distance = sum(run_distances) / len(run_distances)
    else:
        average_angle = 0.0
        average_distance = 0.0
    return average_angle, average_distance, len(best_run), found_open
def wall_following():
    scan = rc.lidar.get_samples()
    # quick check, is there something directly in front of us (narrow +/-5 deg window)
    LIDAR_angle, LIDAR_dist = rc_utils.get_lidar_closest_point(scan, (-5, 5))
    # the reordered 180 deg front arc, for gap finding and side wall checks
    front_samples = _front_samples(scan)
    # whole scan but with bad readings swapped out for max range
    cleaned_scan = [_clean_distance(distance) for distance in scan]
    max_lidar_distance = max(cleaned_scan)
    # distance right at the wheels, used to catch us hugging a wall too hard
    left_wall_distance = _nearest_sample_distance(front_samples, -45.0)
    right_wall_distance = _nearest_sample_distance(front_samples, 45.0)
    # find the widest open gap ahead and steer toward the middle of it
    target_angle, target_distance, open_run_length, found_open =
_largest_open_run(front_samples, OPEN_DISTANCE_THRESHOLD)
    # NOTE: this should probably be dividing by delta not multiplying?
    # (standard PD deriv is (target - last) / delta) might want to double check the math here
    delta = rc.get_delta_time()
    deriv = target_angle - (last_target_angle) * delta
    last_target_angle = target_angle
    # classic PD steering. P term pulls us toward center of the gap,
    # D term smooths it out so we're not jerking the wheel around
    angle = TURN_KP * target_angle + TURN_KD * deriv
    # too close to the wall on the left? steer away (positive = right)
    # scaled by how deep into the danger zone we are
    if left_wall_distance < SIDE_WALL_THRESHOLD:
        angle += SIDE_WALL_KP * (SIDE_WALL_THRESHOLD - left_wall_distance) / SIDE_WALL_THRESHOLD
    # same deal but on the right (steer left this time)
    if right_wall_distance < SIDE_WALL_THRESHOLD:
        angle -= SIDE_WALL_KP * (SIDE_WALL_THRESHOLD - right_wall_distance) / SIDE_WALL_THRESHOLD
    # keep steering in bounds
    angle = rc_utils.clamp(angle, -1.0, 1.0)
    dt = rc.get_delta_time()
    # slow down the sharper we're turning, then clamp to allowed speed range
    speed = 1 - 0.6 * (abs(angle))
    speed = rc_utils.clamp(speed, MIN_SPEED, 1)
    # keep track of race time while we're actually running
    if isRunning:
        time += dt
    rc.drive.set_speed_angle(speed, angle)

```

VERSION 2: UPDATED GAP FOLLOWER (During Speed Quest)

```python
def gap_follow_update():
    global left_dist, right_dist

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
            best_run = current_run

        if best_run:
            target_angle = sum(best_run) / len(best_run)
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
```

Gap Follower Version 2 is the most concise and easy to understand, which is why we selected to use it for the grand prix. Our Version 1 of the gap follower used lots of functions, which made reading the code easier but was more complex.

# Start Detection

Written by: Jason Zeng

## Start Detection Logic

1. Crops the frame to the middle-lower region (skips sky and near-bottom).

2. Finds green contours in that crop via HSV thresholding.

3. Takes the largest contour and measures its area.

4. Returns True only if a green contour exists and its area exceeds START_AREA_THRESHOLD (i.e., the marker is close enough); returns False if no green is found at all.

VERSION 1: START DETECTION

```python
def start_detection():
    image = rc.camera.get_color_image() #looks for the color
    if image is None:
        return False
    h = image.shape[0]
    crop = image[h // 3 : 7 * h // 8, :]
    green_contours = rc_utils.find_contours(crop, GREEN[0], GREEN[1])
    green_c = rc_utils.get_largest_contour(green_contours)
    green_a = rc_utils.get_contour_area(green_c)
    # if green is not detected, don't start
    if green_c is None:
        return False

    if green_a > START_AREA_THRESHOLD:
        return True
```

# Dot Matrix

Written by: Paul Sopin

Start Gating Logic

Each loop iteration, while race_started is False:

1. Call start_detection()

2. If a large enough green contour is detected → set race_started = True, display "STARTED", and fall through to run driving logic in that same frame.

3. If there is no green contour, or a green contour that isn’t big enough→ force set_speed_angle(0, 0), display "NOT STARTED", and return early (skip driving logic).

VERSION 1: Not Started/Started Dot Matrix

```python
if not race_started:
    if start_detection():
        race_started = True
        rc.display.show_text("STARTED")
    else:
        rc.drive.set_speed_angle(0, 0)
        rc.display.show_text("NOT STARTED")
        return
```

# Telemetry

## Finding the problem:

Tuning the gap follower during the Grand Prix requires rapid iteration under a strict time limit. Watching the car drive does not reveal how the proportional term actually responds to steering error, so we needed a way to inspect controller error as a function of time. With that data in hand, gain values can be chosen from measured response rather than guesswork.

## Brainstorming

Logging and plotting: Record numeric sensor and controller values throughout a run and plot them afterward. This approach works with nearly every sensor on the platform and exposes trends that are not apparent from visual observation of the vehicle.

Record and replay: Stream the full LIDAR and camera feeds to disk during each run so the entire run can be reconstructed offline. This yields the most complete picture of vehicle state, but it is considerably more complex to build and maintain.

Record and Replay:

- Pros: Allows us to have a full understanding of what the RACECAR sees

- Cons: It takes time to develop and is computationally expensive (causes lag)

Logging and plotting:

- Pros: It is simple to code and allows us to understand how the PID controller is reacting

- Cons: The logging and plotting do not give an in-depth view of the RACECAR’s POV.

To help us select a solution, we will use a decision matrix to rank each option out of 5 on 3 categories:

- Ease to code:

  - How easy is it for us to develop and use this code?

- Assists us in understanding the POV of the Racecar

  - How much can we see from the perspective of the RACECAR?

- Computationally expensive? (0 for yes, 5 for no)

  - Will it create lag?

## Decision Matrix:

| Option | Ease to code | POV | Lag? |
| --- | --- | --- | --- |
| Record and Replay | 3 | 5 | 0 |
| Logging and Plotting | 5 | 2 | 5 |

## Results:

Record and Replay: 3+5+0=8

Logging and Plotting: 5+2+5=12

## Select Solution:

Based on our need to tune the P Controller for wall following, logging and plotting is the best option for us because it is less computationally heavy and creates less lag than Record and Replay. Logging and plotting also show us a direct graph of the error over time, which can streamline our process for making the values effective.

Here is our code for the telemetry:

```python
"""
telemetry.py
lightweight telemetry + debugging harness for the RACECAR wall-follower.
this used to roll its own CSV logger plus a companion analyze_telemetry.py
script for plotting. turns out racecar_neo already has a built in telemetry
module (rc.telemetry) that handles recording and graphing for us, so now
we're just wrapping that instead of reinventing csv writers. Still keeping:
  1. the live on screen debug HUD (rc.telemetry doesn't do live viewing,
     it's record now, graph later, so the HUD is still pulling real weight)
  2. a thin wrapper class so wall_following.py doesn't need to change much
Gone:
  csv.DictWriter code, file handles, flush(), close(), and the whole
  analyze_telemetry.py script. rc.telemetry.visualize() spits out the graph
  automatically when the program exits, so no separate script needed anymore.
Usage (inside wall_following.py):
    from telemetry import TelemetryLogger
    logger = TelemetryLogger(rc)   # create once, outside the main loop
    def wall_following():
        ...
        logger.log(
            target_angle=target_angle,
            target_distance=target_distance,
            left_wall=left_wall_distance,
            right_wall=right_wall_distance,
            angle=angle,
            speed=speed,
            kp_term=TURN_KP * target_angle,
        )
        logger.draw_hud(rc, target_angle, angle, speed)
    # nothing to call at the end, rc.telemetry.visualize() runs automatically
    # when the program exits and drops a graph for you
"""
# order here matters. rc.telemetry.record() is positional, not a dict like
# our old logger.log(**fields) used to be. so we hardcode the field order in
# one spot (this tuple) and everything downstream just follows it. don't go
# rearranging this list without checking wall_following.py too.
FIELD_ORDER = (
    "target_angle",
    "target_distance",
    "left_wall",
    "right_wall",
    "angle",
    "speed",
    "kp_term",
)
class TelemetryLogger:
    def __init__(self, rc):
        """
        heads up, this needs the rc object now (didn't used to) since the
        recording lives on rc.telemetry instead of some file we open ourselves.
        grab it once and stash it, no more os.makedirs or open() calls.
        """
        self._rc = rc
        # declare_variables only actually does something the first time it's
        # called ever. so if you change FIELD_ORDER later just restart the
        # script, calling this again does nothing (that's expected, not a bug)
        self._rc.telemetry.declare_variables(*FIELD_ORDER)
        self._frame = 0  # just for our own sanity, rc.telemetry timestamps on its own
    def log(self, **fields):
        """
        push one frame of telemetry into rc.telemetry.
        still takes keyword args like the old version since that's nicer to
        read at the call site, but internally we convert to the positional
        order rc.telemetry.record() wants, using FIELD_ORDER above.
        if you forget a field from FIELD_ORDER or typo a key this throws a
        KeyError, which is a good thing, better than silently
        logging garbage or mismatched columns.
        """
        try:
            values = tuple(fields[name] for name in FIELD_ORDER)
        except KeyError as missing:
            raise KeyError(
                f"missing telemetry field {missing}, expected all of: {FIELD_ORDER}"
            ) from missing
        self._rc.telemetry.record(*values)
        self._frame += 1
        # no flush(), no file handle, none of that. rc.telemetry handles
        # persistence under the hood now, we just feed it data.
    # ---------------------------------------------------------------- --
    # live debug HUD (rc.telemetry doesn't do real time viz, so this stays)
    # ---------------------------------------------------------------- --
    def draw_hud(self, rc, target_angle, angle, speed):
        """
        draws a little debug overlay on the car's display, a text readout
        plus a bar showing where the controller thinks the open gap is,
        relative to straight ahead. handy to watch live instead of waiting
        for the post run graph from rc.telemetry.visualize().
        """
        hud_text = f"tgt:{target_angle:5.1f} ang:{angle:+.2f} spd:{speed:.2f}"
        # bar goes from -90 (left) through 0 (center) to +90 (right)
        bar_width = 21  # odd width on purpose so there's a clean center tick, don't change to even
        center = bar_width // 2
        pos = int(center + (target_angle / 90.0) * center)
        pos = max(0, min(bar_width - 1, pos))  # clamp so we don't go out of bounds if angle is wild
        bar = ["-"] * bar_width
        bar[center] = "|"   # straight ahead marker, always here no matter what
        bar[pos] = "X"      # where the controller is currently aiming
        hud_bar = "".join(bar)
        rc.display.show_text(hud_text + "\n" + hud_bar)
    # ---------------------------------------------------------------- --
    # graph gen. technically optional since rc.telemetry auto calls
    # visualize() on program exit, but if you want to force a graph mid
    # session (like during a long test run without killing the script)
    # this does it manually.
    # ---------------------------------------------------------------- --
    def save_graph(self):
        """
        manually trigger a graph dump early if you don't want to wait for
        the program to exit. not required for normal use, just here in case
        someone's debugging a run that takes forever and wants a peek.
        """
        self._rc.telemetry.visualize()
```

log() reorders the logs to match the FIELD_ORDER, and feeds them into rc.telemetry.record() so the library can create a time graph when the program exits. draw_hud()prints a live text readout and displays the target angle and speed. While the program runs, you get a visual printout log, and after the run ends you can view the entire graph to better tune PID.

# AHRS Integration

Written By: Jason Ma

## Idea:

The main problem with our original AHRS node is that it was computationally expensive and hard to run (since it used an EKF). For the Grand Prix, we decided to run a lightweight version of our AHRS code alongside our Gap Follower. This fuses the IMU’s gyroscope and accelerometer using a complementary filter to find pitch and roll, and we estimate heading (yaw), turn rate, and roll/pitch; it feeds this information back into the speed controller.

## Why did we do this?

Currently, the wall follower only sees the RACECAR’s surroundings, but it doesn’t see if the car can actually do the motion. In past runs, we noticed that the RACECAR will sometimes slip off course due to high speeds. Now, we can detect the change in heading to see how fast the heading changes, and compare it to how fast the heading should actually change, and the car slows down accordingly to prevent slippage.

## Problems with the original code:

Our original code requires a ROS2 node to run, and we found it easier to make it run using the RACECAR core instead of ROS2.

```python
"""
Team 4 - baby AHRS for the grand prix
the same idea as our state_estimation attitude_node but much smaller
because that one is a whole ros2 package and takes cant run ros on top of racecar_core
during the race. this just reads the imu and outputs heading + how fast
we're spinning.
"""
import math
import time
CALIB_FRAMES = 120     # ~2 sec of just sitting there. dont move the car during this
ACCEL_TRUST = 0.02     # how much we believe the accelerometer. keep it small or it gets jittery
GRAVITY = 9.81
def wrap(a):
    # keeps angles between -pi and pi so it doesnt count to 900 degrees
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
class AHRS:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0         # radians, 0 = whichever way we were pointed at the start
        self.yaw_rate = 0.0    # rad/s, + is one way - is the other
        self.ready = False     # goes True once the gyro bias is figured out
        self._bias = 0.0
        self._sum = 0.0
        self._n = 0
        self._last = None
    def update(self, rc):
        ax, ay, az = rc.physics.get_linear_acceleration()
        gx, gy, gz = rc.physics.get_angular_velocity()
        now = time.time()
        if self._last is None:
            self._last = now
            return
        dt = now - self._last
        self._last = now
        # if dt is huge something froze, just skip that frame
        if dt <= 0 or dt > 0.5:
            return
        # THE TRICKY PART: even sitting perfectly still the gyro reads like
        # 0.01 rad/s and if you integrate that you drift badly. so we sit
        # there at the red light, average it, and subtract it off forever
        if not self.ready:
            self._sum += gz
            self._n += 1
            if self._n >= CALIB_FRAMES:
                self._bias = self._sum / self._n
                self.ready = True
            return
        self.yaw_rate = gz - self._bias
        self.yaw = wrap(self.yaw + self.yaw_rate * dt)
        # roll/pitch: gyro is smooth but drifts, gravity doesnt drift but is noisy
        # so just mix them (complementary filter, way easier than a kalman)
        self.roll += gx * dt
        self.pitch += gy * dt
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        # only trust gravity when we're not slamming into things
        if abs(mag - GRAVITY) < 2.0:
            self.roll = (1 - ACCEL_TRUST) * self.roll + ACCEL_TRUST * math.atan2(ay, az)
            self.pitch = (1 - ACCEL_TRUST) * self.pitch + ACCEL_TRUST * math.atan2(
                -ax, math.sqrt(ay * ay + az * az)
            )
    def heading(self):
        # degrees is just easier to read on the display
        return math.degrees(self.yaw)
    def turn_rate(self):
        return math.degrees(self.yaw_rate)
```

## How the code works:

Essentially, the first 2 seconds it averages the bias of the system, which is then subtracted from future samples.

Next, yaw is the integral of yaw rate, which is between [-pi, pi]

Yaw is relative, so zero is wherever the car is started.

For the complementary filter, gyro is smooth, but it drifts; however, the accelerometer is jittery but accurate. We have 0.02 for the accel trust, as otherwise the system will become very jittery. (although we can tune it later)

The gap follower sets the speed based on the open distance ahead, so it will carry a fast speed into a turn that the wheel cannot hold. If the change in heading is greater than what we expect, then we know that the car is slipping, so we then proportionally slow the turn down.

SKID_DEADZONE is the turn rate that is accepted, if it is too low then it will slow down every time, if it is too high then the car will still slip.

# Preparing for dynamic obstacles

We plan on traversing the fork dynamic obstacle, which requires us to traverse the fork through one of the gates. To accomplish this, we added 3 modes to the gap follower:

“largest”

“rightmost”

“leftmost”

The largest mode follows the largest gap like a normal gap follower, the rightmost mode follows the rightmost gap, and the leftmost mode follows the leftmost gap. This effectively allows us to traverse the fork dynamic obstacle really effectively based on the AR marker orientation.

# AHRS Integration Update

## Issues:

Before, we only used the gyroscope for the yaw readings. This caused a lot of drift,

which accumulated over time. To address this issue, we decided to look into different sensors.

Gyroscope:

- Pros:

  - Smooth

  - Fast

  - Free from lag

- Cons:

  - Drifts endlessly

Compass:

- Pros:

  - Never drifts

  - Always converges to present value (ground truth)

- Cons:

  - Jittery

  - Noisy

## Possible solutions:

Due to the nature of the 2 sensors, we believe that it is best to fuse them together, since they complement each other through their strengths and weaknesses.

## Selecting Solutions:

Extended Kalman Filter:

- Pros:

  - Re-evaluates the weight of each sensor every time

- Cons:

  - Hard to code

  - Hard to tune

Complementary Filter:

- Pros:

  - Simple to tune

  - Easy to code

- Cons:

  - Weights always stay the same

## Decision Matrix:

We will rank each other out of 5 in 3 areas: Ease to code, Ease to tune, and Accuracy (if tuned well).

- Ease to code:

  - How easy is it for us to develop and code this?

- Ease to tune:

  - How long will it take for us to tune it?

- Accuracy:

  - How accurate will the measurements be?

| Option | Ease to code | Ease to tune | Accuracy |
| --- | --- | --- | --- |
| EKF | 2 | 3 | 5 |
| Complementary | 5 | 5 | 3 |

EKF: 2+3+5 = 10

Complementary = 5+5+3 = 13

Due to the complementary filter’s ease of coding and tuning, we will select it for the grand prix (since we have limited time with it).
