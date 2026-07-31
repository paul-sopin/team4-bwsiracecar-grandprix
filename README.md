# BWSI RACECAR 2026 - TEAM 4 GRAND PRIX

> Important note: Jason Ma's laptop broke at the start of week 4. Most of the work after that happened on Jason Zeng's laptop, which makes Github show the commit as the person whose device was used. The person that actually did the commit’s name is in the description.

Our Grand Prix code goes as follows: Car sits at the line, waits for the stoplight to turn green, then runs the course on a gap follower. An AHRS node goes with it and reads heading, turn rate, and it makes sure the racecar doesn’t slip. Two things gets put into that: an AR tag on the left wall tells us the elevator is coming up, and a compass keeps the heading from drifting. The elevator itself is run off its GO / STOP sign, which we read on the Coral.

## Quick Start

```bash
# cd to your racecar folder first

git clone https://github.com/paul-sopin/team4-bwsiracecar-grandprix
cd team4-bwsiracecar-grandprix

# Run on the car
racecar sim grand_prix/grand_prix_AHRS.py
```

At the start of the match, do not touch the racecar for 2 seconds. Gyro bias gets measured while we're sitting at the light.

What the dot matrix shows/means:

| Shows | Means |
| :--- | :--- |
| CALIB | gyro's still calibrating |
| READY | done calibrating, waiting on green stoplight|
| GO | up for about a second once we start |
| ELV | tag seen, hugging the left, watching for the sign |
| WAIT | the sign said STOP, sitting 30 inches off the wall |
| IN | the sign said GO, driving into the elevator |
| DONE | parked inside |
| -12 | that's live heading, in degrees |

Everything's simplified since the matrix is 8x24 and it scrolls anything longer at maybe two characters a second, which is no use at all once the car is moving.

## Repository Layout

| Path | Purpose |
| :--- | :--- |
| grand_prix/grand_prix_AHRS.py | the race script, start detection, gap follower, traction limiter, display |
| grand_prix/ahrs.py | AHRS: gyro bias calibration, heading, turn rate, roll and pitch, compass filter |
| grand_prix/mag.py | reads the compass off /mag, the only ROS we touch during a race |
| grand_prix/ar_detector.py | the AR tag gate before the elevator. no racecar imports |
| grand_prix/elevator_signs.py | the elevator's GO / STOP sign, on the Coral |
| grand_prix/best_v5_edgetpu.tflite | the weights that reads, added from the Trial 3A repo |
| grand_prix/show_ar_tags.py | draws what both readers see, and prints test tags. doesn't drive |
| grand_prix/telemetry.py | thin layer over rc.telemetry, plus the live debug HUD |
| integration-challenges-progress.md | where we track Trial 4A |
| integration-challenges.png | the Trial 4A sheet itself |

---

## Architecture

We do not run a ROS 2 stack for this code. The AHRS is a plain Python class which update() calls it directly. Nothing extra to start when you're standing at the line and the light is about to change.

The one exception is mag.py. The compass is not on rc.physics, it is an LSM9DS1 publishing on the /mag topic, so that one file subscribes to it and nothing else. It is a subscriber, not a node we have to launch, and if it finds nothing there the AHRS goes back to gyro-only and the car still races.

There is a real ROS 2 AHRS, which is state_estimation, from Trial 2D. Even though trial 2D’s estimator is slightly more accurate, it also wants a colcon build and a launch file already running before it gives us any reading.

What reads what:

```mermaid
flowchart LR
CAM[camera] --> SD["start_detection()"]
CAM --> AR["elevator_update()"]
LID[lidar] --> GF["gap_follow_update()"]
LID --> FD["front_distance()"]
IMU[imu] --> AH["AHRS.update()"]
MAG["/mag compass"] --> AH
AR -->|"set_gap_mode()"| GF
FD -->|"cm to the wall"| AR
SD --> U["update()"]
GF --> U
AH --> U
U --> DRV["set_speed_angle()"]
U --> DISP["dot matrix"]
U --> TEL["telemetry"]
```

And one frame of update():

```mermaid
flowchart TD
I["imu.update(rc)"] --> R{"race_started?"}
R -->|no| S["start_detection()"]
S -->|green| Y["race_started = True<br/>show GO"]
S -->|no green| H["set_speed_angle(0, 0)<br/>show CALIB / READY<br/>return early"]
R -->|yes| F["front_distance()"]
Y --> F
F --> A["elevator_update()<br/>tag, then GO / STOP"]
A --> G["gap_follow_update()"]
G --> E{"at the elevator?"}
E -->|no| K{"turn rate ><br/>SKID_DEADZONE?"}
K -->|yes| B["cut speed"]
K -->|no| V["set_speed_angle(speed, angle)"]
B --> V
E -->|yes| W["speed from the front wall<br/>30 in on STOP, 5 cm on GO"]
W --> V
V --> L["logger.log(...)<br/>show heading"]
```

---

## Gap Follower
Andrew Pan, Jason Ma, Jason Zeng

Reads the lidar readings from -90 to 90. Anything past the OPEN_THRESHOLD is open, and the largest open gap is selected, and the car steers at the middle of that run. Speed is calculated from the average of the furthest left and right readings, which means it is really fast on straights, but slows down during turns (it is basically an EATS controller).

We decided to use a gap follower based on a decision matrix against Midline Tracking and EATS. Scored on speed, consistency, ease to code, efficiency:

- Gap Follower: 33
- EATS: 32
- Midline Tracking: 21

Two versions of the follower exist. V1 came before Speed Quest and splits the logic over separate functions. Even though it is nicer to read, it is harder to edit. V2 got written during the Speed Quest.

### Gap modes

By default it takes the largest gap it can find. set_gap_mode() switches that to leftmost or rightmost, which pins the car to one side of the course instead. That is for the work dynamic obstacle.

The side modes skip gaps narrower than MIN_GAP_WIDTH degrees. Without that, leftmost will steer at a one degree gap of noise at the edge of the scan. If nothing on that side is wide enough it falls back to the largest gap. largest skips the check completely, so it behaves the same as it always did. 

## AR Tag Detection
Paul Sopin

One tag, taped to the left wall just before the elevator, and it means one thing: the elevator is next. Seeing it calls set_gap_mode("leftmost") so we hug that side the rest of the way in, and it turns the sign reader on.

That makes this much simpler than what we had at the split, where a tag's orientation picked left or right and a weighted voting window decided whether to believe it. A tag with one meaning does not need reading, it needs noticing, so ar_detector.py decodes, throws out anything too small, and latches after AR_NEED frames in a row. It never unlatches. An aruco tag carries error correcting bits so a decode is either a real tag or nothing, and the frame counter is only there so one reflection cannot commit us early.

## The Elevator
Paul Sopin

Once we get past the tag, the elevator shows a sign and we do what it says:

| Sign | What we do |
| :--- | :--- |
| STOP | wait 30 inches (HOLD_DIST_CM) off the wall |
| GO | drive in until the wall is ENTER_DIST_CM away, then park |

We keep reading the sign the whole way in, since the board shows STOP first and GO later and we have to catch that change while sitting in front of it. A STOP that shows up after we have started in is only obeyed while there is still room to stop.


```bash
sudo kill $(sudo lsof -t /dev/apex_0)
sudo lsof /dev/apex_0          # blank means it is free
```

If it is not, the race script prints why and drives anyway, it just cannot read the sign. The same is true of the tag reader, so neither of them can keep the car off the line.

One thing to know about ENTER_DIST_CM: the lidar cannot actually see 5 cm. Its minimum range is somewhere around 15 and under that the samples come back 0.0, which racecar_utils reads as no data. So we call it arrived when the front goes blank right after reading something shorter than LIDAR_BLIND_CM. Stopping early means lowering that number, stopping late means raising it.

To check both before a run:

```bash
cd grand_prix

# no tag handy? print one and tape it up
python3 show_ar_tags.py --make-tag 0

# what the car sees, at http://10.42.0.1:8000
python3 show_ar_tags.py --racecar --http

# same, with the Coral running too
python3 show_ar_tags.py --racecar --http --signs
```


## Start Detection
Paul Sopin

Crop down to the middle and lower part of the frame so the sky is gone, and it reads the  HSV threshold for green inside that crop. It then takes the biggest contour and measures the area. It returns True on two conditions: there's a green contour, and the contour is bigger than START_AREA_THRESHOLD.

## Dot Matrix
Andrew Pan

When race_started is False, the camera reads start_detection(). If the green contour is big enough, then race_started becomes True, and GO appears on the screen. No green: set_speed_angle(0, 0), display shows whatever the calibration state is, return early.

For a while it just said NOT STARTED. However, we added CALIB to see when the IMU is calibrating, which is very important so we know when the IMU is finished calibrating.

## Telemetry
Jason Zeng

We decided on a graph of error over time for our telemetry system.

The other idea we had was downloading full LIDAR and camera to disk and rebuilding entire runs offline. The issue was this is that we did not have enough time to develop this, and it lags the system too much while running.

telemetry.py is a helper for the built in rc.telemetry. log() which puts fields into FIELD_ORDER, passes them to rc.telemetry.record(), and the graph shows up when the script exits. Gap angle, both wall distances, steering angle, speed, heading, turn rate and gap bias are all logged. Gap bias is the gap mode as a number, -1 for leftmost, 0 for largest, 1 for rightmost, so on the graph you can line it up against the steering angle and see whether a tag fired where you expected it to.

draw_hud() prints out the target angle, steering angle, speed, and a bar for where the controller is pointing in the consoler.

## AHRS
Jason Ma

For the first two seconds we average the gyro bias, then subtract it off every sample from then on. A gyro that is completely motionless still has a bit of drift, and the more we integrate that the more the heading drifts.

Yaw: integrate yaw rate, turn it into -pi to pi. It's relative and not absolute (it starts at 0 at calibration).

Roll and pitch it calculated through a complementary filter. Gyro is smooth, and it drifts. Gravity is noisy, and it doesn't. ACCEL_TRUST sits at 0.02. We tried higher, it got jittery.

### Compass, for yaw

Gravity pulls roll and pitch straight every frame. Nothing in an accel and gyro IMU knows which way is north, so yaw is pure integration and it drifts a lot. The compass is the only sensor on the car that can correct it, so we use the same complementary filter we used for the pitch and roll: 98% of the gyro, 2% of the compass, MAG_TRUST set to 0.02 for the same reason ACCEL_TRUST is.

The compass is not on rc.physics, instead it is an LSM9DS1 on the /mag topic, which is what mag.py is for. These are what is needed for the filter:

- Same frame. Gyro yaw is relative to startup, compass heading is relative to north. At lock in we record the offset between them, so yaw keeps meaning degrees from wherever we started and the logs don't change meaning halfway through a run.
- Same direction. If the compass reads mirrored the correction would push yaw the wrong way.


### Traction limiter

Speed comes from open distance ahead, which means the gap follower will carry all of it into a corner. If the turn rate is higher than normal turning should be able to produce, we call it a slip, and we  cut speed in proportion to how far over the line it is.

SKID_DEADZONE is how much turning is considered to be normal. Of every constant in here, it's the one that needs the most tuning.

## Tuning

| Constant | File | Effect |
| :--- | :--- | :--- |
| OPEN_THRESHOLD | grand_prix_AHRS.py | how far away still counts as open space |
| GAP_TURN_KP | grand_prix_AHRS.py | how hard it yanks toward the gap |
| MIN_SPEED / MAX_SPEED | grand_prix_AHRS.py | speed envelope |
| SKID_DEADZONE | grand_prix_AHRS.py | turn rate we're still willing to call cornering |
| SKID_KD | grand_prix_AHRS.py | brake strength per deg/s over that |
| CALIB_FRAMES | ahrs.py | calibration length. has to finish before green |
| ACCEL_TRUST | ahrs.py | accelerometer weight in roll/pitch |
| MAG_TRUST | ahrs.py | compass weight in yaw. same idea as ACCEL_TRUST |
| MAG_OFFSET | ahrs.py | hard iron offset. fill it in to skip the calibration circle |
| MIN_GAP_WIDTH | grand_prix_AHRS.py | narrowest gap the side modes will aim at |
| AR_DICT | grand_prix_AHRS.py | which AR dictionary the course tag comes from |
| AR_MIN_SIZE | grand_prix_AHRS.py | smallest tag we act on, so really the trigger distance |
| AR_NEED | grand_prix_AHRS.py | frames in a row before the tag counts |
| AR_DETECT_EVERY_N | grand_prix_AHRS.py | frame skipping. the main CPU lever |
| SIGN_NEED | grand_prix_AHRS.py | evidence before we act on GO or STOP. lower reacts sooner and trusts less |
| SIGN_TRIGGER_H | grand_prix_AHRS.py | sign size worth a full vote, so the distance we read it from |
| HOLD_DIST_CM | grand_prix_AHRS.py | where we wait on STOP. 76.2 is 30 inches |
| ENTER_DIST_CM | grand_prix_AHRS.py | where we stop on GO |
| LIDAR_BLIND_CM | grand_prix_AHRS.py | how short a reading has to be before a blank one means we arrived |
| ELEV_KP / ELEV_MAX_SPEED | grand_prix_AHRS.py | how fast we close on the wall |
| CLASS_ROW | elevator_signs.py | which model output row is GO and which is STOP |

Once a second, update_slow() dumps speed, angle, wall distances, the race state, the front distance, what both readers are sitting on, the compass state, heading, turn rate, roll and pitch. To set AR_MIN_SIZE or SIGN_TRIGGER_H, run show_ar_tags.py --racecar --signs, walk the car back until sz drops under the number you are trying, and measure the floor. To set SKID_DEADZONE: drive a lap clean, watch the Turn rate line, put the deadzone a little over the biggest number honest cornering produced. Set it low and the car brakes in every corner. Set it high and you get the slide you were trying to prevent.

---

## Trial 4A Integration Challenges

| # | Challenge | Status |
| :--- | :--- | :--- |
| 1 | Waiting state, green light start | done, grand_prix_AHRS.py |
| 2 | Dot Matrix Display utility | done. calibration state, run state, live heading |
| 3 | Novel telemetry or debugging sequence | done. telemetry.py, live HUD, graph after the run |
| 4 | AHRS node or heading parameters | done, ahrs.py, and the Trial 2D ROS package too |
| 5 | G-Splat with RealSense 435i | no |
| 6 | Occupancy grid of the track | no |
| 7 | Object detector influencing decisions | done. the Coral reads the elevator's GO / STOP sign and the car obeys it |
| 8 | Dynamic obstacle traversal | going for it race day. the gap modes are the groundwork |
| 9 | New sensor under $100 | no |

More detail in [integration-challenges-progress.md](integration-challenges-progress.md).

---

## Known Issues

- The AR tag code, the elevator code and the compass filter have not been run on the car yet. Check them with show_ar_tags.py and the Compass line in update_slow() before a race depends on any of them.
- AR_DICT is DICT_6X6_250, the usual one, but not confirmed against the real course tag. Wrong dictionary means zero detections rather than bad ones.
- GO is row 3 of the model, which was trained as GO_AROUND. It has never seen an elevator GO board. Watch it in show_ar_tags.py --signs against the real sign, and if it will not fire, retrain and change CLASS_ROW.
- ENTER_DIST_CM is 5, which is inside the lidar's blind spot, so arrival is inferred rather than measured. See LIDAR_BLIND_CM above, and test it against a wall before trusting it in a doorway.
