# How the AR code works

Three files are involved:

**`grand_prix/ar_detector.py`** — the detector. It's deliberately minimal: it answers one boolean question, *has tag 0 been seen yet*, and latches on the answer.

- `_Aruco` (ar_detector.py:42) is just a compatibility shim. OpenCV 4.7 moved marker detection onto an `ArucoDetector` object and renamed the dictionary constructors, so this picks the new API if `cv2.aruco.ArucoDetector` exists and the old free-function API otherwise. Same `detect(gray) -> (corners, ids)` either way.
- `ARTagGate.poll(image)` (ar_detector.py:100) is the whole algorithm:
  1. If already latched, return immediately — it stops doing work for the rest of the run.
  2. Dedupe frames with a cheap fingerprint: `int(image[::40, ::40, 0].sum())`. The camera runs ~30 Hz but `update()` runs at 60, so racecar_core hands the same frame twice; without this, one look at the tag would count as two votes.
  3. Grayscale → `detectMarkers`.
  4. For each decoded marker, drop it unless its id is in `self.ids`. Measure size as **the mean of the four edge lengths divided by frame height** (ar_detector.py:131) — not the bounding box, because a tag seen at an angle has a fat bounding box but honest edges.
  5. Keep the largest such tag. If `size >= min_size` (0.03), increment `hits`; otherwise reset `hits` to 0 — the frames must be **consecutive**.
  6. At `hits >= need` (3), set `latched = True` forever.

  The 3-frame counter isn't really false-positive protection — an ArUco codeword carries error-correcting bits, so it either decodes or doesn't. It's there so a single lucky decode off a reflection can't commit the car early. That's also why `MIN_SIZE` can be 0.03 here while the sign detector needs 0.10.
- `reset()` clears the latch; the race script calls it in `start()` because the gate object is built once and a stale latch would send the car hunting for the elevator off the starting line.

**`grand_prix/grand_prix_AHRS.py`** — the consumer. Config at lines 98–108, use at line 348. In state `RACE`, every update it polls the gate; the first time it returns true, the car transitions `RACE → APPROACH` and calls `set_gap_mode("leftmost")`. The tag is taped to the **right** wall, but the elevator is on the **left** — the tag is a thing to look at, not a direction to drive. After that the Coral GO/STOP sign reader takes over.

The import is wrapped in try/except (line 56) — if OpenCV lacks `aruco`, the car prints a warning and races without the gate rather than dying at import.

**`grand_prix/show_ar_tags.py`** — the debug viewer. Runs the same gate over a USB cam, a video file, or racecar_core, and prints/streams the `sz` numbers so you can tune `AR_MIN_SIZE` by walking the car back until `sz` drops below your candidate. Note it builds a *second* `_Aruco` for drawing (line 224), because the gate stops decoding once latched.

# The exact tag

```python
AR_DICT = "DICT_6X6_250"
AR_IDS  = (0,)
AR_MIN_SIZE = 0.03
AR_NEED = 3
```

So: **ArUco marker id 0 from the 6×6 dictionary**, the one on the right wall before the elevator. Tags 1–4 exist elsewhere on the course and are explicitly filtered out — leaving `AR_IDS = None` would make the first tag the car passes trigger the elevator sequence.

`DICT_6X6_250` vs `_50` vs `_100` doesn't matter for id 0: the smaller 6×6 dictionaries are prefixes of the larger ones, so a tag printed from any of them decodes identically here.

`grand_prix/elevator_tag.png` is the reference photo of the course tag — the 6×6 marker you see above. The identification workflow is `show_ar_tags.py --image elevator_tag.png` (show_ar_tags.py:58), which brute-forces every `DICT_*` OpenCV has (including the AprilTag families, which are a different family entirely and won't decode as ArUco) and prints the `AR_DICT`/`AR_IDS` lines to paste in. I can't run that here — no Python on this machine — so I'm reporting the id the code is configured for, not a decode I performed. An ArUco id is an error-correcting codeword and genuinely can't be read off the image by eye, and a wrong `AR_IDS` fails silently, so that's worth re-running on the car before the race.

# Verification

No racecar library on this machine, so this is a by-hand review — nothing here was executed. Scoping to the AR path end to end: decode → latch → what the car does about it.

## Verdict

The detector logic itself is sound. Three things would hurt on a real run, one of them badly.

**1. `AR_MIN_SIZE = 0.03` latches from much too far away.** That's the real problem. On a 480-row frame, 0.03 means an average tag edge of ~14 px. For a ~15 cm tag on a RealSense at 640×480 (f ≈ 465 px), that's roughly 4.8 m out. The file's own reasoning — "aruco either decodes or does not" — is true but points the wrong way: it means the gate fires the instant the tag becomes decodable at all, which is both far away *and* non-deterministic, since bit extraction at <2 px per module succeeds intermittently. So the trigger distance isn't 4.8 m reliably, it's "somewhere between 2 and 5 m, differently each run."

**2. An early latch then degrades the whole rest of the run.** This is what makes #1 expensive rather than cosmetic. The moment `APPROACH` starts:
- `grand_prix_AHRS.py:484` caps speed at `approach_speed(front_dist, HOLD_DIST_CM)`, which at any distance over ~2 m returns `ELEV_MAX_SPEED = 0.45` — below `MIN_SPEED = 0.533`. So the car drops below its own racing floor for everything that follows.
- `set_gap_mode("leftmost")` plus a constant 0.30 left push (`grand_prix_AHRS.py:507`) applies through any corners still between the tag and the elevator.

**3. `hits = 0` on a miss is fragile at speed.** `need=3` requires three *consecutive* decoded frames. One motion-blurred frame during the pass throws away the good frames either side of it and restarts the count.

## What I changed

Diff is 2 files, 39 insertions. Summary of what's now different:

| | before | after |
|---|---|---|
| `MIN_SIZE` | 0.03 (~4.8 m) | 0.06 (~2.5 m) |
| miss handling | `hits = 0` | `hits -= 1` |
| APPROACH throttle cap | from the moment the tag fires | only inside `APPROACH_SLOW_CM = 250` |

The third change is what makes the second-guessing on `MIN_SIZE` cheap: an early latch now costs you leftmost-gap mode and the left bias, but no longer costs racing speed for the remainder of the course.

## What I could not verify

**The dictionary is the one binary risk and it's still open.** If the course tags aren't `DICT_6X6_250`, `detectMarkers` returns nothing, the gate never latches, and the car drives past the elevator with no error message anywhere. Nothing in the repo proves `elevator_tag.png` decodes as id 0 — that rests on someone having run the identify step. Re-run it on the car:

```
python3 show_ar_tags.py --image elevator_tag.png
```

**`MIN_SIZE = 0.06` is a reasoned starting point, not a measured one.** I don't know the physical tag size, and the geometry cuts an extra way I should flag: the tag is on the right wall and the car drives *parallel* to it, so it's viewed at a steep incidence angle until quite close. Foreshortening shrinks the mean edge length, so 0.06 fires nearer than the head-on 2.5 m estimate — possibly much nearer. That's the safe direction, but it means the real number has to come off `show_ar_tags.py --racecar --http`: park the car where you want the gate to fire and read `sz`. If the gate never fires on a practice run, `AR_MIN_SIZE` is the first thing to drop.

**`left_bias()` uses `get_lidar_closest_point(scan, (270, 360))`** — I can't confirm racecar_utils accepts an endpoint of exactly 360 rather than 359, since the library isn't on this machine. Worth one sim run to confirm it doesn't throw or wrap to the wrong window.

**Timing.** `gap_follow_update()` makes 362 `get_lidar_average_distance` calls per frame, and `gate.poll()` adds a full-frame `detectMarkers` on top. `update()` is nominally 60 Hz — a 16.6 ms budget. The repeat-frame filter halves the ArUco cost, which is the right instinct, but if the loop overruns on the Jetson the whole state machine slows down. `update_slow()` already prints `new/dup`; if `dup` climbs while `new` stalls, the camera stalled rather than there being nothing to see.

None of this was executed — no Python, Node, or Docker on this machine, so these are hand-review conclusions and the constants above are untested on hardware.
