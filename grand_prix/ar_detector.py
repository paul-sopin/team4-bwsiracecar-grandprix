#!/usr/bin/env python3
"""ArUco tag detection for RACECAR, with the tag's in-plane rotation.

This is the perception half of the Trial 3A sign detector, rebuilt around AR
tags. Same job -- look at the camera, tell the driving code which way to go --
but the evidence is a printed marker instead of a sign the CNN has to recognise,
and the thing we read off it is its ORIENTATION, not its class:

    upright  ->  one gap mode
    upside down (180 deg)  ->  the other

Why tags instead of the Coral model:

  1. No Edge TPU. The model needed `/dev/apex_0` free, a 4.2 MB weights file,
     tflite_runtime and libedgetpu. A tag is decoded by OpenCV, which the car
     already has loaded for everything else.
  2. Rotation is recoverable for free. ArUco returns the four corners starting
     from the marker's own printed top-left, so the direction of that first
     edge in the image IS the tag's rotation. A CNN box gives you no such thing.
  3. Identity is checksummed. A dictionary marker carries error-correcting bits,
     so a false positive on a random yellow tube is close to impossible. That
     lets us trust smaller (further away) tags than the sign detector could.

Cost control, in order of impact:

  1. `every_n` -- detection is the expensive part and a tag we drive toward is
     in view for seconds, so there is no reason to run it every frame.
  2. `scale` -- detect on a downscaled copy. Halving the frame is ~4x cheaper
     and costs detection range, which is why it is not the default.
  3. Grayscale conversion done once here rather than inside detectMarkers.

Unlike sign_detector.py this does NOT renice or pin the process by default.
That module ran inside a script whose only job was perception, so making the
whole process nice 10 on core 3 was fine. This one is imported by the race
script, and deprioritising the control loop would be a real problem. Pass
`core=`/`niceness=` explicitly if you are running a standalone tool.

Also unlike sign_detector.py: no `int | None`, so this imports on Python 3.9
and can be tested on a laptop venv, not just the car.

Usage:
    from ar_detector import ARTagDetector, UPRIGHT, FLIPPED
    det = ARTagDetector()
    for tag in det.detect(frame_bgr):
        print(tag.id, tag.orientation, tag.roll)
"""

import math
import os
from collections import namedtuple

import cv2
import numpy as np

# What a tag can be facing. Anything in between (a tag on its side, or one
# caught mid-tumble) classifies as None and is ignored -- guessing on an
# ambiguous tag is worse than waiting for the next frame.
UPRIGHT = "UPRIGHT"
FLIPPED = "FLIPPED"

DEFAULT_DICT = "DICT_6X6_250"   # what the RACECAR course tags are printed from
ANGLE_TOL = 50.0                # deg from 0/180 still counted as that facing

# id      dictionary id of the tag
# roll    in-plane rotation in degrees, 0 = upright, +/-180 = upside down.
#         positive is counter-clockwise as you look at the image.
# orientation  UPRIGHT / FLIPPED / None (ambiguous)
# box     (x1, y1, x2, y2) bounding box in frame pixels, for drawing
# size    mean edge length as a fraction of frame height. this is the
#         "how close is it" number -- rotation invariant, unlike the box height
# corners 4x2 float array, marker order (top-left, top-right, bottom-right,
#         bottom-left of the tag as printed)
ARTag = namedtuple("ARTag", "id roll orientation box size corners")


def limit_cpu(core=None, niceness=0):
    """Keep a standalone tool off the control loop's back.

    Only worth calling from tools that are NOT the race script -- both of these
    affect the whole process, so calling it from a script that also drives would
    deprioritise the driving.
    """
    if niceness:
        try:
            os.nice(niceness)
        except OSError:
            pass
    if core is not None and hasattr(os, "sched_setaffinity"):
        try:
            os.sched_setaffinity(0, {core})
        except OSError:
            pass


def wrap180(deg):
    """Fold an angle into -180..180 so 190 and -170 compare as the same thing."""
    return (deg + 180.0) % 360.0 - 180.0


def roll_degrees(corners):
    """In-plane rotation of a tag, from its corners.

    detectMarkers always hands the corners back starting at the marker's own
    printed top-left and going clockwise, whatever way the tag is turned. So
    corner0 -> corner1 is the tag's top edge, and where that edge points in the
    image is the tag's rotation. Nothing else has to be computed.

    Image y grows downward, so negate dy to get the usual counter-clockwise
    positive angle.
    """
    dx = float(corners[1][0] - corners[0][0])
    dy = float(corners[1][1] - corners[0][1])
    return math.degrees(math.atan2(-dy, dx))


def classify(roll, angle_tol=ANGLE_TOL):
    """UPRIGHT / FLIPPED / None for a roll angle in degrees."""
    off = abs(wrap180(roll))
    if off <= angle_tol:
        return UPRIGHT
    if off >= 180.0 - angle_tol:
        return FLIPPED
    return None     # on its side: we cannot say, so we do not


class _Aruco:
    """detectMarkers across OpenCV versions.

    4.7 moved the API to an ArucoDetector object and renamed the constructors.
    The car and a laptop venv are not usually on the same OpenCV, and finding
    that out on race day is not the plan.
    """

    def __init__(self, dict_name=DEFAULT_DICT):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "this OpenCV has no aruco module -- pip install opencv-contrib-python"
            )
        key = getattr(cv2.aruco, dict_name, None)
        if key is None:
            raise ValueError("unknown aruco dictionary: " + str(dict_name))

        if hasattr(cv2.aruco, "ArucoDetector"):          # OpenCV >= 4.7
            dictionary = cv2.aruco.getPredefinedDictionary(key)
            self._detector = cv2.aruco.ArucoDetector(
                dictionary, cv2.aruco.DetectorParameters())
            self._legacy = None
        else:                                            # OpenCV < 4.7
            self._detector = None
            self._legacy = (cv2.aruco.Dictionary_get(key),
                            cv2.aruco.DetectorParameters_create())

    def detect(self, gray):
        """Return (corners, ids) exactly as detectMarkers does."""
        if self._detector is not None:
            corners, ids, _rejected = self._detector.detectMarkers(gray)
        else:
            dictionary, params = self._legacy
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray, dictionary, parameters=params)
        return corners, ids


class ARTagDetector:
    def __init__(self, dictionary=DEFAULT_DICT, ids=None, angle_tol=ANGLE_TOL,
                 every_n=1, scale=1.0, core=None, niceness=0):
        """
        dictionary  ArUco dictionary name, e.g. "DICT_6X6_250"
        ids         iterable of tag ids to keep, or None for every tag
        angle_tol   degrees from 0/180 that still count as that facing
        every_n     run detection on one frame in n, repeat the last result
                    in between. detect() is the expensive call
        scale       detect on a copy shrunk by this much. < 1.0 is faster and
                    sees less far, coordinates come back in full-frame pixels
        core        pin the WHOLE PROCESS to this core. tools only, see limit_cpu
        niceness    renice the WHOLE PROCESS. tools only, see limit_cpu
        """
        limit_cpu(core, niceness)
        self._aruco = _Aruco(dictionary)
        self.ids = None if ids is None else set(int(i) for i in ids)
        self.angle_tol = float(angle_tol)
        self.every_n = max(1, int(every_n))
        self.scale = float(scale)
        self._frame = 0
        self._last = []

    def detect(self, frame_bgr):
        """Return [ARTag, ...] for this frame, nearest (largest) first."""
        self._frame += 1
        if self._frame % self.every_n:
            return self._last                  # skipped frame: last answer stands
        if frame_bgr is None:
            return self._last

        height = frame_bgr.shape[0]
        img = frame_bgr
        if self.scale != 1.0:
            img = cv2.resize(frame_bgr, None, fx=self.scale, fy=self.scale,
                             interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids = self._aruco.detect(gray)
        found = []
        if ids is not None:
            for quad, tag_id in zip(corners, ids.flatten()):
                tag_id = int(tag_id)
                if self.ids is not None and tag_id not in self.ids:
                    continue
                pts = quad.reshape(4, 2).astype(np.float32)
                if self.scale != 1.0:
                    pts = pts / self.scale     # back to full-frame pixels

                roll = roll_degrees(pts)
                # mean of the four edges. the bounding box grows when a tag is
                # turned, the edges do not, so this is the honest size measure
                edges = np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1)
                size = float(edges.mean()) / height

                x1, y1 = pts.min(axis=0)
                x2, y2 = pts.max(axis=0)
                found.append(ARTag(
                    tag_id, roll, classify(roll, self.angle_tol),
                    (int(x1), int(y1), int(x2), int(y2)), size, pts))

        found.sort(key=lambda t: -t.size)      # nearest tag first
        self._last = found
        return found


def make_tag(tag_id, px=600, dictionary=DEFAULT_DICT, border=0.15):
    """Render a printable tag, white quiet zone included.

    The quiet zone is not decoration: detectMarkers finds candidates by looking
    for a dark quad on a light background, so a tag printed edge to edge on the
    paper often will not be found at all.
    """
    aruco = cv2.aruco
    key = getattr(aruco, dictionary)
    dic = (aruco.getPredefinedDictionary(key) if hasattr(aruco, "ArucoDetector")
           else aruco.Dictionary_get(key))
    draw = getattr(aruco, "generateImageMarker", None) or aruco.drawMarker
    img = draw(dic, int(tag_id), int(px))
    pad = int(px * border)
    return cv2.copyMakeBorder(img, pad, pad, pad, pad,
                              cv2.BORDER_CONSTANT, value=255)


if __name__ == "__main__":
    import argparse
    import time

    ap = argparse.ArgumentParser(description="benchmark the AR tag detector")
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--dict", default=DEFAULT_DICT)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--image", default=None,
                    help="test image. default is a synthetic upright tag 0")
    ap.add_argument("--core", type=int, default=None)
    args = ap.parse_args()

    det = ARTagDetector(dictionary=args.dict, scale=args.scale, core=args.core,
                        niceness=10 if args.core is not None else 0)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit("cannot read " + args.image)
    else:
        # a real tag pasted into a 480p frame, so the timing is measured on
        # something the detector actually has to decode
        frame = np.full((480, 640, 3), 200, np.uint8)
        tag = cv2.cvtColor(make_tag(0, px=200, dictionary=args.dict),
                           cv2.COLOR_GRAY2BGR)
        h, w = tag.shape[:2]
        frame[140:140 + h, 220:220 + w] = tag

    det.detect(frame)                                   # warm up
    t_cpu0, t0 = time.process_time(), time.perf_counter()
    for _ in range(args.n):
        tags = det.detect(frame)
    wall = (time.perf_counter() - t0) / args.n
    cpu = (time.process_time() - t_cpu0) / args.n
    print("{:.1f} ms/frame ({:.0f} FPS)".format(wall * 1000, 1 / wall))
    print("CPU {:.1f} ms/frame -> {:.0f}% of one core".format(
        cpu * 1000, cpu / wall * 100))
    print("tags:", [(t.id, t.orientation, round(t.roll, 1), round(t.size, 3))
                    for t in tags])
