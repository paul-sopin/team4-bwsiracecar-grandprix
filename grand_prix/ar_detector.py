#!/usr/bin/env python3
"""
Team 4 - AR tag detection

Finds AR tags in a camera frame and works out which way up each one is. That is
the whole job. Upright means one thing to the gap follower and upside down means
the other, and the mapping lives in grand_prix_AHRS.py.

This replaces the sign detector we built for Trial 3A. That one ran a YOLO model
on the Coral, which meant /dev/apex_0 had to be free, tflite_runtime and
libedgetpu both had to be installed, and the 4.2 MB weights file had to be
sitting next to the script. The first of those needs a sudo kill after every
reboot. A tag gets decoded by the OpenCV we already have loaded.

It is also a better fit for what we actually want. A sign has to be recognized,
and the model would call a yellow tube a sign often enough that we needed
confidence weighting to bury the false positives. An AR tag carries error
correcting bits, so we either read a real tag or we read nothing. That is why
MIN_SIZE in ar_support.py can be 0.03 where the sign version needed 0.10.

And we get the rotation for free, which a bounding box was never going to give
us. See roll_degrees().

Two things keep the cost down. every_n skips frames, which matters because
detection is the expensive part and a tag we are driving at is in view for
seconds. scale shrinks the frame first, which is faster and sees less far, so it
is off by default.

One thing this does not do is renice or pin the process. sign_detector.py did
both in its constructor, which was fine there because that script only did
perception. This one gets imported by the race script and slowing the control
loop down would be a real problem, so you have to ask for it. See limit_cpu.

There is no `int | None` anywhere in here either, so unlike sign_detector.py it
imports on Python 3.9 and we can test it on a laptop instead of only on the car.

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

# The two ways a tag can be. Anything in between, so a tag on its side or one
# caught mid fall, comes back as None and we leave it alone. Guessing at a tag
# we cannot read is worse than waiting, and 30 more frames are coming this second.
UPRIGHT = "UPRIGHT"
FLIPPED = "FLIPPED"

DEFAULT_DICT = "DICT_6X6_250"   # what the course tags should be printed from
ANGLE_TOL = 50.0                # degrees off 0 or 180 we will still read

# id      which tag it is
# roll    how far it is turned, in degrees. 0 is upright, 180 either way is
#         upside down, and positive is counter clockwise on screen
# orientation  UPRIGHT, FLIPPED, or None if we cannot tell
# box     (x1, y1, x2, y2) in frame pixels, for drawing
# size    average edge length over the frame height, so how close it is. we use
#         this and not the box height because a turned tag has a bigger box but
#         the edges stay the same
# corners 4x2 floats, in tag order: top left, top right, bottom right, bottom
#         left of the tag as it was printed
ARTag = namedtuple("ARTag", "id roll orientation box size corners")


def limit_cpu(core=None, niceness=0):
    """Keep a tool off the control loop's back.

    Only worth calling from something that is not the race script. Both of these
    hit the whole process, so calling it from a script that also drives would
    slow the driving down too.
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
    """Fold an angle into -180 to 180, so 190 and -170 come out the same."""
    return (deg + 180.0) % 360.0 - 180.0


def roll_degrees(corners):
    """How far a tag is turned, from its corners.

    detectMarkers always hands the corners back starting at the tag's own
    printed top left and going clockwise, however the tag is sitting. So corner0
    to corner1 is the top edge of the tag, and wherever that edge points in the
    image is how far the tag is turned. Nothing else needs working out.

    Image y counts downward, so we flip dy to get the angle the way you would
    expect to see it.
    """
    dx = float(corners[1][0] - corners[0][0])
    dy = float(corners[1][1] - corners[0][1])
    return math.degrees(math.atan2(-dy, dx))


def classify(roll, angle_tol=ANGLE_TOL):
    """UPRIGHT, FLIPPED, or None for a roll angle in degrees."""
    off = abs(wrap180(roll))
    if off <= angle_tol:
        return UPRIGHT
    if off >= 180.0 - angle_tol:
        return FLIPPED
    return None     # on its side, so we do not know


class _Aruco:
    """detectMarkers, across OpenCV versions.

    4.7 moved this to an ArucoDetector object and renamed the constructors. The
    car and a laptop venv are usually on different OpenCVs and race morning is a
    bad time to find that out.
    """

    def __init__(self, dict_name=DEFAULT_DICT):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "this OpenCV has no aruco module, try opencv-contrib-python"
            )
        key = getattr(cv2.aruco, dict_name, None)
        if key is None:
            raise ValueError("unknown aruco dictionary: " + str(dict_name))

        if hasattr(cv2.aruco, "ArucoDetector"):          # OpenCV 4.7 and up
            dictionary = cv2.aruco.getPredefinedDictionary(key)
            self._detector = cv2.aruco.ArucoDetector(
                dictionary, cv2.aruco.DetectorParameters())
            self._legacy = None
        else:                                            # older
            self._detector = None
            self._legacy = (cv2.aruco.Dictionary_get(key),
                            cv2.aruco.DetectorParameters_create())

    def detect(self, gray):
        """(corners, ids), the same as detectMarkers gives them."""
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
        dictionary  which aruco dictionary, so "DICT_6X6_250"
        ids         tag ids to keep, or None for all of them
        angle_tol   degrees off 0 or 180 we will still read as that
        every_n     only detect on one frame in n and reuse the last answer in
                    between. detect() is the call that costs us
        scale       shrink the frame this much before detecting. under 1.0 is
                    faster and sees less far. coordinates still come back in
                    full frame pixels
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
        """Tags in this frame, closest first."""
        self._frame += 1
        if self._frame % self.every_n:
            return self._last                  # skipped, so the old answer stands
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
                    pts = pts / self.scale     # back into full frame pixels

                roll = roll_degrees(pts)
                # average of the four edges. turning a tag grows its bounding
                # box but leaves the edges alone, so this is the honest size
                edges = np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1)
                size = float(edges.mean()) / height

                x1, y1 = pts.min(axis=0)
                x2, y2 = pts.max(axis=0)
                found.append(ARTag(
                    tag_id, roll, classify(roll, self.angle_tol),
                    (int(x1), int(y1), int(x2), int(y2)), size, pts))

        found.sort(key=lambda t: -t.size)      # closest tag first
        self._last = found
        return found


def make_tag(tag_id, px=600, dictionary=DEFAULT_DICT, border=0.15):
    """Draw a tag we can print, white border included.

    The border is not decoration. detectMarkers looks for a dark square on a
    light background, so a tag printed right to the edge of the paper often will
    not get found at all.
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

    ap = argparse.ArgumentParser(description="time the AR tag detector")
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--dict", default=DEFAULT_DICT)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--image", default=None,
                    help="test image. default is a tag we draw ourselves")
    ap.add_argument("--core", type=int, default=None)
    args = ap.parse_args()

    det = ARTagDetector(dictionary=args.dict, scale=args.scale, core=args.core,
                        niceness=10 if args.core is not None else 0)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit("cannot read " + args.image)
    else:
        # a real tag pasted into a 480p frame, so we are timing the work of
        # actually decoding one and not just the work of finding nothing
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
