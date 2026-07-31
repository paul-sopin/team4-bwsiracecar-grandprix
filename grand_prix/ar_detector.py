#!/usr/bin/env python3
"""
Team 4 - the AR tag before the elevator

There is one tag on the course, taped to the left wall, and it means one thing:
the elevator is next, so hug the left and start watching for the GO / STOP sign.
This file answers one question, has the tag been seen, and it only answers it
once.

It is much smaller than the version we had at the split. That one measured which
way up each tag was and ran the answer through a weighted voting window, so an
upright tag sent us left and an upside down one sent us right. We do not need any
of that now. A tag that only ever means one thing does not need reading, it needs
noticing.

So: decode, drop anything too small or the wrong id, count frames. `need` frames
in a row and seen() stays true for the rest of the run. The counter is not
really there for false positives, since an aruco tag carries error correcting
bits and either decodes or does not. It is there so one lucky frame off a
reflection cannot commit us to the elevator early.

That is also why MIN_SIZE can be 0.03 here when the sign detector needed 0.10.

    from ar_detector import ARTagGate
    gate = ARTagGate()
    if gate.poll(rc.camera.get_color_image()):
        ...   # tag found, and it stays found
"""

import cv2
import numpy as np

DEFAULT_DICT = "DICT_6X6_250"   # what the course tags should be printed from
MIN_SIZE = 0.03                 # average tag edge over frame height, under which
                                # the decode is too marginal to count


class _Aruco:
    """detectMarkers, across OpenCV versions.

    4.7 moved this onto an ArucoDetector object and renamed the constructors.
    The car and a laptop venv are usually on different OpenCVs and race morning
    is a bad time to find that out.
    """

    def __init__(self, dict_name=DEFAULT_DICT):
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "this OpenCV has no aruco module, try opencv-contrib-python")
        key = getattr(cv2.aruco, dict_name, None)
        if key is None:
            raise ValueError("unknown aruco dictionary: " + str(dict_name))

        if hasattr(cv2.aruco, "ArucoDetector"):          # OpenCV 4.7 and up
            self._detector = cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(key),
                cv2.aruco.DetectorParameters())
            self._legacy = None
        else:                                            # older
            self._detector = None
            self._legacy = (cv2.aruco.Dictionary_get(key),
                            cv2.aruco.DetectorParameters_create())

    def detect(self, gray):
        if self._detector is not None:
            corners, ids, _rejected = self._detector.detectMarkers(gray)
        else:
            dictionary, params = self._legacy
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray, dictionary, parameters=params)
        return corners, ids


class ARTagGate:
    """Have we passed the tag yet. Latches true and stays there."""

    def __init__(self, dictionary=DEFAULT_DICT, ids=None, min_size=MIN_SIZE,
                 need=3):
        """
        ids       tag ids to accept, or None for any of them
        min_size  average tag edge over frame height, under which we ignore it
        need      frames in a row with a tag in them before we call it seen
        """
        self._aruco = _Aruco(dictionary)
        self.ids = None if ids is None else set(int(i) for i in ids)
        self.min_size = float(min_size)
        self.need = int(need)

        self.hits = 0            # frames in a row with a big enough tag in them
        self.latched = False     # set once, never cleared
        self.last_size = 0.0     # nearest tag last time we looked, for tuning
        self.last_id = None
        self.new = self.dup = 0  # frame counts, for summary()
        self._seen = None

    def poll(self, image):
        """Score one new frame and return seen().

        Repeat frames are skipped. The camera runs at about 30 Hz and update()
        at 60, so racecar_core hands us most frames twice, and counting one look
        at the tag as two votes would halve what `need` is worth.
        """
        if self.latched or image is None:
            return self.latched

        fingerprint = int(image[::40, ::40, 0].sum())   # cheaper than a hash
        if fingerprint == self._seen:
            self.dup += 1
            return self.latched
        self._seen = fingerprint
        self.new += 1

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids = self._aruco.detect(gray)

        height = image.shape[0]
        best = 0.0
        best_id = None
        if ids is not None:
            for quad, tag_id in zip(corners, ids.flatten()):
                tag_id = int(tag_id)
                if self.ids is not None and tag_id not in self.ids:
                    continue
                pts = quad.reshape(4, 2).astype(np.float32)
                # average of the four edges, not the bounding box height. a tag
                # seen at an angle has a bigger box but the edges stay honest
                edges = np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1)
                size = float(edges.mean()) / height
                if size > best:
                    best, best_id = size, tag_id

        self.last_size, self.last_id = best, best_id
        if best >= self.min_size:
            self.hits += 1
            if self.hits >= self.need:
                self.latched = True
                print("[ar] tag id{} seen (size {:.3f}) -> elevator ahead".format(
                    best_id, best))
        else:
            self.hits = 0     # the frames have to be in a row, so one stray
                              # decode from across the room cannot add up
        return self.latched

    def seen(self):
        return self.latched

    def reset(self):
        """Forget the tag. The race script calls this in start(), because we
        build the gate once and a latch left over from the last run would send
        the car looking for the elevator off the line."""
        self.hits = 0
        self.latched = False
        self.last_size, self.last_id = 0.0, None
        self._seen = None

    def summary(self):
        if self.latched:
            return "LATCHED id{} frames new/dup={}/{}".format(
                self.last_id, self.new, self.dup)
        return "{}/{} near={} sz{:.3f} frames new/dup={}/{}".format(
            self.hits, self.need,
            "id{}".format(self.last_id) if self.last_id is not None else "-",
            self.last_size, self.new, self.dup)


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
