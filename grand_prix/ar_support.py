"""Evidence gathering for the AR tag reader. Ported from Trial 3A's sign_support.

One ARWatcher.poll() per camera frame turns a stream of tag detections into a
single stable answer: which way the tag at the side of the course is facing,
and therefore which side of a split we should take.

Two things carried over from the sign version, both of which earned their place:

  * NEW FRAMES ONLY. The camera publishes at ~30 Hz while update() runs at 60,
    so racecar_core hands out every frame roughly twice. Letting a repeat
    through would run detection twice on one observation and, worse, count that
    observation twice in the voting window -- which halves the window's ability
    to reject a one-frame misread. The fingerprint sums ~200 sampled pixels,
    two orders of magnitude cheaper than hashing the frame.

  * WEIGHTED EVIDENCE, not a K-of-N vote. A plain vote throws away the one thing
    that says most about whether a reading is trustworthy: how big the tag was.
    Here each frame contributes, per facing,

        min(1, size / trigger_size)

    and a facing fires once its sum over the window reaches `need`. A tag far
    down the course still accumulates, just slower, instead of being discarded
    for being small, and a single misread contributes a fraction and drops out
    of the window long before anything reaches the threshold.

What is NOT carried over is the confidence term. The Coral model returned a
score per box and that score was doing real work, because the model would
cheerfully put a 0.5 box on a yellow tube. ArUco decodes error-correcting bits
instead, so a detection is either a valid dictionary marker or nothing at all.
There is no score to weight by and no need for one -- which is also why
MIN_SIZE here is 0.03 where the sign version needed 0.10.

The wall-strip reader (ground_lights.py) did not come across either. This repo
already starts on green in grand_prix_AHRS.start_detection(), and one green
detector is enough.
"""

from collections import deque

from ar_detector import (ARTagDetector, ARTag, FLIPPED, UPRIGHT, classify,
                         roll_degrees, wrap180)

__all__ = ["ARTagDetector", "ARTag", "ARWatcher", "UPRIGHT", "FLIPPED",
           "classify", "roll_degrees", "wrap180"]


MIN_SIZE = 0.03   # smaller than this and the corner fit is too coarse to get a
                  # reliable angle out of -- the id may well be right, but the
                  # 0-vs-180 read is what we care about and that needs corners
MARGIN = 1.5      # the winner must also beat the runner-up by this factor, so
                  # a tag being read both ways reports nothing instead of
                  # flip-flopping the gap mode at a split


class ARWatcher:
    """Accumulates weighted evidence per tag facing over a sliding window."""

    def __init__(self, dictionary="DICT_6X6_250", ids=None, angle_tol=50.0,
                 trigger_size=0.10, vote_n=7, every_n=1, scale=1.0,
                 core=None, niceness=0):
        """
        trigger_size  tag size (mean edge / frame height) worth a full vote.
                      anything bigger is still worth exactly one
        vote_n        how many frames the window holds
        the rest are passed straight through to ARTagDetector
        """
        self.detector = ARTagDetector(dictionary=dictionary, ids=ids,
                                      angle_tol=angle_tol, every_n=every_n,
                                      scale=scale, core=core, niceness=niceness)
        self.trigger_size = float(trigger_size)
        self.window = deque(maxlen=vote_n)   # per frame: {facing: weight}
        self.new = self.dup = 0              # frame counters, for summary()
        self.last_tags = []                  # newest detections, for the viewer
        self._seen = None

    def poll(self, image):
        """Score one NEW frame. Repeats are skipped (see the module docstring)."""
        if image is None:
            return
        fingerprint = int(image[::40, ::40, 0].sum())   # cheaper than a hash
        if fingerprint == self._seen:
            self.dup += 1
            return
        self._seen = fingerprint
        self.new += 1

        tags = self.detector.detect(image)
        self.last_tags = tags

        frame = {}
        for tag in tags:
            if tag.orientation is None or tag.size < MIN_SIZE:
                continue
            weight = min(1.0, tag.size / self.trigger_size)
            # two tags facing the same way in one frame is still one observation
            # of that facing, just take the nearer one's weight
            frame[tag.orientation] = max(frame.get(tag.orientation, 0.0), weight)
        self.window.append(frame)

    def totals(self):
        out = {}
        for frame in self.window:
            for facing, weight in frame.items():
                out[facing] = out.get(facing, 0.0) + weight
        return out

    def count(self, facing):
        """Frames in the window that saw this facing -- the 'still in view' test."""
        return sum(facing in frame for frame in self.window)

    def winner(self, need):
        """Facing whose accumulated evidence reaches `need` and clearly leads."""
        totals = self.totals()
        if not totals:
            return None
        ranked = sorted(totals.values(), reverse=True)
        best = max(totals, key=totals.get)
        runner_up = ranked[1] if len(ranked) > 1 else 0.0
        if totals[best] < need or totals[best] < MARGIN * runner_up:
            return None
        return best

    def clear(self):
        """Drop the window. Call this after acting, so the tag that just fired
        cannot immediately fire again on its own leftover evidence."""
        self.window.clear()

    def summary(self):
        totals = self.totals()
        top = " ".join("{}:{:.1f}".format(f[:4], v) for f, v
                       in sorted(totals.items(), key=lambda kv: -kv[1]))
        if self.last_tags:
            t = self.last_tags[0]
            near = "id{} {:+.0f}deg sz{:.2f}".format(t.id, t.roll, t.size)
        else:
            near = "-"
        return "[{}] near={} frames new/dup={}/{}".format(
            top or "-", near, self.new, self.dup)
