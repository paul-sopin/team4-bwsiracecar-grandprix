"""
Team 4 - deciding what the AR tags mean

ar_detector.py looks at one frame. This turns a stream of those into one answer
we are willing to act on: which way up the tag at the split is, and so which
side we take. Ported over from sign_support.py in the Trial 3A repo, with the
sign classes swapped out for tag facings.

Two things came across from that version because both of them earned it.

The first is that we only score new frames. The camera runs at about 30 Hz and
update() runs at 60, so racecar_core hands us most frames twice. Running
detection on a repeat wastes the CPU, and counting it twice is worse, because
one observation ends up as two votes and the window is half as good at throwing
out a bad read. The fingerprint adds up about 200 pixels, which is a lot cheaper
than hashing the whole frame.

The second is that the votes are weighted instead of counted. Counting throws
away the thing that says most about whether a reading is any good, which is how
big the tag was. Here every frame adds

    min(1, size / trigger_size)

to whichever facing it saw, and a facing fires once its total over the window
gets to `need`. A tag further down the course still adds up, just slower,
instead of being thrown out for being small, and one bad read adds a fraction
and falls out of the window well before anything reaches the threshold.

What did not come across is the confidence term. The Coral model gave us a score
per box and that score was doing real work, because the model would put a 0.5
box on a yellow tube. Aruco decodes error correcting bits, so there is nothing
to weight by and nothing to bury. Same reason MIN_SIZE here is 0.03 and the sign
version needed 0.10.

ground_lights.py did not come across either. It read the red and green wall
strips, and start_detection() in grand_prix_AHRS.py already gets us off the line
on green.
"""

from collections import deque

from ar_detector import (ARTagDetector, ARTag, FLIPPED, UPRIGHT, classify,
                         roll_degrees, wrap180)

__all__ = ["ARTagDetector", "ARTag", "ARWatcher", "UPRIGHT", "FLIPPED",
           "classify", "roll_degrees", "wrap180"]


MIN_SIZE = 0.03   # under this the corners are too rough to get an angle out of.
                  # the id is probably still right, but 0 against 180 is the
                  # part we care about and that needs corners
MARGIN = 1.5      # the winner has to beat the runner up by this much, so a tag
                  # being read both ways gives us nothing instead of flipping
                  # the gap mode back and forth at the split


class ARWatcher:
    """Adds up weighted votes per tag facing over a sliding window."""

    def __init__(self, dictionary="DICT_6X6_250", ids=None, angle_tol=50.0,
                 trigger_size=0.10, vote_n=7, every_n=1, scale=1.0,
                 core=None, niceness=0):
        """
        trigger_size  tag size (average edge over frame height) worth a whole
                      vote. bigger than that is still worth exactly one
        vote_n        how many frames the window holds
        everything else goes straight through to ARTagDetector
        """
        self.detector = ARTagDetector(dictionary=dictionary, ids=ids,
                                      angle_tol=angle_tol, every_n=every_n,
                                      scale=scale, core=core, niceness=niceness)
        self.trigger_size = float(trigger_size)
        self.window = deque(maxlen=vote_n)   # per frame: {facing: weight}
        self.new = self.dup = 0              # frame counts, for summary()
        self.last_tags = []                  # newest detections, for the viewer
        self._seen = None

    def poll(self, image):
        """Score one new frame. Repeats get skipped, see the top of the file."""
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
            # two tags the same way up in one frame is still one look at that
            # facing, so take the closer one's weight
            frame[tag.orientation] = max(frame.get(tag.orientation, 0.0), weight)
        self.window.append(frame)

    def totals(self):
        out = {}
        for frame in self.window:
            for facing, weight in frame.items():
                out[facing] = out.get(facing, 0.0) + weight
        return out

    def count(self, facing):
        """Frames in the window that saw this facing, so is it still in view."""
        return sum(facing in frame for frame in self.window)

    def winner(self, need):
        """The facing that got to `need` and is clearly ahead, or None."""
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
        """Empty the window. Call it after acting on a tag, or the same tag can
        fire again off the votes it already cast."""
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
