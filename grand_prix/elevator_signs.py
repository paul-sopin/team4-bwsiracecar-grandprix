#!/usr/bin/env python3
"""
Team 4 - the elevator's GO / STOP sign, on the Coral

Added from sign_detector.py and sign_support.py in the Trial 3A repo, cut down
to the one job left. That version watched nine sign classes and the coloured
wall strips. We only need two answers now, GO and STOP, and only at the elevator.

The model runs on the Edge TPU, so this needs tflite_runtime and libedgetpu but
not torch. The CPU only does one resize and an argmax, which matters because the
race script is already sharing four cores with the ROS stack.

Two things came across from sign_support.py because both of them earned it.

Only new frames get scored. The camera runs at about 30 Hz and update() at 60,
so racecar_core hands us most frames twice, and counting one look at the sign as
two votes would make the window half as good at throwing out a bad read.

And votes are weighted, not counted. Each new frame adds

    confidence * min(1, height / trigger_h)

to whichever sign it saw, and a sign fires once its total reaches `need`. A sign
further off still adds up, just slower, instead of being thrown out for being
small, and one weak false positive falls out of the window long before it gets
anywhere near the threshold.

    from elevator_signs import ElevatorSigns, GO, STOP
    signs = ElevatorSigns("best_v5_edgetpu.tflite")
    signs.poll(rc.camera.get_color_image())
    if signs.winner(2.0) == GO:
        ...

Free the Coral first. It stays claimed across a reboot:

    sudo kill $(sudo lsof -t /dev/apex_0)
    sudo lsof /dev/apex_0          # blank means it is free
"""

import os

import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter, load_delegate

EDGETPU_LIB = "libedgetpu.so.1"

# The two answers. Strings and not numbers so a print or a telemetry column says
# what happened without a lookup table.
GO = "GO"
STOP = "STOP"

# Which row of the model's output each one lives in.
#
# best_v5_edgetpu.tflite was trained on the Trial 3A sign set, which had nine
# classes. We read two of those rows and never look at the other seven, so as far
# as the rest of the code is concerned this model knows GO and STOP and nothing
# else.
#
# Row 7 is a real STOP sign. Row 3 is the old GO_AROUND standing in for the
# elevator's GO, because there was no GO placard in the training set. Retrain on
# the real elevator boards and this pair of numbers is the only edit needed.
CLASS_ROW = {GO: 3, STOP: 7}

MIN_H = 0.10   # a box shorter than this fraction of the frame is too far off to
               # mean anything, and small boxes are where the false positives are


def limit_cpu(core=None, niceness=0):
    """Keep a tool off the control loop's back.

    Both of these hit the whole process, so this is for the viewer and the
    benchmark below, not for the race script. Defaults do nothing on purpose.
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


class SignDetector:
    """One frame in, GO and STOP boxes out. No state beyond frame skipping."""

    def __init__(self, model_path, conf=0.35, iou=0.45, every_n=1, core=None,
                 niceness=0):
        """
        conf     score floor, applied in int8 space before any float math
        iou      overlap at which two boxes of the same sign count as one
        every_n  only run the TPU on one frame in n and reuse the last answer.
                 this is the call that costs us
        """
        limit_cpu(core, niceness)
        self.interp = Interpreter(
            model_path=model_path,
            experimental_delegates=[load_delegate(EDGETPU_LIB)],
            num_threads=1,      # the TPU does the work. more CPU threads here
        )                       # would only steal cycles from the control loop
        self.interp.allocate_tensors()

        inp = self.interp.get_input_details()[0]
        out = self.interp.get_output_details()[0]
        self.in_idx, self.out_idx = inp["index"], out["index"]
        self.in_dtype = inp["dtype"]
        self.size = int(inp["shape"][1])
        self.out_scale, self.out_zero = out["quantization"]

        self.names = tuple(CLASS_ROW)                       # ("GO", "STOP")
        self.rows = np.asarray([CLASS_ROW[n] for n in self.names], np.int32)
        self.iou = float(iou)
        self.every_n = max(1, int(every_n))
        self._frame = 0
        self._last = []

        # the score floor pushed into int8 space, so the frame with nothing in it
        # (which is nearly all of them) is thrown out with one integer compare
        # instead of dequantising 2100 candidates first
        self._q_thresh = (conf / self.out_scale + self.out_zero
                          if self.out_scale else conf)

    def _letterbox(self, img):
        """Fit the frame into the model's square input without stretching it."""
        h, w = img.shape[:2]
        r = min(self.size / float(h), self.size / float(w))
        nh, nw = int(round(h * r)), int(round(w * r))
        top, left = (self.size - nh) // 2, (self.size - nw) // 2
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(
            img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        return canvas, r, left, top

    @staticmethod
    def _nms(boxes, scores, iou_thr):
        """Keep the best box out of each overlapping cluster."""
        idx = scores.argsort()[::-1]
        keep = []
        while idx.size:
            i = idx[0]
            keep.append(i)
            if idx.size == 1:
                break
            rest = idx[1:]
            xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
            yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
            xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
            yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
            inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
            area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
            area_r = ((boxes[rest, 2] - boxes[rest, 0])
                      * (boxes[rest, 3] - boxes[rest, 1]))
            idx = rest[inter / (area_i + area_r - inter + 1e-9) <= iou_thr]
        return keep

    def detect(self, frame_bgr):
        """[(name, confidence, (x1, y1, x2, y2)), ...] in frame pixels."""
        self._frame += 1
        if self._frame % self.every_n:
            return self._last                 # skipped, so the old answer stands
        if frame_bgr is None:
            return self._last

        img, ratio, pad_x, pad_y = self._letterbox(frame_bgr)
        img = img[:, :, ::-1]                             # BGR -> RGB
        if self.in_dtype == np.int8:
            x = (img.astype(np.int16) - 128).astype(np.int8)
        elif self.in_dtype == np.uint8:
            x = img
        else:                                             # float model fallback
            x = img.astype(np.float32) / 255.0
        self.interp.set_tensor(self.in_idx, x[None])
        self.interp.invoke()
        raw = self.interp.get_tensor(self.out_idx)[0]     # (4 + 9, 2100)

        # the whole two-class thing is this one line: read our two rows and never
        # look at the other seven
        cls_rows = raw[4 + self.rows]                     # (2, 2100)
        best = cls_rows.max(axis=0)
        hit = np.flatnonzero(best >= self._q_thresh)      # int8-space cut
        if hit.size == 0:
            self._last = []
            return self._last

        scores = ((best[hit].astype(np.float32) - self.out_zero) * self.out_scale
                  if self.out_scale else best[hit].astype(np.float32))
        which = cls_rows[:, hit].argmax(axis=0)           # 0 = GO, 1 = STOP

        box = raw[:4, hit].astype(np.float32)
        if self.out_scale:
            box = (box - self.out_zero) * self.out_scale
        if box.max() <= 2.0:                              # normalised -> px
            box *= self.size
        cx, cy, bw, bh = box
        xyxy = np.stack([cx - bw / 2, cy - bh / 2,
                         cx + bw / 2, cy + bh / 2], axis=1)

        # undo the letterbox, so the boxes come back in the caller's pixels
        h, w = frame_bgr.shape[:2]
        xyxy[:, [0, 2]] = np.clip((xyxy[:, [0, 2]] - pad_x) / ratio, 0, w)
        xyxy[:, [1, 3]] = np.clip((xyxy[:, [1, 3]] - pad_y) / ratio, 0, h)

        found = []
        for k in np.unique(which):                        # NMS per sign
            m = np.flatnonzero(which == k)
            for j in self._nms(xyxy[m], scores[m], self.iou):
                i = m[j]
                found.append((self.names[int(k)], float(scores[i]),
                              tuple(xyxy[i].round().astype(int))))
        found.sort(key=lambda d: -d[1])
        self._last = found
        return found


class ElevatorSigns:
    """Adds up weighted evidence for GO and STOP over a sliding window."""

    def __init__(self, model_path, conf=0.35, trigger_h=0.22, vote_n=9,
                 every_n=2, iou=0.45, core=None, niceness=0):
        """
        trigger_h  box height over frame height that is worth a whole vote.
                   taller than that is still worth exactly one
        vote_n     how many scored frames the window holds
        every_n    goes through to SignDetector. 2 gives us about 15 Hz, which is
                   plenty for a sign we are driving straight at
        """
        self.detector = SignDetector(model_path, conf=conf, iou=iou,
                                     every_n=every_n, core=core,
                                     niceness=niceness)
        self.trigger_h = float(trigger_h)
        self.window = []                 # per frame: {name: weight}
        self.vote_n = int(vote_n)
        self.new = self.dup = 0          # frame counts, for summary()
        self.last_hits = []              # newest detections, for the viewer
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

        hits = self.detector.detect(image)
        self.last_hits = hits

        height = image.shape[0]
        frame = {}
        for name, confidence, box in hits:
            ratio = (box[3] - box[1]) / float(height)
            if ratio < MIN_H:
                continue
            weight = confidence * min(1.0, ratio / self.trigger_h)
            # the same sign twice in one frame is still one look at it, so keep
            # the stronger of the two
            frame[name] = max(frame.get(name, 0.0), weight)
        self.window.append(frame)
        if len(self.window) > self.vote_n:
            del self.window[0]

    def totals(self):
        out = {}
        for frame in self.window:
            for name, weight in frame.items():
                out[name] = out.get(name, 0.0) + weight
        return out

    def count(self, name):
        """Frames in the window that saw this sign, so is it still in view."""
        return sum(name in frame for frame in self.window)

    def winner(self, need, margin=1.5):
        """GO, STOP, or None.

        Whichever got to `need` first, as long as it is `margin` ahead of the
        other one. Reading both at once has to give us nothing rather than a coin
        flip, because one of them means drive into the elevator.
        """
        totals = self.totals()
        if not totals:
            return None
        best = max(totals, key=totals.get)
        other = max([v for k, v in totals.items() if k != best] or [0.0])
        if totals[best] < need or totals[best] < margin * other:
            return None
        return best

    def clear(self):
        """Empty the window, so votes already cast cannot fire a second time."""
        self.window = []

    def summary(self):
        totals = self.totals()
        top = " ".join("{}:{:.1f}".format(k, v) for k, v
                       in sorted(totals.items(), key=lambda kv: -kv[1]))
        if self.last_hits:
            name, confidence, box = self.last_hits[0]
            near = "{} {:.2f}".format(name, confidence)
        else:
            near = "-"
        return "[{}] near={} frames new/dup={}/{}".format(
            top or "-", near, self.new, self.dup)


if __name__ == "__main__":
    import argparse
    import time

    ap = argparse.ArgumentParser(description="time the elevator sign detector")
    ap.add_argument("--model", default="best_v5_edgetpu.tflite")
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--image", default=None)
    ap.add_argument("--core", type=int, default=3,
                    help="pin this tool to a core. 3 keeps it off the ROS cores")
    args = ap.parse_args()

    det = SignDetector(args.model, conf=args.conf, core=args.core, niceness=10)
    frame = (cv2.imread(args.image) if args.image
             else np.random.randint(0, 255, (480, 640, 3), np.uint8))
    if frame is None:
        raise SystemExit("cannot read " + str(args.image))

    det.detect(frame)                                       # warm up
    t_cpu0, t0 = time.process_time(), time.perf_counter()
    for _ in range(args.n):
        hits = det.detect(frame)
    wall = (time.perf_counter() - t0) / args.n
    cpu = (time.process_time() - t_cpu0) / args.n
    print("input {}px | {:.1f} ms/frame ({:.0f} FPS)".format(
        det.size, wall * 1000, 1 / wall))
    print("CPU {:.1f} ms/frame -> {:.0f}% of one core".format(
        cpu * 1000, cpu / wall * 100))
    print("last:", [(n, round(s, 2)) for n, s, _ in hits] or "-")
