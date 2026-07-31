#!/usr/bin/env python3
"""
Team 4 - watching what the AR tag reader sees

Ported from show_detections.py in the Trial 3A repo. Three ways to watch it,
cheapest first:

    python3 show_ar_tags.py --racecar                # terminal only
    python3 show_ar_tags.py --racecar --http         # browser, 10.42.0.1:8000
    python3 show_ar_tags.py --show                   # cv2 window, needs a display

Where the frames come from:

    --source 0            USB camera, this is the default
    --source clip.mp4     a video file
    --racecar             through racecar_core, which is what you want when the
                          ROS stack has the RealSense

It prints the same numbers the race script makes its decision on, so this is how
you set AR_TRIGGER_SIZE. Walk the car back from a tag until sz drops under the
number you are thinking of using, then measure the floor.

No tags around? Print your own:

    python3 show_ar_tags.py --make-tag 0
    python3 show_ar_tags.py --make-tag 0 --rotate 180

Tape both to a wall and you can test the whole thing without the car. Turning
the PNG and turning the paper look the same to the detector, so one printed tag
flipped over does the same job.
"""

import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

from ar_detector import (ARTagDetector, DEFAULT_DICT, FLIPPED, UPRIGHT,
                         make_tag)

COLORS = {UPRIGHT: (0, 220, 0), FLIPPED: (0, 140, 255), None: (120, 120, 120)}

latest_jpeg = None           # shared with the HTTP thread
lock = threading.Lock()


def draw(frame, tags, fps, trigger_size):
    for tag in tags:
        color = COLORS.get(tag.orientation, (200, 200, 200))
        near = tag.size >= trigger_size
        pts = tag.corners.astype(np.int32)
        # some OpenCV builds will not take numpy scalars in a point tuple
        p0 = (int(pts[0][0]), int(pts[0][1]))
        p1 = (int(pts[1][0]), int(pts[1][1]))
        cv2.polylines(frame, [pts], True, color, 2 if near else 1)
        # the tag's own top edge, which is where the whole angle comes from. if
        # this white line is not sitting along the printed top of the tag then
        # we have the corner order wrong and every angle after it is wrong too
        cv2.line(frame, p0, p1, (255, 255, 255), 2)
        cv2.circle(frame, p0, 4, (255, 255, 255), -1)
        label = "id{} {} {:+.0f} sz{:.2f}{}".format(
            tag.id, tag.orientation or "?", tag.roll, tag.size,
            " *" if near else "")
        cv2.putText(frame, label, (tag.box[0], max(14, tag.box[1] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText(frame, "{:4.1f} FPS   * = big enough for a full vote".format(fps),
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return frame


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/", "/stream"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=f")
        self.end_headers()
        try:
            while True:
                with lock:
                    buf = latest_jpeg
                if buf is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(buf)).encode()
                                 + b"\r\n\r\n" + buf + b"\r\n")
                time.sleep(0.08)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_):
        pass                                  # keep the console readable


def frames_from_racecar():
    sys.path.insert(0, "../../library")
    import racecar_core
    import rclpy

    rc = racecar_core.create_racecar(isSimulation=False)
    # racecar_core only spins its executor inside rc.go() and this tool never
    # calls that, so we have to spin it here or no callback ever fires and the
    # camera stays None forever. the _async getter is for the same reason. the
    # normal one hands back the frame rc's update loop promotes, and there is no
    # update loop here.
    executor = rclpy.get_global_executor()
    while True:
        executor.spin_once(timeout_sec=0.05)
        img = rc.camera.get_color_image_async()
        if img is not None:
            yield img


def frames_from_capture(source):
    cap = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
    if not cap.isOpened():
        sys.exit("cannot open source {!r}".format(source))
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        yield frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--racecar", action="store_true")
    ap.add_argument("--http", action="store_true", help="serve MJPEG on --port")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--show", action="store_true", help="local cv2 window")
    ap.add_argument("--dict", default=DEFAULT_DICT)
    ap.add_argument("--ids", default=None,
                    help="comma separated tag ids to keep, default all")
    ap.add_argument("--angle-tol", type=float, default=50.0)
    ap.add_argument("--trigger-size", type=float, default=0.10)
    ap.add_argument("--every-n", type=int, default=1)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--core", type=int, default=None,
                    help="pin this tool to a core (3 keeps it off the ROS cores)")
    ap.add_argument("--jpeg-quality", type=int, default=70)
    ap.add_argument("--make-tag", type=int, default=None,
                    help="write a printable tag PNG and exit")
    ap.add_argument("--rotate", type=int, default=0,
                    help="with --make-tag: rotate the PNG this many degrees")
    ap.add_argument("--px", type=int, default=600, help="with --make-tag: size")
    args = ap.parse_args()

    if args.make_tag is not None:
        img = make_tag(args.make_tag, px=args.px, dictionary=args.dict)
        if args.rotate % 90:
            sys.exit("--rotate takes a multiple of 90")
        if args.rotate % 360:
            img = np.rot90(img, (args.rotate % 360) // 90).copy()
        name = "tag_{}{}.png".format(args.make_tag,
                                     "_rot{}".format(args.rotate % 360)
                                     if args.rotate % 360 else "")
        cv2.imwrite(name, img)
        print("wrote", name)
        return

    ids = ([int(i) for i in args.ids.split(",")] if args.ids else None)
    det = ARTagDetector(dictionary=args.dict, ids=ids, angle_tol=args.angle_tol,
                        every_n=args.every_n, scale=args.scale, core=args.core,
                        niceness=10 if args.core is not None else 0)

    if args.http:
        srv = HTTPServer(("0.0.0.0", args.port), MJPEGHandler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print("watch it at  http://10.42.0.1:{}".format(args.port))

    frames = (frames_from_racecar() if args.racecar
              else frames_from_capture(args.source))
    print("dictionary: {}   ids: {}   (ctrl-c to stop)".format(
        args.dict, ids if ids else "all"))

    fps, last_line = 0.0, ""
    try:
        for frame in frames:
            t0 = time.perf_counter()
            tags = det.detect(frame)

            if args.http or args.show:
                view = draw(frame.copy(), tags, fps, args.trigger_size)
                if args.http:
                    ok, buf = cv2.imencode(
                        ".jpg", view,
                        [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                    if ok:
                        with lock:
                            globals()["latest_jpeg"] = buf.tobytes()
                if args.show:
                    cv2.imshow("ar tags", view)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

            line = "  ".join(
                "id{} {} {:+.0f} sz{:.2f}{}".format(
                    t.id, t.orientation or "?", t.roll, t.size,
                    "*" if t.size >= args.trigger_size else "")
                for t in tags) or "-"
            if line != last_line:
                print("[{:4.1f} FPS] {}".format(fps, line))
                last_line = line

            dt = time.perf_counter() - t0
            fps = 0.9 * fps + 0.1 / dt if fps else 1 / dt
    except KeyboardInterrupt:
        pass
    finally:
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
