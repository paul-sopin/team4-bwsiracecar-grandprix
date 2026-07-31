#!/usr/bin/env python3
"""
Team 4 - watching the AR tag gate and the elevator signs

Two detectors, one viewer, since they run one after the other on the course and
you want to check them the same way. Tags are on by default, add --signs to also
run the Coral.

Three ways to watch, cheapest first:

    python3 show_ar_tags.py --racecar                # terminal only
    python3 show_ar_tags.py --racecar --http         # browser, 10.42.0.1:8000
    python3 show_ar_tags.py --show                   # cv2 window, needs a display

Where the frames come from:

    --source 0            USB camera, this is the default
    --source clip.mp4     a video file
    --racecar             through racecar_core, which is what you want when the
                          ROS stack has the RealSense

This prints the numbers the race script decides on, so it is how you set
AR_MIN_SIZE and SIGN_TRIGGER_H. Walk the car back from the tag until sz drops
under the number you are thinking of using, then measure the floor.

For the signs, free the Coral first or the interpreter will not build:

    sudo kill $(sudo lsof -t /dev/apex_0)

No tags around? Print your own:

    python3 show_ar_tags.py --make-tag 0

Tape it to a wall and you can test the gate without the car.
"""

import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

from ar_detector import ARTagGate, DEFAULT_DICT, MIN_SIZE, _Aruco, make_tag

latest_jpeg = None           # shared with the HTTP thread
lock = threading.Lock()


def draw(frame, quads, sizes, hits, fps, min_size, gate, signs):
    for pts, size in zip(quads, sizes):
        near = size >= min_size
        color = (0, 220, 0) if near else (120, 120, 120)
        cv2.polylines(frame, [pts.astype(np.int32)], True, color,
                      2 if near else 1)
        x, y = pts.min(axis=0)
        cv2.putText(frame, "sz{:.3f}{}".format(size, " *" if near else ""),
                    (int(x), max(14, int(y) - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
    for name, confidence, box in hits:
        color = (0, 220, 0) if name == "GO" else (0, 0, 255)
        cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, 2)
        cv2.putText(frame, "{} {:.2f}".format(name, confidence),
                    (box[0], max(14, box[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2)
    line = "{:4.1f} FPS  gate:{}".format(fps, "LATCHED" if gate else "waiting")
    if signs is not None:
        line += "  " + signs
    cv2.putText(frame, line, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)
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
    ap.add_argument("--min-size", type=float, default=MIN_SIZE)
    ap.add_argument("--need", type=int, default=3)
    ap.add_argument("--signs", action="store_true",
                    help="also run the Coral GO/STOP detector")
    ap.add_argument("--model", default="best_v5_edgetpu.tflite")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--jpeg-quality", type=int, default=70)
    ap.add_argument("--make-tag", type=int, default=None,
                    help="write a printable tag PNG and exit")
    ap.add_argument("--px", type=int, default=600, help="with --make-tag: size")
    args = ap.parse_args()

    if args.make_tag is not None:
        name = "tag_{}.png".format(args.make_tag)
        cv2.imwrite(name, make_tag(args.make_tag, px=args.px,
                                   dictionary=args.dict))
        print("wrote", name)
        return

    ids = ([int(i) for i in args.ids.split(",")] if args.ids else None)
    gate = ARTagGate(dictionary=args.dict, ids=ids, min_size=args.min_size,
                     need=args.need, every_n=1)
    # the gate stops looking once it latches, which is right on the course and
    # useless in a viewer, so decode separately just for drawing
    aruco = _Aruco(args.dict)

    signs = None
    if args.signs:
        from elevator_signs import ElevatorSigns
        signs = ElevatorSigns(args.model, conf=args.conf, every_n=1)

    if args.http:
        srv = HTTPServer(("0.0.0.0", args.port), MJPEGHandler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print("watch it at  http://10.42.0.1:{}".format(args.port))

    frames = (frames_from_racecar() if args.racecar
              else frames_from_capture(args.source))
    print("dictionary: {}   ids: {}   signs: {}   (ctrl-c to stop)".format(
        args.dict, ids if ids else "all", "on" if signs else "off"))

    fps, last_line = 0.0, ""
    try:
        for frame in frames:
            t0 = time.perf_counter()
            gate.poll(frame)
            if signs is not None:
                signs.poll(frame)

            corners, tag_ids = aruco.detect(
                cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            quads, sizes, labels = [], [], []
            height = frame.shape[0]
            if tag_ids is not None:
                for quad, tag_id in zip(corners, tag_ids.flatten()):
                    pts = quad.reshape(4, 2).astype(np.float32)
                    edges = np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1)
                    size = float(edges.mean()) / height
                    quads.append(pts)
                    sizes.append(size)
                    labels.append("id{} sz{:.3f}{}".format(
                        int(tag_id), size, "*" if size >= args.min_size else ""))

            hits = signs.last_hits if signs is not None else []

            if args.http or args.show:
                view = draw(frame.copy(), quads, sizes, hits, fps,
                            args.min_size, gate.seen(),
                            signs.summary() if signs else None)
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

            line = "  ".join(labels) or "-"
            line = "gate:{:8} {}".format(
                "LATCHED" if gate.seen() else "{}/{}".format(gate.hits,
                                                             gate.need), line)
            if signs is not None:
                line += "   signs " + signs.summary()
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
