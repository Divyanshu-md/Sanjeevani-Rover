#!/usr/bin/env python3
"""
operator_view.py - live driving view for the rover operator.

WHAT IT DOES
  Reads the USB webcam on the Raspberry Pi and serves it as a live video page
  that any web browser can open.  Nothing needs to be installed on the laptop.

  The Pi Camera is NOT used here: that camera belongs to rover_node.py (human
  detection).  This program only shows the operator what is in front of the
  rover so he/she can drive it.  It never detects anything.

RUN (on the Pi, with the venv active):
    python3 operator_view.py
  then, on the laptop, open a browser at
    http://<Pi address>:8080          (Pi address: run  hostname -I  on the Pi)
  - over Ethernet that address is   http://192.168.50.2:8080
  Stop with Ctrl+C.

ADDRESSES SERVED
    /               a small page that shows the live picture
    /stream.mjpg    the raw video stream (what the page uses)
    /snapshot.jpg   one single picture (handy for testing)

FLAGS
    --device /dev/video1   which webcam (find it with: v4l2-ctl --list-devices)
    --port 8080            web port
    --fps 15               frames per second asked from the webcam
    --test-pattern         no webcam needed: serves a moving test picture
                           (to try the page on any PC)
    -v                     print every web request

NOTES FOR BEGINNERS
  * USB webcams start dark: they need 1-2 seconds to adjust their exposure.
    This program simply keeps reading frames all the time, so after the first
    seconds the picture is normal.
  * A clock is drawn in the corner of the picture.  If the clock stops, the
    picture is frozen (the webcam or the network has a problem) - do not drive
    on a frozen picture.
  * Only the standard library, OpenCV (cv2) and numpy are used.
  * Expect a delay of a few tenths of a second.  Drive slowly.
"""

import argparse
import glob
import logging
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ===========================================================================
# SETTINGS  (change here, or with the flags above)
# ===========================================================================
WEBCAM_DEVICE = "auto"          # "auto" = find the USB webcam by its fixed name (the /dev/videoN number can change at every boot)
WEBCAM_FALLBACK = "/dev/video1" # used by "auto" if no fixed name is found
WEBCAM_WIDTH = 640
WEBCAM_HEIGHT = 480
WEBCAM_FPS = 15
JPEG_QUALITY = 70               # 1-100; lower = smaller/faster, uglier
HTTP_PORT = 8080
BIND_ADDRESS = "0.0.0.0"        # listen on every network connection
RETRY_S = 3.0                   # wait before trying to reopen a missing webcam
MAX_READ_FAILURES = 30          # bad reads in a row before reopening the webcam
SHOW_CLOCK = True               # draw the clock in the corner (freeze detector)

log = logging.getLogger("operator_view")


# ===========================================================================
# FrameHub: the newest JPEG picture, shared between the camera thread and
# every browser connected.  The picture is compressed ONCE per frame, no
# matter how many browsers are watching.
# ===========================================================================
class FrameHub:
    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg = None          # newest picture (bytes) or None
        self.counter = 0          # increases with every new picture
        self.status = "starting"  # short text for the log

    def publish(self, jpeg_bytes):
        with self.cond:
            self.jpeg = jpeg_bytes
            self.counter += 1
            self.cond.notify_all()

    def latest(self):
        with self.cond:
            return self.jpeg, self.counter

    def wait_next(self, last_counter, timeout=2.0):
        """Block until a picture newer than `last_counter` exists (or timeout).
        Always returns (jpeg_or_None, counter).  Slow viewers simply skip
        frames, so the picture never lags behind more than one frame."""
        with self.cond:
            self.cond.wait_for(lambda: self.counter != last_counter, timeout)
            return self.jpeg, self.counter


# ===========================================================================
# Camera side
# ===========================================================================
def open_webcam(device, width, height, fps):
    """Open the USB webcam asking for MJPG (the webcam compresses by itself,
    so the Pi has little to do).  Returns an opened cv2.VideoCapture, or None."""
    import cv2
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def make_test_frame(width, height, n):
    """A moving test picture (used by --test-pattern)."""
    import cv2
    import numpy as np
    x = np.linspace(0, 255, width, dtype=np.uint8)
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 0] = x[None, :]
    frame[:, :, 1] = np.linspace(255, 0, height, dtype=np.uint8)[:, None]
    frame[:, :, 2] = 80
    bar_x = (n * 8) % width
    cv2.rectangle(frame, (bar_x, 0), (min(bar_x + 20, width - 1), height - 1), (255, 255, 255), -1)
    cv2.putText(frame, "TEST PATTERN", (20, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                (0, 0, 0), 3, cv2.LINE_AA)
    return frame


def draw_clock(frame):
    """Write the time in the corner.  If it stops changing the picture is frozen."""
    import cv2
    text = time.strftime("%H:%M:%S")
    cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)


def capture_loop(hub, stop, device, width, height, fps, quality, test_pattern):
    """Runs in its own thread: read the webcam forever and publish JPEGs.
    Never dies: any problem is logged and the webcam is reopened later."""
    import cv2
    cap = None
    failures = 0
    n = 0
    last_status = None

    def set_status(text):
        nonlocal last_status
        if text != last_status:
            last_status = text
            hub.status = text
            log.info(text)

    while not stop.is_set():
        try:
            if test_pattern:
                frame = make_test_frame(width, height, n)
                n += 1
                set_status("serving the test pattern (no webcam)")
                stop.wait(1.0 / max(1, fps))
            else:
                if cap is None:
                    cap = open_webcam(device, width, height, fps)
                    if cap is None:
                        set_status("cannot open webcam %s - retrying every %.0f s" % (device, RETRY_S))
                        stop.wait(RETRY_S)
                        continue
                    failures = 0
                ok, frame = cap.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= MAX_READ_FAILURES:
                        set_status("webcam stopped delivering pictures - reopening")
                        cap.release()
                        cap = None
                        stop.wait(RETRY_S)
                    else:
                        stop.wait(0.05)
                    continue
                failures = 0
                set_status("webcam %s running" % device)

            if SHOW_CLOCK:
                draw_clock(frame)
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
            if ok:
                hub.publish(buf.tobytes())
        except Exception as exc:  # keep running whatever happens
            log.error("capture problem: %s", exc)
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass
            cap = None
            stop.wait(RETRY_S)
    if cap is not None:
        cap.release()


# ===========================================================================
# Web side
# ===========================================================================
INDEX_HTML = b"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover operator view</title>
<style>
  body{margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center}
  h1{font-size:1rem;font-weight:normal;margin:.6rem}
  img{width:100%;max-width:960px;height:auto;background:#000}
  p{font-size:.85rem;color:#999;margin:.5rem}
</style></head><body>
<h1>Rover operator view (USB webcam, driving aid only)</h1>
<img src="/stream.mjpg" alt="live video">
<p>If the clock in the corner stops, the picture is frozen: stop driving.<br>
Human detection alerts appear in the ground station window, not here.</p>
</body></html>
"""


class Handler(BaseHTTPRequestHandler):
    server_version = "RoverView/1.0"
    protocol_version = "HTTP/1.0"   # one connection per request: simple and safe

    def log_message(self, fmt, *args):
        log.debug("%s %s", self.address_string(), fmt % args)

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        hub = self.server.hub
        try:
            if path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", INDEX_HTML)
            elif path == "/snapshot.jpg":
                jpeg, _ = hub.latest()
                if jpeg is None:
                    self._send(503, "text/plain", b"no picture yet, try again in a moment\n")
                else:
                    self._send(200, "image/jpeg", jpeg)
            elif path == "/stream.mjpg":
                self._stream(hub)
            else:
                self._send(404, "text/plain", b"not found\n")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # the browser went away: normal

    def _stream(self, hub):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        counter = -1
        while not self.server.stop_event.is_set():
            jpeg, new_counter = hub.wait_next(counter, timeout=2.0)
            if jpeg is None or new_counter == counter:
                continue  # nothing new yet
            counter = new_counter
            try:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode("ascii") + b"\r\n\r\n" + jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                break


class QuietServer(ThreadingHTTPServer):
    daemon_threads = True          # viewer threads never block shutdown
    allow_reuse_address = True


def resolve_device(device, pattern="/dev/v4l/by-id/*-video-index0"):
    """"auto" -> the first USB webcam's fixed name (it always points to the real
    picture node, whatever number the Pi gave it this time).  Anything else is
    used as given.  `pattern` can be changed for testing."""
    if device != "auto":
        return device
    found = sorted(glob.glob(pattern))
    if found:
        return found[0]
    return WEBCAM_FALLBACK


def main(argv=None):
    ap = argparse.ArgumentParser(description="Live USB-webcam view for the rover operator.")
    ap.add_argument("--device", default=WEBCAM_DEVICE, help="webcam device (default %s)" % WEBCAM_DEVICE)
    ap.add_argument("--port", type=int, default=HTTP_PORT, help="web port (default %d)" % HTTP_PORT)
    ap.add_argument("--fps", type=int, default=WEBCAM_FPS, help="frames per second (default %d)" % WEBCAM_FPS)
    ap.add_argument("--test-pattern", action="store_true", help="serve a test picture, no webcam needed")
    ap.add_argument("-v", "--verbose", action="store_true", help="log every web request")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    if args.verbose:
        log.setLevel(logging.DEBUG)

    try:
        httpd = QuietServer((BIND_ADDRESS, args.port), Handler)
    except OSError as exc:
        print("Cannot use web port %d: %s" % (args.port, exc))
        print("Is another copy of operator_view.py already running?")
        return 1

    stop = threading.Event()
    hub = FrameHub()
    httpd.hub = hub
    httpd.stop_event = stop

    cam_thread = threading.Thread(
        target=capture_loop, name="webcam", daemon=True,
        args=(hub, stop, resolve_device(args.device), WEBCAM_WIDTH, WEBCAM_HEIGHT, args.fps, JPEG_QUALITY, args.test_pattern))
    cam_thread.start()

    print("Operator view running.  Open  http://<Pi address>:%d  in a browser on the laptop." % args.port)
    print("(find the Pi address with:  hostname -I      over Ethernet it is 192.168.50.2)")
    print("Press Ctrl+C to stop.")
    try:
        httpd.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        stop.set()
        httpd.server_close()
        cam_thread.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
