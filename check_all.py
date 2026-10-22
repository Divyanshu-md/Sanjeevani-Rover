#!/usr/bin/env python3
"""
check_all.py  -  one-minute health check of every part of the rover
===================================================================
HOW TO RUN (on the Pi, venv active, rover_node.py and operator_view.py STOPPED):
    cd ~/rescue && source venv/bin/activate
    python3 check_all.py
Optional:   python3 check_all.py --no-send     (do not send the test message to the laptop)

It checks, one by one, and prints PASS / WARN / FAIL for each:
    1. nothing else is using the cameras      5. USB webcam (driving view)
    2. power and heat of the Pi               6. GPS module
    3. program files and the AI model         7. link to the laptop (+ one test message)
    4. Pi Camera and the AI model speed       8. drive train (NOT checked by software)

For the test message in step 7, start ground_station.py on the laptop first.
The test message shows up there as a yellow block saying "SYSTEM CHECK".

THIS PROGRAM DOES NOT TOUCH THE MOTORS.  It only reads the cameras, the GPS and
the Pi's own sensors.
"""
import argparse
import glob
import os
import socket
import subprocess
import sys
import time

import numpy as np

import protocol
import rover_node as rn

results = []          # (status, name, detail)


def record(status, name, detail=""):
    results.append((status, name, detail))
    print("[%-4s] %s%s" % (status, name, (" - " + detail) if detail else ""), flush=True)


def run_cmd(args, timeout=5):
    try:
        out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return out.returncode, (out.stdout or "").strip()
    except Exception as exc:                                  # tool missing, timeout ...
        return -1, str(exc)


# ---------------------------------------------------------------------------
def check_busy():
    rc, out = run_cmd(["pgrep", "-af", "rover_node.py|operator_view.py|see_detection.py|calibrate_motion.py"])
    lines = [l for l in out.splitlines() if "pgrep" not in l]
    if lines:
        record("FAIL", "Other programs running",
               "stop rover_node.py / operator_view.py first (Ctrl+C in their windows), they hold the cameras")
        return False
    record("PASS", "No other rover program is using the cameras")
    return True


def check_power_and_heat():
    rc, out = run_cmd(["vcgencmd", "get_throttled"])
    if rc != 0 or "=" not in out:
        record("WARN", "Power check", "vcgencmd not available (%s)" % out)
    else:
        v = int(out.split("=")[1], 16)
        now_bits, since_boot = v & 0xF, v >> 16
        if now_bits & 0x1:
            record("FAIL", "Power", "UNDER-VOLTAGE right now (%s) - use a better power supply or cable" % out)
        elif now_bits & 0x6:
            record("FAIL", "Power", "the Pi is slowing itself down right now (%s)" % out)
        elif now_bits & 0x8:
            record("WARN", "Power", "soft temperature limit active right now (%s)" % out)
        elif since_boot & 0x1:
            record("WARN", "Power", "under-voltage happened since boot (%s) - plug the webcam in BEFORE power-on, "
                                    "check the supply" % out)
        elif since_boot:
            record("WARN", "Power", "an old heat/throttle flag since boot (%s) - ok if the temperature below is fine" % out)
        else:
            record("PASS", "Power", out)
    rc, out = run_cmd(["vcgencmd", "measure_temp"])
    try:
        temp = float(out.split("=")[1].split("'")[0])
        if temp < 70:
            record("PASS", "Temperature", "%.1f C" % temp)
        elif temp < 80:
            record("WARN", "Temperature", "%.1f C - getting warm; add a heatsink/airflow" % temp)
        else:
            record("FAIL", "Temperature", "%.1f C - too hot, let it cool" % temp)
    except Exception:
        record("WARN", "Temperature", "could not read it (%s)" % out)


def check_files():
    missing = []
    for name in ("rover_node.py", "protocol.py", "operator_view.py"):
        if not os.path.exists(os.path.join(rn.HERE, name)):
            missing.append(name)
    if not os.path.exists(rn.MODEL_PATH):
        missing.append(os.path.basename(rn.MODEL_PATH))
    if missing:
        record("FAIL", "Program files", "missing: " + ", ".join(missing))
    else:
        record("PASS", "Program files and AI model file are present")


def check_pi_camera():
    """Returns one picture (or None)."""
    cam = None
    try:
        cam = rn.open_camera()
        rn.grab_frame(cam)
        frame = rn.grab_frame(cam)
    except Exception as exc:
        record("FAIL", "Pi Camera", "could not open/read it (%s)" % exc)
        if cam is not None:
            rn.close_camera(cam)
        return None
    rn.close_camera(cam)
    h, w = frame.shape[:2]
    mean, spread = float(np.mean(frame)), float(np.std(frame))
    if mean < 8 or spread < 3:
        record("WARN", "Pi Camera", "%dx%d picture is almost black or flat (brightness %.0f) - lens cap / too dark / "
                                    "loose ribbon cable?" % (w, h, mean))
    else:
        record("PASS", "Pi Camera", "%dx%d picture, brightness %.0f" % (w, h, mean))
    return frame


def check_ai(frame):
    try:
        det = rn.PoseDetector(rn.MODEL_PATH)
    except Exception as exc:
        record("FAIL", "AI model", "could not load (%s)" % exc)
        return
    if frame is None:
        frame = np.zeros((rn.CAM_HEIGHT, rn.CAM_WIDTH, 3), dtype=np.uint8)
        note = " (timed on a blank picture because the camera failed)"
    else:
        note = ""
    try:
        det.detect(frame)                                      # warm-up
        t0 = time.monotonic()
        people = []
        for _ in range(5):
            people = det.detect(frame)
        per = (time.monotonic() - t0) / 5.0
    except Exception as exc:
        record("FAIL", "AI model", "crashed while running (%s)" % exc)
        return
    rate = 1.0 / per if per > 0 else 0.0
    if rate >= 2.5:
        record("PASS", "AI model", "%dx%d, about %.1f pictures/s%s" % (det.size, det.size, rate, note))
    else:
        record("WARN", "AI model", "only %.1f pictures/s (normally about 3) - Pi busy or too hot?%s" % (rate, note))
    record("INFO", "People in that picture", "%d possible person(s) - only for your information" % len(people))


def check_usb_webcam():
    matches = sorted(glob.glob("/dev/v4l/by-id/*-video-index0"))
    if not matches:
        record("FAIL", "USB webcam", "not found - plug it in (best BEFORE powering the Pi on)")
        return
    try:
        import cv2
        cap = cv2.VideoCapture(matches[0], cv2.CAP_V4L2)
        ok = False
        shape = None
        for _ in range(8):
            ok, img = cap.read()
            if ok:
                shape = img.shape
                break
            time.sleep(0.2)
        cap.release()
    except Exception as exc:
        record("FAIL", "USB webcam", "found but could not read (%s)" % exc)
        return
    if ok:
        record("PASS", "USB webcam", "%s, %dx%d" % (os.path.basename(matches[0])[:40], shape[1], shape[0]))
    else:
        record("FAIL", "USB webcam", "found but no picture - is operator_view.py still running?")


def check_gps(seconds=8):
    try:
        import serial
        ser = serial.Serial(rn.GPS_PORT, rn.GPS_BAUD, timeout=1)
    except Exception as exc:
        record("FAIL", "GPS", "cannot open %s (%s) - is pyserial installed / serial port enabled?" % (rn.GPS_PORT, exc))
        return
    good, has_fix, sats = 0, False, None
    try:
        ser.reset_input_buffer()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            line = ser.readline().decode("ascii", "ignore").strip()
            if not line.startswith("$") or not rn._nmea_checksum_ok(line):
                continue
            good += 1
            if rn.parse_nmea_line(line) is not None:
                has_fix = True
            if line[3:6] == "GGA":
                f = line[1:].split("*")[0].split(",")
                if len(f) > 7 and f[7].isdigit():
                    sats = int(f[7])
    finally:
        ser.close()
    if good == 0:
        record("FAIL", "GPS", "no GPS messages in %d s - check wires (module TX -> Pi pin 10, RX -> pin 8, "
                              "VCC pin 2, GND pin 6)" % seconds)
    elif has_fix:
        record("PASS", "GPS", "%d messages, has a position fix (%s satellites)" % (good, sats if sats is not None else "?"))
    else:
        record("WARN", "GPS", "wiring OK (%d messages) but no position yet - needs a clear view of the sky; "
                              "the module's LED blinks when it has a fix" % good)


def check_laptop(send):
    rc, out = run_cmd(["ping", "-c", "2", "-W", "1", rn.PEER_IP], timeout=8)
    if rc == 0:
        record("PASS", "Link to laptop", "%s answers a ping" % rn.PEER_IP)
    else:
        # Windows blocks ping by default, so a missing ping reply proves nothing.
        # Ask the Pi's address table instead: if the laptop answered at the cable level it is listed there.
        rc2, neigh = run_cmd(["ip", "neigh", "show", rn.PEER_IP])
        if "lladdr" in neigh and "FAILED" not in neigh:
            record("INFO", "Link to laptop", "no ping reply (normal: Windows blocks ping) but the laptop IS on the "
                                             "cable. The test message below is the real test.")
        else:
            record("FAIL", "Link to laptop", "laptop %s not found on the cable - Ethernet cable plugged in? laptop "
                                             "address 192.168.50.1?" % rn.PEER_IP)
            return
    if not send:
        record("INFO", "Test message", "skipped (--no-send)")
        return
    try:
        payload = protocol.build_payload("SYSTEM CHECK (test, not a real alert)", None, None, None, None, "NONE", 0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(payload, (rn.PEER_IP, rn.PEER_PORT))
        sock.close()
        record("INFO", "Test message", "sent - the laptop's ground station should now show a block saying "
                                      "SYSTEM CHECK (look at it yourself)")
    except Exception as exc:
        record("FAIL", "Test message", "could not send (%s)" % exc)


def main():
    ap = argparse.ArgumentParser(description="Health check of every rover part.")
    ap.add_argument("--no-send", action="store_true", help="do not send the test message to the laptop")
    args = ap.parse_args()

    print("Rover health check - this program does not touch the motors.\n")
    if not check_busy():
        print("\nStopped: close the other programs and run this again.")
        return 2
    check_power_and_heat()
    check_files()
    frame = check_pi_camera()
    check_ai(frame)
    check_usb_webcam()
    check_gps()
    check_laptop(not args.no_send)
    record("INFO", "Drive train (FlySky, Cytron driver, motors)",
           "NOT checked by software - it is wired directly. Test it by hand with the wheels lifted off the ground.")

    fails = sum(1 for r in results if r[0] == "FAIL")
    warns = sum(1 for r in results if r[0] == "WARN")
    print("\n==== SUMMARY: %d FAIL, %d WARN ====" % (fails, warns))
    if fails:
        print("Fix the FAIL lines first (send me a screenshot of this list).")
    elif warns:
        print("Working, with warnings - read the WARN lines.")
    else:
        print("Everything checked is working.")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
