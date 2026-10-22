#!/usr/bin/env python3
"""
calibrate_motion.py  -  find the right MOTION_THRESH for YOUR camera and Pi
===========================================================================
WHY
  rover_node.py decides "Conscious" or "Unconscious" for a lying (or slumped)
  person by measuring how much the body points MOVE.  The AI model's body
  points always jitter a little, even when the person is perfectly still, so
  the limit MOTION_THRESH must sit between "still" and "moving".  This tool
  measures both on your real camera and tells you the number to use.

HOW TO RUN (on the Pi, venv active, rover_node.py STOPPED - the camera can
only be used by one program at a time):
    cd ~/rescue && source venv/bin/activate
    python3 calibrate_motion.py

WHAT YOU DO (the program tells you when)
  1. Lie down 2.5 - 3 m from the camera, side-on, whole body in view.
     You have 15 seconds to get ready.
  2. STAY COMPLETELY STILL for 25 seconds (only breathe).
  3. Then MOVE your arms and legs (wave them, like you are waking up)
     for 25 seconds.
  At the end it prints a recommended MOTION_THRESH and the command to set it.

THIS PROGRAM DOES NOT TOUCH THE MOTORS.  It only reads the Pi Camera.
"""

import argparse
import json
import sys
import time

import numpy as np

import rover_node as rn

PREPARE_S = 15.0
STILL_S = 25.0
MOVE_S = 25.0
SETTLE_S = 4.0          # the first seconds of each phase are ignored (the movement
                        # test needs a few seconds of history, and you are still settling)
MIN_SAMPLES = 15        # need at least this many movement values per phase


def collect(pipeline, read_frame, seconds, label, clock=time.monotonic, sleep=time.sleep,
            out=print, settle=SETTLE_S, records=None):
    """Run for `seconds`, return the list of movement scores measured after the
    settle time.  Prints one status line per second."""
    values = []
    start = clock()
    last_print = -1.0
    seen = 0
    total = 0
    while True:
        now = clock()
        if now - start >= seconds:
            break
        frame = read_frame()
        t = clock()
        det = pipeline.process(frame, t)
        total += 1
        if det is not None:
            seen += 1
            if records is not None:                      # raw data for offline analysis
                records.append({"phase": label, "t": round(t - start, 3), "posture": det.posture,
                                "bbox": [float(v) for v in det.bbox],
                                "kp": [[round(float(v), 1) for v in row] for row in det.keypoints]})
            if det.motion is not None and t - start >= settle:
                values.append(det.motion)
        if t - last_print >= 1.0:
            last_print = t
            if det is None:
                out("  %-24s %2ds left   no person found in this picture" % (label, int(seconds - (t - start))))
            else:
                out("  %-24s %2ds left   motion = %s   (shape: %s)" % (
                    label, int(seconds - (t - start)),
                    "n/a (still measuring)" if det.motion is None else "%.3f" % det.motion,
                    det.posture))
        sleep(0.0)
    return values, seen, total


def summarize(still, moving, out=print):
    """Print the result.  Returns the recommended threshold, or None."""
    if len(still) < MIN_SAMPLES or len(moving) < MIN_SAMPLES:
        out("")
        out("NOT ENOUGH DATA (still: %d values, moving: %d values, need %d each)." % (
            len(still), len(moving), MIN_SAMPLES))
        out("The AI must see you in most pictures.  Lie 2.5 - 3 m from the camera with")
        out("your whole body in view, in good light, and run this program again.")
        return None
    s = np.array(still)
    m = np.array(moving)
    s_med, s_p90, s_max = float(np.median(s)), float(np.percentile(s, 90)), float(s.max())
    m_med, m_p10, m_min = float(np.median(m)), float(np.percentile(m, 10)), float(m.min())
    out("")
    out("RESULT")
    out("  STILL  : median %.3f   90%% of values below %.3f   highest %.3f   (%d values)" % (s_med, s_p90, s_max, len(s)))
    out("  MOVING : median %.3f   90%% of values above %.3f   lowest  %.3f   (%d values)" % (m_med, m_p10, m_min, len(m)))
    if s_p90 < m_p10:
        thresh = (s_p90 + m_p10) / 2.0
        out("")
        out("GOOD SEPARATION.  Recommended  MOTION_THRESH = %.3f" % thresh)
    elif s_med < m_med:
        thresh = (s_p90 + m_med) / 2.0 if s_p90 < m_med else (s_med + m_med) / 2.0
        out("")
        out("OVERLAP: some still values are as high as some moving values, so the")
        out("decision will sometimes be wrong.  Best compromise  MOTION_THRESH = %.3f" % thresh)
        out("(Tip: lie closer to 3 m, in good light, and wave your arms and legs more widely.)")
    else:
        out("")
        out("The 'still' values were NOT lower than the 'moving' values, so no threshold")
        out("can be recommended.  Did you really stay still, and then really move?  Run again.")
        return None
    out("")
    out("To set it, copy this line and run it on the Pi (in ~/rescue):")
    out("  sed -i 's/^MOTION_THRESH = [0-9.]*/MOTION_THRESH = %.3f/' rover_node.py" % thresh)
    out("Check it with:   grep -n '^MOTION_THRESH' rover_node.py")
    return thresh


def countdown(seconds, message, clock=time.monotonic, sleep=time.sleep, out=print):
    out("")
    out(message)
    end = clock() + seconds
    last = None
    while True:
        left = int(end - clock() + 0.999)
        if left <= 0:
            break
        if left != last and (left <= 5 or left % 5 == 0):
            out("   ... %d" % left)
        last = left
        sleep(0.2)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Measure movement values to choose MOTION_THRESH.")
    ap.parse_args(argv)
    print("Loading the AI model and opening the Pi Camera ...")
    try:
        pipeline = rn.VisionPipeline(rn.PoseDetector(rn.MODEL_PATH))
        cam = rn.open_camera()
    except Exception as exc:
        print("Could not start: %s" % exc)
        print("Is rover_node.py still running?  Stop it first (Ctrl+C), then try again.")
        return 1

    def read_frame():
        return rn.grab_frame(cam)

    try:
        countdown(PREPARE_S, "GET READY: lie down 2.5 - 3 m from the camera, side-on, whole body in view.")
        pipeline.tracker.reset()
        print("")
        print(">>> STAY COMPLETELY STILL for %d seconds (only breathe) <<<" % STILL_S)
        records = []
        still, seen_s, total_s = collect(pipeline, read_frame, STILL_S, "STILL", records=records)
        print("")
        print(">>> NOW MOVE your arms and legs for %d seconds <<<" % MOVE_S)
        pipeline.tracker.reset()
        moving, seen_m, total_m = collect(pipeline, read_frame, MOVE_S, "MOVING", records=records)
        print("")
        print("(You were found in %d of %d pictures while still, %d of %d while moving.)" % (
            seen_s, total_s, seen_m, total_m))
        summarize(still, moving)
        try:
            path = rn.os.path.join(rn.HERE, "calib_data.json")
            with open(path, "w") as fh:
                json.dump(records, fh)
            print("")
            print("Raw body-point data saved to %s (%d pictures)." % (path, len(records)))
        except OSError as exc:
            print("Could not save the raw data: %s" % exc)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 130
    finally:
        rn.close_camera(cam)
    return 0


if __name__ == "__main__":
    sys.exit(main())
