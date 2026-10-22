#!/usr/bin/env python3
"""
see_detection.py  -  take ONE picture and draw what the AI sees on it
=====================================================================
HOW TO RUN (on the Pi, venv active, rover_node.py STOPPED):
    cd ~/rescue && source venv/bin/activate
    python3 see_detection.py
It waits 10 seconds (get into position), takes a picture, runs the AI on it and
saves  ~/rescue/seen.jpg  with:
    * a RED box around every possible person, with its score
    * GREEN dots on every body point the AI is sure about, YELLOW dots on weak ones
    * the text on the picture says how many body points were seen
(The picture is turned the right way up when CAM_ROTATE_180 = True in rover_node.py.)
Copy seen.jpg to the laptop (scp) and look at it.

THIS PROGRAM DOES NOT TOUCH THE MOTORS.  It only reads the Pi Camera.
"""
import sys
import time

import numpy as np

import rover_node as rn

WAIT_S = 10


def draw(frame, people):
    import cv2
    img = np.ascontiguousarray(frame.copy())
    for p in people:
        x, y, w, h = [int(v) for v in p["bbox"]]
        n_vis = int(np.sum(p["kp"][:, 2] >= rn.KP_VIS_THRESH))
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        label = "score %.2f  points %d" % (p["conf"], n_vis)
        cv2.putText(img, label, (x, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        for kx, ky, kv in p["kp"]:
            if kv >= rn.KP_VIS_THRESH:
                colour = (0, 255, 0)
            elif kv >= 0.1:
                colour = (0, 255, 255)
            else:
                continue
            cv2.circle(img, (int(kx), int(ky)), 5, colour, -1)
    msg = "%d possible person(s)" % len(people)
    cv2.putText(img, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return img


def main():
    import cv2
    print("Loading the AI model and opening the Pi Camera ...")
    try:
        det = rn.PoseDetector(rn.MODEL_PATH)
        cam = rn.open_camera()
    except Exception as exc:
        print("Could not start: %s" % exc)
        print("Is rover_node.py still running?  Stop it first (Ctrl+C), then try again.")
        return 1
    try:
        print("Model size: %d x %d" % (det.size, det.size))
        print("Get into position now.  Picture in %d seconds ..." % WAIT_S)
        end = time.monotonic() + WAIT_S
        while time.monotonic() < end:
            rn.grab_frame(cam)                         # keep the camera running
        frame = rn.grab_frame(cam)
        people = det.detect(frame)
        out = draw(frame, people)
        path = rn.os.path.join(rn.HERE, "seen.jpg")
        cv2.imwrite(path, out)
        print("")
        if not people:
            print("The AI found NO possible person in this picture.")
        for p in people:
            print("possible person: score %.2f, body points seen %d (needs score >= %.2f and points >= %d)" % (
                p["conf"], int(np.sum(p["kp"][:, 2] >= rn.KP_VIS_THRESH)),
                rn.CAMONLY_MIN_CONF, rn.CAMONLY_MIN_KEYPOINTS))
        print("Saved: %s" % path)
    finally:
        rn.close_camera(cam)
    return 0


if __name__ == "__main__":
    sys.exit(main())
