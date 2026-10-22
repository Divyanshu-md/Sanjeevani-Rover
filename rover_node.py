#!/usr/bin/env python3
"""
rover_node.py - runs on the Raspberry Pi 4B on the rover.

WHAT IT DOES (in plain words)
  1. The Pi Camera takes pictures (640x480, 15 fps).
  2. An AI model (YOLOv8n-pose, ONNX) finds a person and 17 body points
     (nose, shoulders, hips, knees ...).
  3. Simple rules turn the body points into one of four SUSPECTED statuses:
        Conscious (Upright/Sitting)        Unconscious (Collapsed/Slumped)
        Conscious Lying Down               Unconscious Lying Down
     IMPORTANT: this is a guess made from a camera picture (body angle and
     whether the body moves).  It is NOT a medical diagnosis.
  4. A rough distance is estimated from the size of the person in the picture
     (single camera, approximate: +-30% or worse, wrong for sitting or
     partly hidden people).
  5. Optional AMG8833 thermal sensor checks for body heat (THERMAL_MODE).
     YOU DO NOT HAVE THIS SENSOR YET, so the default is "off" (camera-only):
     the alert then says "Temp: N/A" and the camera rules are made stricter.
  6. Optional GPS adds latitude/longitude (GPS_ENABLED).  Without a GPS the
     alert says "Fix: NONE" and "Lat/Lon: N/A".
  7. When a human is confirmed, a text alert is sent over the network (UDP)
     to the ground station, and written to rover_events.jsonl.

  The wheels / motors are NOT touched by this program.  The operator drives
  with the FlySky remote.  (operator_view.py shows the USB webcam picture.)

THREE PARTS (they run at the same time):
  SensorThread : reads GPS and the thermal sensor (if switched on).
  VisionThread : camera + AI + posture.
  Main thread  : every 0.1 s combines everything, decides, sends the alert.

RUN (on the Pi, venv active, from the folder with all the files):
    python3 rover_node.py --dry-run -v          # test: prints alerts, sends nothing
    python3 rover_node.py --peer 192.168.50.1   # real run over Ethernet
    python3 rover_node.py --peer 10.226.49.17   # real run over Wi-Fi (laptop address)
FLAGS
    --dry-run     do not send over the network, just print the alerts
    --peer IP     ground station address   (default PEER_IP below)
    --port N      ground station UDP port  (default PEER_PORT below)
    -v            more messages (one status line per second)
  Stop with Ctrl+C.

HONEST LIMITS
  * Posture is a heuristic.  A frozen-in-fear person looks "unconscious"; a
    person moved by wind or by debris looks "conscious".
  * The rover should be STOPPED while it judges movement: if the camera itself
    moves, the whole picture moves and the movement test is fooled.
  * Camera-only mode cannot tell a mannequin or a poster from a person as well
    as the thermal check would.  The rules below are stricter to compensate,
    but it will still make mistakes.
  * Distance is approximate.  GPS often fails under debris; the Fix field says
    how old the position is.  This is a prototype, not rescue equipment.
"""

import argparse
import collections
import datetime
import json
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from link import UdpTx
from protocol import POSTURE_TEXT, build_payload

HERE = os.path.dirname(os.path.abspath(__file__))

# ===========================================================================
# TUNABLE CONSTANTS  (everything you may want to change is here)
# ===========================================================================

# ---- network ---------------------------------------------------------------
PEER_IP = "192.168.50.1"          # ground station (laptop) address
PEER_PORT = 5005                  # UDP port

# ---- camera (Pi Camera, picamera2) ----------------------------------------
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FPS = 15
CAM_HFOV_DEG = 62.2               # horizontal field of view of Pi Camera V2

# ---- AI model --------------------------------------------------------------
MODEL_PATH = os.path.join(HERE, "yolov8n-pose.onnx")
MODEL_IMGSZ = 320                 # size the model was exported with (imgsz=320)
ORT_THREADS = 3                   # CPU threads for onnxruntime
DETECT_CONF = 0.35                # model keeps boxes above this score
NMS_IOU = 0.45                    # boxes overlapping more than this are merged
KP_VIS_THRESH = 0.30              # a body point counts only above this visibility
PERSON_MIN_CONF = 0.50            # "confident person" (used when thermal is on)

# ---- camera-only strictness (used when THERMAL_MODE == "off") -------------
CAMONLY_MIN_CONF = 0.50           # higher score needed without thermal check
CAMONLY_MIN_KEYPOINTS = 5         # at least this many of the 17 body points seen

# ---- "partial view": a person is seen, but too little of the body to judge posture
PARTIAL_ENABLED = True            # False = never send "Person detected (partial view)"
PARTIAL_MIN_CONF = 0.40           # lower the score needed ...
PARTIAL_MIN_KEYPOINTS = 2         # ... and the number of body points seen (any 2 body parts)

# ---- posture rules ---------------------------------------------------------
TORSO_LYING_DEG = 60.0            # torso this far from vertical (or more) = lying
ASPECT_LYING = 1.6                # box width/height this big (or more) = lying
ASPECT_LYING_NO_TORSO = 1.1       # same, but when no torso points are visible
TORSO_SLUMP_DEG = 35.0            # torso this far from vertical = slumped
MOTION_THRESH = 0.02              # average pose movement above this = "moving"
MOTION_WINDOW_S = 5.0             # look at the last 5 seconds (a Pi 3 sees only a few pictures per second)
MOTION_MIN_FRAMES = 4             # need at least this many frames ...
MOTION_MIN_SPAN_S = 2.0           # ... spread over at least this many seconds
PERSON_LOST_RESET_S = 3.0         # forget the movement history after this gap
POSTURE_VOTE_S = 3.0              # the body SHAPE (upright / lying / slumped) is decided by a vote over this many seconds

# ---- camera orientation ---------------------------------------------------
CAM_ROTATE_180 = True             # True = the Pi Camera is mounted UPSIDE DOWN (turn every picture 180 degrees)

# ---- distance (single camera, APPROXIMATE) ---------------------------------
DISTANCE_MODE = "mono"            # "mono" = estimate from person size, "off" = N/A
PERSON_HEIGHT_M = 1.7             # assumed height of a person

# ---- thermal sensor AMG8833 (you do not have it yet) -----------------------
THERMAL_MODE = "off"              # "off" = camera-only, "amg8833" = use the sensor
AMG_I2C_ADDRESS = 0x69            # 0x69 (default) or 0x68
THERMAL_FOV_DEG = 60.0            # AMG8833 field of view (both directions)
THERMAL_FLIP_X = False            # set True if the thermal picture is mirrored left-right
THERMAL_FLIP_Y = False            # set True if it is upside down
THERMAL_ABS_C = 34.0              # a pixel this warm (or warmer) counts as "warm"
THERMAL_DELTA_C = 2.5             # ... or this much warmer than the scene median
THERMAL_DELTA_MEDIAN_MAX_C = 30.0 # the "delta" rule is used only if median is below this
THERMAL_MAX_PEAK_C = 40.0         # hotter than this is not a person (engine, fire)
THERMAL_MAX_CLUSTER_PX = 40       # a warm blob bigger than this is not a person
THERMAL_MARGIN_PX = 1             # widen the window around the person by this many pixels
SENSOR_POLL_HZ = 9.0              # thermal sensor reads per second
# NOTE: an 8x8 sensor at a distance averages each (large) pixel with the cold
# background, so a person 3 m away reads far below 35 C.  That is why the
# "warmer than the scene median" rule exists.

# ---- GPS (Neo-6M / Neo-8M) -------------------------------------------------
GPS_ENABLED = True                # without a GPS the alert says Fix: NONE
GPS_PORT = "/dev/serial0"
GPS_BAUD = 9600
GPS_LIVE_MAX_AGE_S = 3.0          # fix newer than this = LIVE, older = CACHED

# ---- devices ---------------------------------------------------------------
DEVICE_RETRY_S = 5.0              # wait this long before reopening a failed device

# ---- fusion (deciding when to send) ---------------------------------------
FUSION_PERIOD_S = 0.1             # decision every 0.1 s
MAX_DATA_AGE_S = 1.0              # detection / thermal data older than this is ignored
CONFIRM_WINDOW = 5                # look at the last 5 decisions ...
CONFIRM_MIN = 3                   # ... at least 3 must be positive
TX_MIN_INTERVAL_S = 4.0           # send at most one alert every 4 s ...
TX_CHANGE_INTERVAL_S = 1.5        # ... except a CHANGE of status may go after 1.5 s

# ---- files -----------------------------------------------------------------
EVENT_LOG_PATH = os.path.join(HERE, "rover_events.jsonl")

# COCO body point numbers used by the model
NOSE, L_SHOULDER, R_SHOULDER, L_HIP, R_HIP = 0, 5, 6, 11, 12

log = logging.getLogger("rover")


# ===========================================================================
# Data shared between threads
# ===========================================================================
@dataclass
class Detection:
    """One person found in one camera picture."""
    t: float                       # time.monotonic() when the picture was taken
    bbox: tuple                    # (x, y, w, h) in picture pixels
    conf: float                    # model score 0..1
    keypoints: np.ndarray          # 17 x (x, y, visibility)
    n_visible: int                 # how many body points are visible
    posture: Optional[str]         # UPRIGHT / COLLAPSED / LYING_* / PARTIAL or None (not sure yet)
    distance_m: Optional[float]    # rough distance or None
    img_w: int
    img_h: int
    motion: Optional[float] = None # movement score (for the log)


class SharedState:
    """Newest values from every thread, protected by a lock."""

    def __init__(self):
        self._lock = threading.Lock()
        self._det = None
        self._det_time = None      # when the last picture was processed
        self._thermal = None
        self._thermal_time = None
        self._gps = None           # (lat, lon, monotonic time of that fix)
        self._fps = 0.0            # camera pictures analysed per second

    def set_fps(self, fps):
        with self._lock:
            self._fps = fps

    def set_detection(self, t, det):
        """det is a Detection, or None when the picture contained no person."""
        with self._lock:
            self._det, self._det_time = det, t

    def set_thermal(self, t, pixels):
        with self._lock:
            self._thermal, self._thermal_time = pixels, t

    def set_gps(self, lat, lon, t):
        with self._lock:
            self._gps = (lat, lon, t)

    def snapshot(self):
        with self._lock:
            return {
                "det": self._det, "det_time": self._det_time,
                "thermal": self._thermal, "thermal_time": self._thermal_time,
                "gps": self._gps, "fps": self._fps,
            }


# ===========================================================================
# Posture rules
# ===========================================================================
def _mid_point(kp, indices):
    """Average position of the visible points among `indices`, or None."""
    pts = [kp[i, :2] for i in indices if kp[i, 2] >= KP_VIS_THRESH]
    if not pts:
        return None
    return np.mean(pts, axis=0)


def torso_angle_deg(kp):
    """Angle between the torso (shoulder-middle -> hip-middle) and the vertical:
    0 = standing straight, 90 = horizontal.  None if the torso is not visible."""
    shoulders = _mid_point(kp, (L_SHOULDER, R_SHOULDER))
    hips = _mid_point(kp, (L_HIP, R_HIP))
    if shoulders is None or hips is None:
        return None
    v = hips - shoulders
    if float(np.hypot(v[0], v[1])) < 1e-6:
        return None
    return math.degrees(math.atan2(abs(float(v[0])), abs(float(v[1]))))


def classify_geometry(kp, bbox):
    """Returns (lying, slumped, angle).  Only the SHAPE of the body is used here."""
    _, _, w, h = bbox
    aspect = (w / h) if h > 0 else 99.0
    angle = torso_angle_deg(kp)
    if angle is None:                                  # no torso visible
        lying = aspect > ASPECT_LYING_NO_TORSO
    else:
        lying = angle >= TORSO_LYING_DEG or aspect >= ASPECT_LYING
    slumped = False
    if not lying:
        if angle is not None and angle >= TORSO_SLUMP_DEG:
            slumped = True
        else:                                          # head lower than the shoulders?
            shoulders = _mid_point(kp, (L_SHOULDER, R_SHOULDER))
            if shoulders is not None and kp[NOSE, 2] >= KP_VIS_THRESH and kp[NOSE, 1] > shoulders[1]:
                slumped = True
    return lying, slumped, angle


def decide_posture(kp, bbox, motion):
    """Turn shape + movement into one of the four keys (or None = not sure yet).
    motion is the movement score, or None when there is not enough history."""
    lying, slumped, _ = classify_geometry(kp, bbox)
    return decide_posture_shape(lying, slumped, motion)


def vote_shape(history, now):
    """One picture can be wrong (a bad body fit), so the body SHAPE is decided
    by a vote over the last POSTURE_VOTE_S seconds.  history is a list of
    (time, lying, slumped).  Returns (lying, slumped)."""
    recent = [(l, s) for t, l, s in history if now - t <= POSTURE_VOTE_S]
    if not recent:
        return False, False
    n = float(len(recent))
    lying_share = sum(1 for l, _ in recent if l) / n
    slumped_share = sum(1 for l, s in recent if s and not l) / n
    if lying_share >= 0.5:
        return True, False
    if lying_share + slumped_share >= 0.5:
        return False, True
    return False, False


def decide_posture_shape(lying, slumped, motion):
    """Same as decide_posture, but the shape (lying / slumped) is already known."""
    if not lying and not slumped:
        return "UPRIGHT"                               # no history needed
    if motion is None:
        return None                                    # wait for more pictures
    moving = motion >= MOTION_THRESH
    if lying:
        return "LYING_CONSCIOUS" if moving else "LYING_UNCONSCIOUS"
    return "UPRIGHT" if moving else "COLLAPSED"


class MotionTracker:
    """Measures how much the body points move over the last few seconds (MOTION_WINDOW_S).

    Every body point is written relative to the middle of the person's box and
    divided by the box's long side, so walking closer/farther or sideways does
    not count as body movement.  For each point we take the standard deviation
    of its position (x and y combined); the score is the average over points."""

    def __init__(self):
        self.samples = collections.deque()   # (time, {point number: (nx, ny)})

    def reset(self):
        self.samples.clear()

    def add(self, t, kp, bbox):
        x, y, w, h = bbox
        cx, cy = x + w / 2.0, y + h / 2.0
        scale = max(w, h)
        if scale <= 0:
            return
        pose = {}
        for i in range(kp.shape[0]):
            if kp[i, 2] >= KP_VIS_THRESH:
                pose[i] = ((kp[i, 0] - cx) / scale, (kp[i, 1] - cy) / scale)
        self.samples.append((t, pose))
        while self.samples and t - self.samples[0][0] > MOTION_WINDOW_S:
            self.samples.popleft()

    def score(self):
        """Movement score, or None if there is not enough history yet.

        Only body points that were seen in most of the recent pictures are used
        (a point that flickers in and out would look like movement).  In every
        picture the middle of those points is subtracted, so the box jumping
        around by a few pixels or the whole body shifting does not count; only
        the body points moving AGAINST each other (arms, legs, head) do."""
        if len(self.samples) < MOTION_MIN_FRAMES:
            return None
        if self.samples[-1][0] - self.samples[0][0] < MOTION_MIN_SPAN_S:
            return None
        n = len(self.samples)
        stable = []
        for i in range(17):
            seen = sum(1 for _, p in self.samples if i in p)
            if seen >= max(MOTION_MIN_FRAMES, 0.7 * n):
                stable.append(i)
        if len(stable) < 3:
            return None
        relative = []                                    # one {point: (x, y)} per picture
        for _, p in self.samples:
            pts = [p[i] for i in stable if i in p]
            if len(pts) < 3:
                continue
            mx = sum(q[0] for q in pts) / len(pts)
            my = sum(q[1] for q in pts) / len(pts)
            relative.append({i: (p[i][0] - mx, p[i][1] - my) for i in stable if i in p})
        if len(relative) < MOTION_MIN_FRAMES:
            return None
        per_point = []
        for i in stable:
            xs = [r[i][0] for r in relative if i in r]
            ys = [r[i][1] for r in relative if i in r]
            if len(xs) >= MOTION_MIN_FRAMES:
                per_point.append(math.hypot(float(np.std(xs)), float(np.std(ys))))
        if not per_point:
            return None
        return float(np.mean(per_point))


EDGE_MARGIN_PX = 4                # a box this close to the picture edge = body cut off


def box_touches_edge(bbox, img_w, img_h):
    """True if the person's box reaches the edge of the picture (the body is cut
    off, so its size says nothing about the distance)."""
    x, y, w, h = bbox
    return (x <= EDGE_MARGIN_PX or y <= EDGE_MARGIN_PX
            or x + w >= img_w - EDGE_MARGIN_PX or y + h >= img_h - EDGE_MARGIN_PX)


def estimate_distance_m(bbox, img_w):
    """APPROXIMATE distance from one camera: Z = focal_px * 1.7 m / box long side.
    Wrong for sitting or partly hidden people.  Returns None if switched off."""
    if DISTANCE_MODE != "mono":
        return None
    long_side = max(bbox[2], bbox[3])
    if long_side < 1.0:
        return None
    f_px = img_w / (2.0 * math.tan(math.radians(CAM_HFOV_DEG) / 2.0))
    return float(f_px * PERSON_HEIGHT_M / long_side)


# ===========================================================================
# AI model: pre-processing, decoding, detector
# ===========================================================================
def letterbox(img, size):
    """Shrink the picture to fit a size x size square without changing its
    shape, filling the empty border with grey (114).
    Returns (square picture, scale, (pad_left, pad_top))."""
    import cv2
    h, w = img.shape[:2]
    r = min(size / float(w), size / float(h))
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    dw, dh = (size - nw) / 2.0, (size - nh) / 2.0
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    out = cv2.copyMakeBorder(resized, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, r, (left, top)


def decode_pose_output(out, r, pad, img_w, img_h, conf_thresh, iou_thresh):
    """Turn the model output [1, 56, N] into a list of people.
    Each of the N candidates has: cx, cy, w, h, score, then 17 x (x, y, visibility).
    Positions are in the letterboxed picture and are mapped back to the
    original camera picture here.  Returns a list of dicts with
    'bbox' (x, y, w, h), 'conf', 'kp' (17 x 3)."""
    import cv2
    arr = np.asarray(out)
    if arr.ndim == 3:
        arr = arr[0]
    if arr.shape[0] == 56:            # [56, N] -> [N, 56]
        arr = arr.T
    if arr.ndim != 2 or arr.shape[1] < 56:
        return []
    arr = arr[arr[:, 4] >= conf_thresh]
    if arr.shape[0] == 0:
        return []

    x1 = (arr[:, 0] - arr[:, 2] / 2.0 - pad[0]) / r
    y1 = (arr[:, 1] - arr[:, 3] / 2.0 - pad[1]) / r
    bw = arr[:, 2] / r
    bh = arr[:, 3] / r
    scores = arr[:, 4]

    # NMS (removes duplicate boxes around the same person).  Whole-pixel boxes
    # are used for NMS because that is the safest form for every OpenCV version.
    nms_boxes = [[int(round(a)), int(round(b)), int(round(c)), int(round(d))]
                 for a, b, c, d in zip(x1, y1, bw, bh)]
    keep = cv2.dnn.NMSBoxes(nms_boxes, [float(s) for s in scores], float(conf_thresh), float(iou_thresh))
    keep = np.array(keep).flatten().astype(int)

    people = []
    for i in keep:
        kp = arr[i, 5:56].reshape(17, 3).astype(np.float64).copy()
        kp[:, 0] = (kp[:, 0] - pad[0]) / r
        kp[:, 1] = (kp[:, 1] - pad[1]) / r
        if kp[:, 2].min() < 0.0 or kp[:, 2].max() > 1.0:   # raw scores -> 0..1
            kp[:, 2] = 1.0 / (1.0 + np.exp(-kp[:, 2]))
        bx0, by0 = max(0.0, float(x1[i])), max(0.0, float(y1[i]))
        bx1, by1 = min(float(img_w), float(x1[i] + bw[i])), min(float(img_h), float(y1[i] + bh[i]))
        if bx1 - bx0 < 1.0 or by1 - by0 < 1.0:
            continue
        people.append({"bbox": (bx0, by0, bx1 - bx0, by1 - by0),
                       "conf": float(scores[i]), "kp": kp})
    return people


class PoseDetector:
    """Runs the YOLOv8n-pose ONNX model with onnxruntime on the Pi's CPU.
    `session` can be given for testing (a stand-in for the real model)."""

    def __init__(self, model_path, session=None):
        if session is None:
            import onnxruntime as ort                   # imported here on purpose
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = ORT_THREADS
            session = ort.InferenceSession(model_path, sess_options=opts,
                                           providers=["CPUExecutionProvider"])
        self.session = session
        inp = session.get_inputs()[0]
        self.input_name = inp.name
        shape = list(inp.shape)
        self.size = MODEL_IMGSZ
        if len(shape) == 4 and isinstance(shape[2], int) and shape[2] == shape[3]:
            self.size = int(shape[2])                  # trust the model file

    def detect(self, frame_bgr):
        import cv2
        h, w = frame_bgr.shape[:2]
        padded, r, pad = letterbox(frame_bgr, self.size)
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        blob = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)
        out = self.session.run(None, {self.input_name: blob})[0]
        return decode_pose_output(out, r, pad, w, h, DETECT_CONF, NMS_IOU)


def person_ok(person):
    """Is this candidate a 'confident person'?  Stricter when there is no
    thermal sensor to double-check."""
    n_vis = int(np.sum(person["kp"][:, 2] >= KP_VIS_THRESH))
    if THERMAL_MODE == "off":
        return person["conf"] >= CAMONLY_MIN_CONF and n_vis >= CAMONLY_MIN_KEYPOINTS
    return person["conf"] >= PERSON_MIN_CONF


class VisionPipeline:
    """picture -> Detection (or None).  Keeps the movement history."""

    def __init__(self, detector):
        self.detector = detector
        self.tracker = MotionTracker()
        self.last_seen = None
        self._last_reject_log = -10.0
        self.shapes = collections.deque()    # (time, lying, slumped) of recent pictures
        self.voted = "upright"               # the shape decided by the vote

    def _partial_detection(self, candidates, now, img_w, img_h):
        """A person is seen but NOT well enough for a posture (too few body
        points, or a low score).  Returns a Detection with posture PARTIAL, or None."""
        if not PARTIAL_ENABLED:
            return None
        ok = [p for p in candidates
              if p["conf"] >= PARTIAL_MIN_CONF
              and int(np.sum(p["kp"][:, 2] >= KP_VIS_THRESH)) >= PARTIAL_MIN_KEYPOINTS]
        if not ok:
            return None
        best = max(ok, key=lambda p: p["conf"])
        return Detection(
            t=now, bbox=best["bbox"], conf=best["conf"], keypoints=best["kp"],
            n_visible=int(np.sum(best["kp"][:, 2] >= KP_VIS_THRESH)),
            posture="PARTIAL", distance_m=None,        # a cut-off body gives a wrong distance
            img_w=img_w, img_h=img_h, motion=None)

    def process(self, frame, now):
        img_h, img_w = frame.shape[:2]
        candidates = self.detector.detect(frame)
        good = [p for p in candidates if person_ok(p)]
        if candidates and not good and now - self._last_reject_log >= 1.0 and log.isEnabledFor(logging.DEBUG):
            # (-v only) say WHY a possible person was not accepted
            self._last_reject_log = now
            top = max(candidates, key=lambda p: p["conf"])
            need_conf = CAMONLY_MIN_CONF if THERMAL_MODE == "off" else PERSON_MIN_CONF
            need_pts = CAMONLY_MIN_KEYPOINTS if THERMAL_MODE == "off" else 0
            log.debug("possible person REJECTED: score %.2f (needs >= %.2f), body points seen %d (needs >= %d)",
                      top["conf"], need_conf, int(np.sum(top["kp"][:, 2] >= KP_VIS_THRESH)), need_pts)
        if not good:
            if self.last_seen is not None and now - self.last_seen > PERSON_LOST_RESET_S:
                self.tracker.reset()
                self.shapes.clear()
                self.voted = "upright"
                self.last_seen = None
            partial = self._partial_detection(candidates, now, img_w, img_h)
            return partial
        best = max(good, key=lambda p: p["bbox"][2] * p["bbox"][3])    # largest person
        if self.last_seen is not None and now - self.last_seen > PERSON_LOST_RESET_S:
            self.tracker.reset()                                       # long gap: start again
            self.shapes.clear()
            self.voted = "upright"
        self.last_seen = now
        kp, bbox = best["kp"], best["bbox"]
        lying_now, slumped_now, _ = classify_geometry(kp, bbox)
        self.shapes.append((now, lying_now, slumped_now))
        while self.shapes and now - self.shapes[0][0] > POSTURE_VOTE_S:
            self.shapes.popleft()
        lying, slumped = vote_shape(list(self.shapes), now)
        voted = "lying" if lying else ("slumped" if slumped else "upright")
        if voted != self.voted:                       # the person really changed shape:
            self.tracker.reset()                      # start the movement test again
            self.voted = voted
        this_one = "lying" if lying_now else ("slumped" if slumped_now else "upright")
        if this_one == voted:                         # a picture with a wrong body fit must not
            self.tracker.add(now, kp, bbox)           # look like "movement"
        motion = self.tracker.score()
        return Detection(
            t=now, bbox=bbox, conf=best["conf"], keypoints=kp,
            n_visible=int(np.sum(kp[:, 2] >= KP_VIS_THRESH)),
            posture=decide_posture_shape(lying, slumped, motion),
            distance_m=None if box_touches_edge(bbox, img_w, img_h) else estimate_distance_m(bbox, img_w),
            img_w=img_w, img_h=img_h, motion=motion)


def open_camera():
    """Open the Pi Camera.  'RGB888' gives a numpy array in BGR byte order,
    which is exactly what OpenCV expects."""
    from picamera2 import Picamera2
    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": "RGB888"},
        controls={"FrameRate": CAM_FPS})
    cam.configure(cfg)
    cam.start()
    time.sleep(1.0)                                    # let exposure settle
    return cam


def grab_frame(cam):
    """One picture from the Pi Camera as a BGR array, turned the right way up
    (CAM_ROTATE_180).  Everything (AI, calibration, test tools) uses this."""
    frame = cam.capture_array()
    if frame.ndim == 3 and frame.shape[2] == 4:
        frame = frame[:, :, :3]
    if CAM_ROTATE_180:
        frame = np.ascontiguousarray(frame[::-1, ::-1])
    return frame


def close_camera(cam):
    try:
        cam.stop()
        cam.close()
    except Exception:
        pass


class VisionThread(threading.Thread):
    """Reads the camera and publishes the newest Detection.  Never dies."""

    def __init__(self, state, stop):
        super().__init__(name="vision", daemon=True)
        self.state, self.stop = state, stop

    def run(self):
        pipeline, cam = None, None
        count, count_start = 0, time.monotonic()
        while not self.stop.is_set():
            try:
                if pipeline is None:
                    pipeline = VisionPipeline(PoseDetector(MODEL_PATH))
                    log.info("AI model loaded (%s, input %dx%d)", os.path.basename(MODEL_PATH),
                             pipeline.detector.size, pipeline.detector.size)
                if cam is None:
                    cam = open_camera()
                    log.info("Pi Camera opened (%dx%d @ %d fps)", CAM_WIDTH, CAM_HEIGHT, CAM_FPS)
                frame = grab_frame(cam)
                now = time.monotonic()
                self.state.set_detection(now, pipeline.process(frame, now))
                count += 1
                if now - count_start >= 2.0:
                    self.state.set_fps(count / (now - count_start))
                    count, count_start = 0, now
            except Exception as exc:       # keep going whatever happens
                log.error("vision problem: %s - retrying in %.0f s", exc, DEVICE_RETRY_S)
                if cam is not None:
                    close_camera(cam)
                    cam = None
                self.stop.wait(DEVICE_RETRY_S)
        if cam is not None:
            close_camera(cam)


# ===========================================================================
# GPS
# ===========================================================================
def _nmea_checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return calc == want


def _nmea_to_degrees(value, hemisphere):
    """NMEA 'ddmm.mmmm' (or 'dddmm.mmmm') plus N/S/E/W -> signed degrees."""
    if not value or not hemisphere:
        return None
    dot = value.find(".")
    if dot < 3:
        return None
    degrees = float(value[:dot - 2]) + float(value[dot - 2:]) / 60.0
    return -degrees if hemisphere in ("S", "W") else degrees


def _parse_nmea_fallback(line):
    """Small built-in parser, used only if pynmea2 is not installed."""
    if not _nmea_checksum_ok(line):
        return None
    f = line[1:].split("*")[0].split(",")
    kind = f[0][-3:]
    try:
        if kind == "GGA" and len(f) > 6:
            if not f[6] or int(f[6]) <= 0:
                return None
            lat, lon = _nmea_to_degrees(f[2], f[3]), _nmea_to_degrees(f[4], f[5])
        elif kind == "RMC" and len(f) > 6:
            if f[2] != "A":
                return None
            lat, lon = _nmea_to_degrees(f[3], f[4]), _nmea_to_degrees(f[5], f[6])
        else:
            return None
    except ValueError:
        return None
    if lat is None or lon is None:
        return None
    return lat, lon


def parse_nmea_line(line):
    """One NMEA text line -> (lat, lon) in degrees, or None.
    Accepts GGA (fix quality > 0) and RMC (status A) from ANY talker (GP, GN ...)."""
    line = line.strip()
    if not line.startswith("$"):
        return None
    try:
        import pynmea2
    except ImportError:
        return _parse_nmea_fallback(line)
    try:
        msg = pynmea2.parse(line, check=True)          # check=True rejects damaged lines
        kind = getattr(msg, "sentence_type", "")
        if kind == "GGA":
            quality = msg.gps_qual
            if quality is None or int(quality) <= 0:
                return None
        elif kind == "RMC":
            if msg.status != "A":
                return None
        else:
            return None
        return float(msg.latitude), float(msg.longitude)
    except Exception:
        return None


def gps_fix_label(gps, now):
    """gps = (lat, lon, time) or None.  Returns (label, lat, lon):
    LIVE if the fix is <= 3 s old, CACHED-<n>s if older, NONE if never."""
    if gps is None:
        return "NONE", None, None
    lat, lon, t = gps
    age = now - t
    if age <= GPS_LIVE_MAX_AGE_S:
        return "LIVE", lat, lon
    return "CACHED-%ds" % int(age), lat, lon


# ===========================================================================
# Thermal check (only used when THERMAL_MODE == "amg8833")
# ===========================================================================
def thermal_window(bbox, img_w, img_h):
    """Which part of the 8x8 thermal grid looks at the person's box?
    The camera angle of each box edge is worked out from the camera field of
    view (the vertical field of view follows from the picture shape), and then
    placed on the 8x8 grid, which covers THERMAL_FOV_DEG in both directions.
    Returns (row0, row1, col0, col1), inclusive, with the +-1 pixel margin."""
    f_px = img_w / (2.0 * math.tan(math.radians(CAM_HFOV_DEG) / 2.0))
    deg_per_px = THERMAL_FOV_DEG / 8.0
    x, y, w, h = bbox

    def to_cells(a, b, centre):
        lo = 4.0 + math.degrees(math.atan((a - centre) / f_px)) / deg_per_px
        hi = 4.0 + math.degrees(math.atan((b - centre) / f_px)) / deg_per_px
        i0 = int(math.floor(min(lo, hi)))
        i1 = int(math.floor(max(lo, hi)))
        i0 = min(7, max(0, i0 - THERMAL_MARGIN_PX))
        i1 = min(7, max(0, i1 + THERMAL_MARGIN_PX))
        return i0, i1

    c0, c1 = to_cells(x, x + w, img_w / 2.0)
    r0, r1 = to_cells(y, y + h, img_h / 2.0)
    if THERMAL_FLIP_X:
        c0, c1 = 7 - c1, 7 - c0
    if THERMAL_FLIP_Y:
        r0, r1 = 7 - r1, 7 - r0
    return r0, r1, c0, c1


def find_clusters(mask):
    """Groups of touching (8 directions) True pixels.  Returns lists of (row, col)."""
    rows, cols = mask.shape
    seen = np.zeros(mask.shape, dtype=bool)
    clusters = []
    for r in range(rows):
        for c in range(cols):
            if mask[r, c] and not seen[r, c]:
                stack, pts = [(r, c)], []
                seen[r, c] = True
                while stack:
                    rr, cc = stack.pop()
                    pts.append((rr, cc))
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            nr, nc = rr + dr, cc + dc
                            if (dr or dc) and 0 <= nr < rows and 0 <= nc < cols \
                                    and mask[nr, nc] and not seen[nr, nc]:
                                seen[nr, nc] = True
                                stack.append((nr, nc))
                clusters.append(pts)
    return clusters


def thermal_verify(pixels, bbox, img_w, img_h):
    """Is there body-like warmth where the camera sees the person?
    Returns the peak temperature (deg C) of the best warm blob, or None."""
    pixels = np.asarray(pixels, dtype=float)
    if pixels.shape != (8, 8) or not np.all(np.isfinite(pixels)):
        return None
    median = float(np.median(pixels))
    warm = pixels >= THERMAL_ABS_C
    if median < THERMAL_DELTA_MEDIAN_MAX_C:
        warm = warm | (pixels >= median + THERMAL_DELTA_C)
    r0, r1, c0, c1 = thermal_window(bbox, img_w, img_h)
    best = None                                   # (pixels inside window, peak)
    for pts in find_clusters(warm):
        inside = sum(1 for (r, c) in pts if r0 <= r <= r1 and c0 <= c <= c1)
        if inside == 0:
            continue
        peak = max(float(pixels[r, c]) for (r, c) in pts)
        if peak > THERMAL_MAX_PEAK_C or len(pts) > THERMAL_MAX_CLUSTER_PX:
            continue
        if best is None or (inside, peak) > best:
            best = (inside, peak)
    return None if best is None else best[1]


# ===========================================================================
# Sensor thread: GPS + thermal sensor
# ===========================================================================
class SensorThread(threading.Thread):
    """Reads the GPS (without blocking) and the thermal sensor (~9 times a
    second).  A device that fails is reopened every DEVICE_RETRY_S seconds."""

    def __init__(self, state, stop):
        super().__init__(name="sensors", daemon=True)
        self.state, self.stop = state, stop

    def run(self):
        gps, amg = None, None
        buf = b""
        next_gps_try = next_amg_try = 0.0
        last_amg = 0.0
        gps_failed = amg_failed = False
        while not self.stop.is_set():
            now = time.monotonic()

            # ---- GPS ----
            if GPS_ENABLED:
                if gps is None and now >= next_gps_try:
                    try:
                        import serial                      # pyserial, imported here on purpose
                        gps = serial.Serial(GPS_PORT, GPS_BAUD, timeout=0)
                        log.info("GPS opened on %s", GPS_PORT)
                        gps_failed = False
                        buf = b""
                    except Exception as exc:
                        if not gps_failed:
                            log.warning("GPS not available (%s) - alerts will say Fix: NONE; "
                                        "retrying every %.0f s", exc, DEVICE_RETRY_S)
                            gps_failed = True
                        gps = None
                        next_gps_try = now + DEVICE_RETRY_S
                if gps is not None:
                    try:
                        waiting = gps.in_waiting
                        if waiting:
                            buf += gps.read(waiting)
                            buf = buf[-4096:]
                        *lines, buf = buf.split(b"\n")
                        for raw in lines:
                            fix = parse_nmea_line(raw.decode("ascii", "ignore"))
                            if fix is not None:
                                self.state.set_gps(fix[0], fix[1], time.monotonic())
                    except Exception as exc:
                        log.warning("GPS read problem (%s) - reopening", exc)
                        try:
                            gps.close()
                        except Exception:
                            pass
                        gps = None
                        next_gps_try = now + DEVICE_RETRY_S

            # ---- thermal sensor ----
            if THERMAL_MODE == "amg8833":
                if amg is None and now >= next_amg_try:
                    try:
                        import board                       # imported here on purpose
                        import busio
                        import adafruit_amg88xx
                        i2c = busio.I2C(board.SCL, board.SDA)
                        amg = adafruit_amg88xx.AMG88XX(i2c, addr=AMG_I2C_ADDRESS)
                        log.info("AMG8833 thermal sensor opened at 0x%02X", AMG_I2C_ADDRESS)
                        amg_failed = False
                    except Exception as exc:
                        if not amg_failed:
                            log.warning("AMG8833 not available (%s) - no alerts without it; "
                                        "retrying every %.0f s", exc, DEVICE_RETRY_S)
                            amg_failed = True
                        amg = None
                        next_amg_try = now + DEVICE_RETRY_S
                if amg is not None and now - last_amg >= 1.0 / SENSOR_POLL_HZ:
                    last_amg = now
                    try:
                        px = np.array(amg.pixels, dtype=float)     # 8x8 degrees C
                        if px.shape == (8, 8):
                            self.state.set_thermal(time.monotonic(), px)
                    except Exception as exc:
                        log.warning("thermal sensor read problem (%s) - reopening", exc)
                        amg = None
                        next_amg_try = now + DEVICE_RETRY_S

            self.stop.wait(0.05)
        if gps is not None:
            try:
                gps.close()
            except Exception:
                pass


# ===========================================================================
# Fusion: decide when a human is confirmed and an alert must be sent
# ===========================================================================
class Fusion:
    """Called every 0.1 s.  A decision is POSITIVE when the newest camera
    picture (not older than 1 s) shows a person with a known status and, if
    the thermal sensor is switched on, warmth is found at the person.  An alert
    needs the latest decision positive AND at least 3 of the last 5 positive.
    (A picture without a person counts as 'no person', so one wrong picture
    cannot keep the decisions positive.)"""

    def __init__(self):
        self.ticks = collections.deque(maxlen=CONFIRM_WINDOW)
        self.last_tx_time = None
        self.last_tx_label = None
        self.seq = 0

    def step(self, now, snap):
        det, det_t = snap["det"], snap["det_time"]
        positive, peak = False, None
        if det is not None and det_t is not None and now - det_t < MAX_DATA_AGE_S \
                and det.posture is not None:
            if THERMAL_MODE == "amg8833":
                th, th_t = snap["thermal"], snap["thermal_time"]
                if th is not None and th_t is not None and now - th_t < MAX_DATA_AGE_S:
                    peak = thermal_verify(th, det.bbox, det.img_w, det.img_h)
                    positive = peak is not None
            else:
                positive = True                          # camera-only mode
        self.ticks.append(positive)
        if not (positive and sum(self.ticks) >= CONFIRM_MIN):
            return None

        label = det.posture
        if self.last_tx_time is None:
            due = True
        else:
            since = now - self.last_tx_time
            due = since >= TX_MIN_INTERVAL_S or (label != self.last_tx_label and since >= TX_CHANGE_INTERVAL_S)
        if not due:
            return None

        fix, lat, lon = gps_fix_label(snap["gps"], now)
        self.seq += 1
        self.last_tx_time, self.last_tx_label = now, label
        return {"seq": self.seq, "label": label, "distance_m": det.distance_m,
                "lat": lat, "lon": lon, "temp_c": peak, "fix": fix}


# ===========================================================================
# Sending and logging alerts
# ===========================================================================
class EventLog:
    """Append-only JSON-lines file; every line is flushed to disk."""

    def __init__(self, path):
        self.fh = open(path, "a", encoding="utf-8")

    def write(self, record):
        self.fh.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.fh.flush()
        try:
            os.fsync(self.fh.fileno())
        except OSError:
            pass

    def close(self):
        self.fh.close()


def send_alert(alert, sender, events, dry_run):
    """Build the text alert, send it (or just print it in --dry-run), log it."""
    try:
        payload = build_payload(POSTURE_TEXT[alert["label"]], alert["distance_m"], alert["lat"],
                                alert["lon"], alert["temp_c"], alert["fix"], alert["seq"])
    except ValueError as exc:
        log.error("could not build alert #%d: %s", alert["seq"], exc)
        return False
    text = payload.decode("utf-8")
    if dry_run:
        sent = None
        log.info("DRY-RUN alert (not sent): %s", text)
    else:
        sent = sender.send(payload)
        if sent:
            log.info("ALERT sent: %s", text)
        else:
            log.error("ALERT NOT SENT (network problem?): %s", text)
    events.write({
        "time_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds"),
        "seq": alert["seq"], "status": POSTURE_TEXT[alert["label"]],
        "distance_m": alert["distance_m"], "lat": alert["lat"], "lon": alert["lon"],
        "temp_c": alert["temp_c"], "fix": alert["fix"],
        "sent": sent, "dry_run": dry_run, "payload": text})
    return bool(sent) if not dry_run else True


# ===========================================================================
# Main
# ===========================================================================
def run(args, stop):
    """The main loop.  Returns a program exit code."""
    if not os.path.exists(MODEL_PATH):
        print("AI model file not found: %s" % MODEL_PATH)
        print("Copy yolov8n-pose.onnx into the same folder as rover_node.py.")
        return 2
    try:
        import onnxruntime  # noqa: F401  (only checking that it is installed)
    except ImportError:
        print("onnxruntime is not installed.  Run:  pip install onnxruntime")
        return 2

    sender = None if args.dry_run else UdpTx(args.peer, args.port)
    events = EventLog(EVENT_LOG_PATH)
    state = SharedState()
    fusion = Fusion()

    log.info("rover_node starting: %s, thermal=%s, gps=%s, distance=%s",
             "DRY-RUN (nothing is sent)" if args.dry_run else "sending to %s:%d" % (args.peer, args.port),
             THERMAL_MODE, "on" if GPS_ENABLED else "off", DISTANCE_MODE)
    if THERMAL_MODE == "off":
        log.info("camera-only mode: alerts say Temp: N/A (no body-heat check)")

    threads = [VisionThread(state, stop)]
    if GPS_ENABLED or THERMAL_MODE == "amg8833":
        threads.append(SensorThread(state, stop))
    for t in threads:
        t.start()

    try:
        next_tick = time.monotonic()
        last_status = 0.0
        while not stop.is_set():
            now = time.monotonic()
            snap = state.snapshot()
            alert = fusion.step(now, snap)
            if alert is not None:
                send_alert(alert, sender, events, args.dry_run)
            if args.verbose and now - last_status >= 1.0:
                last_status = now
                det = snap["det"]
                if det is None:
                    log.debug("no person in view (%.1f pictures/s)", snap["fps"])
                else:
                    log.debug("person conf=%.2f points=%d posture=%s motion=%s distance=%s (%.1f pictures/s)",
                              det.conf, det.n_visible, det.posture,
                              "n/a" if det.motion is None else "%.3f" % det.motion,
                              "n/a" if det.distance_m is None else "%.1f m" % det.distance_m,
                              snap["fps"])
            next_tick += FUSION_PERIOD_S
            if next_tick < now:                      # fell behind: do not rush to catch up
                next_tick = now + FUSION_PERIOD_S
            stop.wait(max(0.0, next_tick - time.monotonic()))
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=3.0)
        if sender is not None:
            sender.close()
        events.close()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Rescue rover node (camera, posture, alerts).")
    ap.add_argument("--dry-run", action="store_true", help="print alerts instead of sending them")
    ap.add_argument("--peer", default=PEER_IP, help="ground station IP address (default %s)" % PEER_IP)
    ap.add_argument("--port", type=int, default=PEER_PORT, help="ground station UDP port (default %d)" % PEER_PORT)
    ap.add_argument("-v", "--verbose", action="store_true", help="show a status line every second")
    args = ap.parse_args(argv)

    # Only OUR messages get the extra -v detail; other libraries (picamera2 ...)
    # stay at INFO so they do not flood the screen.
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    if args.verbose:
        log.setLevel(logging.DEBUG)
    stop = threading.Event()
    try:
        return run(args, stop)
    except KeyboardInterrupt:
        stop.set()
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
