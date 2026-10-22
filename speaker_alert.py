#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
speaker_alert.py - plays a calm voice message through the Pi's speaker
when a person has been found. Does NOT touch the motors or the camera.

Use from rover_node.py (after the alert has really been sent):
    import speaker_alert
    speaker_alert.announce()

Test by hand:
    python3 speaker_alert.py test      # plays the message
    python3 speaker_alert.py say "Hello"   # speaks any text (needs espeak-ng)

Rules built in (so the rover stays honest and quiet):
  * plays when a person is first seen, and again when a NEW person appears
    (the rover saw nobody for ABSENT_S seconds, then someone again)
  * if the same person just stays there, it only repeats every REPEAT_S seconds
  * never twice within MIN_GAP_S seconds
  * never overlaps with a message that is still playing
  * runs in the background, so detection is never slowed down
  * operator mute: create an empty file called MUTE next to this script
        touch ~/rescue/MUTE      (mute)      rm ~/rescue/MUTE      (unmute)
"""
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
AUDIO_DIR = os.path.join(HERE, "audio")
MUTE_FILE = os.path.join(HERE, "MUTE")

MIN_GAP_S = 20            # never play twice within this many seconds
ABSENT_S = 15             # no alerts for this long = the person left; next alert = new person
REPEAT_S = 120            # same person still there: remind after this many seconds
AUDIO_DEVICE = "plughw:Headphones"   # the Pi 3.5 mm jack. None = system default

# Recorded voice files (put WAV files in ~/rescue/audio/). Played in this order.
WAV_FILES = ["found_en.wav", "found_hi.wav"]

# Fallback text (robot voice) if no recorded files exist and espeak-ng is installed.
# Wording states only facts: it does NOT promise that help is coming soon.
TEXT_EN = "We can see you. Please stay calm. A rescue alert has been sent to the team."
TEXT_HI = "हम आपको देख सकते हैं। कृपया शांत रहें। बचाव दल को सूचना भेज दी गई है।"

_last_play = 0.0
_last_seen = 0.0   # time of the most recent announce() call (= person present)
_proc = None


def _busy():
    return _proc is not None and _proc.poll() is None


def _dev():
    return " -D " + _q(AUDIO_DEVICE) if AUDIO_DEVICE else ""


def _play_chain():
    """Return a list of shell commands: recorded files, else espeak-ng through aplay."""
    steps = []
    wavs = [os.path.join(AUDIO_DIR, f) for f in WAV_FILES]
    wavs = [w for w in wavs if os.path.isfile(w)]
    if wavs and shutil.which("aplay"):
        for w in wavs:
            steps.append("aplay -q%s %s" % (_dev(), _q(w)))
    elif shutil.which("espeak-ng") and shutil.which("aplay"):
        # -a 200 = loudest espeak volume; audio goes through aplay so it uses AUDIO_DEVICE
        steps.append("espeak-ng -a 200 -s 120 -g 6 --stdout %s | aplay -q%s" % (_q(TEXT_EN), _dev()))
        steps.append("espeak-ng -a 200 -s 115 -g 6 -v hi --stdout %s | aplay -q%s" % (_q(TEXT_HI), _dev()))
    return steps


def _run_steps(steps):
    """Run the steps one after another in ONE background process."""
    return subprocess.Popen(["sh", "-c", "; ".join(steps)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def _q(a):
    return "'" + a.replace("'", "'\\''") + "'"


def announce(force=False):
    """Call this each time an alert is sent. Plays the message when appropriate.
    Returns True if it started playing, False if skipped."""
    global _last_play, _last_seen, _proc
    now = time.time()
    gap = now - _last_seen          # how long since the previous alert
    _last_seen = now                # always remember: a person is present now
    if not force:
        if os.path.exists(MUTE_FILE) or _busy():
            return False
        since_play = now - _last_play
        new_person = gap > ABSENT_S
        if since_play < MIN_GAP_S:
            return False
        if not new_person and since_play < REPEAT_S:
            return False
    steps = _play_chain()
    if not steps:
        print("[speaker] no audio files and no espeak-ng found; nothing to play")
        return False
    try:
        _proc = _run_steps(steps)
        _last_play = now
        return True
    except Exception as e:   # audio must never crash the rover program
        print("[speaker] could not play:", e)
        return False


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "test":
        ok = announce(force=True)
        print("playing..." if ok else "could not play (see message above)")
        if _proc:
            _proc.wait()
    elif len(sys.argv) >= 3 and sys.argv[1] == "say":
        if shutil.which("espeak-ng") and shutil.which("aplay"):
            subprocess.call("espeak-ng -a 200 --stdout %s | aplay -q%s" % (_q(" ".join(sys.argv[2:])), _dev()), shell=True)
        else:
            print("espeak-ng is not installed")
    else:
        print(__doc__)
