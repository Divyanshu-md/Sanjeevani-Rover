#!/usr/bin/env python3
"""
patch_speaker.py - adds the speaker message to rover_node.py (two small additions).
It does NOT touch anything related to motors, camera or detection.

Usage (on the Pi, in ~/rescue):
    python3 patch_speaker.py              # patches ~/rescue/rover_node.py
It refuses to change anything if it cannot find the exact places, and it
checks that the result is still valid Python before saving.
"""
import ast
import os
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/rescue/rover_node.py")
src = open(path, encoding="utf-8").read()

if "speaker_alert" in src:
    print("Already patched - nothing to do.")
    sys.exit(0)

IMPORT_LINE = "from protocol import POSTURE_TEXT, build_payload\n"
if src.count(IMPORT_LINE) != 1:
    print("STOP: could not find the import line exactly once. File not changed.")
    sys.exit(1)

pat = re.compile(r'^( +)log\.info\("ALERT sent: %s", text\)\n', re.M)
m = pat.findall(src)
if len(m) != 1:
    print("STOP: could not find the 'ALERT sent' line exactly once. File not changed.")
    sys.exit(1)

imp_add = (
    "\n"
    "# Optional voice message for survivors (never stops the rover if it fails)\n"
    "try:\n"
    "    import speaker_alert\n"
    "except Exception:\n"
    "    speaker_alert = None\n"
)
src = src.replace(IMPORT_LINE, IMPORT_LINE + imp_add, 1)

def add_after_sent(mo):
    ind = mo.group(1)
    return (mo.group(0)
            + ind + "if speaker_alert is not None:\n"
            + ind + "    try:\n"
            + ind + "        speaker_alert.announce()\n"
            + ind + "    except Exception:\n"
            + ind + "        pass\n")

src = pat.sub(add_after_sent, src, count=1)

try:
    ast.parse(src)
except SyntaxError as e:
    print("STOP: the result would not be valid Python (%s). File not changed." % e)
    sys.exit(1)

open(path, "w", encoding="utf-8").write(src)
print("Done. Speaker message added to", path)
