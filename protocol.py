#!/usr/bin/env python3
"""
protocol.py - the alert message format shared by BOTH programs
(rover_node.py builds messages, ground_station.py parses them).

Only the Python standard library is used, so this runs on any PC.

One alert = one UTF-8 text line sent as one UDP datagram:

ALERT: Human found! Status: <class text> | Distance: <X.X m or N/A> | Lat: <6 decimals or N/A> | Lon: <6 decimals or N/A> | Temp: <T.T>°C or N/A | Fix: <LIVE|CACHED-<n>s|NONE> | Seq: <n>*<CRC>

<CRC> = 4 UPPERCASE hex digits, CRC-16/CCITT-FALSE
        (binascii.crc_hqx(data, 0xFFFF)) computed over ALL bytes that come
        BEFORE the '*'.  The CRC lets the ground station reject packets that
        were damaged or tampered with.

IMPORTANT: the posture labels are SUSPECTED statuses guessed from a camera
image.  They are NOT a medical diagnosis.
"""

import binascii
import re

# --------------------------------------------------------------------------
# Posture keys -> the exact text that is sent over the link.
# (rover_node.py imports this dictionary so the text is defined only once.)
# --------------------------------------------------------------------------
POSTURE_TEXT = {
    "UPRIGHT": "Conscious (Upright/Sitting)",
    "COLLAPSED": "Unconscious (Collapsed/Slumped)",
    "LYING_CONSCIOUS": "Conscious Lying Down",
    "LYING_UNCONSCIOUS": "Unconscious Lying Down",
    "PARTIAL": "Person detected (partial view)",
}

PREFIX = "ALERT: Human found! "
FIELD_SEP = " | "

# Regular expressions that every field must match exactly.
_FIX_RE = re.compile(r"^(LIVE|NONE|CACHED-\d+s)$")
_DIST_RE = re.compile(r"^(\d+\.\d) m$")
_LATLON_RE = re.compile(r"^-?\d{1,3}\.\d{6}$")
_TEMP_RE = re.compile(r"^(-?\d+\.\d)\u00b0C$")  # \u00b0 is the degree sign
_SEQ_RE = re.compile(r"^\d+$")
_CRC_RE = re.compile(r"^[0-9A-F]{4}$")


def crc16(data):
    """Return the CRC-16/CCITT-FALSE of `data` (bytes) as an int 0..65535."""
    return binascii.crc_hqx(data, 0xFFFF)


def build_payload(status_text, distance_m, lat, lon, temp_c, fix, seq):
    """
    Build one alert datagram and return it as bytes.

    status_text : one of the POSTURE_TEXT values (any text without '|', '*'
                  or line breaks is accepted)
    distance_m  : float metres, or None  -> "N/A"
    lat, lon    : float degrees, or None -> "N/A"
    temp_c      : float, peak thermal temperature in degrees C, or None when
                  there is no thermal sensor (camera-only mode) -> "N/A"
    fix         : "LIVE", "NONE" or "CACHED-<n>s"
    seq         : integer sequence number (0, 1, 2, ...)
    Raises ValueError if a value cannot be represented in the format.
    """
    if not status_text or any(c in status_text for c in "|*\r\n"):
        raise ValueError("bad status text")
    if not _FIX_RE.match(str(fix)):
        raise ValueError("bad fix value: %r" % (fix,))
    if int(seq) < 0:
        raise ValueError("seq must be >= 0")

    if distance_m is None:
        dist_txt = "N/A"
    else:
        if distance_m < 0:
            raise ValueError("negative distance")
        dist_txt = "%.1f m" % distance_m

    if lat is None or lon is None:
        lat_txt = lon_txt = "N/A"
    else:
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError("lat/lon out of range")
        lat_txt = "%.6f" % lat
        lon_txt = "%.6f" % lon

    if temp_c is None:
        temp_txt = "N/A"
    else:
        temp_txt = "%.1f\u00b0C" % float(temp_c)

    body = (
        PREFIX
        + "Status: " + status_text + FIELD_SEP
        + "Distance: " + dist_txt + FIELD_SEP
        + "Lat: " + lat_txt + FIELD_SEP
        + "Lon: " + lon_txt + FIELD_SEP
        + "Temp: " + temp_txt + FIELD_SEP
        + "Fix: " + str(fix) + FIELD_SEP
        + "Seq: %d" % int(seq)
    ).encode("utf-8")
    return body + b"*" + ("%04X" % crc16(body)).encode("ascii")


def _field(part, label):
    """Return the text after 'Label: ' or raise ValueError."""
    start = label + ": "
    if not part.startswith(start):
        raise ValueError("expected field '%s'" % label)
    return part[len(start):]


def parse_payload(data):
    """
    Check and decode one datagram (bytes).  Returns a dict:
      status (text), distance_m (float|None), lat (float|None),
      lon (float|None), peak_temp_c (float|None), gps_fix (str), seq (int),
      crc (str)
    Raises ValueError on a bad CRC or a bad layout.
    """
    if not isinstance(data, (bytes, bytearray)):
        raise ValueError("payload must be bytes")
    data = bytes(data)

    star = data.rfind(b"*")
    if star < 0:
        raise ValueError("missing '*' before CRC")
    body = data[:star]
    crc_txt = data[star + 1:].rstrip(b"\r\n").decode("ascii", "replace")
    if not _CRC_RE.match(crc_txt):
        raise ValueError("CRC must be 4 uppercase hex digits, got %r" % crc_txt)
    expected = "%04X" % crc16(body)
    if crc_txt != expected:
        raise ValueError("CRC mismatch (got %s, computed %s)" % (crc_txt, expected))

    try:
        text = body.decode("utf-8")
    except UnicodeDecodeError:
        raise ValueError("payload is not valid UTF-8")
    if not text.startswith(PREFIX):
        raise ValueError("missing 'ALERT: Human found!' prefix")

    parts = text[len(PREFIX):].split(FIELD_SEP)
    if len(parts) != 7:
        raise ValueError("expected 7 fields, found %d" % len(parts))

    status = _field(parts[0], "Status")
    if not status:
        raise ValueError("empty status")

    dist_txt = _field(parts[1], "Distance")
    if dist_txt == "N/A":
        distance = None
    else:
        m = _DIST_RE.match(dist_txt)
        if not m:
            raise ValueError("bad distance %r" % dist_txt)
        distance = float(m.group(1))

    lat_txt = _field(parts[2], "Lat")
    lon_txt = _field(parts[3], "Lon")
    if lat_txt == "N/A" and lon_txt == "N/A":
        lat = lon = None
    else:
        if not (_LATLON_RE.match(lat_txt) and _LATLON_RE.match(lon_txt)):
            raise ValueError("bad lat/lon")
        lat, lon = float(lat_txt), float(lon_txt)
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError("lat/lon out of range")

    temp_txt = _field(parts[4], "Temp")
    if temp_txt == "N/A":
        temp = None
    else:
        m = _TEMP_RE.match(temp_txt)
        if not m:
            raise ValueError("bad temperature")
        temp = float(m.group(1))

    fix = _field(parts[5], "Fix")
    if not _FIX_RE.match(fix):
        raise ValueError("bad fix %r" % fix)

    seq_txt = _field(parts[6], "Seq")
    if not _SEQ_RE.match(seq_txt):
        raise ValueError("bad seq %r" % seq_txt)

    return {
        "status": status,
        "distance_m": distance,
        "lat": lat,
        "lon": lon,
        "peak_temp_c": temp,
        "gps_fix": fix,
        "seq": int(seq_txt),
        "crc": crc_txt,
    }


if __name__ == "__main__":
    # Tiny self-test:  python3 protocol.py
    pkt = build_payload(POSTURE_TEXT["UPRIGHT"], 4.2, 25.317645, 82.973914, 36.4, "LIVE", 1)
    print(pkt.decode("utf-8"))
    print(parse_payload(pkt))
