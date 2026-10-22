#!/usr/bin/env python3
"""
ground_station.py - runs on the laptop/PC (or a second Pi).  Plain Python 3,
standard library only (plus protocol.py and link.py in the same folder).

What it does:
  1. Listens for UDP datagrams on 0.0.0.0:<port>   (default 5005).
  2. Checks the CRC and decodes every field (protocol.parse_payload).
  3. Prints a big coloured alert (RED if the status contains "Unconscious",
     YELLOW otherwise) with a Google Maps link.  A warning is printed if
     the GPS fix is CACHED or NONE.
  4. Shows a live "listening..." status line.
  5. Appends EVERY packet (valid or rejected) to a CSV log, flushing and
     fsync-ing after each row so nothing is lost if the PC loses power.

Run:   python3 ground_station.py                (Windows: python ground_station.py)
       python3 ground_station.py --port 5005 --log rescue_telemetry_log.csv
Stop:  Ctrl+C

NOTE: the status text is a SUSPECTED status guessed from a camera image.
It is NOT a medical diagnosis.
"""

import argparse
import csv
import datetime
import os
import sys
import time

from link import UdpRx
from protocol import parse_payload

CSV_COLUMNS = [
    "rx_time_utc", "seq", "status", "distance_m", "lat", "lon",
    "peak_temp_c", "gps_fix", "rssi_dbm", "snr_db", "valid", "error", "raw",
]

# ANSI colour codes (enabled on Windows 10+ by the os.system("") trick below).
RED = "\033[1;97;41m"      # bold white on red background
YELLOW = "\033[1;30;43m"   # bold black on yellow background
WARN = "\033[1;93m"        # bold bright yellow text
DIM = "\033[2m"
RESET = "\033[0m"

STATUS_WIDTH = 100  # width used to blank out the live status line


def utc_now_text():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


class CsvLog:
    """Append-only CSV file; every row is flushed and fsync-ed."""

    def __init__(self, path):
        is_new = (not os.path.exists(path)) or os.path.getsize(path) == 0
        # newline="" is required by the csv module; utf-8 for the degree sign.
        self.fh = open(path, "a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fh)
        if is_new:
            self.writer.writerow(CSV_COLUMNS)
            self._sync()

    def _sync(self):
        self.fh.flush()
        try:
            os.fsync(self.fh.fileno())
        except OSError:
            pass  # some file systems do not support fsync; flush is enough

    def write(self, row_dict):
        self.writer.writerow([row_dict.get(c, "") for c in CSV_COLUMNS])
        self._sync()

    def close(self):
        self.fh.close()


def na(value, fmt):
    """Format a number, or 'N/A' when it is None."""
    return "N/A" if value is None else fmt % value


def clear_status_line():
    sys.stdout.write("\r" + " " * STATUS_WIDTH + "\r")


def print_alert(info, addr):
    """Big coloured console alert for one VALID packet."""
    colour = RED if "Unconscious" in info["status"] else YELLOW
    clear_status_line()
    bar = "=" * 70
    print(colour + bar + RESET)
    print(colour + "  HUMAN FOUND - SUSPECTED STATUS: %-40s" % info["status"] + RESET)
    print(colour + bar + RESET)
    print("  Time (UTC) : %s    From: %s    Seq: %d" % (utc_now_text(), addr[0], info["seq"]))
    print("  Distance   : %s  (approximate, single-camera estimate)" % na(info["distance_m"], "%.1f m"))
    if info["peak_temp_c"] is None:
        print(WARN + "  Body heat  : NOT MEASURED (no thermal sensor) - camera-only, UNVERIFIED" + RESET)
    else:
        print("  Body heat  : peak %.1f C (8x8 thermal sensor)" % info["peak_temp_c"])
    print("  GPS fix    : %s" % info["gps_fix"])
    if info["lat"] is not None:
        print("  Position   : %.6f, %.6f" % (info["lat"], info["lon"]))
        print("  Map        : https://www.google.com/maps?q=%.6f,%.6f" % (info["lat"], info["lon"]))
    else:
        print("  Position   : N/A (no GPS position known)")
    if info["gps_fix"] != "LIVE":
        print(WARN + "  WARNING: GPS fix is %s - the position may be OLD or missing!" % info["gps_fix"] + RESET)
    print("  (Posture/consciousness is a heuristic guess, not a medical diagnosis.)")
    print(colour + bar + RESET)
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Rescue rover ground station (UDP receiver).")
    ap.add_argument("--port", type=int, default=5005, help="UDP port to listen on (default 5005)")
    ap.add_argument("--log", default="rescue_telemetry_log.csv", help="CSV log file path")
    args = ap.parse_args()

    if os.name == "nt":
        os.system("")  # makes ANSI colours work in the Windows console

    try:
        rx = UdpRx(args.port)
    except OSError as e:
        print("Cannot listen on UDP port %d: %s" % (args.port, e))
        print("Is another copy already running, or is the port blocked by a firewall?")
        return 1
    log = CsvLog(args.log)

    valid_count = 0
    rejected_count = 0
    last_rx = None            # time.monotonic() of last packet
    last_status_draw = 0.0

    print("Ground station listening on UDP 0.0.0.0:%d   log: %s   (Ctrl+C to stop)" % (args.port, args.log))

    try:
        while True:
            pkt = rx.poll()
            now = time.monotonic()

            if pkt is None:
                # Nothing waiting: redraw the status line twice a second.
                if now - last_status_draw >= 0.5:
                    last_status_draw = now
                    since = "never" if last_rx is None else "%.0fs ago" % (now - last_rx)
                    line = "listening...  valid: %d  rejected: %d  last packet: %s" % (
                        valid_count, rejected_count, since)
                    sys.stdout.write("\r" + DIM + line.ljust(STATUS_WIDTH) + RESET)
                    sys.stdout.flush()
                time.sleep(0.05)
                continue

            last_rx = now
            raw_bytes = pkt["data"]
            raw_text = raw_bytes.decode("utf-8", "replace").replace("\r", "\\r").replace("\n", "\\n")
            row = {
                "rx_time_utc": utc_now_text(),
                "rssi_dbm": "" if pkt["rssi_dbm"] is None else pkt["rssi_dbm"],
                "snr_db": "" if pkt["snr_db"] is None else pkt["snr_db"],
                "raw": raw_text,
            }
            try:
                info = parse_payload(raw_bytes)
            except ValueError as e:
                rejected_count += 1
                row.update({"valid": 0, "error": str(e)})
                log.write(row)
                clear_status_line()
                print(WARN + "[%s] REJECTED packet from %s: %s" % (row["rx_time_utc"], pkt["addr"][0], e) + RESET)
                continue

            valid_count += 1
            row.update({
                "seq": info["seq"],
                "status": info["status"],
                "distance_m": na(info["distance_m"], "%.1f"),
                "lat": na(info["lat"], "%.6f"),
                "lon": na(info["lon"], "%.6f"),
                "peak_temp_c": na(info["peak_temp_c"], "%.1f"),
                "gps_fix": info["gps_fix"],
                "valid": 1,
                "error": "",
            })
            log.write(row)
            print_alert(info, pkt["addr"])

    except KeyboardInterrupt:
        print("\nStopped. valid=%d rejected=%d, log saved to %s" % (valid_count, rejected_count, args.log))
    finally:
        rx.close()
        log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
