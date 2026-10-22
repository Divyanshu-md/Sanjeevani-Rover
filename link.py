#!/usr/bin/env python3
"""
link.py - the Ethernet/UDP link between rover and ground station.

  UdpTx(peer_ip, port).send(bytes) -> bool      (used by rover_node.py)
  UdpRx(port).poll()               -> dict|None (used by ground_station.py)

Only the standard library `socket` module is used, so it works on
Windows, Linux, macOS and Raspberry Pi.

UDP is "fire and forget": a lost packet is simply lost.  That is fine here
because the rover keeps re-sending alerts while the human is in view.
"""

import socket


class UdpTx:
    """Sends one datagram per call to a fixed peer."""

    def __init__(self, peer_ip, port):
        self.peer = (peer_ip, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)  # never block the program for long

    def send(self, data):
        """Send bytes.  Returns True if the OS accepted it, False on error
        (for example the Ethernet cable is unplugged)."""
        try:
            self.sock.sendto(data, self.peer)
            return True
        except OSError:
            return False

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class UdpRx:
    """Non-blocking UDP receiver."""

    MAX_DATAGRAM = 2048  # alerts are ~170 bytes; this is plenty

    def __init__(self, port, bind_ip="0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, int(port)))
        self.sock.setblocking(False)

    def poll(self):
        """Return None if nothing is waiting, otherwise a dict:
          {"data": bytes, "addr": (ip, port), "rssi_dbm": None, "snr_db": None}
        rssi/snr are always None because UDP has no radio signal numbers
        (the keys exist so the CSV log has the same columns as a radio link)."""
        try:
            data, addr = self.sock.recvfrom(self.MAX_DATAGRAM)
        except (BlockingIOError, InterruptedError):
            return None
        except ConnectionResetError:
            # Windows quirk: reports an old "port unreachable" here. Ignore.
            return None
        except OSError:
            return None
        return {"data": data, "addr": addr, "rssi_dbm": None, "snr_db": None}

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
