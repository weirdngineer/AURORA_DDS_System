#!/usr/bin/env python3
# AURORA Insight Dashboard V4 - "Mission Control"
#
# Two tabs in one window:
#   1. MISSION CONTROL - big-number tiles, flight-phase banner, clean plots,
#      rocket GPS coordinates (live / last known / landing spot), QR code for
#      the phone tracker page,
#      event log, flight records and receiver radio control.
#   2. SENSOR VIEW     - the original 12-plot PyQtGraph dashboard + HUD/terminal,
#      every raw sensor channel for validation.
#
# Everything the previous dashboard did is still here:
#   - separate serial-reader process (multiprocessing.Queue)
#   - decimated plotting + throttled HUD
#   - HTTPS web tracker server + rocket_state.json
#   - CSV logging, non-GUI mode, flight summary on exit
#
# New:
#   - RSSI / SNR from the receiver's :NAV: block (shown + logged)
#   - Altitude above ground level (auto-zeroed on the pad)
#   - Send "FREQ <MHz>" / "FREQ?" to the receiver from the GUI
#   - Flight logs ON by default, one folder per session in flight_logs/:
#     telemetry CSV (same columns as before + extras) and a raw copy of every
#     line the receiver printed
#   - Flight report (PDF + HTML): key results, event timeline, plots with
#     event markers, recovery coordinates, pad sensor check, link stats.
#     Auto-generated 10 s after touchdown, or press FLIGHT REPORT.
#     Rebuild one from any saved CSV:  --report flight_logs/<session>/ReceiverLog_*.csv
#   - Google Earth export (KMZ): landing spot + search circle, 3D flight path,
#     ground track, pad / apogee / deployment markers, your position.
#     Written at touchdown (and by the EARTH button), plus a live file during
#     flight. Phones can download it from https://<pi>:8443/rocket_track.kmz
#   - --demo mode: simulated flight in the exact TX/RX packet format,
#     so you can test the dashboard on the bench with no hardware
#   - Only the visible tab is redrawn (saves CPU on the Pi)
#   - QR code that opens the phone tracker page (needs: pip install segno
#     or sudo apt install python3-qrcode)
#
# Usage:
#   python3 AURORA_Insight_Dashboard_V4.py                 # normal (serial)
#   python3 AURORA_Insight_Dashboard_V4.py --demo          # simulated flight
#   python3 AURORA_Insight_Dashboard_V4.py --fullscreen
#   python3 AURORA_Insight_Dashboard_V4.py --report flight_logs/<session>/ReceiverLog_<ts>.csv
#   python3 AURORA_Insight_Dashboard_V4.py --port /dev/ttyACM1 --no-gl
#
# Keys: F11 fullscreen, Ctrl+1 / Ctrl+2 switch tabs, Ctrl+Q quit.

import os
import serial
import time
import re
import threading
import numpy as np
import csv
from datetime import datetime, timezone
from collections import deque
import math
import textwrap
import warnings
import html
import signal
import argparse
import sys

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui

import http.server
import socketserver
import os
import socket  # for IP detection
import json

from multiprocessing import Process, Queue
import queue  # for queue.Empty in get_nowait()

warnings.filterwarnings("ignore", category=UserWarning)

# === USER CONFIGURABLE SETTINGS ===
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIME_WINDOW = 15.0          # seconds of data visible in Sensor View plots
SAVE_TO_FILE = True         # CSV + raw serial log (turn off with --no-log)
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_logs")
AUTO_REPORT = True          # write the flight report automatically after touchdown
REPORT_DELAY_S = 10.0       # ...this many seconds after touchdown (captures landing data)
FLIGHT_REC_MAX = 400000     # rows kept in memory for the report (~10 h at 11 Hz)
SEARCH_RADIUS_M = 30.0      # search circle drawn around the landing spot in Google Earth
LIVE_KML_EVERY_S = 3.0      # how often the live Google Earth file is rewritten
KML_MAX_POINTS = 2000       # flight-path points written to KML (decimated)
TRACKER_KMZ = "rocket_track.kmz"   # name of the Google Earth file served to phones
LIVE_PLOT = True
USE_OPENGL = True           # set False (or --no-gl) if plots misbehave
START_FULLSCREEN = False

# Performance settings
PLOT_UPDATE_INTERVAL = 60   # ms GUI refresh (tune: 50–100 for Pi)
MAX_PLOT_POINTS = 500       # max points kept in memory per trace
EXPECTED_PACKET_RATE = 10   # packets/s for connection quality
MAX_DRAW_POINTS = 400       # max points actually drawn per curve
HUD_UPDATE_EVERY = 3        # update HUD/terminal/tiles every N frames

# Mission Control settings
HISTORY_RATE_HZ = 5.0       # long-history sample rate (for 60 s / 5 min / ALL)
HISTORY_MAX_POINTS = 7200   # 24 min at 5 Hz
GROUND_REF_SAMPLES = 50     # rolling baro samples averaged on the pad for AGL zero
VS_SMOOTH_ALPHA = 0.25      # EMA for the vertical speed tile/plot
BATT_FULL_V = 4.20          # 1S LiPo
BATT_EMPTY_V = 3.30
BATT_WARN_V = 3.70
BATT_CRIT_V = 3.50
LINK_STALE_S = 0.6          # LED goes amber after this many seconds without a packet
LINK_LOST_S = 3.0           # LED goes red
DEFAULT_FREQ_MHZ = 915.0
DEMO_RATE_HZ = 11.0         # ~what the real LoRa link manages at SF7/BW500

# Terminal pane controls
TERMINAL_MAX_LINES = 12
TERMINAL_MAX_CHARS = 100

# Web server directory (where rocket_tracker.html lives)
WEB_DIR = "/home/weirdngineer/aurora_web"
CERT_DIR = "/home/weirdngineer/aurora_certs"

# TX order (23 values after timestamp)
DATA_LABELS = [
    'HG_Ax', 'HG_Ay', 'HG_Az',
    'LG_Ax', 'LG_Ay', 'LG_Az',
    'GyroX', 'GyroY', 'GyroZ',
    'Roll', 'Pitch', 'Yaw',
    'BaroAlt', 'Pressure', 'Temp',
    'Lat', 'Lon', 'GPSAlt',
    'Speed', 'MagX', 'MagY', 'MagZ',
    'Battery'
]

# Navigation data labels
NAV_LABELS = ['RX_LAT', 'RX_LON', 'RX_HDG', 'DIST', 'BEAR', 'REL_BEAR', 'TURN_DEG']
LINK_LABELS = ['RSSI', 'SNR']

# Indices
IDX_ROLL, IDX_PITCH, IDX_YAW = 9, 10, 11
IDX_LAT, IDX_LON, IDX_ALT, IDX_BATTERY = 15, 16, 17, 22
IDX_BARO_ALT, IDX_PRESSURE, IDX_TEMP = 12, 13, 14
IDX_HG_AX, IDX_HG_AY, IDX_HG_AZ = 0, 1, 2
IDX_LG_AX, IDX_LG_AY, IDX_LG_AZ = 3, 4, 5
IDX_GYRO_X, IDX_GYRO_Y, IDX_GYRO_Z = 6, 7, 8
IDX_SPEED = 18

# ===================== Theme =====================
# Brand: true black, one red accent, white numbers. Amber is used only for warnings.
C_BG = '#000000'
C_PANEL = '#0a0a0a'
C_PANEL_HI = '#151515'
C_BORDER = '#242424'
C_TEXT = '#f4f4f4'
C_MUTED = '#8e8e8e'
C_DIM = '#4a4a4a'
C_BRAND = '#ff1a2e'      # AURORA red
C_BRAND_DK = '#7a0a14'
C_WHITE = '#ffffff'
C_AMBER = '#ffb020'      # warnings only
C_RED = C_BRAND          # alarms use the brand red

# phase badge: (background, text)
PHASE_STYLE = {
    'Ground': ('#1c1c1c', C_TEXT),
    'Launch': (C_BRAND, C_WHITE),
    'Ascent': (C_BRAND, C_WHITE),
    'Descent': (C_WHITE, '#000000'),
}

# Light panel option (black ink on white)
SUN_BG = '#ffffff'
SUN_INK = '#000000'

GPS_STALE_S = 3.0        # rocket coordinates older than this are shown as "LAST KNOWN"

TRACKER_PORT = 8443
TRACKER_PAGE = "rocket_tracker.html"
IP_REFRESH_S = 10.0      # re-check the Pi's IP (Wi-Fi may change) this often

FONT_UI = "DejaVu Sans"
FONT_MONO = "DejaVu Sans Mono"


# ===================== Small helpers =====================

def fmt_dist(m):
    """Return (value_text, unit) for a distance in meters."""
    if m is None or not np.isfinite(m):
        return "--", "m"
    if m >= 1000:
        return f"{m / 1000:.2f}", "km"
    return f"{m:.0f}", "m"


def fmt_clock(seconds):
    seconds = max(0.0, seconds)
    m = int(seconds // 60)
    s = seconds - 60 * m
    return f"{m:02d}:{s:04.1f}"


def decimate(x, *ys, max_points=MAX_DRAW_POINTS):
    if len(x) <= max_points:
        return (x,) + ys
    step = max(1, len(x) // max_points)
    out = [x[::step]] + [y[::step] for y in ys]
    # always include the newest point so the trace reaches "now"
    if out[0][-1] != x[-1]:
        out = [np.append(o, src[-1]) for o, src in zip(out, (x,) + ys)]
    return tuple(out)


def qcolor(hex_color, alpha=255):
    c = QtGui.QColor(hex_color)
    c.setAlpha(alpha)
    return c


def compass8(bearing):
    names = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return names[int(((bearing % 360) + 22.5) // 45) % 8]


def fmt_dms(value, pos, neg):
    hemi = pos if value >= 0 else neg
    v = abs(value)
    d = int(v)
    m = int((v - d) * 60)
    sec = (v - d - m / 60.0) * 3600
    return f"{d}\u00B0{m:02d}'{sec:04.1f}\"{hemi}"


def fmt_age(seconds):
    if seconds < 60:
        return f"{seconds:.1f} s ago"
    m = int(seconds // 60)
    return f"{m} min {int(seconds - 60 * m)} s ago"


def make_qr_matrix(text):
    """QR modules as rows of bools, or None if no QR library is installed."""
    try:
        import segno
        qr = segno.make(text, error='m', micro=False)
        return [[bool(v) for v in row] for row in qr.matrix]
    except ImportError:
        pass
    try:
        import qrcode
        q = qrcode.QRCode(border=0, error_correction=qrcode.constants.ERROR_CORRECT_M)
        q.add_data(text)
        q.make(fit=True)
        return [[bool(v) for v in row] for row in q.get_matrix()]
    except ImportError:
        return None


def local_xy(lat, lon, lat0, lon0):
    """Equirectangular projection to meters east/north of (lat0, lon0)."""
    x = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    y = (lat - lat0) * 110540.0
    return x, y


# ===================== Serial Reader Process =====================

def serial_reader_process(port, baud, out_queue, cmd_queue=None):
    """
    Separate process that owns the serial port and pushes full lines
    into a multiprocessing.Queue for the GUI process to consume.
    Also forwards commands (e.g. "FREQ 915.000") from cmd_queue to the receiver.
    """
    import serial
    import time
    import queue as _queue

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.reset_input_buffer()
        print(f"[SERIAL PROC] Opened {port} at {baud} baud")
    except Exception as e:
        print(f"[SERIAL PROC] Failed to open {port}: {e}")
        out_queue.put((time.time(), f":DASH:ERR:Failed to open {port}: {e}"))
        return

    while True:
        try:
            if cmd_queue is not None:
                while True:
                    try:
                        cmd = cmd_queue.get_nowait()
                    except _queue.Empty:
                        break
                    ser.write((cmd.strip() + "\n").encode('ascii', errors='ignore'))
                    ser.flush()

            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                out_queue.put((time.time(), line))   # stamp on arrival
        except Exception as e:
            print(f"[SERIAL PROC] Error: {e}")
            time.sleep(0.1)


# ===================== Demo (simulated flight) Process =====================

def demo_reader_process(out_queue, cmd_queue=None, rate_hz=DEMO_RATE_HZ):
    """
    Generates lines in exactly the format the receiver prints:
      DATA:<tod>:<23 values>:NAV:RSSI:..:SNR:..:RX_LAT:..:RX_LON:..:RX_HDG:..:DIST:..
    Simulates: pad wait -> boost -> coast -> apogee/drogue -> main -> landing,
    with wind drift, GPS acquisition delays and some packet loss.
    """
    import random
    import time
    import math
    import queue as _queue

    def wrap360(a):
        a = math.fmod(a, 360.0)
        return a + 360.0 if a < 0 else a

    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dlat, dlon = p2 - p1, math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def bearing(lat1, lon1, lat2, lon2):
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(p2)
        x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
        return wrap360(math.degrees(math.atan2(y, x)))

    def direction(b):
        names = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        return names[int(((b + 22.5) % 360) // 45)]

    pad_lat, pad_lon = 32.990300, -106.975000
    ground_msl = 1400.0
    m_per_deg_lat = 110540.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(pad_lat))
    rx_lat = pad_lat - 260.0 / m_per_deg_lat
    rx_lon = pad_lon - 300.0 / m_per_deg_lon

    freq = DEFAULT_FREQ_MHZ
    K = 0.0011           # drag factor
    THRUST = 95.0        # m/s^2
    BURN = 1.9           # s
    PAD_TIME = 12.0      # s on the pad before launch

    state = 'PAD'
    alt = v = east = north = 0.0
    roll = 0.0
    pitch = 88.0
    yaw = 15.0
    t_boost = t_eject = t_main = None

    def put(line):
        out_queue.put((time.time(), line))

    put("Receiver w/ GPS NAV + RSSI/SNR starting (LoRa.h)... [DEMO]")
    put(":RXCFG:FREQ_MHZ:%.3f" % freq)

    dt = 1.0 / rate_hz
    t0 = time.time()
    next_t = t0
    while True:
        # ---- commands from the dashboard ----
        if cmd_queue is not None:
            while True:
                try:
                    cmd = cmd_queue.get_nowait().strip()
                except _queue.Empty:
                    break
                if cmd.startswith("FREQ?"):
                    put(":RXCFG:FREQ_MHZ:%.3f" % freq)
                elif cmd.startswith("FREQ"):
                    try:
                        mhz = float(cmd[4:].strip())
                    except ValueError:
                        mhz = -1
                    if 902.0 <= mhz <= 928.0:
                        freq = mhz
                        put(":RXCFG:FREQ_MHZ:%.3f" % freq)
                    else:
                        put(":RXCFG:ERR:BAD_FREQ")
                else:
                    put(":RXCFG:ERR:UNKNOWN_CMD")

        # ---- pacing ----
        now = time.time()
        if now < next_t:
            time.sleep(next_t - now)
        next_t += dt
        if next_t < time.time() - 1.0:
            next_t = time.time()
        el = time.time() - t0

        # ---- vertical physics ----
        if state == 'PAD' and el >= PAD_TIME:
            state, t_boost = 'BOOST', el
        a = 0.0
        if state == 'BOOST':
            a = THRUST - 9.81 - K * v * abs(v)
            if el - t_boost > BURN:
                state = 'COAST'
        elif state == 'COAST':
            a = -9.81 - K * v * abs(v)
            if v <= 0:
                state, t_eject = 'DROGUE', el
        elif state == 'DROGUE':
            a = (-22.0 - v) * 1.5
            if alt <= 250.0:
                state, t_main = 'MAIN', el
        elif state == 'MAIN':
            a = (-6.5 - v) * 2.0
            if alt <= 0.0:
                state = 'LANDED'
        if state in ('PAD', 'LANDED'):
            a = v = 0.0
            if state == 'PAD':
                alt = 0.0
        v += a * dt
        alt = max(0.0, alt + v * dt)

        # ---- horizontal drift ----
        if state in ('BOOST', 'COAST'):
            ve, vn = 0.05 * v, 0.09 * v
        elif state in ('DROGUE', 'MAIN'):
            ve, vn = 7.0, 1.8
        else:
            ve = vn = 0.0
        east += ve * dt
        north += vn * dt

        # ---- sensed specific force along rocket "up" axis ----
        if state in ('PAD', 'LANDED'):
            f_up = 9.81
        else:
            f_up = a + 9.81
        if state == 'DROGUE' and el - t_eject < 0.25:
            f_up += random.uniform(40, 90)
        if state == 'MAIN' and el - t_main < 0.25:
            f_up += random.uniform(30, 60)
        n = random.gauss
        hg = (f_up + n(0, 1.2), n(0, 1.2), n(0, 1.2))
        lg_lim = 16 * 9.81
        lg = (max(-lg_lim, min(lg_lim, f_up + n(0, 0.12))), n(0, 0.12), n(0, 0.12))

        # ---- rotation ----
        if state == 'BOOST':
            gz = 260.0 * (el - t_boost) / BURN
        elif state == 'COAST':
            gz = 260.0 * math.exp(-(el - t_boost - BURN) / 4.0)
        elif state in ('DROGUE', 'MAIN'):
            gz = 35 * math.sin(el * 1.7)
        else:
            gz = 0.0
        gx = n(0, 1.0) + (25 * math.sin(el * 2.3) if state in ('DROGUE', 'MAIN') else 0)
        gy = n(0, 1.0) + (20 * math.cos(el * 1.9) if state in ('DROGUE', 'MAIN') else 0)
        roll = (roll + gz * dt + 180.0) % 360.0 - 180.0
        if state == 'COAST':
            pitch = max(55.0, pitch - 1.2 * dt)
        elif state in ('DROGUE', 'MAIN'):
            pitch = 10 + 25 * math.sin(el * 0.9)
        elif state == 'LANDED':
            pitch = 3.0
        yaw = (yaw + n(0, 0.3) + 180.0) % 360.0 - 180.0

        # ---- environment ----
        baro_alt = ground_msl + alt + n(0, 0.05)
        pressure = 101325.0 * (1.0 - baro_alt / 44330.0) ** 5.255
        temp = 27.0 - 0.0065 * alt + n(0, 0.05)
        battery = 4.12 - el * 0.0003 + n(0, 0.004)

        # ---- GPS (rocket) ----
        if el > 3.0:
            r_lat = pad_lat + north / m_per_deg_lat
            r_lon = pad_lon + east / m_per_deg_lon
            gps_alt = ground_msl + alt + n(0, 1.5)
            speed = math.sqrt(v * v + ve * ve + vn * vn) * 3.6
        else:
            r_lat = r_lon = 0.0
            gps_alt = speed = -1.0

        tod = int(time.time() % 86400)
        pkt = ("DATA:%d:"
               "%.1f:%.1f:%.1f:"
               "%.1f:%.1f:%.1f:"
               "%.1f:%.1f:%.1f:"
               "%.1f:%.1f:%.1f:"
               "%.1f:%.0f:%.1f:"
               "%.6f:%.6f:%.1f:%.1f:"
               "%.1f:%.1f:%.1f:"
               "%.2f") % (
            tod, hg[0], hg[1], hg[2], lg[0], lg[1], lg[2], gx, gy, gz,
            roll, pitch, yaw, baro_alt, pressure, temp,
            r_lat, r_lon, gps_alt, speed,
            22.0 + n(0, 0.4), -5.0 + n(0, 0.4), 40.0 + n(0, 0.4), battery)

        # ---- packet loss (worse right after ejection) ----
        loss = 0.02
        if t_eject is not None and 0 <= el - t_eject < 1.5:
            loss = 0.35
        if random.random() < loss:
            continue

        # ---- receiver NAV block ----
        d_ground = haversine(rx_lat, rx_lon, r_lat, r_lon) if el > 3.0 else 300.0
        d3 = math.sqrt(d_ground ** 2 + alt ** 2)
        rssi = int(-38 - 24 * math.log10(max(d3, 5.0) / 5.0) + n(0, 1.5))
        snr = max(-14.0, min(12.0, 11.0 - max(0, -rssi - 75) * 0.45 + n(0, 0.6)))
        rx_hdg = wrap360(35 + 25 * math.sin(el / 9.0))
        nav = ":NAV:RSSI:%d:SNR:%.1f" % (rssi, snr)
        rx_fix = el > 5.0
        if rx_fix:
            nav += ":RX_LAT:%.6f:RX_LON:%.6f" % (rx_lat, rx_lon)
        else:
            nav += ":RX_LAT:NO_FIX:RX_LON:NO_FIX"
        nav += ":RX_HDG:%.1f" % rx_hdg
        if rx_fix:
            if el > 3.0:
                dist = haversine(rx_lat, rx_lon, r_lat, r_lon)
                brg = bearing(rx_lat, rx_lon, r_lat, r_lon)
                rel = wrap360(brg - rx_hdg)
                nav += ":DIST:%.1f:BEAR:%.1f:DIR:%s:REL_BEAR:%.1f" % (dist, brg, direction(brg), rel)
                if rel > 180.0:
                    nav += ":TURN:LEFT:TURN_DEG:%.1f" % (360.0 - rel)
                elif rel > 10.0:
                    nav += ":TURN:RIGHT:TURN_DEG:%.1f" % rel
                else:
                    nav += ":TURN:STRAIGHT:TURN_DEG:0.0"
            else:
                nav += (":DIST:NO_ROCKET_GPS:BEAR:NO_ROCKET_GPS:DIR:NO_ROCKET_GPS:REL_BEAR:NO_ROCKET_GPS"
                        ":TURN:NO_ROCKET_GPS:TURN_DEG:NO_ROCKET_GPS")
        else:
            nav += (":DIST:NO_RX_GPS:BEAR:NO_RX_GPS:DIR:NO_RX_GPS:REL_BEAR:NO_RX_GPS"
                    ":TURN:NO_RX_GPS:TURN_DEG:NO_RX_GPS")

        put(pkt + nav)


# ===================== Web Server Thread =====================

class WebServerThread(threading.Thread):
    """
    Simple HTTPS server in a background thread.
    Serves static files from `directory` on the given port.
    Expects cert/key in CERT_DIR/cert.pem and key.pem.
    """
    def __init__(self, directory, port=8443):
        super().__init__(daemon=True)
        self.directory = directory
        self.port = port
        self.status = "starting"   # starting / running / error
        self.error = ""

    def _fail(self, msg):
        self.status, self.error = "error", msg
        print(f"[WEB] ERROR: {msg}")

    def run(self):
        import ssl
        import functools

        if not os.path.isdir(self.directory):
            self._fail(f"Directory {self.directory} not found. Server not started.")
            return

        # Serve from the directory without chdir'ing the whole process
        handler = functools.partial(http.server.SimpleHTTPRequestHandler,
                                    directory=self.directory)

        class ReusableTCPServer(socketserver.TCPServer):
            allow_reuse_address = True

        cert_file = os.path.join(CERT_DIR, "cert.pem")
        key_file = os.path.join(CERT_DIR, "key.pem")

        try:
            with ReusableTCPServer(("", self.port), handler) as httpd:
                try:
                    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
                    ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)
                    httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
                except Exception as e:
                    self._fail(f"SSL setup failed (check {CERT_DIR}): {e}")
                    return

                print(f"[WEB] Serving HTTPS tracker at https://0.0.0.0:{self.port}/rocket_tracker.html")
                print("      From directory:", self.directory)
                self.status = "running"
                try:
                    httpd.serve_forever()
                except KeyboardInterrupt:
                    pass
                finally:
                    httpd.server_close()
        except Exception as e:
            self._fail(f"Could not start HTTPS server on port {self.port}: {e}")


# ===================== Custom widgets (Mission Control) =====================

class AuroraTitle(QtWidgets.QWidget):
    """'AURORA INSIGHT' with a slowly drifting aurora gradient."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.phase = 0.0
        self.setMinimumSize(290, 46)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Preferred)

    def tick(self):
        self.phase = (self.phase + 0.004) % 1.0
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        period = w * 3.0
        x0 = -self.phase * period
        g = QtGui.QLinearGradient(x0, 0, x0 + period, 0)
        g.setSpread(QtGui.QGradient.Spread.RepeatSpread)
        g.setColorAt(0.0, QtGui.QColor(C_WHITE))
        g.setColorAt(0.44, QtGui.QColor(C_WHITE))
        g.setColorAt(0.50, QtGui.QColor(C_BRAND))
        g.setColorAt(0.56, QtGui.QColor(C_WHITE))
        g.setColorAt(1.0, QtGui.QColor(C_WHITE))

        f = QtGui.QFont(FONT_UI)
        f.setPixelSize(int(h * 0.62))
        f.setWeight(QtGui.QFont.Weight.Black)
        f.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 3)
        path = QtGui.QPainterPath()
        baseline = h * 0.76
        path.addText(0, baseline, f, "AURORA")
        p.fillPath(path, QtGui.QBrush(g))
        aw = QtGui.QFontMetrics(f).horizontalAdvance("AURORA")

        f2 = QtGui.QFont(FONT_UI)
        f2.setPixelSize(int(h * 0.30))
        f2.setWeight(QtGui.QFont.Weight.DemiBold)
        f2.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 5)
        p.setFont(f2)
        p.setPen(QtGui.QColor(C_BRAND))
        p.drawText(QtCore.QPointF(aw + 14, baseline), "INSIGHT")
        p.end()


class StatusLed(QtWidgets.QWidget):
    def __init__(self, size=16, parent=None):
        super().__init__(parent)
        self._color = C_DIM
        self._on = True
        self.setFixedSize(size + 8, size + 8)

    def set_state(self, color, on=True):
        if color != self._color or on != self._on:
            self._color, self._on = color, on
            self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        c = QtCore.QPointF(self.width() / 2, self.height() / 2)
        r = (self.width() - 8) / 2
        col = self._color if self._on else C_DIM
        glow = QtGui.QRadialGradient(c, r + 4)
        glow.setColorAt(0.0, qcolor(col, 160))
        glow.setColorAt(1.0, qcolor(col, 0))
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        if self._on:
            p.setBrush(QtGui.QBrush(glow))
            p.drawEllipse(c, r + 4, r + 4)
        p.setBrush(QtGui.QColor(col))
        p.drawEllipse(c, r * 0.7, r * 0.7)
        p.end()


class MetricTile(QtWidgets.QFrame):
    """Big-number tile: TITLE / huge value + unit / small subtitle."""
    def __init__(self, title, unit, accent, parent=None):
        super().__init__(parent)
        self.setObjectName("tile")
        self._accent = None
        self._value_color = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 8, 14, 10)
        lay.setSpacing(0)

        self.title = QtWidgets.QLabel(title.upper())
        tf = QtGui.QFont(FONT_UI)
        tf.setPixelSize(12)
        tf.setWeight(QtGui.QFont.Weight.Bold)
        tf.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 2)
        self.title.setFont(tf)
        lay.addWidget(self.title)

        row = QtWidgets.QHBoxLayout()
        row.setSpacing(6)
        self.value = QtWidgets.QLabel("--")
        self.unit = QtWidgets.QLabel(unit)
        self.value.setStyleSheet(f"color:{C_TEXT};")
        self.unit.setStyleSheet(f"color:{C_MUTED};")
        row.addWidget(self.value, 0, QtCore.Qt.AlignmentFlag.AlignBottom)
        row.addWidget(self.unit, 0, QtCore.Qt.AlignmentFlag.AlignBottom)
        row.addStretch(1)
        lay.addLayout(row, 1)

        self.sub = QtWidgets.QLabel("")
        sf = QtGui.QFont(FONT_UI)
        sf.setPixelSize(13)
        self.sub.setFont(sf)
        self.sub.setStyleSheet(f"color:{C_MUTED};")
        lay.addWidget(self.sub)

        self.set_accent(accent)
        self.setMinimumHeight(118)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Preferred)

    def set_accent(self, color):
        if color == self._accent:
            return
        self._accent = color
        self.setStyleSheet(
            f"QFrame#tile {{ background:{C_PANEL}; border:1px solid {C_BORDER};"
            f" border-top:3px solid {color}; border-radius:6px; }}"
        )
        self.title.setStyleSheet(f"color:{C_MUTED}; border:none;")

    def set(self, value, unit=None, sub=None, color=None):
        self.value.setText(value)
        if unit is not None and unit != self.unit.text():
            self.unit.setText(unit)
        if sub is not None:
            self.sub.setText(sub)
        color = color or C_TEXT
        if color != self._value_color:
            self._value_color = color
            self.value.setStyleSheet(f"color:{color}; border:none;")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        px = int(max(22, min(self.height() * 0.40, self.width() * 0.19)))
        vf = QtGui.QFont(FONT_MONO)
        vf.setPixelSize(px)
        vf.setWeight(QtGui.QFont.Weight.Bold)
        self.value.setFont(vf)
        uf = QtGui.QFont(FONT_UI)
        uf.setPixelSize(max(11, int(px * 0.36)))
        uf.setWeight(QtGui.QFont.Weight.DemiBold)
        self.unit.setFont(uf)


class Panel(QtWidgets.QFrame):
    """Rounded dark panel with a small caps title."""
    def __init__(self, title=None, parent=None, margins=(10, 8, 10, 10), light=False):
        super().__init__(parent)
        self.setObjectName("panel")
        self.title_lbl = None
        self.body = QtWidgets.QVBoxLayout(self)
        self.body.setContentsMargins(*margins)
        self.body.setSpacing(6)
        if title:
            self.header = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel(title.upper())
            f = QtGui.QFont(FONT_UI)
            f.setPixelSize(12)
            f.setWeight(QtGui.QFont.Weight.Bold)
            f.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 2)
            lbl.setFont(f)
            self.title_lbl = lbl
            self.header.addWidget(lbl)
            self.header.addStretch(1)
            self.body.addLayout(self.header)
        self.set_light(light)

    def set_light(self, light):
        bg, border, title_col = (SUN_BG, SUN_INK, SUN_INK) if light else (C_PANEL, C_BORDER, C_MUTED)
        self.setStyleSheet(
            f"QFrame#panel {{ background:{bg}; border:{2 if light else 1}px solid {border}; border-radius:6px; }}"
        )
        if self.title_lbl is not None:
            self.title_lbl.setStyleSheet(f"color:{title_col}; border:none;")


class QrCodeWidget(QtWidgets.QWidget):
    """Black-on-white QR code with a quiet zone. Click to enlarge."""
    clicked = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.text = None
        self.matrix = None
        self.dimmed = False
        self.setMinimumSize(150, 150)
        self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.PointingHandCursor))
        self.setToolTip("Click to show a big QR code")

    def set_text(self, text, dimmed=False):
        if text == self.text and dimmed == self.dimmed:
            return
        if text != self.text:
            self.matrix = make_qr_matrix(text) if text else None
        self.text, self.dimmed = text, dimmed
        self.update()

    def mousePressEvent(self, event):
        self.clicked.emit()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        side = min(self.width(), self.height())
        x0 = (self.width() - side) // 2
        y0 = (self.height() - side) // 2
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor("#ffffff"))
        p.drawRect(x0, y0, side, side)
        if self.matrix is None:
            p.setPen(QtGui.QColor("#000000"))
            f = QtGui.QFont(FONT_UI)
            f.setPixelSize(max(11, side // 14))
            p.setFont(f)
            msg = ("No QR library found.\n\nOn the Pi run:\npip install segno\n"
                   "or\nsudo apt install python3-qrcode") if self.text else "No tracker URL"
            p.drawText(QtCore.QRect(x0 + 8, y0 + 8, side - 16, side - 16),
                       int(QtCore.Qt.AlignmentFlag.AlignCenter) | int(QtCore.Qt.TextFlag.TextWordWrap), msg)
            p.end()
            return
        n = len(self.matrix)
        quiet = 4
        cell = max(1, side // (n + 2 * quiet))   # whole pixels -> crisp edges, easy to scan
        size = cell * n
        ox = x0 + (side - size) // 2
        oy = y0 + (side - size) // 2
        p.setBrush(QtGui.QColor("#b8b8b8" if self.dimmed else "#000000"))
        for r, row in enumerate(self.matrix):
            for c, on in enumerate(row):
                if on:
                    p.drawRect(ox + c * cell, oy + r * cell, cell, cell)
        p.end()


# ===================== Flight report =====================

# columns of TelemetryReceiver.flight_rec
FR_T, FR_AGL, FR_VS, FR_G, FR_SPD, FR_RSSI, FR_SNR, FR_LAT, FR_LON, FR_GALT, FR_BARO, FR_TEMP, FR_BATT, FR_DIST = range(14)

M_TO_FT = 3.28084


def _qenum(obj, scoped, name):
    """PyQt5/PyQt6-compatible enum lookup."""
    holder = getattr(obj, scoped, None)
    if holder is not None and hasattr(holder, name):
        return getattr(holder, name)
    return getattr(obj, name)


class FlightReportBuilder:
    """Builds a printable PDF + a single-file HTML report from a TelemetryReceiver."""

    def __init__(self, rx, out_dir):
        self.rx = rx
        self.out_dir = out_dir

    # ---------- helpers ----------
    @staticmethod
    def _median(a):
        a = a[np.isfinite(a)]
        return float(np.median(a)) if a.size else float('nan')

    @staticmethod
    def _f(v, fmt="{:.1f}", unit="", dash="&mdash;"):
        if v is None or (isinstance(v, float) and not np.isfinite(v)):
            return dash
        return fmt.format(v) + unit

    def _plot(self, x, series, ylabel, events, x_range, label_events=False, zero=False, height=320):
        """Render one white, print-friendly plot to a QImage."""
        import pyqtgraph.exporters  # noqa: F401
        pw = pg.PlotWidget(background='w')
        pw.resize(1100, height)
        pi = pw.getPlotItem()
        pi.setGeometry(QtCore.QRectF(0, 0, 1100, height))   # widget is never shown, so size the plot directly
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.hideButtons()
        f = QtGui.QFont(FONT_UI)
        f.setPixelSize(13)
        for name in ('left', 'bottom'):
            ax = pi.getAxis(name)
            ax.setPen(pg.mkPen('#000000', width=1))
            ax.setTextPen(pg.mkPen('#000000'))
            ax.setStyle(tickFont=f)
        pi.setLabel('bottom', "Time from liftoff (s)", color='#000000')
        pi.setLabel('left', ylabel, color='#000000')
        if len(series) > 1:
            try:
                pi.addLegend(offset=(-10, 10), labelTextColor='#000000', labelTextSize='12pt',
                             brush=pg.mkBrush(255, 255, 255, 230), pen=pg.mkPen('#999999'))
            except TypeError:
                pi.addLegend(offset=(-10, 10))
        if zero:
            pi.addItem(pg.InfiniteLine(angle=0, pos=0, pen=pg.mkPen('#888888', width=1)))
        for y, color, width, dashed, name in series:
            pen = pg.mkPen(color, width=width * 2,   # exported at 2x; cosmetic pens don't scale
                           style=QtCore.Qt.PenStyle.DashLine if dashed else QtCore.Qt.PenStyle.SolidLine)
            pi.plot(x, y, pen=pen, name=name, connect='finite', antialias=True)
        span = (x_range[1] - x_range[0]) or 1.0
        for i, (tt, name) in enumerate(events):
            kw = {}
            if label_events:
                right_side = (tt - x_range[0]) / span > 0.8      # keep labels on the page
                anc = (1.05, 0.5) if right_side else (-0.05, 0.5)
                kw = dict(label=name, labelOpts={'position': 0.93 - 0.12 * (i % 4), 'color': '#000000',
                                                 'fill': pg.mkBrush(255, 255, 255, 230),
                                                 'anchors': [anc, anc]})
            pi.addItem(pg.InfiniteLine(pos=tt, angle=90, movable=False,
                                       pen=pg.mkPen(C_BRAND, width=2, style=QtCore.Qt.PenStyle.DashLine), **kw))
        pi.setXRange(*x_range, padding=0.01)
        pi.enableAutoRange(y=True)
        QtWidgets.QApplication.processEvents()
        exp = pg.exporters.ImageExporter(pi)
        exp.parameters()['width'] = 2200
        img = exp.export(toBytes=True)
        pw.close()
        pw.deleteLater()
        return img

    @staticmethod
    def _png_bytes(img):
        ba = QtCore.QByteArray()
        buf = QtCore.QBuffer(ba)
        buf.open(_qenum(QtCore.QIODevice, 'OpenModeFlag', 'WriteOnly'))
        img.save(buf, "PNG")
        buf.close()
        return bytes(ba)

    # ---------- analysis ----------
    def analyse(self):
        rx = self.rx
        with rx.data_lock:
            if len(rx.flight_rec) < 5:
                raise ValueError("not enough telemetry recorded yet")
            d = np.array(rx.flight_rec, dtype=float)
            snap = dict(
                launch=rx.launch_time, touchdown=rx.touchdown_time, ground_alt=rx.ground_alt,
                origin=rx.track_origin, landing=rx.landing_pos, last_fix=rx.last_rocket_fix,
                packets=rx.packet_count, drops=rx.connection_drops, max_gap=rx.max_packet_gap,
                bad=rx.bad_packets, freq=rx.rx_freq_mhz, csv=rx.log_filename, raw=rx.raw_log_filename,
                start_wall=rx.start_wall_time, rx_pos=(rx.receiver_positions[-1] if rx.receiver_positions else None),
                sensor_used=list(rx.g_sensor_used),
            )
        t = d[:, FR_T]
        launch = snap['launch']
        flew = launch is not None
        t0 = launch if flew else t[0]
        td = snap['touchdown']
        start = max(t[0], t0 - 5.0) if flew else t[0]
        end = min(t[-1], td + 10.0) if td is not None else t[-1]
        m = (t >= start) & (t <= end)
        w = d[m]
        tp = w[:, FR_T] - t0
        A = dict(d=d, w=w, tp=tp, t0=t0, flew=flew, snap=snap, x_range=(tp[0], tp[-1]))

        ev = []
        if flew:
            ev.append(("Liftoff", 0.0))
            after = (tp > 0.3) & (tp < 30)
            bo = np.where(after & (w[:, FR_G] < 1.0))[0]
            if bo.size:
                ev.append(("Burnout", float(tp[bo[0]])))
            asc = tp >= 0
            i_ap = int(np.nanargmax(np.where(asc, w[:, FR_AGL], -np.inf)))
            A['i_ap'] = i_ap
            ev.append(("Apogee", float(tp[i_ap])))
            # deployment shocks: G spikes > 3 g after apogee, clustered
            post = np.where((tp >= tp[i_ap] - 1.0) & (w[:, FR_G] > 3.0))[0]
            deps, last = [], -1e9
            for i in post:
                if tp[i] - last > 1.5:
                    deps.append(i)
                elif w[i, FR_G] > w[deps[-1], FR_G]:
                    deps[-1] = i
                last = tp[i]
            names = (["Drogue deploy", "Main deploy"] if len(deps) == 2 else
                     ["Deployment"] if len(deps) == 1 else
                     [f"Deployment {k + 1}" for k in range(len(deps))])
            A['deps'] = [(n, float(tp[i]), float(w[i, FR_G])) for n, i in zip(names, deps)]
            for n, tt, _ in A['deps']:
                ev.append((n, tt))
            if td is not None:
                ev.append(("Touchdown", float(td - t0)))
        ev.sort(key=lambda e: e[1])
        A['events'] = ev
        return A

    # ---------- document ----------
    def build(self):
        A = self.analyse()
        w, tp, snap = A['w'], A['tp'], A['snap']
        d = A['d']
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = datetime.fromtimestamp(snap['start_wall'] or time.time())
        base = os.path.join(self.out_dir, f"FlightReport_{stamp.strftime('%Y%m%d-%H%M%S')}")
        DASH = "—"

        def F(v, fmt="{:.1f}", unit=""):
            if v is None or (isinstance(v, float) and not np.isfinite(v)):
                return DASH
            return fmt.format(v) + unit

        def at(tt):
            return w[int(np.argmin(np.abs(tp - tt)))]

        # ----- key numbers -----
        agl = w[:, FR_AGL]
        vs = w[:, FR_VS]
        spd = w[:, FR_SPD]
        i_ap = A.get('i_ap', int(np.nanargmax(agl)))
        apogee = float(agl[i_ap])
        t_ap = float(tp[i_ap]) if A['flew'] else None
        i_g = int(np.nanargmax(w[:, FR_G]))
        max_g, t_g = float(w[i_g, FR_G]), float(tp[i_g])
        max_spd = mach = None
        if np.isfinite(spd).any():
            i_s = int(np.nanargmax(spd))
            max_spd = float(spd[i_s])
            temp = w[i_s, FR_TEMP]
            a_snd = 331.3 * math.sqrt(1 + (temp if np.isfinite(temp) else 15.0) / 273.15)
            mach = max_spd / 3.6 / a_snd
        asc = (tp >= 0) & (tp <= (t_ap if t_ap is not None else tp[-1]))
        max_vs = float(np.nanmax(vs[asc])) if asc.any() else None
        evd = dict(A['events'])
        burn = evd.get("Burnout")
        flight_time = evd.get("Touchdown")

        def med_descent(a, b_):
            mm = (tp > a) & (tp < b_)
            return -self._median(vs[mm]) if mm.sum() >= 3 else None
        descent = []
        if A['flew'] and t_ap is not None:
            end_d = (flight_time - 1.0) if flight_time is not None else tp[-1]
            deps = A.get('deps', [])
            if len(deps) >= 2:
                descent = [("Under drogue", med_descent(deps[0][1] + 1.0, deps[1][1] - 1.0)),
                           ("Under main", med_descent(deps[1][1] + 1.5, end_d))]
            else:
                descent = [("Avg descent", med_descent(t_ap + 2.0, end_d)),
                           ("Final descent", med_descent(end_d - 8.0, end_d))]
        while len(descent) < 2:
            descent.append(("Descent", None))

        tiles = [
            ("APOGEE (AGL)", f"{apogee:,.0f} m",
             f"{apogee * M_TO_FT:,.0f} ft" + (f"  ·  T+{t_ap:.1f} s" if t_ap is not None else "")),
            ("MAX ACCELERATION", f"{max_g:.1f} g", f"T+{t_g:.1f} s"),
            ("MAX SPEED (GPS)", F(max_spd, "{:.0f}", " km/h"),
             f"Mach {mach:.2f}  ·  {max_spd / 3.6:.0f} m/s" if max_spd else ""),
            ("FLIGHT TIME", F(flight_time, "{:.1f}", " s"), "liftoff to touchdown"),
            ("MAX ASCENT RATE", F(max_vs, "{:.0f}", " m/s"), "barometric"),
            ("BURN TIME", F(burn, "{:.1f}", " s"), "liftoff to burnout"),
            (descent[0][0].upper(), F(descent[0][1], "{:.1f}", " m/s"), "median rate"),
            (descent[1][0].upper(), F(descent[1][1], "{:.1f}", " m/s"), "median rate"),
        ]

        events = []
        for n, tt in A['events']:
            r = at(tt)
            alt_r = round(float(r[FR_AGL])) + 0   # avoid "-0 m"
            events.append((n, f"{tt:+.1f}", f"{alt_r:,.0f} m", f"{r[FR_VS]:+.1f} m/s", f"{r[FR_G]:.1f} g"))

        # ----- recovery -----
        origin, landing, last_fix = snap['origin'], snap['landing'], snap['last_fix']
        rec = []   # (key, value, url)

        def maps(lat, lon):
            return f"https://maps.google.com/?q={lat:.6f},{lon:.6f}"
        if origin:
            rec.append(("Launch site", f"{origin[0]:.6f}, {origin[1]:.6f}", maps(*origin)))
        pos = landing or last_fix
        if pos:
            rec.append(("Landing position" if landing else "Last known position",
                        f"{pos[0]:.6f}, {pos[1]:.6f}", maps(pos[0], pos[1])))
            if origin:
                x, y = local_xy(pos[0], pos[1], *origin)
                dv, du = fmt_dist(math.hypot(x, y))
                rec.append(("Drift from pad", f"{dv} {du} {compass8(math.degrees(math.atan2(x, y)))}", None))
        if np.isfinite(d[:, FR_DIST]).any():
            dv, du = fmt_dist(float(np.nanmax(d[:, FR_DIST])))
            rec.append(("Max range from receiver", f"{dv} {du}", None))
        if snap['rx_pos']:
            rec.append(("Receiver position", f"{snap['rx_pos'][0]:.6f}, {snap['rx_pos'][1]:.6f}", None))
        if snap['ground_alt'] is not None:
            rec.append(("Pad elevation (baro)",
                        f"{snap['ground_alt']:,.0f} m MSL ({snap['ground_alt'] * M_TO_FT:,.0f} ft)", None))

        # ----- pad sensor check (last 10 s before liftoff) -----
        checks = []   # (key, value, colour)
        if A['flew']:
            pm = (d[:, FR_T] >= A['t0'] - 10.0) & (d[:, FR_T] <= A['t0'] - 0.5)
            if pm.sum() >= 5:
                g_mean, g_std = float(np.mean(d[pm, FR_G])), float(np.std(d[pm, FR_G]))
                ok = abs(g_mean - 1.0) < 0.05
                checks += [
                    ("Accel at rest", f"{g_mean:.3f} g ± {g_std:.3f}  " + ("OK" if ok else "CHECK (expect 1.000)"),
                     '#0a7d00' if ok else C_BRAND),
                    ("Baro noise on pad", f"±{float(np.nanstd(d[pm, FR_AGL])):.2f} m (1σ)", None),
                    ("Battery at launch", f"{float(np.nanmean(d[pm, FR_BATT])):.2f} V", None),
                    ("Temp at launch", f"{float(np.nanmean(d[pm, FR_TEMP])):.1f} °C", None),
                    ("Signal on pad", F(self._median(d[pm, FR_RSSI]), "{:.0f}", " dBm"), None),
                ]
        if "HG" in snap['sensor_used']:
            checks.append(("High-g sensor", "used (low-g saturated)", None))

        # ----- link -----
        dur = d[-1, FR_T] - d[0, FR_T]
        rssi_all, snr_all = d[:, FR_RSSI], d[:, FR_SNR]
        link = [
            ("Packets received", f"{snap['packets']:,}" + (f" ({snap['bad']} unreadable)" if snap['bad'] else "")),
            ("Avg packet rate", F(snap['packets'] / dur if dur > 0 else None, "{:.1f}", " /s")),
            ("Gaps > 2 intervals", f"{snap['drops']} (longest {snap['max_gap']:.2f} s)"),
            ("RSSI median / worst", f"{F(self._median(rssi_all), '{:.0f}')} / "
                                    f"{F(float(np.nanmin(rssi_all)) if np.isfinite(rssi_all).any() else None, '{:.0f}')} dBm"),
            ("Worst SNR", F(float(np.nanmin(snr_all)) if np.isfinite(snr_all).any() else None, "{:+.1f}", " dB")),
            ("Receiver frequency", F(snap['freq'], "{:.3f}", " MHz")),
        ]

        # ----- plots -----
        evs = [(tt, n) for n, tt in A['events']]
        g50 = d[:min(len(d), 50), FR_GALT]
        gps_agl = w[:, FR_GALT] - (np.nanmedian(g50) if np.isfinite(g50).any() else 0.0)
        xr = A['x_range']
        plots = [
            ("Altitude above pad", self._plot(
                tp, [(agl, C_BRAND, 2.5, False, "Barometric"), (gps_agl, '#555555', 1.5, True, "GPS")],
                "Altitude AGL (m)", evs, xr, label_events=True, height=420)),
            ("Vertical velocity (barometric)", self._plot(
                tp, [(vs, '#000000', 2, False, "Vertical velocity")], "m/s", evs, xr, zero=True)),
            ("Acceleration magnitude", self._plot(
                tp, [(w[:, FR_G], C_BRAND, 2, False, "G")], "g", evs, xr)),
            ("GPS speed", self._plot(
                tp, [(spd, '#000000', 2, False, "Speed")], "km/h", evs, xr)),
            ("Radio link", self._plot(
                tp, [(w[:, FR_RSSI], C_BRAND, 1.5, False, "RSSI (dBm)"),
                     (w[:, FR_SNR], '#000000', 1.5, False, "SNR (dB)")], "dBm / dB", evs, xr)),
        ]

        files = [f for f in (snap['csv'], snap['raw']) if f]
        notes = ("Times are relative to detected liftoff. Altitude is barometric above the pad; speed is "
                 "GPS-derived; vertical velocity is the smoothed barometric rate. Events are detected "
                 "automatically from the telemetry (burnout: acceleration < 1 g; deployments: shocks > 3 g "
                 "after apogee).")
        R = dict(title="FLIGHT REPORT" if A['flew'] else "SESSION REPORT",
                 subtitle=None if A['flew'] else "No launch detected - whole session shown",
                 date=stamp.strftime('%A %d %B %Y, %H:%M:%S'), tiles=tiles, events=events, rec=rec,
                 checks=checks, link=link, plots=plots, notes=notes,
                 files=[os.path.basename(f) for f in files],
                 generated=datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
        self._write_pdf(base + ".pdf", R)
        self._write_html(base + ".html", R)
        return base + ".pdf", base + ".html"

    # ---------- PDF (drawn directly with QPainter: exact, no extra packages) ----------
    def _write_pdf(self, path, R):
        writer = QtGui.QPdfWriter(path)
        writer.setPageSize(QtGui.QPageSize(_qenum(QtGui.QPageSize, 'PageSizeId', 'Letter')))
        writer.setResolution(300)
        try:
            writer.setPageMargins(QtCore.QMarginsF(0, 0, 0, 0))
        except TypeError:
            pass
        writer.setTitle("AURORA Insight - " + R['title'].title())
        writer.setCreator("AURORA Insight Dashboard V4")
        k = 300 / 72.0                        # points -> device pixels
        PW, PH, M = 612.0, 792.0, 40.0        # US Letter in points, margin
        CW = PW - 2 * M
        AL = QtCore.Qt.AlignmentFlag
        WRAP = int(QtCore.Qt.TextFlag.TextWordWrap)

        p = QtGui.QPainter(writer)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)

        def rect(x, y, w_, h):
            return QtCore.QRectF(x * k, y * k, w_ * k, h * k)

        def font(size, bold=False, mono=False, spacing=0.0, black=False):
            f = QtGui.QFont(FONT_MONO if mono else FONT_UI)
            f.setPointSizeF(size)
            if black:
                f.setWeight(QtGui.QFont.Weight.Black)
            elif bold:
                f.setWeight(QtGui.QFont.Weight.Bold)
            if spacing:
                f.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, spacing * k)
            return f

        def text(x, y, w_, h, s_, f, color='#000000', align=None, wrap=False):
            p.setFont(f)
            p.setPen(QtGui.QColor(color))
            flags = int(align if align is not None else (AL.AlignLeft | AL.AlignVCenter))
            if wrap:
                flags |= WRAP
            p.drawText(rect(x, y, w_, h), flags, s_)

        def text_w(s_, f):
            return QtGui.QFontMetricsF(f, writer).horizontalAdvance(s_) / k

        def text_h(s_, f, w_):
            fm = QtGui.QFontMetricsF(f, writer)
            br = fm.boundingRect(QtCore.QRectF(0, 0, w_ * k, 1e7), int(AL.AlignLeft) | WRAP, s_)
            return br.height() / k

        def fill(x, y, w_, h, color):
            p.fillRect(rect(x, y, w_, h), QtGui.QColor(color))

        state = {'page': 1, 'y': 0.0}

        def footer():
            fill(M, PH - 30, CW, 0.6, '#cccccc')
            text(M, PH - 27, CW, 12, f"AURORA Insight  ·  {R['title'].title()}  ·  {R['date']}",
                 font(7), '#777777')
            text(M, PH - 27, CW, 12, f"Page {state['page']}", font(7), '#777777',
                 align=AL.AlignRight | AL.AlignVCenter)

        def new_page():
            footer()
            writer.newPage()
            state['page'] += 1
            state['y'] = M

        def need(h):
            if state['y'] + h > PH - 44:
                new_page()

        def section(title):
            need(26)
            state['y'] += 8
            text(M, state['y'], CW, 14, title.upper(), font(9, bold=True, spacing=1.6), C_BRAND)
            state['y'] += 17

        # ---- header band ----
        fill(0, 0, PW, 74, '#000000')
        fill(0, 74, PW, 4, C_BRAND)
        fa = font(24, black=True, spacing=1.5)
        text(M, 20, 300, 34, "AURORA", fa, '#ffffff')
        text(M + text_w("AURORA", fa) + 10, 28, 200, 22, "INSIGHT", font(10, bold=True, spacing=3), C_BRAND)
        text(PW - M - 320, 18, 320, 22, R['title'], font(14, bold=True, spacing=1.5), '#ffffff',
             align=AL.AlignRight | AL.AlignVCenter)
        text(PW - M - 320, 40, 320, 14, R['date'], font(8.5), '#bbbbbb', align=AL.AlignRight | AL.AlignVCenter)
        state['y'] = 92
        if R['subtitle']:
            text(M, state['y'], CW, 14, R['subtitle'], font(9, bold=True), C_BRAND)
            state['y'] += 16

        # ---- key results tiles ----
        section("Key results")
        gap, th = 10.0, 62.0
        tw = (CW - 3 * gap) / 4
        for i, (lab, val, sub) in enumerate(R['tiles']):
            col, row = i % 4, i // 4
            x = M + col * (tw + gap)
            y = state['y'] + row * (th + 8)
            fill(x, y, tw, 2.2, C_BRAND)
            text(x, y + 6, tw, 10, lab, font(6.5, bold=True, spacing=0.6), '#666666')
            text(x, y + 17, tw, 24, val, font(17, bold=True), '#000000')
            text(x, y + 42, tw, 18, sub, font(7), '#444444', align=AL.AlignLeft | AL.AlignTop, wrap=True)
        state['y'] += 2 * (th + 8)

        # ---- event timeline ----
        if R['events']:
            section("Event timeline")
            cols = [("EVENT", 150, AL.AlignLeft), ("T+ (s)", 70, AL.AlignRight),
                    ("ALTITUDE AGL", 100, AL.AlignRight), ("VERT. VELOCITY", 110, AL.AlignRight),
                    ("ACCEL", CW - 430, AL.AlignRight)]
            rh = 16.0
            need(rh * (len(R['events']) + 1))
            fill(M, state['y'], CW, rh, '#000000')
            x = M
            for name, cw_, al in cols:
                text(x + 6, state['y'], cw_ - 12, rh, name, font(7.5, bold=True, spacing=0.5), '#ffffff',
                     align=al | AL.AlignVCenter)
                x += cw_
            state['y'] += rh
            for j, row in enumerate(R['events']):
                if j % 2:
                    fill(M, state['y'], CW, rh, '#f2f2f2')
                x = M
                for (name, cw_, al), cell in zip(cols, row):
                    text(x + 6, state['y'], cw_ - 12, rh, cell, font(8.5, bold=(cell is row[0])), '#000000',
                         align=al | AL.AlignVCenter)
                    x += cw_
                state['y'] += rh

        # ---- two columns: recovery + pad check | link ----
        col_w = (CW - 20) / 2
        kw = 108.0

        def kv_block(x, y, items):
            for it in items:
                key, val = it[0], it[1]
                color = it[2] if len(it) > 2 and it[2] and str(it[2]).startswith('#') else '#000000'
                fv = font(8.5, bold=True)
                h = max(13.0, text_h(val, fv, col_w - kw) + 3)
                text(x, y, kw - 6, h, key, font(8), '#555555', align=AL.AlignLeft | AL.AlignTop, wrap=True)
                text(x + kw, y, col_w - kw, h, val, fv, color, align=AL.AlignLeft | AL.AlignTop, wrap=True)
                y += h + 2
            return y

        need(150)
        y0 = state['y']
        section("Recovery")
        yl = kv_block(M, state['y'], [(a, b_) for a, b_, _ in R['rec']])
        if R['checks']:
            state['y'] = yl
            section("Pad sensor check")
            yl = kv_block(M, state['y'], [(a, b_, c) for a, b_, c in R['checks']])
        state['y'] = y0
        state['y'] += 8
        text(M + col_w + 20, state['y'], col_w, 14, "TELEMETRY LINK", font(9, bold=True, spacing=1.6), C_BRAND)
        yr = kv_block(M + col_w + 20, state['y'] + 17, R['link'])
        state['y'] = max(yl, yr) + 6

        # ---- plots ----
        new_page()
        for title, img in R['plots']:
            ih = CW * img.height() / max(1, img.width())
            need(ih + 30)
            section(title)
            p.drawImage(rect(M, state['y'], CW, ih), img)
            state['y'] += ih + 6

        # ---- notes ----
        fn = font(7.5)
        notes = R['notes'] + ("\nFiles: " + ", ".join(R['files']) if R['files'] else "") + \
            f"\nGenerated by AURORA Insight Dashboard V4 on {R['generated']}."
        nh = text_h(notes, fn, CW) + 4
        need(nh + 10)
        state['y'] += 6
        text(M, state['y'], CW, nh, notes, fn, '#666666', align=AL.AlignLeft | AL.AlignTop, wrap=True)
        footer()
        p.end()

    # ---------- HTML (single file, images embedded) ----------
    def _write_html(self, path, R):
        import base64
        E = html.escape

        def png_uri(img):
            return "data:image/png;base64," + base64.b64encode(self._png_bytes(img)).decode()

        tiles = "".join(f"<div class='tile'><div class='lab'>{E(a)}</div><div class='val'>{E(b_)}</div>"
                        f"<div class='sub'>{E(c)}</div></div>" for a, b_, c in R['tiles'])
        ev = ""
        if R['events']:
            ev = ("<h2>Event timeline</h2><table class='ev'><tr><th>Event</th><th>T+ (s)</th><th>Altitude AGL</th>"
                  "<th>Vert. velocity</th><th>Accel</th></tr>"
                  + "".join("<tr>" + "".join(f"<td>{E(c)}</td>" for c in row) + "</tr>" for row in R['events'])
                  + "</table>")

        def kv(items):
            out = ""
            for it in items:
                key, val = it[0], it[1]
                extra = it[2] if len(it) > 2 else None
                v = E(val)
                if extra and str(extra).startswith('http'):
                    v = f"<a href='{E(extra)}'>{v}</a>"
                elif extra:
                    v = f"<span style='color:{extra}'>{v}</span>"
                out += f"<tr><td class='k'>{E(key)}</td><td class='v'>{v}</td></tr>"
            return f"<table class='kv'>{out}</table>"

        left = "<h2>Recovery</h2>" + kv(R['rec'])
        if R['checks']:
            left += "<h2>Pad sensor check</h2>" + kv(R['checks'])
        right = "<h2>Telemetry link</h2>" + kv(R['link'])
        plots = "".join(f"<h2>{E(t)}</h2><img src='{png_uri(img)}'>" for t, img in R['plots'])
        files = ("<br>Files: " + E(", ".join(R['files']))) if R['files'] else ""
        sub = f"<div class='subtitle'>{E(R['subtitle'])}</div>" if R['subtitle'] else ""
        doc = f"""<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>AURORA {E(R['title'].title())}</title>
<style>
 body {{ margin:0; background:#e9e9e9; font-family:'{FONT_UI}', 'Helvetica Neue', Arial, sans-serif; color:#000; }}
 .page {{ max-width:880px; margin:24px auto; background:#fff; box-shadow:0 2px 12px rgba(0,0,0,.15); }}
 header {{ background:#000; color:#fff; padding:22px 36px; display:flex; justify-content:space-between;
          align-items:center; border-bottom:5px solid {C_BRAND}; flex-wrap:wrap; gap:8px; }}
 .logo {{ font-size:30px; font-weight:900; letter-spacing:2px; }}
 .logo span {{ color:{C_BRAND}; font-size:13px; letter-spacing:4px; margin-left:10px; vertical-align:middle; }}
 .title {{ text-align:right; font-weight:700; letter-spacing:2px; }}
 .title small {{ display:block; color:#bbb; font-weight:400; letter-spacing:0; margin-top:2px; }}
 main {{ padding:8px 36px 28px; }}
 h2 {{ color:{C_BRAND}; font-size:13px; letter-spacing:2px; text-transform:uppercase; margin:22px 0 8px; }}
 .subtitle {{ color:{C_BRAND}; font-weight:700; margin-top:14px; }}
 .tiles {{ display:grid; grid-template-columns:repeat(4, 1fr); gap:12px; }}
 .tile {{ border-top:3px solid {C_BRAND}; padding-top:6px; }}
 .lab {{ font-size:10px; font-weight:700; color:#666; letter-spacing:1px; }}
 .val {{ font-size:24px; font-weight:800; margin:2px 0; }}
 .sub {{ font-size:11px; color:#444; }}
 table.ev {{ width:100%; border-collapse:collapse; font-size:13px; }}
 table.ev th {{ background:#000; color:#fff; text-align:right; padding:6px 8px; font-size:11px; letter-spacing:1px; }}
 table.ev td {{ text-align:right; padding:5px 8px; }}
 table.ev th:first-child, table.ev td:first-child {{ text-align:left; font-weight:700; }}
 table.ev tr:nth-child(odd) td {{ background:#f2f2f2; }}
 .cols {{ display:grid; grid-template-columns:1fr 1fr; gap:24px; }}
 table.kv {{ border-collapse:collapse; font-size:13px; }}
 table.kv td {{ padding:3px 12px 3px 0; vertical-align:top; }}
 .k {{ color:#555; }} .v {{ font-weight:700; }}
 a {{ color:{C_BRAND}; }}
 img {{ width:100%; display:block; }}
 .notes {{ font-size:11px; color:#666; margin-top:20px; line-height:1.5; }}
 @media (max-width:640px) {{ .tiles {{ grid-template-columns:repeat(2, 1fr); }} .cols {{ grid-template-columns:1fr; }}
   header, main {{ padding-left:16px; padding-right:16px; }} }}
 @media print {{ body {{ background:#fff; }} .page {{ box-shadow:none; margin:0; max-width:none; }} }}
</style></head><body><div class='page'>
<header><div class='logo'>AURORA<span>INSIGHT</span></div>
<div class='title'>{E(R['title'])}<small>{E(R['date'])}</small></div></header>
<main>{sub}<h2>Key results</h2><div class='tiles'>{tiles}</div>{ev}
<div class='cols'><div>{left}</div><div>{right}</div></div>
{plots}
<div class='notes'>{E(R['notes'])}{files}<br>Generated by AURORA Insight Dashboard V4 on {E(R['generated'])}.</div>
</main></div></body></html>"""
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(doc)


# ===================== Google Earth (KML / KMZ) =====================

def _kml_color(hex_rgb, alpha=255):
    """'#rrggbb' -> KML 'aabbggrr'."""
    h = hex_rgb.lstrip('#')
    return f"{alpha:02x}{h[4:6]}{h[2:4]}{h[0:2]}"


def _circle_coords(lat, lon, radius_m, n=48):
    pts = []
    for i in range(n + 1):
        a = 2 * math.pi * i / n
        dlat = (radius_m * math.cos(a)) / 110540.0
        dlon = (radius_m * math.sin(a)) / (111320.0 * math.cos(math.radians(lat)))
        pts.append(f"{lon + dlon:.7f},{lat + dlat:.7f},0")
    return " ".join(pts)


def build_kml(rx, live=False):
    """KML document for Google Earth (Pro, web, iOS/Android) from the receiver's flight record."""
    X = html.escape
    with rx.data_lock:
        d = np.array(rx.flight_rec, dtype=float) if rx.flight_rec else np.zeros((0, 14))
        origin, landing, last_fix = rx.track_origin, rx.landing_pos, rx.last_rocket_fix
        rx_pos = rx.receiver_positions[-1] if rx.receiver_positions else None
        start_wall, ground_alt, max_agl = rx.start_wall_time, rx.ground_alt, rx.max_agl
    stamp = datetime.fromtimestamp(start_wall or time.time())

    # events with positions (reuse the report's event detection)
    events = []
    try:
        A = FlightReportBuilder(rx, "").analyse()
        for name, tt in A['events']:
            if name in ("Liftoff", "Touchdown"):
                continue
            t_abs = A['t0'] + tt
            ok = np.isfinite(d[:, FR_LAT])
            if ok.any():
                idx = np.where(ok)[0]
                i = idx[int(np.argmin(np.abs(d[idx, FR_T] - t_abs)))]
                events.append((name, tt, d[i, FR_LAT], d[i, FR_LON], max(0.0, d[i, FR_AGL])))
    except Exception:
        pass

    ok = np.isfinite(d[:, FR_LAT]) & np.isfinite(d[:, FR_LON]) if len(d) else np.zeros(0, bool)
    path = d[ok] if len(d) else d
    if len(path) > KML_MAX_POINTS:
        step = int(math.ceil(len(path) / KML_MAX_POINTS))
        path = np.vstack([path[::step], path[-1:]])
    coords3d = " ".join(f"{r[FR_LON]:.7f},{r[FR_LAT]:.7f},{max(0.0, r[FR_AGL]):.1f}" for r in path)
    coords2d = " ".join(f"{r[FR_LON]:.7f},{r[FR_LAT]:.7f},0" for r in path)

    red = _kml_color(C_BRAND)
    icon = "http://maps.google.com/mapfiles/kml/paddle/"
    out = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">',
        '<Document>',
        f"<name>AURORA {'LIVE ' if live else ''}flight {stamp.strftime('%Y-%m-%d %H:%M')}</name>",
        '<open>1</open>',
        f'<Style id="path"><LineStyle><color>{red}</color><width>4</width></LineStyle>'
        f'<PolyStyle><color>{_kml_color("#ffffff", 50)}</color></PolyStyle></Style>',
        f'<Style id="ground"><LineStyle><color>{_kml_color("#ffffff", 230)}</color><width>2.5</width></LineStyle></Style>',
        f'<Style id="search"><LineStyle><color>{red}</color><width>3</width></LineStyle>'
        f'<PolyStyle><color>{_kml_color(C_BRAND, 60)}</color></PolyStyle></Style>',
        f'<Style id="landing"><IconStyle><scale>1.6</scale><Icon><href>{icon}red-stars.png</href></Icon>'
        '<hotSpot x="32" y="1" xunits="pixels" yunits="pixels"/></IconStyle><LabelStyle><scale>1.3</scale></LabelStyle></Style>',
        f'<Style id="pad"><IconStyle><scale>1.2</scale><Icon><href>{icon}grn-circle.png</href></Icon>'
        '<hotSpot x="32" y="1" xunits="pixels" yunits="pixels"/></IconStyle></Style>',
        f'<Style id="you"><IconStyle><scale>1.2</scale><Icon><href>{icon}blu-circle.png</href></Icon>'
        '<hotSpot x="32" y="1" xunits="pixels" yunits="pixels"/></IconStyle></Style>',
        f'<Style id="event"><IconStyle><scale>0.9</scale><Icon><href>{icon}wht-blank.png</href></Icon>'
        '<hotSpot x="32" y="1" xunits="pixels" yunits="pixels"/></IconStyle><LabelStyle><scale>0.9</scale></LabelStyle></Style>',
    ]

    target = landing or last_fix
    if target:
        rng = 600.0
        if origin:
            x, y = local_xy(target[0], target[1], *origin)
            rng = max(400.0, 3.0 * math.hypot(x, y))
        out.append(f"<LookAt><longitude>{target[1]:.7f}</longitude><latitude>{target[0]:.7f}</latitude>"
                   f"<altitude>0</altitude><heading>0</heading><tilt>35</tilt><range>{rng:.0f}</range></LookAt>")

    # ---- recovery folder ----
    out.append("<Folder><name>Recovery</name><open>1</open>")
    if target:
        lat, lon = target[0], target[1]
        if landing:
            title, what = "ROCKET LANDED HERE", "Landing position (captured at touchdown)"
        else:
            age = time.time() - target[3] if len(target) > 3 else 0
            title, what = "ROCKET - LAST KNOWN", f"Last received position ({fmt_age(age)})"
        lines = [what,
                 f"{lat:.6f}, {lon:.6f}",
                 f"{fmt_dms(lat, 'N', 'S')}  {fmt_dms(lon, 'E', 'W')}"]
        if origin:
            x, y = local_xy(lat, lon, *origin)
            dv, du = fmt_dist(math.hypot(x, y))
            lines.append(f"{dv} {du} {compass8(math.degrees(math.atan2(x, y)))} of the pad")
        if rx_pos:
            x, y = local_xy(lat, lon, rx_pos[0], rx_pos[1])
            dv, du = fmt_dist(math.hypot(x, y))
            lines.append(f"{dv} {du} {compass8(math.degrees(math.atan2(x, y)))} of the receiver")
        nav = f"https://www.google.com/maps/dir/?api=1&amp;destination={lat:.6f},{lon:.6f}"
        desc = "<br>".join(X(l) for l in lines) + f'<br><br><a href="{nav}">Navigate here (Google Maps)</a>'
        out.append(f'<Placemark><name>{title}</name><styleUrl>#landing</styleUrl>'
                   f'<description><![CDATA[{desc}]]></description>'
                   f'<Point><coordinates>{lon:.7f},{lat:.7f},0</coordinates></Point></Placemark>')
        out.append(f'<Placemark><name>Search area ({SEARCH_RADIUS_M:.0f} m)</name><styleUrl>#search</styleUrl>'
                   f'<Polygon><tessellate>1</tessellate><outerBoundaryIs><LinearRing><coordinates>'
                   f'{_circle_coords(lat, lon, SEARCH_RADIUS_M)}</coordinates></LinearRing></outerBoundaryIs>'
                   f'</Polygon></Placemark>')
    if rx_pos:
        out.append(f'<Placemark><name>Receiver (you)</name><styleUrl>#you</styleUrl>'
                   f'<Point><coordinates>{rx_pos[1]:.7f},{rx_pos[0]:.7f},0</coordinates></Point></Placemark>')
    out.append("</Folder>")

    # ---- flight folder ----
    out.append("<Folder><name>Flight</name><open>0</open>")
    if origin:
        pad_desc = f"{origin[0]:.6f}, {origin[1]:.6f}"
        if ground_alt is not None:
            pad_desc += f"<br>Pad elevation (baro): {ground_alt:,.0f} m MSL"
        out.append(f'<Placemark><name>Launch pad</name><styleUrl>#pad</styleUrl>'
                   f'<description><![CDATA[{pad_desc}]]></description>'
                   f'<Point><coordinates>{origin[1]:.7f},{origin[0]:.7f},0</coordinates></Point></Placemark>')
    for name, tt, lat, lon, alt in events:
        label = f"{name} ({alt:,.0f} m)" if name in ("Apogee",) else name
        out.append(f'<Placemark><name>{X(label)}</name><styleUrl>#event</styleUrl>'
                   f'<description>T+{tt:.1f} s, {alt:,.0f} m above pad</description>'
                   f'<Point><altitudeMode>relativeToGround</altitudeMode>'
                   f'<coordinates>{lon:.7f},{lat:.7f},{alt:.1f}</coordinates></Point></Placemark>')
    if len(path) >= 2:
        out.append(f'<Placemark><name>Flight path (3D, apogee {max_agl:,.0f} m)</name><styleUrl>#path</styleUrl>'
                   f'<LineString><extrude>1</extrude><tessellate>1</tessellate>'
                   f'<altitudeMode>relativeToGround</altitudeMode><coordinates>{coords3d}</coordinates>'
                   f'</LineString></Placemark>')
        out.append(f'<Placemark><name>Ground track</name><styleUrl>#ground</styleUrl>'
                   f'<LineString><tessellate>1</tessellate><altitudeMode>clampToGround</altitudeMode>'
                   f'<coordinates>{coords2d}</coordinates></LineString></Placemark>')
    out.append("</Folder>")
    out.append(f"<description><![CDATA[Generated by AURORA Insight Dashboard V4 on "
               f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}. Altitudes are barometric, above the pad."
               f"]]></description>")
    out.append("</Document></kml>")
    return "\n".join(out)


def write_kmz(path, kml_text):
    import zipfile
    tmp = path + ".tmp"
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("doc.kml", kml_text)
    os.replace(tmp, path)


def write_text_atomic(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(text)
    os.replace(tmp, path)


# ===================== Telemetry Receiver =====================

class TelemetryReceiver:
    def __init__(self, demo=False):
        # --- Serial / run state ---
        self.running = False
        self.demo = demo

        # multiprocessing queues: serial lines in, commands out
        self.data_queue = Queue()
        self.cmd_queue = Queue()
        self.reader_proc = None

        # Thread-safe data storage
        self.data_lock = threading.Lock()
        self.all_time = deque(maxlen=MAX_PLOT_POINTS)
        self.all_data = {label: deque(maxlen=MAX_PLOT_POINTS) for label in DATA_LABELS}

        # Navigation data storage
        self.nav_data = {label: deque(maxlen=MAX_PLOT_POINTS) for label in NAV_LABELS}
        self.nav_status = {
            'DIR': deque(maxlen=MAX_PLOT_POINTS),
            'TURN': deque(maxlen=MAX_PLOT_POINTS),
            'GPS_FIX': deque(maxlen=MAX_PLOT_POINTS)
        }

        # Derived data storage
        self.g_force_magnitude = deque(maxlen=MAX_PLOT_POINTS)
        self.vertical_speed = deque(maxlen=MAX_PLOT_POINTS)
        self.altitude_rate = deque(maxlen=MAX_PLOT_POINTS)
        self.connection_quality = deque(maxlen=100)
        self.packet_intervals = deque(maxlen=50)
        self.g_sensor_used = deque(maxlen=MAX_PLOT_POINTS)

        # Mission Control (new) fast buffers - same length/timebase as all_time
        self.agl_hist = deque(maxlen=MAX_PLOT_POINTS)
        self.vs_smooth_hist = deque(maxlen=MAX_PLOT_POINTS)
        self.rssi_hist = deque(maxlen=MAX_PLOT_POINTS)
        self.snr_hist = deque(maxlen=MAX_PLOT_POINTS)

        # Mission Control long history (low rate)
        self.h_t = deque(maxlen=HISTORY_MAX_POINTS)
        self.h_agl = deque(maxlen=HISTORY_MAX_POINTS)
        self.h_vs = deque(maxlen=HISTORY_MAX_POINTS)
        self.h_g = deque(maxlen=HISTORY_MAX_POINTS)
        self._h_last_t = -1e9
        self._h_peak_g = 0.0

        # GPS tracking data
        self.rocket_positions = deque(maxlen=200)
        self.receiver_positions = deque(maxlen=200)
        self.distance_history = deque(maxlen=MAX_PLOT_POINTS)
        self.bearing_history = deque(maxlen=MAX_PLOT_POINTS)
        self.track_origin = None      # (lat, lon) of first rocket fix (pad)
        self.last_rocket_fix = None   # (lat, lon, gps_alt, wall_time) of newest valid coords
        self.landing_pos = None       # last_rocket_fix captured at touchdown

        # GPS caches
        self.gps_lats = deque(maxlen=100)
        self.gps_lons = deque(maxlen=100)
        self.gps_alts = deque(maxlen=100)

        # Connection monitoring
        self.packet_count = 0
        self.bad_packets = 0
        self.packet_timestamps = deque(maxlen=100)
        self.last_packet_time = None
        self.connection_drops = 0
        self.max_packet_gap = 0
        self.current_rssi = None
        self.current_snr = None
        self.rx_freq_mhz = None

        # Flight statistics
        self.max_altitude = 0          # baro MSL (as before)
        self.max_g_force = 0
        self.max_speed = 0             # GPS speed, km/h (TX sends km/h)
        self.max_distance = 0
        self.flight_phase = "Ground"
        self._last_logged_g = 5.0
        self._last_logged_dist = 1000.0
        self._last_alt_milestone = 0

        # Altitude above ground
        self.ground_samples = deque(maxlen=GROUND_REF_SAMPLES)
        self.ground_alt = None
        self.current_agl = 0.0
        self.max_agl = 0.0
        self.apogee_time = None
        self.launch_time = None
        self._launch_entry_t = None
        self.vs_smooth = 0.0

        # Navigation statistics
        self.current_distance = 0
        self.current_bearing = 0
        self.receiver_has_fix = False
        self.rocket_has_fix = False

        # Status
        self.start_time = None          # TX timestamp base (for logging)
        self.start_wall_time = None     # Pi-side wall-clock base for plotting
        self.last_tx_tod = None
        self.last_update_time = time.time()
        self.packets_per_second = 0.0

        # File logging (one folder per session)
        self.session_stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
        self.session_dir = os.path.join(LOG_DIR, self.session_stamp)
        self.log_file = None
        self.log_filename = None
        self.csv_writer = None
        self.raw_log = None
        self.raw_log_filename = None
        self._raw_lines = 0

        # Flight record for the report (full rate, compact)
        self.flight_rec = deque(maxlen=FLIGHT_REC_MAX)
        self.touchdown_time = None
        self.report_paths = None
        self._report_due = None
        self.kmz_path = None
        self._live_kml_t = 0.0
        self._live_link_written = False

        # Web
        self.web_dir_ok = os.path.isdir(WEB_DIR)
        self.web_thread = None
        self.local_ip = None
        self._ip_checked = 0.0
        self._state_write_errors = 0

        # Terminal output buffer
        self.terminal_messages = deque(maxlen=50)
        self.terminal_dirty = True
        self._cached_terminal_text = ""
        self.log_seq = 0
        self.log_events = deque(maxlen=300)   # (seq, timestamp, message)

        # Regex for floats
        self.num_pattern = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')

        # GUI / PyQtGraph members
        self.app = None
        self.main_win = None
        self.tabs = None
        self.win = None                # Sensor View GraphicsLayoutWidget
        self.plot_items = {}           # Sensor View: name -> PlotItem
        self.curves = {}               # Sensor View: channel -> PlotDataItem list
        self.status_label = None
        self.terminal_label = None
        self.timer = None
        self.frame_counter = 0
        self.mc = {}                   # Mission Control widgets
        self.mc_window = 15.0          # seconds, None = whole flight
        self._big_qr = None
        self._big_qr_widget = None
        self._big_qr_label = None
        self._shown_log_seq = 0

    # ===================== Terminal / Logging =====================
    def add_terminal_message(self, message):
        timestamp = datetime.now().strftime('%H:%M:%S')
        msg = f"[{timestamp}] {message}"
        self.terminal_messages.append(msg)
        self.terminal_dirty = True
        self.log_seq += 1
        self.log_events.append((self.log_seq, timestamp, message))
        print(msg)

    def list_serial_ports(self):
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            self.add_terminal_message("Available serial ports:")
            for port in ports:
                self.add_terminal_message(f"  {port.device} - {port.description}")
            return [port.device for port in ports]
        except ImportError:
            self.add_terminal_message("pyserial not installed. Install with: pip install pyserial")
            return []

    def setup_logging(self):
        if SAVE_TO_FILE:
            ts = self.session_stamp
            os.makedirs(self.session_dir, exist_ok=True)
            filename = os.path.join(self.session_dir, f'ReceiverLog_{ts}.csv')
            self.log_filename = filename
            # raw copy of every line the receiver prints (exactly as transmitted + NAV block)
            self.raw_log_filename = os.path.join(self.session_dir, f'RawSerial_{ts}.txt')
            self.raw_log = open(self.raw_log_filename, 'w', encoding='utf-8')
            self.raw_log.write("# PC_time<TAB>line exactly as received from the receiver\n")
            self.log_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.log_file)
            # Same columns as before, with new columns appended at the end
            header = (['Timestamp'] + DATA_LABELS +
                      ['G_Force', 'Vertical_Speed', 'Connection_Quality', 'G_Sensor_Used'] +
                      NAV_LABELS + ['Direction', 'Turn_Direction', 'Distance_km'] +
                      ['RSSI_dBm', 'SNR_dB', 'Alt_AGL_m', 'Flight_Phase', 'PC_Time'])
            self.csv_writer.writerow(header)
            self.add_terminal_message(f"Logging to {self.session_dir}")

    def get_local_ip(self):
        ip = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))  # no real traffic, used to pick interface
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            pass
        if not ip or ip.startswith("127."):
            # no default route (e.g. Pi running its own hotspot): ask the OS
            try:
                import subprocess
                out = subprocess.run(["hostname", "-I"], capture_output=True,
                                     text=True, timeout=1).stdout.split()
                cands = [a for a in out if a.count('.') == 3 and not a.startswith("127.")]
                if cands:
                    ip = cands[0]
            except Exception:
                pass
        if not ip:
            try:
                ip = socket.gethostbyname(socket.gethostname())
            except Exception:
                ip = "0.0.0.0"
        return ip

    def tracker_url(self, ip=None):
        return f"https://{ip or self.get_local_ip()}:{TRACKER_PORT}/{TRACKER_PAGE}"

    def send_receiver_command(self, cmd):
        try:
            self.cmd_queue.put(cmd)
            if self.raw_log:
                self.raw_log.write(f"{time.time():.3f}\t# SENT> {cmd}\n")
            self.add_terminal_message(f"Sent to receiver: {cmd}")
        except Exception as e:
            self.add_terminal_message(f"Command send error: {e}")

    # ===================== Parsing =====================
    def parse_data_packet(self, line):
        try:
            data_idx = line.find('DATA:')
            if data_idx == -1:
                return None, None

            nav_idx = line.find(':NAV:')
            if nav_idx != -1:
                telemetry_part = line[data_idx + 5:nav_idx]
                nav_part = line[nav_idx + 5:]
            else:
                telemetry_part = line[data_idx + 5:]
                nav_part = ""

            pipe_idx = telemetry_part.find('|')
            if pipe_idx != -1:
                telemetry_part = telemetry_part[:pipe_idx]

            numbers = [float(x) for x in self.num_pattern.findall(telemetry_part)]
            if len(numbers) < 24:
                return None, None

            tx_timestamp = numbers[0]
            values = numbers[1:24]

            nav_values = {}
            if nav_part:
                nav_values = self.parse_navigation_data(nav_part)

            return (tx_timestamp, values), nav_values
        except Exception:
            return None, None

    def parse_navigation_data(self, nav_string):
        nav_values = {}
        try:
            parts = nav_string.split(':')
            for i in range(0, len(parts) - 1, 2):
                key = parts[i]
                value_str = parts[i + 1]
                if key in ['DIR', 'TURN']:
                    nav_values[key] = value_str
                elif key in ['RX_LAT', 'RX_LON'] and value_str == 'NO_FIX':
                    nav_values[key] = None
                elif key in NAV_LABELS or key in LINK_LABELS:
                    try:
                        nav_values[key] = float(value_str)
                    except ValueError:
                        nav_values[key] = None
        except Exception as e:
            self.add_terminal_message(f"Navigation parsing error: {e}")
        return nav_values

    def handle_aux_line(self, line):
        """Non-DATA lines from the receiver (config replies, boot messages)."""
        if line.startswith(':RXCFG:FREQ_MHZ:'):
            try:
                self.rx_freq_mhz = float(line.split(':')[-1])
                self.add_terminal_message(f"Receiver frequency: {self.rx_freq_mhz:.3f} MHz")
            except ValueError:
                self.add_terminal_message(f"RX> {line[:TERMINAL_MAX_CHARS]}")
        elif line.startswith(':RXCFG:ERR'):
            self.add_terminal_message(f"Receiver config error: {line.split(':')[-1]}")
        elif line.startswith(':DASH:ERR:'):
            self.add_terminal_message(f"ERROR: {line[10:]}")
        else:
            self.add_terminal_message(f"RX> {line[:TERMINAL_MAX_CHARS]}")

    # ===================== Derived / Navigation Metrics =====================
    def update_ground_reference(self, baro_alt):
        """Rolling average of baro altitude while sitting on the pad = 0 m AGL."""
        if self.launch_time is None and self.flight_phase == "Ground":
            self.ground_samples.append(baro_alt)
            self.ground_alt = float(np.mean(self.ground_samples))
        if self.ground_alt is None:
            self.ground_alt = baro_alt
        self.current_agl = baro_alt - self.ground_alt

    def zero_altitude(self):
        with self.data_lock:
            self.ground_samples.clear()
            if self.all_data['BaroAlt']:
                recent = list(self.all_data['BaroAlt'])[-10:]
                self.ground_alt = float(np.mean(recent))
                self.ground_samples.extend(recent)
            self.max_agl = 0.0
            self._last_alt_milestone = 0
        self.add_terminal_message("Altitude zeroed (AGL reference reset)")

    def reset_records(self):
        with self.data_lock:
            self.max_altitude = 0
            self.max_g_force = 0
            self.max_speed = 0
            self.max_distance = 0
            self.max_agl = 0.0
            self.apogee_time = None
            self.landing_pos = None
            self.touchdown_time = None
            self.report_paths = None
            self._report_due = None
            self.flight_rec.clear()
            self.kmz_path = None
            self.launch_time = None
            self._launch_entry_t = None
            self.connection_drops = 0
            self.max_packet_gap = 0
            self.bad_packets = 0
            self._last_logged_g = 5.0
            self._last_logged_dist = 1000.0
            self._last_alt_milestone = 0
            self.flight_phase = "Ground"
        self.add_terminal_message("Records, mission clock and phase reset")

    def calculate_derived_metrics(self, values, t_current):
        lg_ax, lg_ay, lg_az = values[IDX_LG_AX], values[IDX_LG_AY], values[IDX_LG_AZ]
        hg_ax, hg_ay, hg_az = values[IDX_HG_AX], values[IDX_HG_AY], values[IDX_HG_AZ]

        lg_mag = math.sqrt(lg_ax*lg_ax + lg_ay*lg_ay + lg_az*lg_az) / 9.81
        if lg_mag >= 13.0:
            g_force = math.sqrt(hg_ax*hg_ax + hg_ay*hg_ay + hg_az*hg_az) / 9.81
            sensor_used = "HG"
        else:
            g_force = lg_mag
            sensor_used = "LG"

        vertical_speed = 0.0
        altitude_rate = 0.0
        if len(self.all_data['BaroAlt']) >= 2:
            prev_alt = self.all_data['BaroAlt'][-1]
            prev_time = self.all_time[-1]
            current_alt = values[IDX_BARO_ALT]
            dt = t_current - prev_time
            if dt < 0.02 and self.altitude_rate:
                # packets arrived back-to-back (USB burst): a tiny dt would turn
                # 0.1 m of baro noise into a huge fake climb rate
                altitude_rate = self.altitude_rate[-1]
                vertical_speed = self.vertical_speed[-1] if self.vertical_speed else altitude_rate
            elif dt > 0:
                altitude_rate = (current_alt - prev_alt) / dt
                if self.altitude_rate:
                    prev_rate = self.altitude_rate[-1]
                    vertical_speed = 0.7 * prev_rate + 0.3 * altitude_rate
                else:
                    vertical_speed = altitude_rate

        if g_force > self.max_g_force:
            self.max_g_force = g_force
            # log only when the record grows by >= 1 g (avoids log spam during boost)
            if g_force > 5.0 and g_force >= self._last_logged_g + 1.0:
                self._last_logged_g = g_force
                self.add_terminal_message(f"New max G-force: {g_force:.2f} g")

        if values[IDX_BARO_ALT] > self.max_altitude:
            self.max_altitude = values[IDX_BARO_ALT]

        agl = self.current_agl
        if agl > self.max_agl:
            self.max_agl = agl
            self.apogee_time = t_current
            milestone = int(agl // 100) * 100
            if milestone >= 100 and milestone > self._last_alt_milestone:
                self._last_alt_milestone = milestone
                self.add_terminal_message(f"Passing {milestone} m AGL")

        speed = values[IDX_SPEED]
        if speed >= 0:
            self.max_speed = max(self.max_speed, speed)

        # Flight phase - same rules as before, but "Ground" uses altitude above
        # the pad instead of above sea level (works at any launch site elevation)
        old_phase = self.flight_phase
        if g_force > 2.0 and self.flight_phase == "Ground":
            self.flight_phase = "Launch"
        elif altitude_rate < -5 and self.flight_phase in ["Launch", "Ascent"]:
            self.flight_phase = "Descent"
        elif altitude_rate > 5 and self.flight_phase in ["Ground", "Launch"]:
            self.flight_phase = "Ascent"
        elif abs(altitude_rate) < 2 and agl < 100:
            self.flight_phase = "Ground"
        if old_phase != self.flight_phase:
            self.add_terminal_message(f"Flight phase changed: {old_phase} → {self.flight_phase}")
            if self.flight_phase == "Launch":
                self._launch_entry_t = t_current
            if self.flight_phase == "Ascent" and self.launch_time is None:
                self.launch_time = self._launch_entry_t if self._launch_entry_t is not None else t_current
                self.add_terminal_message("LIFTOFF! Mission clock started")
            if self.flight_phase == "Descent" and old_phase in ("Launch", "Ascent"):
                self.add_terminal_message(f"APOGEE ≈ {self.max_agl:.0f} m AGL")
            if self.flight_phase == "Ground" and old_phase == "Descent":
                self.add_terminal_message("TOUCHDOWN")
                self.touchdown_time = t_current
                if AUTO_REPORT and self.report_paths is None:
                    self._report_due = time.time() + REPORT_DELAY_S
                if self.last_rocket_fix is not None:
                    self.landing_pos = self.last_rocket_fix
                    self.add_terminal_message(
                        f"Landing position: {self.landing_pos[0]:.6f}, {self.landing_pos[1]:.6f}")
            if self.flight_phase == "Ground" and old_phase in ("Launch", "Ascent") and self.launch_time is not None:
                self.launch_time = None
                self.add_terminal_message("False launch trigger - mission clock reset")

        return g_force, vertical_speed, altitude_rate, sensor_used

    def process_navigation_data(self, nav_values, tx_timestamp, rocket_values):
        if not nav_values:
            for key in NAV_LABELS:
                self.nav_data[key].append(0)
            self.nav_status['DIR'].append('UNKNOWN')
            self.nav_status['TURN'].append('UNKNOWN')
            self.distance_history.append(0)
            self.bearing_history.append(0)
            return

        for key in NAV_LABELS:
            value = nav_values.get(key)
            if value is not None and isinstance(value, (int, float)):
                self.nav_data[key].append(value)
            else:
                self.nav_data[key].append(0)

        self.nav_status['DIR'].append(nav_values.get('DIR', 'UNKNOWN'))
        self.nav_status['TURN'].append(nav_values.get('TURN', 'UNKNOWN'))

        rx_lat = nav_values.get('RX_LAT')
        rx_lon = nav_values.get('RX_LON')
        if (rx_lat is not None and rx_lon is not None and
            isinstance(rx_lat, (int, float)) and isinstance(rx_lon, (int, float)) and
            abs(rx_lat) > 0.001 and abs(rx_lon) > 0.001):
            rx_hdg = nav_values.get('RX_HDG', 0) or 0
            self.receiver_positions.append((rx_lat, rx_lon, rx_hdg, tx_timestamp))
            if not self.receiver_has_fix:
                self.receiver_has_fix = True
                self.add_terminal_message("Receiver GPS fix acquired")
        else:
            if self.receiver_has_fix:
                self.receiver_has_fix = False
                self.add_terminal_message("Receiver GPS fix lost")

        rocket_lat, rocket_lon = rocket_values[IDX_LAT], rocket_values[IDX_LON]
        if (rocket_lat is not None and rocket_lon is not None and
            abs(rocket_lat) > 0.001 and abs(rocket_lon) > 0.001):
            self.rocket_positions.append((rocket_lat, rocket_lon, rocket_values[IDX_ALT], tx_timestamp))
            if self.track_origin is None:
                self.track_origin = (rocket_lat, rocket_lon)
            if not self.rocket_has_fix:
                self.rocket_has_fix = True
                self.add_terminal_message("Rocket GPS fix acquired")
        else:
            if self.rocket_has_fix:
                self.rocket_has_fix = False
                self.add_terminal_message("Rocket GPS fix lost")

        distance = nav_values.get('DIST', 0)
        bearing = nav_values.get('BEAR', 0)
        if distance is not None and isinstance(distance, (int, float)) and distance > 0:
            self.distance_history.append(distance)
            self.current_distance = distance
            if distance > self.max_distance:
                self.max_distance = distance
                if distance > 1000 and distance >= self._last_logged_dist + 100:
                    self._last_logged_dist = distance
                    self.add_terminal_message(
                        f"New max distance: {distance:.1f} m ({distance/1000:.2f} km)"
                    )
        else:
            self.distance_history.append(0)

        if bearing is not None and isinstance(bearing, (int, float)):
            self.bearing_history.append(bearing)
            self.current_bearing = bearing
        else:
            self.bearing_history.append(0)

    def calculate_connection_quality(self, current_time=None):
        if current_time is None:
            current_time = time.time()
        if self.last_packet_time:
            interval = current_time - self.last_packet_time
            self.packet_intervals.append(interval)
            self.max_packet_gap = max(self.max_packet_gap, interval)
            expected_interval = 1.0 / EXPECTED_PACKET_RATE
            if interval > 2 * expected_interval:
                self.connection_drops += 1
                self.add_terminal_message(f"Connection drop detected: {interval:.2f} s gap")
        self.last_packet_time = current_time

        # track timestamps for rate calculation
        self.packet_timestamps.append(current_time)
        # compute packets per second over last 1s
        window_start = current_time - 1.0
        count = sum(1 for ts in self.packet_timestamps if ts >= window_start)
        self.packets_per_second = float(count)

        if len(self.packet_intervals) >= 5:
            arr = np.array(self.packet_intervals)[-10:]
            avg_interval = np.mean(arr)
            expected_interval = 1.0 / EXPECTED_PACKET_RATE
            timing_quality = max(0, 100 - abs(avg_interval - expected_interval) * 1000)
            drop_penalty = min(50, self.connection_drops * 10)
            quality = max(0, timing_quality - drop_penalty)
        else:
            quality = 50.0
        return quality

    # ===================== Data ingestion from Queue =====================
    def poll_serial_queue(self):
        """
        Drain any pending lines from the multiprocessing Queue and
        feed them into process_data().
        """
        while True:
            try:
                item = self.data_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, tuple):
                recv_time, line = item
            else:
                recv_time, line = time.time(), item

            if self.raw_log:
                self.raw_log.write(f"{recv_time:.3f}\t{line}\n")
                self._raw_lines += 1
                if self._raw_lines % 50 == 0:
                    self.raw_log.flush()

            if 'DATA:' not in line:
                self.handle_aux_line(line)
                continue

            telemetry_data, nav_data = self.parse_data_packet(line)
            if telemetry_data:
                tx_timestamp, values = telemetry_data
                self.process_data(tx_timestamp, values, nav_data, recv_time)
            else:
                self.bad_packets += 1

    def process_data(self, tx_timestamp, values, nav_data, recv_time=None):
        with self.data_lock:
            if self.start_time is None:
                self.start_time = tx_timestamp
                self.add_terminal_message("Data reception started")
            self.last_tx_tod = tx_timestamp

            # use the time the line actually arrived (stamped in the reader
            # process) so a busy GUI frame doesn't squash packet spacing
            now = recv_time if recv_time is not None else time.time()
            if self.start_wall_time is None:
                self.start_wall_time = now
            t = now - self.start_wall_time

            self.update_ground_reference(values[IDX_BARO_ALT])
            g_force, v_speed, alt_rate, sensor_used = self.calculate_derived_metrics(values, t)
            connection_quality = self.calculate_connection_quality(now)
            self.process_navigation_data(nav_data, tx_timestamp, values)

            rssi = nav_data.get('RSSI') if nav_data else None
            snr = nav_data.get('SNR') if nav_data else None
            self.current_rssi = rssi
            self.current_snr = snr

            self.vs_smooth = (1 - VS_SMOOTH_ALPHA) * self.vs_smooth + VS_SMOOTH_ALPHA * alt_rate

            if self.csv_writer:
                nav_row = [nav_data.get(label, '') if nav_data else '' for label in NAV_LABELS]
                direction = nav_data.get('DIR', '') if nav_data else ''
                turn_dir = nav_data.get('TURN', '') if nav_data else ''
                distance_km = self.current_distance / 1000 if self.current_distance > 0 else 0
                row = ([tx_timestamp] + values +
                       [g_force, v_speed, connection_quality, sensor_used] +
                       nav_row + [direction, turn_dir, distance_km] +
                       ['' if rssi is None else rssi, '' if snr is None else snr,
                        round(self.current_agl, 2), self.flight_phase, round(now, 3)])
                self.csv_writer.writerow(row)
                if (self.packet_count % 20) == 0:
                    self.log_file.flush()

            self.all_time.append(t)
            for i, label in enumerate(DATA_LABELS):
                self.all_data[label].append(values[i])

            self.g_force_magnitude.append(g_force)
            self.vertical_speed.append(v_speed)
            self.altitude_rate.append(alt_rate)
            self.connection_quality.append(connection_quality)
            self.g_sensor_used.append(sensor_used)

            self.agl_hist.append(self.current_agl)
            self.vs_smooth_hist.append(self.vs_smooth)
            self.rssi_hist.append(np.nan if rssi is None else rssi)
            self.snr_hist.append(np.nan if snr is None else snr)

            # long, low-rate history (peak-hold G so short spikes aren't lost)
            self._h_peak_g = max(self._h_peak_g, g_force)
            if t - self._h_last_t >= 1.0 / HISTORY_RATE_HZ:
                self._h_last_t = t
                self.h_t.append(t)
                self.h_agl.append(self.current_agl)
                self.h_vs.append(self.vs_smooth)
                self.h_g.append(self._h_peak_g)
                self._h_peak_g = 0.0

            lat, lon, alt = values[IDX_LAT], values[IDX_LON], values[IDX_ALT]
            if abs(lat) > 0.001 and abs(lon) > 0.001:
                self.last_rocket_fix = (lat, lon, alt, now)
                self.gps_lats.append(lat)
                self.gps_lons.append(lon)
                self.gps_alts.append(alt)

                # --- write rocket_state.json for web tracker ---
                if self.web_dir_ok:
                    try:
                        state = {
                            "rocket_lat": float(lat),
                            "rocket_lon": float(lon),
                            "alt": float(alt) if alt is not None else 0.0,
                            "updated": time.time()
                        }
                        out_path = os.path.join(WEB_DIR, "rocket_state.json")
                        tmp_path = out_path + ".tmp"
                        with open(tmp_path, "w") as f:
                            json.dump(state, f)
                        os.replace(tmp_path, out_path)   # atomic: tracker never reads half a file
                    except Exception as e:
                        self._state_write_errors += 1
                        if self._state_write_errors <= 3:
                            self.add_terminal_message(f"Rocket state write error: {e}")
                # --- end block ---

            # compact full-rate record for the flight report
            ok_fix = abs(lat) > 0.001 and abs(lon) > 0.001
            dist = self.distance_history[-1] if self.distance_history and self.distance_history[-1] > 0 else np.nan
            spd = values[IDX_SPEED]
            self.flight_rec.append((
                t, self.current_agl, self.vs_smooth, g_force,
                spd if spd >= 0 else np.nan,
                np.nan if rssi is None else rssi, np.nan if snr is None else snr,
                lat if ok_fix else np.nan, lon if ok_fix else np.nan,
                alt if (ok_fix and alt > -0.5) else np.nan,
                values[IDX_BARO_ALT], values[IDX_TEMP], values[IDX_BATTERY], dist))

            self.packet_count += 1

    # ===================== GUI: window + tabs =====================
    def setup_plots(self):
        pg.setConfigOptions(
            antialias=False,
            useOpenGL=USE_OPENGL,   # relies on system OpenGL config on the Pi
            background='k',
            foreground='w'
        )

        self.app = QtWidgets.QApplication.instance()
        if self.app is None:
            self.app = QtWidgets.QApplication(sys.argv)

        self.main_win = QtWidgets.QMainWindow()
        self.main_win.setWindowTitle("AURORA Insight – Mission Control")
        self.main_win.resize(1600, 900)
        self.main_win.setStyleSheet(f"""
            QMainWindow {{ background:{C_BG}; }}
            QWidget {{ color:{C_TEXT}; font-family:'{FONT_UI}'; }}
            QTabWidget::pane {{ border:none; background:{C_BG}; }}
            QTabBar {{ background:{C_BG}; }}
            QTabBar::tab {{ background:{C_PANEL}; color:{C_MUTED}; padding:7px 22px;
                            border:1px solid {C_BORDER}; border-bottom:none;
                            border-top-left-radius:4px; border-top-right-radius:4px;
                            margin-right:4px; }}
            QTabBar::tab:selected {{ background:{C_PANEL_HI}; color:{C_WHITE};
                                     border-top:2px solid {C_BRAND}; }}
            QPushButton {{ background:{C_PANEL_HI}; color:{C_TEXT}; border:1px solid {C_BORDER};
                           border-radius:4px; padding:5px 12px; font-weight:bold; }}
            QPushButton:hover {{ border-color:{C_BRAND}; }}
            QPushButton:pressed {{ background:{C_BRAND_DK}; }}
            QPushButton:checked {{ background:{C_BRAND}; color:{C_WHITE}; border-color:{C_BRAND}; }}
            QDoubleSpinBox {{ background:{C_BG}; color:{C_TEXT}; border:1px solid {C_BORDER};
                              border-radius:6px; padding:4px 6px; font-family:'{FONT_MONO}'; font-size:15px; }}
            QScrollBar:vertical {{ background:{C_BG}; width:8px; margin:0; }}
            QScrollBar::handle:vertical {{ background:{C_DIM}; border-radius:4px; min-height:20px; }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0; }}
            QPlainTextEdit {{ background:{C_BG}; border:1px solid {C_BORDER}; border-radius:6px;
                              font-family:'{FONT_MONO}'; font-size:12px; }}
            QToolTip {{ background:{C_PANEL_HI}; color:{C_TEXT}; border:1px solid {C_BORDER}; }}
        """)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        tab_font = QtGui.QFont(FONT_UI)
        tab_font.setPixelSize(13)
        tab_font.setWeight(QtGui.QFont.Weight.Bold)
        self.tabs.tabBar().setFont(tab_font)   # set via QFont so tab widths are measured correctly
        self.main_win.setCentralWidget(self.tabs)

        self.tabs.addTab(self.build_mission_tab(), "MISSION CONTROL")
        self.tabs.addTab(self.build_sensor_tab(), "SENSOR VIEW")

        # keyboard shortcuts
        Shortcut = getattr(QtGui, 'QShortcut', None) or getattr(QtWidgets, 'QShortcut')
        self._shortcuts = [
            Shortcut(QtGui.QKeySequence("F11"), self.main_win, activated=self.toggle_fullscreen),
            Shortcut(QtGui.QKeySequence("Ctrl+1"), self.main_win, activated=lambda: self.tabs.setCurrentIndex(0)),
            Shortcut(QtGui.QKeySequence("Ctrl+2"), self.main_win, activated=lambda: self.tabs.setCurrentIndex(1)),
            Shortcut(QtGui.QKeySequence("Ctrl+Q"), self.main_win, activated=self.main_win.close),
        ]

        if START_FULLSCREEN:
            self.main_win.showFullScreen()
        else:
            self.main_win.show()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(PLOT_UPDATE_INTERVAL)

        self.add_terminal_message("Enhanced receiver with GPS navigation running (PyQtGraph)...")
        self.add_terminal_message("Close window or press Ctrl+C to stop")

    def toggle_fullscreen(self):
        if self.main_win.isFullScreen():
            self.main_win.showNormal()
        else:
            self.main_win.showFullScreen()

    # ===================== Sensor View tab (original dashboard) =====================
    def build_sensor_tab(self):
        self.win = pg.GraphicsLayoutWidget(title="AURORA Telemetry (PyQtGraph)")
        self.win.setBackground('k')

        self.plot_items = {}
        self.curves = {}

        def add_plot(row, col, title, channels, pens=None):
            if pens is None:
                pens = ['y', 'c', 'm', 'r', 'g', 'b']
            p = self.win.addPlot(row=row, col=col, title=title)
            p.showGrid(x=True, y=True, alpha=0.3)

            p.setClipToView(True)
            p.setDownsampling(mode='peak')
            p.enableAutoRange(x=False, y=True)
            p.getViewBox().setAutoVisible(y=True)

            self.plot_items[title] = p
            for i, ch in enumerate(channels):
                c = p.plot(pen=pens[i % len(pens)], name=ch)
                self.curves.setdefault(ch, []).append(c)

        add_plot(0, 0, "Acceleration X (m/s²)", ['LG_Ax', 'HG_Ax'])
        add_plot(0, 1, "Acceleration Y (m/s²)", ['LG_Ay', 'HG_Ay'])
        add_plot(0, 2, "Acceleration Z (m/s²)", ['LG_Az', 'HG_Az'])
        add_plot(0, 3, "Gyroscope (°/s)", ['GyroX', 'GyroY', 'GyroZ'])

        add_plot(1, 0, "Orientation (deg)", ['Roll', 'Pitch', 'Yaw'])
        add_plot(1, 1, "G-Force (g)", ['G_Force'])
        add_plot(1, 2, "Altitude (m)", ['BaroAlt', 'GPSAlt'])
        add_plot(1, 3, "Vertical Speed (m/s)", ['Vertical_Speed'])

        add_plot(2, 0, "Pressure (Pa)", ['Pressure'])
        add_plot(2, 1, "Temperature (°C)", ['Temp'])
        add_plot(2, 2, "Distance to Rocket (m)", ['Distance'])
        add_plot(2, 3, "Bearing to Rocket (deg)", ['Bearing'])

        self.status_label = pg.LabelItem(justify='left')
        self.win.addItem(self.status_label, row=3, col=0, colspan=2)

        self.terminal_label = pg.LabelItem(justify='left')
        self.win.addItem(self.terminal_label, row=3, col=2, colspan=2)

        self.status_label.setText(
            "<span style='color:white; font-family:monospace; font-size:9pt;'>"
            "Waiting for telemetry...<br>Status will appear here."
            "</span>"
        )
        self.terminal_label.setText(
            "<span style='color:#00FF00; font-family:monospace; font-size:9pt;'>"
            "HUD will appear here...<br>Waiting for telemetry..."
            "</span>"
        )

        layout = self.win.ci.layout
        layout.setRowStretchFactor(0, 4)
        layout.setRowStretchFactor(1, 4)
        layout.setRowStretchFactor(2, 4)
        layout.setRowStretchFactor(3, 1)
        return self.win

    def update_sensor_view(self, update_hud):
        with self.data_lock:
            if not self.all_time:
                if update_hud:
                    self.status_label.setText(
                        "<span style='color:white; font-family:monospace; font-size:9pt;'>"
                        "Waiting for telemetry...<br><br>No data received yet."
                        "</span>"
                    )
                    if self.terminal_dirty:
                        self._cached_terminal_text = self._wrap_terminal_lines(list(self.terminal_messages))
                        self.terminal_dirty = False
                    term_html = (
                        "<span style='color:#00FF00; font-family:monospace; font-size:9pt;'>"
                        + html.escape(self._cached_terminal_text).replace("\n", "<br>")
                        + "</span>"
                    )
                    self.terminal_label.setText(term_html)
                return

            t = np.array(self.all_time, dtype=float)
            t_current = t[-1]
            t_start = max(0.0, t_current - TIME_WINDOW)
            mask = t >= t_start

            for p in self.plot_items.values():
                p.setXRange(t_start, t_current, padding=0)

            # Derived arrays
            arr_g = np.array(self.g_force_magnitude, dtype=float)
            arr_vs = np.array(self.vertical_speed, dtype=float)
            arr_dist = np.array(self.distance_history, dtype=float)
            arr_bear = np.array(self.bearing_history, dtype=float)
            arr_conn = np.array(self.connection_quality, dtype=float)

            channel_arrays = {}
            for label in DATA_LABELS:
                channel_arrays[label] = np.array(self.all_data[label], dtype=float)

            channel_arrays['G_Force'] = arr_g
            channel_arrays['Vertical_Speed'] = arr_vs
            channel_arrays['Distance'] = arr_dist
            channel_arrays['Bearing'] = arr_bear
            channel_arrays['Connection_Quality'] = arr_conn

            # Plot updates with manual decimation
            x_full = t[mask]
            for channel, curve_list in self.curves.items():
                arr = channel_arrays.get(channel)
                if arr is None or len(arr) != len(t):
                    continue

                y_full = arr[mask]

                x = x_full
                y = y_full
                if len(x_full) > MAX_DRAW_POINTS:
                    step = max(1, len(x_full) // MAX_DRAW_POINTS)
                    x = x_full[::step]
                    y = y_full[::step]

                for c in curve_list:
                    c.setData(x, y)   # no clear=True (cheaper)

            if not update_hud:
                return

            # HUD / terminal text (throttled)
            conn_quality = self.connection_quality[-1] if self.connection_quality else 0.0
            time_since_last = time.time() - self.last_packet_time if self.last_packet_time else 999.0

            battery_voltage = (
                self.all_data[DATA_LABELS[IDX_BATTERY]][-1]
                if self.all_data[DATA_LABELS[IDX_BATTERY]] else 0.0
            )
            current_g_force = self.g_force_magnitude[-1] if self.g_force_magnitude else 0.0
            current_alt = self.all_data['BaroAlt'][-1] if self.all_data['BaroAlt'] else 0.0
            g_sensor = self.g_sensor_used[-1] if self.g_sensor_used else "LG"

            if self.packet_intervals:
                last_interval = self.packet_intervals[-1]
                avg_interval = float(np.mean(self.packet_intervals))
            else:
                last_interval = 0.0
                avg_interval = 0.0

            status_conn = (
                "Connection:\n"
                f"Packets: {self.packet_count}\n"
                f"Rate: {self.packets_per_second:.1f} /s\n"
                f"Quality: {conn_quality:.1f} %\n"
                f"Avg dt: {avg_interval*1000:.1f} ms\n"
                f"Last dt: {last_interval*1000:.1f} ms\n"
                f"Drops: {self.connection_drops}\n"
                f"Last pkt: {time_since_last:.1f} s\n"
            )

            status_flight = (
                "Flight:\n"
                f"Phase: {self.flight_phase}\n"
                f"Time: {t_current:.1f} s\n"
                f"Alt: {current_alt:.1f} m (Max: {self.max_altitude:.1f} m)\n"
                f"G: {current_g_force:.2f} g ({g_sensor})\n"
                f"Bat: {battery_voltage:.2f} V\n"
                f"Max speed: {self.max_speed:.1f} km/h\n"
            )

            if self.gps_lats and self.gps_lons:
                lat = self.gps_lats[-1]
                lon = self.gps_lons[-1]
                gps_str = f"Rocket GPS: {lat:.6f}°, {lon:.6f}°"
            else:
                gps_str = "Rocket GPS: No Fix"

            if self.receiver_has_fix and self.nav_data['RX_LAT']:
                rx_lat = self.nav_data['RX_LAT'][-1]
                rx_lon = self.nav_data['RX_LON'][-1]
                rx_hdg = self.nav_data['RX_HDG'][-1]
                rx_str = f"RX GPS: {rx_lat:.6f}°, {rx_lon:.6f}°\nHdg: {rx_hdg:.1f}°"
            else:
                rx_str = "RX GPS: No Fix"

            if self.current_distance > 0:
                direction = self.nav_status['DIR'][-1] if self.nav_status['DIR'] else "UNKNOWN"
                turn_instr = self.nav_status['TURN'][-1] if self.nav_status['TURN'] else "UNKNOWN"
                turn_deg = self.nav_data['TURN_DEG'][-1] if self.nav_data['TURN_DEG'] else 0.0
                nav_str = (
                    "Navigation:\n"
                    f"Dist: {self.current_distance:.1f} m (Max: {self.max_distance:.1f} m)\n"
                    f"Bear: {self.current_bearing:.1f}° ({direction})\n"
                    f"Turn: {turn_instr} ({turn_deg:.1f}°)\n"
                )
            else:
                nav_str = "Navigation:\nWaiting for GPS fix..."

            left_block = (status_conn + "\n" + status_flight).replace("\n", "<br>")
            right_block = (gps_str + "\n" + rx_str + "\n\n" + nav_str).replace("\n", "<br>")

            status_html = (
                "<span style='color:white; font-family:monospace; font-size:9pt;'>"
                "<table><tr>"
                "<td style='vertical-align:top; padding-right:20px;'>"
                + left_block +
                "</td>"
                "<td style='vertical-align:top;'>"
                + right_block +
                "</td>"
                "</tr></table>"
                "</span>"
            )
            self.status_label.setText(status_html)

            vspd_val = self.vertical_speed[-1] if self.vertical_speed else 0.0
            hud = (
                f"t={t_current:.1f}s | Alt={current_alt:.1f}m | G={current_g_force:.2f}g | "
                f"Vspd={vspd_val:.2f}m/s | Dist={self.current_distance:.1f}m | "
                f"Bear={self.current_bearing:.1f}° | Bat={battery_voltage:.2f}V"
            )

            if self.terminal_dirty:
                self._cached_terminal_text = self._wrap_terminal_lines(list(self.terminal_messages))
                self.terminal_dirty = False

            terminal_html = (
                "<span style='color:#00FF00; font-family:monospace; font-size:9pt;'>"
                + hud.replace("\n", "<br>") + "<br>"
                + html.escape(self._cached_terminal_text).replace("\n", "<br>")
                + "</span>"
            )
            self.terminal_label.setText(terminal_html)

    def _wrap_terminal_lines(self, lines):
        wrapped = []
        for ln in lines[-TERMINAL_MAX_LINES:]:
            for seg in textwrap.wrap(
                ln,
                width=TERMINAL_MAX_CHARS,
                replace_whitespace=False,
                drop_whitespace=False
            ):
                wrapped.append(seg)
                if len(wrapped) >= TERMINAL_MAX_LINES:
                    break
            if len(wrapped) >= TERMINAL_MAX_LINES:
                break
        return '\n'.join(wrapped)

    # ===================== Mission Control tab =====================
    def _make_plot(self, title, units, color):
        frame = Panel(title, margins=(6, 6, 6, 4))
        pw = pg.PlotWidget(background=C_PANEL)
        pi = pw.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.18)
        pi.setMenuEnabled(False)
        pi.hideButtons()
        pi.setMouseEnabled(x=False, y=False)
        pi.enableAutoRange(x=False, y=False)   # y range set manually with a minimum span
        pi.setClipToView(True)
        for name in ('left', 'bottom'):
            ax = pi.getAxis(name)
            ax.setPen(pg.mkPen(C_BORDER))
            ax.setTextPen(pg.mkPen(C_MUTED))
            f = QtGui.QFont(FONT_UI)
            f.setPixelSize(11)
            ax.setStyle(tickFont=f, tickLength=-4)
        pi.getAxis('left').setWidth(46)
        pi.setLabel('left', f"<span style='color:{C_DIM}'>{units}</span>")
        frame.body.addWidget(pw, 1)
        frame.plot = pi
        frame.color = color
        return frame

    def build_mission_tab(self):
        root = QtWidgets.QWidget()
        root.setObjectName("mcroot")
        root.setStyleSheet(f"QWidget#mcroot {{ background:{C_BG}; }} QLabel {{ background:transparent; }}")
        v = QtWidgets.QVBoxLayout(root)
        v.setContentsMargins(10, 8, 10, 10)
        v.setSpacing(8)
        mc = self.mc

        # ---------- header ----------
        header = QtWidgets.QFrame()
        header.setObjectName("hdr")
        header.setStyleSheet(
            f"QFrame#hdr {{ background:{C_PANEL}; border:1px solid {C_BORDER}; border-radius:6px;"
            f" border-bottom:2px solid {C_BRAND}; }}"
            f" QLabel {{ border:none; background:transparent; }}"
        )
        h = QtWidgets.QHBoxLayout(header)
        h.setContentsMargins(14, 6, 14, 6)
        h.setSpacing(18)

        mc['title'] = AuroraTitle()
        h.addWidget(mc['title'])

        mc['phase'] = QtWidgets.QLabel("GROUND")
        pf = QtGui.QFont(FONT_UI)
        pf.setPixelSize(22)
        pf.setWeight(QtGui.QFont.Weight.Black)
        pf.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 4)
        mc['phase'].setFont(pf)
        mc['phase'].setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        mc['phase'].setMinimumWidth(190)
        mc['phase_color'] = None
        h.addWidget(mc['phase'])

        clock_box = QtWidgets.QVBoxLayout()
        clock_box.setSpacing(0)
        mc['clock_lbl'] = QtWidgets.QLabel("SESSION")
        mc['clock_lbl'].setStyleSheet(f"color:{C_MUTED}; font-size:11px; font-weight:bold; letter-spacing:2px;")
        mc['clock'] = QtWidgets.QLabel("00:00.0")
        cf = QtGui.QFont(FONT_MONO)
        cf.setPixelSize(28)
        cf.setWeight(QtGui.QFont.Weight.Bold)
        mc['clock'].setFont(cf)
        clock_box.addWidget(mc['clock_lbl'])
        clock_box.addWidget(mc['clock'])
        h.addLayout(clock_box)

        h.addStretch(1)

        utc_box = QtWidgets.QVBoxLayout()
        utc_box.setSpacing(0)
        l = QtWidgets.QLabel("ROCKET GPS TIME (UTC)")
        l.setStyleSheet(f"color:{C_MUTED}; font-size:11px; font-weight:bold; letter-spacing:2px;")
        mc['utc'] = QtWidgets.QLabel("--:--:--")
        uf = QtGui.QFont(FONT_MONO)
        uf.setPixelSize(18)
        mc['utc'].setFont(uf)
        mc['utc'].setStyleSheet(f"color:{C_TEXT};")
        utc_box.addWidget(l)
        utc_box.addWidget(mc['utc'])
        h.addLayout(utc_box)

        mc['rec'] = QtWidgets.QLabel("")
        mc['rec'].setStyleSheet(f"color:{C_BRAND}; font-weight:bold; font-size:14px; letter-spacing:2px;")
        h.addWidget(mc['rec'])

        link_box = QtWidgets.QHBoxLayout()
        link_box.setSpacing(6)
        mc['led'] = StatusLed(18)
        link_box.addWidget(mc['led'])
        link_txt = QtWidgets.QVBoxLayout()
        link_txt.setSpacing(0)
        mc['link'] = QtWidgets.QLabel("WAITING")
        lf = QtGui.QFont(FONT_UI)
        lf.setPixelSize(18)
        lf.setWeight(QtGui.QFont.Weight.Black)
        lf.setLetterSpacing(QtGui.QFont.SpacingType.AbsoluteSpacing, 2)
        mc['link'].setFont(lf)
        mc['link_sub'] = QtWidgets.QLabel("no packets yet")
        mc['link_sub'].setStyleSheet(f"color:{C_MUTED}; font-size:12px;")
        link_txt.addWidget(mc['link'])
        link_txt.addWidget(mc['link_sub'])
        link_box.addLayout(link_txt)
        h.addLayout(link_box)
        v.addWidget(header)

        # ---------- big tiles ----------
        tiles = QtWidgets.QHBoxLayout()
        tiles.setSpacing(8)
        mc['t_alt'] = MetricTile("Altitude AGL", "m", C_BRAND)
        mc['t_vs'] = MetricTile("Vertical Speed", "m/s", C_BRAND)
        mc['t_g'] = MetricTile("G-Force", "g", C_BRAND)
        mc['t_spd'] = MetricTile("GPS Speed", "km/h", C_BRAND)
        mc['t_dist'] = MetricTile("Range", "m", C_BRAND)
        mc['t_bat'] = MetricTile("Battery", "V", C_BRAND)
        mc['t_sig'] = MetricTile("Signal", "dBm", C_BRAND)
        for key, stretch in (('t_alt', 5), ('t_vs', 4), ('t_g', 4), ('t_spd', 4),
                             ('t_dist', 4), ('t_bat', 3), ('t_sig', 3)):
            tiles.addWidget(mc[key], stretch)
        v.addLayout(tiles, 0)

        # ---------- middle: plots + side column ----------
        mid = QtWidgets.QHBoxLayout()
        mid.setSpacing(8)

        plots_col = QtWidgets.QVBoxLayout()
        plots_col.setSpacing(8)

        # altitude plot with window selector
        alt_frame = self._make_plot("Altitude above pad", "m", C_BRAND)
        win_group = QtWidgets.QButtonGroup(alt_frame)
        win_group.setExclusive(True)
        for label, secs in (("15 s", 15.0), ("60 s", 60.0), ("5 min", 300.0), ("FLIGHT", None)):
            b = QtWidgets.QPushButton(label)
            b.setCheckable(True)
            b.setFixedHeight(24)
            b.setStyleSheet(
                f"QPushButton {{ padding:2px 10px; font-size:11px; background:{C_PANEL_HI}; color:{C_MUTED};"
                f" border:1px solid {C_BORDER}; border-radius:4px; }}"
                f" QPushButton:checked {{ background:{C_BRAND}; color:{C_WHITE}; border-color:{C_BRAND}; }}")
            if secs == self.mc_window:
                b.setChecked(True)
            b.clicked.connect(lambda _=False, s=secs: setattr(self, 'mc_window', s))
            win_group.addButton(b)
            alt_frame.header.addWidget(b)
        mc['win_group'] = win_group
        api = alt_frame.plot
        mc['c_alt'] = api.plot(pen=pg.mkPen(C_BRAND, width=2.5),
                               fillLevel=0, brush=qcolor(C_BRAND, 45))
        mc['c_alt_now'] = pg.ScatterPlotItem(size=11, brush=pg.mkBrush(C_WHITE),
                                             pen=pg.mkPen(C_BRAND, width=2.5))
        api.addItem(mc['c_alt_now'])
        mc['apogee_line'] = pg.InfiniteLine(
            angle=0, movable=False, pen=pg.mkPen(C_WHITE, width=1.2, style=QtCore.Qt.PenStyle.DashLine),
            label='APOGEE {value:.0f} m',
            labelOpts={'position': 0.06, 'color': C_WHITE, 'fill': qcolor(C_BG, 200),
                       'anchors': [(0, 0), (0, 0)]})
        mc['apogee_line'].setVisible(False)
        api.addItem(mc['apogee_line'])
        mc['p_alt'] = api
        plots_col.addWidget(alt_frame, 5)

        lower = QtWidgets.QHBoxLayout()
        lower.setSpacing(8)
        vs_frame = self._make_plot("Vertical speed", "m/s", C_WHITE)
        vpi = vs_frame.plot
        vpi.addItem(pg.InfiniteLine(angle=0, pos=0, pen=pg.mkPen(C_DIM, width=1)))
        mc['c_vs'] = vpi.plot(pen=pg.mkPen(C_WHITE, width=2), fillLevel=0, brush=qcolor(C_WHITE, 28))
        mc['p_vs'] = vpi
        lower.addWidget(vs_frame, 1)

        g_frame = self._make_plot("G-force", "g", C_BRAND)
        gpi = g_frame.plot
        gpi.addItem(pg.InfiniteLine(angle=0, pos=1.0, pen=pg.mkPen(C_DIM, width=1,
                                                                   style=QtCore.Qt.PenStyle.DotLine)))
        mc['c_g'] = gpi.plot(pen=pg.mkPen(C_BRAND, width=2), fillLevel=0, brush=qcolor(C_BRAND, 45))
        mc['p_g'] = gpi
        lower.addWidget(g_frame, 1)

        plots_col.addLayout(lower, 4)

        mid.addLayout(plots_col, 7)

        side = QtWidgets.QVBoxLayout()
        side.setSpacing(8)
        side.setContentsMargins(0, 0, 0, 0)

        # --- phone tracker QR ---
        qr_panel = Panel("Phone tracker")
        qr_row = QtWidgets.QHBoxLayout()
        qr_row.setSpacing(10)
        mc['qr'] = QrCodeWidget()
        mc['qr'].setFixedSize(132, 132)
        mc['qr'].clicked.connect(self.show_big_qr)
        qr_row.addWidget(mc['qr'], 0, QtCore.Qt.AlignmentFlag.AlignTop)
        qr_info = QtWidgets.QVBoxLayout()
        qr_info.setSpacing(4)
        mc['qr_status'] = QtWidgets.QLabel("starting...")
        sf = QtGui.QFont(FONT_UI)
        sf.setPixelSize(16)
        sf.setWeight(QtGui.QFont.Weight.Black)
        mc['qr_status'].setFont(sf)
        mc['qr_status'].setWordWrap(True)
        mc['qr_url'] = QtWidgets.QLabel("")
        mc['qr_url'].setWordWrap(True)
        mc['qr_url'].setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        mc['qr_url'].setStyleSheet(f"color:{C_TEXT}; font-family:'{FONT_MONO}'; font-size:12px; border:none;")
        mc['qr_hint'] = QtWidgets.QLabel("")
        mc['qr_hint'].setWordWrap(True)
        mc['qr_hint'].setStyleSheet(f"color:{C_MUTED}; font-size:12px; border:none;")
        qr_info.addWidget(mc['qr_status'])
        qr_info.addWidget(mc['qr_url'])
        qr_info.addWidget(mc['qr_hint'])
        qr_info.addStretch(1)
        qr_row.addLayout(qr_info, 1)
        qr_panel.body.addLayout(qr_row)
        side.addWidget(qr_panel, 0)

        # --- rocket GPS coordinates ---
        gps_panel = Panel("Rocket GPS")
        mc['gps_chip'] = QtWidgets.QLabel("NO FIX")
        mc['gps_chip_state'] = None
        mc['gps_chip'].setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        gps_panel.header.addWidget(mc['gps_chip'])
        mc['gps_copy'] = QtWidgets.QPushButton("COPY")
        mc['gps_copy'].setFixedHeight(24)
        mc['gps_copy'].setToolTip("Copy 'lat, lon' - paste straight into Google Maps / Apple Maps")
        mc['gps_copy'].clicked.connect(self.copy_rocket_coords)
        gps_panel.header.addWidget(mc['gps_copy'])
        mc['gps_earth'] = QtWidgets.QPushButton("EARTH")
        mc['gps_earth'].setFixedHeight(24)
        mc['gps_earth'].setToolTip("Save a Google Earth file (KMZ) of the landing spot and flight path")
        mc['gps_earth'].clicked.connect(lambda: self.export_google_earth(open_after=True))
        gps_panel.header.addWidget(mc['gps_earth'])

        coords = QtWidgets.QGridLayout()
        coords.setHorizontalSpacing(12)
        coords.setVerticalSpacing(0)
        cf = QtGui.QFont(FONT_MONO)
        cf.setPixelSize(26)
        cf.setWeight(QtGui.QFont.Weight.Bold)
        for r, key in enumerate(("LAT", "LON")):
            k = QtWidgets.QLabel(key)
            k.setStyleSheet(f"color:{C_BRAND}; font-size:13px; font-weight:bold; letter-spacing:2px; border:none;")
            cv = QtWidgets.QLabel("--")
            cv.setFont(cf)
            cv.setStyleSheet(f"color:{C_WHITE}; border:none;")
            cv.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            coords.addWidget(k, r, 0, QtCore.Qt.AlignmentFlag.AlignVCenter)
            coords.addWidget(cv, r, 1)
            mc['gps_' + key.lower()] = cv
        coords.setColumnStretch(1, 1)
        gps_panel.body.addLayout(coords)
        mc['gps_dms'] = QtWidgets.QLabel("")
        mc['gps_dms'].setStyleSheet(f"color:{C_MUTED}; font-family:'{FONT_MONO}'; font-size:12px; border:none;")
        gps_panel.body.addWidget(mc['gps_dms'])

        line = QtWidgets.QFrame()
        line.setFixedHeight(1)
        line.setStyleSheet(f"background:{C_BORDER}; border:none;")
        gps_panel.body.addWidget(line)

        stats = QtWidgets.QGridLayout()
        stats.setHorizontalSpacing(14)
        stats.setVerticalSpacing(3)
        mc['gps_vals'] = {}
        items = [("alt", "GPS ALTITUDE", 0, 0, 1), ("age", "UPDATED", 0, 1, 1),
                 ("you", "FROM YOU", 1, 0, 1), ("pad", "FROM PAD", 1, 1, 1),
                 ("rx", "YOUR POSITION", 2, 0, 2), ("landed", "LANDED AT", 3, 0, 2)]
        vf = QtGui.QFont(FONT_MONO)
        vf.setPixelSize(15)
        vf.setWeight(QtGui.QFont.Weight.Bold)
        for key, name, r, c, span in items:
            box = QtWidgets.QVBoxLayout()
            box.setSpacing(0)
            nl = QtWidgets.QLabel(name)
            nl.setStyleSheet(f"color:{C_MUTED}; font-size:10px; font-weight:bold; letter-spacing:1px; border:none;")
            vl = QtWidgets.QLabel("--")
            vl.setFont(vf)
            vl.setStyleSheet(f"color:{C_TEXT}; border:none;")
            vl.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
            box.addWidget(nl)
            box.addWidget(vl)
            stats.addLayout(box, r, c, 1, span)
            mc['gps_vals'][key] = vl
        stats.setColumnStretch(0, 1)
        stats.setColumnStretch(1, 1)
        gps_panel.body.addLayout(stats)
        gps_panel.body.addStretch(1)
        side.addWidget(gps_panel, 1)

        side_w = QtWidgets.QWidget()
        side_w.setLayout(side)
        side_w.setMinimumWidth(340)
        side_w.setMaximumWidth(460)
        mid.addWidget(side_w, 3)

        v.addLayout(mid, 1)

        # ---------- bottom strip ----------
        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(8)

        log_panel = Panel("Mission log")
        mc['log'] = QtWidgets.QPlainTextEdit()
        mc['log'].setReadOnly(True)
        mc['log'].setMaximumBlockCount(400)
        mc['log'].setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        log_panel.body.addWidget(mc['log'], 1)
        bottom.addWidget(log_panel, 5)

        rec_panel = Panel("Flight records")
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(2)
        mc['rec_vals'] = {}
        rec_items = [
            ("apogee", "APOGEE", C_BRAND), ("maxg", "MAX G", C_TEXT),
            ("maxspd", "MAX SPEED", C_TEXT), ("maxdist", "MAX RANGE", C_TEXT),
            ("ftime", "FLIGHT TIME", C_TEXT), ("packets", "PACKETS", C_TEXT),
            ("drops", "DROPS / MAX GAP", C_TEXT), ("quality", "LINK QUALITY", C_TEXT),
        ]
        for i, (key, name, col) in enumerate(rec_items):
            r, c = divmod(i, 3)
            nl = QtWidgets.QLabel(name)
            nl.setStyleSheet(f"color:{C_MUTED}; font-size:10px; font-weight:bold; letter-spacing:1px; border:none;")
            vl = QtWidgets.QLabel("--")
            vf = QtGui.QFont(FONT_MONO)
            vf.setPixelSize(17)
            vf.setWeight(QtGui.QFont.Weight.Bold)
            vl.setFont(vf)
            vl.setStyleSheet(f"color:{col}; border:none;")
            box = QtWidgets.QVBoxLayout()
            box.setSpacing(0)
            box.addWidget(nl)
            box.addWidget(vl)
            grid.addLayout(box, r, c)
            mc['rec_vals'][key] = vl
        rec_panel.body.addLayout(grid)
        rec_panel.body.addStretch(1)
        btns = QtWidgets.QHBoxLayout()
        b_zero = QtWidgets.QPushButton("ZERO ALT")
        b_zero.setToolTip("Use the current barometric altitude as 0 m AGL (do this on the pad)")
        b_zero.clicked.connect(self.zero_altitude)
        b_reset = QtWidgets.QPushButton("RESET RECORDS")
        b_reset.setToolTip("Clear max values, mission clock and flight phase")
        b_reset.clicked.connect(self.reset_records)
        b_report = QtWidgets.QPushButton("FLIGHT REPORT")
        b_report.setToolTip("Build a PDF + HTML flight report now and open it")
        b_report.setStyleSheet(f"QPushButton {{ background:{C_BRAND}; color:{C_WHITE}; border-color:{C_BRAND}; }}"
                               f" QPushButton:pressed {{ background:{C_BRAND_DK}; }}")
        b_report.clicked.connect(lambda: self.generate_flight_report(open_after=True))
        self.mc['report_btn'] = b_report
        btns.addWidget(b_zero)
        btns.addWidget(b_reset)
        btns.addWidget(b_report)
        rec_panel.body.addLayout(btns)
        bottom.addWidget(rec_panel, 4)

        radio_panel = Panel("Receiver radio")
        mc['freq_now'] = QtWidgets.QLabel("-- MHz")
        ff = QtGui.QFont(FONT_MONO)
        ff.setPixelSize(22)
        ff.setWeight(QtGui.QFont.Weight.Bold)
        mc['freq_now'].setFont(ff)
        mc['freq_now'].setStyleSheet(f"color:{C_WHITE}; border:none;")
        radio_panel.body.addWidget(mc['freq_now'])
        mc['freq_note'] = QtWidgets.QLabel("Must match the transmitter (TX is 915.0)")
        mc['freq_note'].setWordWrap(True)
        mc['freq_note'].setStyleSheet(f"color:{C_MUTED}; font-size:11px; border:none;")
        radio_panel.body.addWidget(mc['freq_note'])
        mc['freq_spin'] = QtWidgets.QDoubleSpinBox()
        mc['freq_spin'].setRange(902.0, 928.0)
        mc['freq_spin'].setDecimals(3)
        mc['freq_spin'].setSingleStep(0.1)
        mc['freq_spin'].setValue(DEFAULT_FREQ_MHZ)
        mc['freq_spin'].setSuffix(" MHz")
        radio_panel.body.addWidget(mc['freq_spin'])
        rb = QtWidgets.QHBoxLayout()
        b_set = QtWidgets.QPushButton("SET")
        b_set.clicked.connect(lambda: self.send_receiver_command(f"FREQ {mc['freq_spin'].value():.3f}"))
        b_q = QtWidgets.QPushButton("QUERY")
        b_q.clicked.connect(lambda: self.send_receiver_command("FREQ?"))
        rb.addWidget(b_set)
        rb.addWidget(b_q)
        radio_panel.body.addLayout(rb)
        radio_panel.body.addStretch(1)
        bottom.addWidget(radio_panel, 2)

        bottom_w = QtWidgets.QWidget()
        bottom_w.setLayout(bottom)
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom_w.setFixedHeight(178)
        v.addWidget(bottom_w)

        return root

    def _log_color(self, msg):
        m = msg.lower()
        if any(k in m for k in ('apogee', 'liftoff', 'touchdown', 'phase')):
            return C_BRAND
        if any(k in m for k in ('lost', 'error', 'fail', 'err', 'drop')):
            return C_AMBER
        if any(k in m for k in ('acquired', 'max', 'passing', 'frequency')):
            return C_TEXT
        return C_MUTED

    def _flush_log_view(self):
        log = self.mc['log']
        new = [e for e in self.log_events if e[0] > self._shown_log_seq]
        if not new:
            return
        for seq, ts, msg in new:
            col = self._log_color(msg)
            log.appendHtml(
                f"<span style='color:{C_DIM}'>{ts}</span>&nbsp; "
                f"<span style='color:{col}'>{html.escape(msg)}</span>")
        self._shown_log_seq = new[-1][0]
        sb = log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _set_phase_badge(self, phase):
        mc = self.mc
        bg, fg = PHASE_STYLE.get(phase, ('#1c1c1c', C_TEXT))
        if mc['phase_color'] != phase:
            mc['phase_color'] = phase
            mc['phase'].setText(phase.upper())
            mc['phase'].setStyleSheet(
                f"color:{fg}; background:{bg}; border-radius:4px; padding:4px 16px;")

    def _update_link_header(self, now):
        mc = self.mc
        if self.last_packet_time is None:
            mc['led'].set_state(C_DIM)
            mc['link'].setText("WAITING")
            mc['link'].setStyleSheet(f"color:{C_MUTED};")
            src = "simulated flight (demo)" if self.demo else f"{SERIAL_PORT} @ {BAUD_RATE}"
            mc['link_sub'].setText(f"no packets yet · {src}")
            return
        age = now - self.last_packet_time
        q = self.connection_quality[-1] if self.connection_quality else 0.0
        blink = int(now * 3) % 2 == 0
        if age < LINK_STALE_S:
            col, txt, on = C_BRAND, "LIVE", True
        elif age < LINK_LOST_S:
            col, txt, on = C_AMBER, "WEAK", True
        else:
            col, txt, on = C_AMBER, "LOST", blink
        mc['led'].set_state(col, on)
        mc['link'].setText(txt)
        mc['link'].setStyleSheet(f"color:{C_WHITE if txt == 'LIVE' else col};")
        if age >= LINK_STALE_S:
            mc['link_sub'].setText(f"last packet {age:.1f} s ago")
        else:
            mc['link_sub'].setText(f"{self.packets_per_second:.0f} pkt/s · Q {q:.0f}% · {self.packet_count} pkts")

    def _update_tracker_qr(self, now):
        mc = self.mc
        if now - self._ip_checked >= IP_REFRESH_S or self.local_ip is None:
            self._ip_checked = now
            ip = self.get_local_ip()
            if ip != self.local_ip:
                if self.local_ip is not None:
                    self.add_terminal_message(f"Network address changed: {ip}  (QR updated)")
                self.local_ip = ip
        ip = self.local_ip
        url = self.tracker_url(ip)
        no_net = ip.startswith("127.") or ip == "0.0.0.0"
        wt = self.web_thread
        if wt is None:
            status, col, hint, dim = "NOT SERVED", C_RED, f"Tracker folder not found:\n{WEB_DIR}", True
        elif wt.status == "error":
            status, col, hint, dim = "SERVER ERROR", C_RED, wt.error, True
        elif no_net:
            status, col, hint, dim = "NO NETWORK", C_AMBER, "Connect the Pi to Wi-Fi or a hotspot", True
        elif wt.status == "running":
            status, col, dim = "SCAN TO TRACK", C_WHITE, False
            hint = ("Phone on the same Wi-Fi.\nAccept the certificate warning.\n"
                    "Click the code for the Google Earth file too.")
        else:
            status, col, hint, dim = "STARTING...", C_AMBER, "", True
        mc['qr'].set_text(url, dimmed=dim)
        if mc['qr_status'].text() != status:
            mc['qr_status'].setText(status)
            mc['qr_status'].setStyleSheet(f"color:{col}; border:none;")
        mc['qr_url'].setText(url)
        mc['qr_hint'].setText(hint)
        if self._big_qr is not None:
            self._refresh_big_qr()

    def show_big_qr(self):
        """Full-size QR codes: live tracker page + Google Earth file. Click a code to close."""
        if self._big_qr is None:
            dlg = QtWidgets.QDialog(self.main_win)
            dlg.setWindowTitle("Phone tracker")
            dlg.setStyleSheet("background:#ffffff;")
            lay = QtWidgets.QVBoxLayout(dlg)
            lay.setContentsMargins(24, 20, 24, 20)
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(32)
            cols = []
            for heading in ("LIVE TRACKER", "GOOGLE EARTH FILE"):
                col = QtWidgets.QVBoxLayout()
                t = QtWidgets.QLabel(heading)
                tf = QtGui.QFont(FONT_UI)
                tf.setPixelSize(22)
                tf.setWeight(QtGui.QFont.Weight.Black)
                t.setFont(tf)
                t.setStyleSheet("color:#000000;")
                t.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                qr = QrCodeWidget()
                qr.setMinimumSize(380, 380)
                qr.clicked.connect(dlg.close)
                url = QtWidgets.QLabel("")
                url.setWordWrap(True)
                url.setStyleSheet(f"color:#000000; font-family:'{FONT_MONO}'; font-size:14px;")
                url.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                col.addWidget(t)
                col.addWidget(qr, 1)
                col.addWidget(url)
                row.addLayout(col)
                cols.append((qr, url))
            lay.addLayout(row, 1)
            hint = QtWidgets.QLabel("Same Wi-Fi as the Pi  \u00B7  accept the certificate warning  \u00B7  "
                                    "open the .kmz with Google Earth  \u00B7  click a code to close")
            hint.setStyleSheet("color:#333333; font-size:14px;")
            hint.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            lay.addWidget(hint)
            self._big_qr = dlg
            (self._big_qr_widget, self._big_qr_label), (self._big_kmz_widget, self._big_kmz_label) = cols
        self._refresh_big_qr()
        self._big_qr.resize(900, 560)
        self._big_qr.show()
        self._big_qr.raise_()

    def _refresh_big_qr(self):
        url = self.tracker_url(self.local_ip)
        self._big_qr_widget.set_text(url, dimmed=self.mc['qr'].dimmed)
        self._big_qr_label.setText(url)
        kurl = self.kmz_url(self.local_ip)
        ready = self._web_serving() and os.path.exists(os.path.join(WEB_DIR, TRACKER_KMZ))
        self._big_kmz_widget.set_text(kurl, dimmed=not ready)
        self._big_kmz_label.setText(kurl if ready else kurl + "\n(available once the rocket has a GPS fix)")

    def report_dir(self):
        if self.log_filename:
            return os.path.dirname(self.log_filename)
        return self.session_dir

    def generate_flight_report(self, open_after=False):
        self._report_due = None
        self.add_terminal_message("Building flight report...")
        if self.app is not None:
            self.app.processEvents()
        try:
            pdf, html_path = FlightReportBuilder(self, self.report_dir()).build()
        except Exception as e:
            self.add_terminal_message(f"Flight report error: {e}")
            import traceback
            traceback.print_exc()
            return None
        self.report_paths = (pdf, html_path)
        self.add_terminal_message(f"Flight report saved: {pdf}")
        if 'report_btn' in self.mc:
            self.mc['report_btn'].setToolTip(f"Last report:\n{pdf}")
        if open_after:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(pdf))
        return pdf

    def _web_serving(self):
        return self.web_thread is not None and self.web_thread.status == "running"

    def kmz_url(self, ip=None):
        return f"https://{ip or self.get_local_ip()}:{TRACKER_PORT}/{TRACKER_KMZ}"

    def export_google_earth(self, open_after=False):
        """Final KMZ for Google Earth: session folder + served to phones."""
        if self.last_rocket_fix is None and not self.flight_rec:
            self.add_terminal_message("Google Earth: no rocket GPS data yet")
            return None
        try:
            kml = build_kml(self)
            out_dir = self.report_dir()
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, f"RocketTrack_{self.session_stamp}.kmz")
            write_kmz(path, kml)
            if self._web_serving():
                write_kmz(os.path.join(WEB_DIR, TRACKER_KMZ), kml)
        except Exception as e:
            self.add_terminal_message(f"Google Earth export error: {e}")
            import traceback
            traceback.print_exc()
            return None
        self.kmz_path = path
        self.add_terminal_message(f"Google Earth file saved: {path}")
        if self._web_serving():
            self.add_terminal_message(f"  phones: {self.kmz_url(self.local_ip)}")
        if 'gps_earth' in self.mc:
            self.mc['gps_earth'].setToolTip(f"Last export:\n{path}")
        if open_after:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(path))
        return path

    def update_live_kml(self):
        """Rewrite the live Google Earth file every few seconds while we have rocket GPS."""
        now = time.time()
        if self.last_rocket_fix is None or now - self._live_kml_t < LIVE_KML_EVERY_S:
            return
        self._live_kml_t = now
        try:
            kml = build_kml(self, live=True)
            if SAVE_TO_FILE:
                os.makedirs(self.session_dir, exist_ok=True)
                live = os.path.join(self.session_dir, "rocket_live.kml")
                write_text_atomic(live, kml)
                if not self._live_link_written:
                    # open THIS in Google Earth Pro: it re-reads rocket_live.kml every few seconds
                    link = os.path.join(self.session_dir, "GoogleEarth_LIVE.kml")
                    href = QtCore.QUrl.fromLocalFile(os.path.abspath(live)).toString()
                    write_text_atomic(link, (
                        '<?xml version="1.0" encoding="UTF-8"?>\n'
                        '<kml xmlns="http://www.opengis.net/kml/2.2"><NetworkLink>'
                        '<name>AURORA live rocket track</name><open>1</open>'
                        f'<Link><href>{html.escape(href)}</href><refreshMode>onInterval</refreshMode>'
                        f'<refreshInterval>{LIVE_KML_EVERY_S:.0f}</refreshInterval></Link>'
                        '</NetworkLink></kml>\n'))
                    self._live_link_written = True
                    self.add_terminal_message(f"Live Google Earth link: {link}")
            if self._web_serving():
                write_kmz(os.path.join(WEB_DIR, TRACKER_KMZ), kml)
        except Exception as e:
            self.add_terminal_message(f"Live KML error: {e}")
            self._live_kml_t = now + 30   # back off

    def copy_rocket_coords(self):
        fix = self.last_rocket_fix
        if fix is None:
            return
        text = f"{fix[0]:.6f}, {fix[1]:.6f}"
        QtWidgets.QApplication.clipboard().setText(text)
        self.add_terminal_message(f"Copied rocket position: {text}")
        btn = self.mc['gps_copy']
        btn.setText("COPIED \u2713")
        QtCore.QTimer.singleShot(1500, lambda: btn.setText("COPY"))

    def _set_gps_chip(self, state):
        mc = self.mc
        if mc['gps_chip_state'] == state:
            return
        mc['gps_chip_state'] = state
        text, bg, fg = {
            'live': ("\u25CF LIVE FIX", C_BRAND, C_WHITE),
            'stale': ("LAST KNOWN", C_AMBER, '#000000'),
            'none': ("NO FIX", '#1c1c1c', C_MUTED),
        }[state]
        mc['gps_chip'].setText(text)
        mc['gps_chip'].setStyleSheet(
            f"color:{fg}; background:{bg}; border-radius:3px; padding:2px 8px;"
            f" font-size:11px; font-weight:bold; letter-spacing:1px;")

    def _update_gps_panel(self, now, nav_ok, last_dist, dir_status):
        mc = self.mc
        gv = mc['gps_vals']
        fix = self.last_rocket_fix

        if fix is None:
            self._set_gps_chip('none')
            mc['gps_lat'].setText("--")
            mc['gps_lon'].setText("--")
            mc['gps_dms'].setText("waiting for rocket GPS fix")
            for key in ('alt', 'age', 'you', 'pad'):
                gv[key].setText("--")
        else:
            lat, lon, alt, t_fix = fix
            age = now - t_fix
            stale = age >= GPS_STALE_S
            self._set_gps_chip('stale' if stale else 'live')
            mc['gps_lat'].setText(f"{abs(lat):.6f}\u00B0{'N' if lat >= 0 else 'S'}")
            mc['gps_lon'].setText(f"{abs(lon):.6f}\u00B0{'E' if lon >= 0 else 'W'}")
            mc['gps_dms'].setText(f"{fmt_dms(lat, 'N', 'S')}   {fmt_dms(lon, 'E', 'W')}")
            gv['alt'].setText(f"{alt:,.0f} m MSL" if alt > -0.5 else "--")
            gv['age'].setText(fmt_age(age))
            gv['age'].setStyleSheet(f"color:{C_AMBER if stale else C_TEXT}; border:none;")
            if nav_ok:
                dv, du = fmt_dist(last_dist)
                gv['you'].setText(f"{dv} {du} {self.current_bearing:.0f}\u00B0 {dir_status}")
            else:
                gv['you'].setText("-- (no RX fix)" if dir_status == "NO_RX_GPS" else "--")
            if self.track_origin is not None:
                x, y = local_xy(lat, lon, *self.track_origin)
                d = math.hypot(x, y)
                dv, du = fmt_dist(d)
                gv['pad'].setText(f"{dv} {du} {compass8(math.degrees(math.atan2(x, y)))}" if d >= 5 else "on pad")
            else:
                gv['pad'].setText("--")

        if self.receiver_has_fix and self.receiver_positions:
            rp = self.receiver_positions[-1]
            gv['rx'].setText(f"{rp[0]:.6f}, {rp[1]:.6f}")
        else:
            gv['rx'].setText("no receiver fix")

        if self.landing_pos is not None:
            gv['landed'].setText(f"{self.landing_pos[0]:.6f}, {self.landing_pos[1]:.6f}")
            gv['landed'].setStyleSheet(f"color:{C_BRAND}; font-size:15px; border:none;")
        else:
            gv['landed'].setText("set automatically at touchdown")
            gv['landed'].setStyleSheet(f"color:{C_DIM}; font-size:12px; font-weight:normal; border:none;")

    def update_mission(self, update_hud):
        mc = self.mc
        now = time.time()
        mc['title'].tick()

        if update_hud:
            self._update_link_header(now)
            self._update_tracker_qr(now)
            self._flush_log_view()
            mc['rec'].setText(("● REC" if int(now * 2) % 2 == 0 else "○ REC")
                              if self.csv_writer else "")
            if self.rx_freq_mhz is not None:
                mc['freq_now'].setText(f"{self.rx_freq_mhz:.3f} MHz")

        with self.data_lock:
            if not self.all_time:
                if update_hud:
                    self._set_phase_badge(self.flight_phase)
                    sess = now - self.start_wall_time if self.start_wall_time else 0.0
                    mc['clock'].setText(fmt_clock(sess))
                return

            t = np.fromiter(self.all_time, dtype=float)
            t_now = t[-1]

            # ---------- plots (every frame) ----------
            window = self.mc_window
            if window is not None and window <= 20.0:
                mask = t >= t_now - window
                x = t[mask]
                y_alt = np.fromiter(self.agl_hist, dtype=float)[mask]
                y_vs = np.fromiter(self.vs_smooth_hist, dtype=float)[mask]
                y_g = np.fromiter(self.g_force_magnitude, dtype=float)[mask]
                x0 = t_now - window
            else:
                ht = np.fromiter(self.h_t, dtype=float)
                ha = np.fromiter(self.h_agl, dtype=float)
                hv = np.fromiter(self.h_vs, dtype=float)
                hg = np.fromiter(self.h_g, dtype=float)
                # append the very latest sample so the trace reaches "now"
                ht = np.append(ht, t_now)
                ha = np.append(ha, self.agl_hist[-1])
                hv = np.append(hv, self.vs_smooth_hist[-1])
                hg = np.append(hg, self.g_force_magnitude[-1])
                if window is not None:
                    m = ht >= t_now - window
                    ht, ha, hv, hg = ht[m], ha[m], hv[m], hg[m]
                    x0 = t_now - window
                else:
                    x0 = ht[0]
                x, y_alt, y_vs, y_g = ht, ha, hv, hg

            x, y_alt, y_vs, y_g = decimate(x, y_alt, y_vs, y_g)
            mc['c_alt'].setData(x, y_alt)
            mc['c_vs'].setData(x, y_vs)
            mc['c_g'].setData(x, y_g)
            mc['c_alt_now'].setData([x[-1]], [y_alt[-1]])
            # y ranges with sensible minimum spans so pad noise isn't magnified
            def yr(arr, lo_min, hi_min, sym=False):
                fin = arr[np.isfinite(arr)]
                lo = float(fin.min()) if fin.size else 0.0
                hi = float(fin.max()) if fin.size else 0.0
                if sym:
                    m = max(abs(lo), abs(hi), hi_min)
                    return -m * 1.1, m * 1.1
                lo, hi = min(lo, lo_min), max(hi, hi_min)
                pad = (hi - lo) * 0.08
                return lo - pad, hi + pad
            a_lo, a_hi = yr(y_alt, -2.0, 10.0)
            a_hi = max(a_hi, self.max_agl * 1.08)
            mc['p_alt'].setYRange(a_lo, a_hi, padding=0)
            mc['p_vs'].setYRange(*yr(y_vs, 0, 5.0, sym=True), padding=0)
            mc['p_g'].setYRange(*yr(y_g, 0.0, 2.0), padding=0)
            x1 = max(t_now, x0 + 1.0)
            for key in ('p_alt', 'p_vs', 'p_g'):
                mc[key].setXRange(x0, x1, padding=0)
            if self.max_agl > 2.0:
                mc['apogee_line'].setPos(self.max_agl)
                mc['apogee_line'].setVisible(True)
            else:
                mc['apogee_line'].setVisible(False)

            if not update_hud:
                return

            # ---------- header ----------
            self._set_phase_badge(self.flight_phase)
            if self.launch_time is not None:
                mc['clock_lbl'].setText("MISSION  T+")
                mc['clock'].setText(fmt_clock(t_now - self.launch_time))
                mc['clock'].setStyleSheet(f"color:{C_WHITE};")
                mc['clock_lbl'].setStyleSheet(f"color:{C_BRAND}; font-size:11px; font-weight:bold; letter-spacing:2px;")
            else:
                mc['clock_lbl'].setText("ON PAD  (SESSION)")
                mc['clock_lbl'].setStyleSheet(f"color:{C_MUTED}; font-size:11px; font-weight:bold; letter-spacing:2px;")
                mc['clock'].setText(fmt_clock(t_now))
                mc['clock'].setStyleSheet(f"color:{C_TEXT};")
            if self.last_tx_tod is not None and self.last_tx_tod >= 0:
                tod = int(self.last_tx_tod)
                mc['utc'].setText(f"{tod // 3600:02d}:{(tod % 3600) // 60:02d}:{tod % 60:02d}")

            # ---------- tiles ----------
            values = {lab: self.all_data[lab][-1] for lab in DATA_LABELS}
            agl = self.current_agl
            if abs(agl) < 0.3:      # hide baro noise on the pad
                agl = 0.0
            mc['t_alt'].set(f"{agl:,.0f}" if abs(agl) >= 10 else f"{agl:.1f}", sub=
                            f"apogee {self.max_agl:,.0f} m  ·  MSL {values['BaroAlt']:,.0f} m")

            vs = self.vs_smooth
            if vs > 1.0:
                vs_txt, vs_col = f"▲{abs(vs):.1f}", C_TEXT
            elif vs < -1.0:
                vs_txt, vs_col = f"▼{abs(vs):.1f}", C_TEXT
            else:
                vs_txt, vs_col = f"{abs(vs):.1f}", C_TEXT
            mc['t_vs'].set(vs_txt, sub=("climbing" if vs > 1 else "descending" if vs < -1 else "steady"),
                           color=vs_col)

            g = self.g_force_magnitude[-1]
            sensor = self.g_sensor_used[-1]
            g_col = C_BRAND if g > 2 else C_TEXT
            mc['t_g'].set(f"{g:.1f}", sub=f"peak {self.max_g_force:.1f} g  ·  {sensor} sensor", color=g_col)

            spd = values['Speed']
            if spd >= 0:
                mc['t_spd'].set(f"{spd:.0f}", sub=f"{spd / 3.6:.1f} m/s  ·  max {self.max_speed:.0f} km/h")
            else:
                mc['t_spd'].set("--", sub="no rocket GPS fix", color=C_MUTED)

            dir_status = self.nav_status['DIR'][-1] if self.nav_status['DIR'] else 'UNKNOWN'
            last_dist = self.distance_history[-1] if self.distance_history else 0
            nav_ok = last_dist > 0 and dir_status not in ('UNKNOWN', 'NO_RX_GPS', 'NO_ROCKET_GPS')
            if nav_ok:
                dv, du = fmt_dist(last_dist)
                mv, mu = fmt_dist(self.max_distance)
                mc['t_dist'].set(dv, unit=du, sub=f"{self.current_bearing:.0f}° {dir_status}  ·  max {mv} {mu}")
            else:
                reason = {"NO_RX_GPS": "receiver GPS: no fix",
                          "NO_ROCKET_GPS": "rocket GPS: no fix"}.get(dir_status, "no nav data")
                mc['t_dist'].set("--", unit="m", sub=reason, color=C_MUTED)

            bat = values['Battery']
            pct = max(0.0, min(1.0, (bat - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V))) * 100
            bat_col = C_BRAND if bat < BATT_CRIT_V else C_AMBER if bat < BATT_WARN_V else C_TEXT
            mc['t_bat'].set(f"{bat:.2f}", sub=f"≈ {pct:.0f} %", color=bat_col)
            mc['t_bat'].set_accent(C_AMBER if bat_col == C_AMBER else C_BRAND)

            if self.current_rssi is not None:
                rssi = self.current_rssi
                bars = 5 if rssi > -70 else 4 if rssi > -80 else 3 if rssi > -90 else 2 if rssi > -100 else 1 if rssi > -110 else 0
                bar_txt = "▮" * bars + "▯" * (5 - bars)
                sig_col = C_TEXT if bars >= 3 else C_AMBER if bars >= 2 else C_BRAND
                snr_txt = f"SNR {self.current_snr:+.1f} dB" if self.current_snr is not None else ""
                mc['t_sig'].set(f"{rssi:.0f}", sub=f"{bar_txt}  {snr_txt}", color=sig_col)
            else:
                mc['t_sig'].set("--", sub="no RSSI in packet", color=C_MUTED)

            # ---------- rocket GPS ----------
            self._update_gps_panel(now, nav_ok, last_dist, dir_status)

            # ---------- records ----------
            rv = mc['rec_vals']
            rv['apogee'].setText(f"{self.max_agl:,.0f} m")
            rv['maxg'].setText(f"{self.max_g_force:.1f} g")
            rv['maxspd'].setText(f"{self.max_speed:.0f} km/h")
            mv, mu = fmt_dist(self.max_distance)
            rv['maxdist'].setText(f"{mv} {mu}")
            rv['ftime'].setText(fmt_clock(t_now - self.launch_time) if self.launch_time is not None else "--")
            bad = f" ({self.bad_packets} bad)" if self.bad_packets else ""
            rv['packets'].setText(f"{self.packet_count}{bad}")
            rv['drops'].setText(f"{self.connection_drops} / {self.max_packet_gap:.1f} s")
            q = self.connection_quality[-1] if self.connection_quality else 0.0
            rv['quality'].setText(f"{q:.0f} %")

    # ===================== Main timer callback =====================
    def update_plots(self):
        # 1) First, drain any new serial data into the buffers
        self.poll_serial_queue()

        self.frame_counter += 1
        update_hud = (self.frame_counter % HUD_UPDATE_EVERY) == 0

        if self._report_due is not None and time.time() >= self._report_due:
            self.generate_flight_report()
            self.export_google_earth()
        self.update_live_kml()

        # 2) Only redraw the tab that's actually visible (saves CPU on the Pi)
        if self.tabs.currentIndex() == 0:
            self.update_mission(update_hud)
        else:
            self.update_sensor_view(update_hud)

    # ===================== Main Run / Cleanup =====================
    def run(self):
        self.add_terminal_message("Starting AURORA Insight Dashboard V4 (Mission Control + Sensor View)...")
        if self.demo:
            self.add_terminal_message("DEMO MODE: simulated flight, no serial port used")
        else:
            self.list_serial_ports()
        self.setup_logging()

        # --- Start web server for tracker page (HTTPS) ---
        web_dir = WEB_DIR
        print(f"[WEB] Checking web directory: {web_dir}")
        try:
            if os.path.isdir(web_dir):
                self.add_terminal_message(f"Web directory found: {web_dir}")
                self.web_thread = WebServerThread(directory=web_dir, port=TRACKER_PORT)
                self.web_thread.start()

                url = self.tracker_url()
                print(f"[WEB] HTTPS tracker page (attempting to serve): {url}")
                print("      From directory:", web_dir)
                self.add_terminal_message("HTTPS tracker page (if SSL OK) at:")
                self.add_terminal_message(f"  {url}")
                self.add_terminal_message("On your phone, connect to the same Wi-Fi and open that URL.")
                self.add_terminal_message("Browser will warn about the cert; accept/trust it.")
                self.add_terminal_message("Or just scan the QR code on the Mission Control tab.")
            else:
                msg = f"Web directory {web_dir} not found; tracker page not served."
                print(f"[WEB] {msg}")
                self.add_terminal_message(msg)
        except Exception as e:
            err = f"Web server error: {e}"
            print(f"[WEB] {err}")
            self.add_terminal_message(err)

        # Start the separate reader process (serial or demo)
        if self.demo:
            self.reader_proc = Process(target=demo_reader_process,
                                       args=(self.data_queue, self.cmd_queue),
                                       daemon=True)
        else:
            self.add_terminal_message(f"Starting serial reader process on {SERIAL_PORT} @ {BAUD_RATE}...")
            self.reader_proc = Process(target=serial_reader_process,
                                       args=(SERIAL_PORT, BAUD_RATE, self.data_queue, self.cmd_queue),
                                       daemon=True)
        self.reader_proc.start()
        if not self.demo:
            self.cmd_queue.put("FREQ?")   # ask the receiver which frequency it's on

        self.running = True

        if LIVE_PLOT:
            self.setup_plots()
            # let Ctrl+C in the terminal close the window cleanly
            signal.signal(signal.SIGINT, lambda *_: self.app.quit())
            self.app.exec()
        else:
            self.add_terminal_message("Receiver running *without* plots... Press Ctrl+C to stop")
            try:
                last_report = 0
                while True:
                    time.sleep(1.0)
                    self.poll_serial_queue()
                    if self.packet_count - last_report >= 100:
                        last_report = self.packet_count
                        quality = self.connection_quality[-1] if self.connection_quality else 0.0
                        distance = self.current_distance
                        self.add_terminal_message(
                            f"Packets: {self.packet_count}, Quality: {quality:.1f}%, Distance: {distance:.1f} m"
                        )
            except KeyboardInterrupt:
                self.add_terminal_message("Shutting down...")

        self.cleanup()

    def cleanup(self):
        self.running = False
        if self.timer is not None:
            self.timer.stop()
        if (self.app is not None and self.launch_time is not None
                and self.report_paths is None and len(self.flight_rec) > 5):
            print("Writing flight report before exit...")
            self.generate_flight_report()
        if self.app is not None and self.last_rocket_fix is not None and self.kmz_path is None:
            self.export_google_earth()
        if self.log_file:
            self.log_file.close()
        if self.raw_log:
            self.raw_log.close()
        if self.reader_proc is not None and self.reader_proc.is_alive():
            self.reader_proc.terminate()

        print("\n=== Enhanced Flight Summary ===")
        print(f"Total packets received: {self.packet_count}")
        print(f"Bad/unparsed packets: {self.bad_packets}")
        print(f"Connection drops: {self.connection_drops}")
        print(f"Maximum altitude: {self.max_altitude:.1f} m (MSL)")
        print(f"Apogee above pad: {self.max_agl:.1f} m (AGL)")
        print(f"Maximum G-force: {self.max_g_force:.2f} g")
        print(f"Maximum speed: {self.max_speed:.1f} km/h")
        print(f"Maximum distance: {self.max_distance:.1f} m ({self.max_distance/1000:.2f} km)")
        print(f"Final flight phase: {self.flight_phase}")
        print(f"Rocket GPS fixes: {'Yes' if self.rocket_has_fix else 'No'}")
        print(f"Receiver GPS fixes: {'Yes' if self.receiver_has_fix else 'No'}")
        if self.rocket_positions and self.receiver_positions:
            print(f"Total rocket positions logged: {len(self.rocket_positions)}")
        if self.last_rocket_fix:
            print(f"Last rocket position: {self.last_rocket_fix[0]:.6f}, {self.last_rocket_fix[1]:.6f}")
        if self.landing_pos:
            print(f"Landing position: {self.landing_pos[0]:.6f}, {self.landing_pos[1]:.6f}")
        if self.log_filename:
            print(f"CSV log: {self.log_filename}")
        if self.raw_log_filename:
            print(f"Raw serial log: {self.raw_log_filename}")
        if self.report_paths:
            print(f"Flight report: {self.report_paths[0]}")
        if self.kmz_path:
            print(f"Google Earth file: {self.kmz_path}")


def report_from_csv(csv_path):
    """Rebuild a flight report from a saved telemetry CSV (new or old dashboard format)."""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True, useOpenGL=False)
    rx = TelemetryReceiver()
    rx.app = app
    rx.log_filename = os.path.abspath(csv_path)
    raw = os.path.join(os.path.dirname(rx.log_filename),
                       os.path.basename(csv_path).replace("ReceiverLog_", "RawSerial_").replace(".csv", ".txt"))
    rx.raw_log_filename = raw if os.path.exists(raw) else None
    with open(csv_path, newline='') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print("CSV is empty")
        return
    has_pc = 'PC_Time' in rows[0] and rows[0]['PC_Time']
    fake_t0 = time.time()

    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None
    for i, r in enumerate(rows):
        values = [num(r.get(lab)) for lab in DATA_LABELS]
        if any(v is None for v in values):
            continue
        nav = {}
        for key in NAV_LABELS:
            if r.get(key, '') != '':
                nav[key] = num(r[key])
        if r.get('Direction'):
            nav['DIR'] = r['Direction']
        if r.get('Turn_Direction'):
            nav['TURN'] = r['Turn_Direction']
        for col, key in (('RSSI_dBm', 'RSSI'), ('SNR_dB', 'SNR')):
            if r.get(col, '') != '':
                nav[key] = num(r[col])
        rt = float(r['PC_Time']) if has_pc else fake_t0 + i / EXPECTED_PACKET_RATE
        rx.process_data(num(r.get('Timestamp')) or 0, values, nav, rt)
    rx.add_terminal_message(f"Replayed {rx.packet_count} packets from {csv_path}")
    if rx.start_wall_time:   # name outputs after the flight, not after today
        rx.session_stamp = datetime.fromtimestamp(rx.start_wall_time).strftime('%Y%m%d-%H%M%S')
    pdf = rx.generate_flight_report()
    kmz = rx.export_google_earth()
    if pdf:
        print(f"\nReport written:\n  {pdf}\n  {rx.report_paths[1]}")
    if kmz:
        print(f"Google Earth file:\n  {kmz}")


def parse_args():
    ap = argparse.ArgumentParser(description="AURORA Insight Dashboard V4")
    ap.add_argument("--demo", action="store_true", help="simulate a flight (no hardware needed)")
    ap.add_argument("--port", default=None, help=f"serial port (default {SERIAL_PORT})")
    ap.add_argument("--log", action="store_true", help="save CSV + raw logs (on by default)")
    ap.add_argument("--no-log", action="store_true", help="don't save any log files")
    ap.add_argument("--report", metavar="CSV", default=None,
                    help="build a flight report from a saved ReceiverLog CSV and exit")
    ap.add_argument("--no-gl", action="store_true", help="disable OpenGL plotting")
    ap.add_argument("--fullscreen", action="store_true", help="start fullscreen (F11 toggles)")
    ap.add_argument("--headless", action="store_true", help="no GUI, just receive/log")
    ap.add_argument("--web-dir", default=None, help=f"folder with {TRACKER_PAGE} (default {WEB_DIR})")
    ap.add_argument("--cert-dir", default=None, help=f"folder with cert.pem/key.pem (default {CERT_DIR})")
    ap.add_argument("--screenshot", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--screenshot-after", type=float, default=20.0, help=argparse.SUPPRESS)
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.port:
        SERIAL_PORT = args.port
    if args.log:
        SAVE_TO_FILE = True
    if args.no_log:
        SAVE_TO_FILE = False
    if args.report:
        report_from_csv(args.report)
        sys.exit(0)
    if args.no_gl:
        USE_OPENGL = False
    if args.fullscreen:
        START_FULLSCREEN = True
    if args.headless:
        LIVE_PLOT = False
    if args.web_dir:
        WEB_DIR = args.web_dir
    if args.cert_dir:
        CERT_DIR = args.cert_dir

    try:
        receiver = TelemetryReceiver(demo=args.demo)
        if args.screenshot:
            # dev helper: grab both tabs after N seconds, then quit
            def _snap():
                base, ext = os.path.splitext(args.screenshot)
                for i in range(receiver.tabs.count()):
                    receiver.tabs.setCurrentIndex(i)
                    for _ in range(HUD_UPDATE_EVERY):
                        receiver.update_plots()
                    receiver.app.processEvents()
                    receiver.main_win.grab().save(f"{base}_{i}{ext or '.png'}")
                receiver.app.quit()
            _orig_setup = receiver.setup_plots

            def _setup_and_schedule():
                _orig_setup()
                QtCore.QTimer.singleShot(int(args.screenshot_after * 1000), _snap)
            receiver.setup_plots = _setup_and_schedule
        receiver.run()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
