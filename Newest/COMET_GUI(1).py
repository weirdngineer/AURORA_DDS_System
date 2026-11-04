#!/usr/bin/env python3
"""
COMET Flight Computer Interface 7.4  (Windows + Linux)

Matches COMET firmware 7.4:
  * Finds and connects to the flight computer automatically (USB VID 0x2E8A:
    COMx on Windows, /dev/ttyACM* on Linux) and reconnects after a replug
  * One-click firmware update (Flight Setup -> Update Firmware) from the
    bundled firmware/COMET_Firmware.uf2
  * Profiles store MAIN_ALT (100-10000 m), ACCEL_ENABLE, the post-launch safety
    lockout and the drogue / main backup timers (each OFF by default)
  * Pyro MOSFET outputs can be armed / made safe from Flight Setup (firmware v7.1+)
  * 20 flight logs + 3 ground-session logs, downloadable as CSV
  * EXPORTALLCSV support (one click saves every log into a new folder)
  * Log list shows max altitude, max g and max speed per flight (firmware v7+);
    click a column header to sort
  * Flight-log plotter: Plot Selected (or double-click a log) / Open CSV...
    shows altitude + speed, acceleration and gyro with apogee, drogue / main
    fire times and MOSFET on-time marked, plus a flight summary

Requires:  pip install pyserial
All pop-ups and the folder browser are built in-app (dark, large text) so they
look the same on every OS -- no white-on-white native dialogs.
"""

import bisect
import csv
import math
import os
import re
import statistics
import string
import subprocess
import sys
import threading
import queue
import time
from datetime import datetime
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import font as tkfont
except ImportError:
    sys.exit("COMET needs Tkinter. On Debian/Ubuntu run:  sudo apt install python3-tk")


def _import_pyserial():
    """Import pyserial; if it is missing, offer to install it (one click)."""
    try:
        import serial  # noqa: F401
        import serial.tools.list_ports  # noqa: F401
        return
    except ImportError:
        pass
    from tkinter import messagebox
    r = tk.Tk()
    r.withdraw()
    if messagebox.askyesno("COMET", "COMET needs the 'pyserial' package to talk to the flight computer.\n\n"
                                    "Install it now? (runs: pip install --user pyserial)"):
        cmd = [sys.executable, "-m", "pip", "install", "--user", "pyserial"]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:  # e.g. Debian "externally managed" Python
            res = subprocess.run(cmd + ["--break-system-packages"], capture_output=True, text=True)
        if res.returncode == 0:
            import site
            import importlib
            site.addsitedir(site.getusersitepackages())
            importlib.invalidate_caches()
            r.destroy()
            return
        messagebox.showerror("COMET", "Could not install pyserial automatically:\n\n"
                                      f"{(res.stderr or res.stdout)[-600:]}\n\n"
                                      "Install it manually:  python -m pip install pyserial")
    r.destroy()
    sys.exit("pyserial is required:  python -m pip install pyserial")


_import_pyserial()
import serial  # noqa: E402
import serial.tools.list_ports  # noqa: E402

# Sharper text on high-DPI Windows screens.
if sys.platform.startswith("win"):
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

# ============================================================
# THEME
# ============================================================

C = {
    "bg": "#121212",
    "panel": "#1c1c1c",
    "panel2": "#262626",
    "panel3": "#303030",
    "field": "#0b0b0b",
    "border": "#3a3a3a",
    "accent": "#c1121f",
    "accent_dark": "#8f0d16",
    "accent_light": "#ff5964",
    "text": "#f5f5f5",
    "muted": "#b8b8b8",
    "success": "#2ecc71",
    "warn": "#ffb020",
    "btn": "#3a3a3a",
    "btn_hover": "#4a4a4a",
}

FONT_UI = "TkDefaultFont"
FONT_MONO = "TkFixedFont"


def F(size=11, bold=False):
    return (FONT_UI, size, "bold") if bold else (FONT_UI, size)


def FM(size=10, bold=False):
    return (FONT_MONO, size, "bold") if bold else (FONT_MONO, size)


def pick_fonts(root):
    global FONT_UI, FONT_MONO
    fams = set(tkfont.families(root))
    for name in ["Segoe UI", "Ubuntu", "Noto Sans", "DejaVu Sans", "Liberation Sans", "Helvetica", "Arial"]:
        if name in fams:
            FONT_UI = name
            break
    for name in ["Consolas", "DejaVu Sans Mono", "Ubuntu Mono", "Liberation Mono", "Courier New"]:
        if name in fams:
            FONT_MONO = name
            break


GUI_VERSION = "7.4"
COMET_USB_VID = 0x2E8A           # Raspberry Pi / Arduino-Pico boards (all PIDs)
AUTO_CONNECT_MIN_SCORE = 30      # see score_port()
UF2_DEFAULT = Path(__file__).resolve().parent / "firmware" / "COMET_Firmware.uf2"

M_TO_FT = 3.280839895

PROFILES = [
    # name, default altitude m, button color, text color
    ("RED", 800, "#e02424", "white"),
    ("ORANGE", 700, "#f97316", "black"),
    ("YELLOW", 600, "#facc15", "black"),
    ("GREEN", 500, "#22c55e", "black"),
    ("BLUE", 400, "#3b82f6", "white"),
    ("INDIGO", 300, "#6366f1", "white"),
    ("VIOLET", 200, "#a855f7", "white"),
]
PROFILE_NAMES = [p[0] for p in PROFILES]
MIN_MAIN_ALT_M = 100.0   # firmware floor; profiles default to 200 m
MAX_MAIN_ALT_M = 10000.0
MAX_LOCKOUT_S = 120.0
MIN_TIMER_S, MAX_TIMER_S = 1.0, 600.0
DEFAULT_PROFILE = {"accel": True, "lockout_s": 6.0, "gap_s": 3.0,
                   "apogee_delay_s": 0.0, "main_delay_s": 0.0,
                   "dtimer": False, "dtimer_s": 15.0, "mtimer": False, "mtimer_s": 25.0}
MIN_GAP_S, MAX_GAP_S = 1, 60
MAX_EVENT_DELAY_S = 30.0         # +T delays (firmware 7.4+), 0.1 s steps


def score_port(p):
    """How likely a serial port is a COMET board (higher = better)."""
    blob = " ".join(str(x or "") for x in (p.device, p.description, p.manufacturer,
                                           getattr(p, "product", ""), p.hwid)).upper()
    score = 0
    if p.vid == COMET_USB_VID:
        score += 100
    if any(k in blob for k in ("RP2040", "PICO", "COMET", "RASPBERRY")):
        score += 40
    if "TTYACM" in blob:          # Linux CDC-ACM (where an RP2040 shows up)
        score += 30
    if "USB SERIAL DEVICE" in blob:  # Windows' name for a CDC-ACM port
        score += 20
    if "BLUETOOTH" in blob:
        score -= 200
    return score


def find_uf2_drive():
    """Mounted RP2040 bootloader drive (RPI-RP2), or None."""
    candidates = []
    if sys.platform.startswith("win"):
        candidates = [f"{c}:\\" for c in string.ascii_uppercase[2:]]
    elif sys.platform == "darwin":
        candidates = [str(p) for p in Path("/Volumes").glob("*")]
    else:
        user = os.environ.get("USER", "")
        for base in (f"/media/{user}", f"/run/media/{user}", "/media", "/mnt"):
            try:
                candidates += [str(p) for p in Path(base).iterdir() if p.is_dir()]
            except OSError:
                pass
    for c in candidates:
        info = Path(c) / "INFO_UF2.TXT"
        try:
            if info.is_file() and "RP2" in info.read_text(errors="ignore").upper():
                return c
        except OSError:
            continue
    return None

SHORT_STATE = {
    "DROGUE_DEPLOYED": "DROGUE OUT",
    "MAIN_DEPLOYED": "MAIN OUT",
    "ASCENT_LOCKOUT": "LOCKOUT",
    "APOGEE_DETECT": "APOGEE",
}

LOG_STATUS = {0: "Empty", 1: "Recording", 2: "Saved", 3: "Downloaded", 4: "Corrupt"}
LOG_ROW_RE = re.compile(
    r"^(\*?)\s*(FLIGHT|STATIC) SLOT (\d+) ID=(\d+) RECORDS=(\d+) SECONDS=(\d+) STATUS=(\d+)"
)

# v7 firmware appends per-flight stats to FLIGHT rows; older firmware omits them.
LOG_STATS_RE = re.compile(r"MAXALT=(-?[\d.]+)\s+MAXG=([\d.]+)\s+MAXV=([\d.]+)")

STATUS_PREFIXES = (
    "ERR", "OK", "SETTINGS", "FLIGHT LOG", "LOGGER", ">>>", "FLIGHT PROFILE",
    "PYRO", "OLD FLIGHT", "LOG RETENTION", "WARNING", "STARTLOG", "STOPLOG",
)


# ============================================================
# SMALL WIDGET HELPERS
# ============================================================

def lbl(parent, text="", size=11, bold=False, fg=None, bg=None, **kw):
    return tk.Label(
        parent, text=text, font=F(size, bold),
        fg=fg or C["text"], bg=bg or parent.cget("bg"), **kw
    )


def make_button(parent, text, command, kind="normal", size=11, padx=16, pady=9):
    palette = {
        "normal": (C["btn"], C["btn_hover"], "white"),
        "primary": (C["accent"], C["accent_dark"], "white"),
        "success": (C["success"], "#22a85a", "black"),
    }
    bg, hover, fg = palette[kind]
    b = tk.Button(
        parent, text=text, command=command, bg=bg, fg=fg,
        activebackground=hover, activeforeground=fg,
        disabledforeground=C["muted"], relief="flat", bd=0,
        padx=padx, pady=pady, font=F(size, True),
        cursor="hand2", highlightthickness=0,
    )
    b._base_bg = bg

    def enter(_e):
        if str(b["state"]) != "disabled":
            b.configure(bg=hover)

    def leave(_e):
        b.configure(bg=b._base_bg)

    b.bind("<Enter>", enter)
    b.bind("<Leave>", leave)
    return b


def make_entry(parent, var=None, width=12, size=12):
    return tk.Entry(
        parent, textvariable=var, width=width, font=F(size),
        bg=C["field"], fg=C["text"], insertbackground=C["text"],
        selectbackground=C["accent"], selectforeground="white",
        relief="flat", highlightthickness=1,
        highlightbackground=C["border"], highlightcolor=C["accent_light"],
    )


def make_card(parent, title):
    outer = tk.Frame(parent, bg=C["panel"], highlightthickness=1, highlightbackground=C["border"])
    tk.Label(outer, text=title, bg=C["panel"], fg=C["accent_light"],
             font=F(12, True), anchor="w").pack(fill="x", padx=14, pady=(10, 6))
    inner = tk.Frame(outer, bg=C["panel"])
    inner.pack(fill="both", expand=True, padx=14, pady=(0, 14))
    return outer, inner


def make_text(parent, font, fg=None, height=10):
    """Dark Text widget with a dark ttk scrollbar. Returns (frame, text)."""
    frame = tk.Frame(parent, bg=C["field"])
    text = tk.Text(
        frame, wrap="word", bg=C["field"], fg=fg or C["text"],
        insertbackground=C["text"], selectbackground=C["accent"],
        relief="flat", bd=0, font=font, height=height, padx=8, pady=6,
        highlightthickness=0,
    )
    sb = ttk.Scrollbar(frame, orient="vertical", command=text.yview)
    text.configure(yscrollcommand=sb.set)
    sb.pack(side="right", fill="y")
    text.pack(side="left", fill="both", expand=True)
    return frame, text


def open_folder(path):
    try:
        if sys.platform.startswith("win"):
            os.startfile(path)  # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            subprocess.Popen(["open", path])
        else:
            subprocess.Popen(["xdg-open", path])
    except Exception:
        pass


# ============================================================
# DARK DIALOGS (replace native messagebox / file dialogs)
# ============================================================

class ModalDialog(tk.Toplevel):
    def __init__(self, app, title, parent=None):
        self.app = app
        self.parent_w = parent or app.root
        super().__init__(self.parent_w)
        self.title(title)
        self.configure(bg=C["bg"])
        self.transient(self.parent_w)
        self.result = None
        self.bind("<Escape>", lambda e: self.close(None))
        self.protocol("WM_DELETE_WINDOW", lambda: self.close(None))

    def close(self, value=None):
        self.result = value
        try:
            self.grab_release()
        except tk.TclError:
            pass
        self.destroy()

    def run(self, width=None, height=None):
        self.update_idletasks()
        w = width or self.winfo_reqwidth()
        h = height or self.winfo_reqheight()
        p = self.parent_w
        x = p.winfo_rootx() + (p.winfo_width() - w) // 2
        y = p.winfo_rooty() + (p.winfo_height() - h) // 3
        self.geometry(f"{w}x{h}+{max(0, x)}+{max(0, y)}")
        try:
            self.wait_visibility()
            self.grab_set()
        except tk.TclError:
            pass
        self.lift()
        self.focus_force()
        self.wait_window(self)
        return self.result


def message_box(app, title, message, buttons, default=None, width=580, parent=None):
    """buttons = [(text, value, kind), ...] shown left-to-right."""
    d = ModalDialog(app, title, parent)
    body = tk.Frame(d, bg=C["bg"], padx=26, pady=22)
    body.pack(fill="both", expand=True)
    lbl(body, title, 16, True, C["accent_light"]).pack(fill="x", anchor="w", pady=(0, 12))
    lbl(body, message, 12, justify="left", anchor="w", wraplength=width - 80).pack(fill="x", anchor="w")
    row = tk.Frame(body, bg=C["bg"])
    row.pack(fill="x", pady=(24, 0))
    for text, value, kind in reversed(buttons):
        make_button(row, text, lambda v=value: d.close(v), kind, 12, 22, 11).pack(side="right", padx=(10, 0))
    d.bind("<Return>", lambda e: d.close(default))
    return d.run(width)


def ask_yes_no(app, title, message, yes="Yes", no="Cancel", danger=False, parent=None):
    return bool(message_box(
        app, title, message,
        [(no, False, "normal"), (yes, True, "primary" if danger else "success")],
        default=False if danger else True, parent=parent,
    ))


def show_info(app, title, message, parent=None):
    message_box(app, title, message, [("OK", True, "success")], default=True, parent=parent)


def show_error(app, title, message, parent=None):
    message_box(app, title, message, [("OK", True, "primary")], default=True, parent=parent)


def ask_string(app, title, prompt, initial="", parent=None):
    d = ModalDialog(app, title, parent)
    body = tk.Frame(d, bg=C["bg"], padx=26, pady=22)
    body.pack(fill="both", expand=True)
    lbl(body, title, 16, True, C["accent_light"]).pack(anchor="w", pady=(0, 10))
    lbl(body, prompt, 12).pack(anchor="w", pady=(0, 8))
    var = tk.StringVar(value=initial)
    e = make_entry(body, var, 40, 13)
    e.pack(fill="x", ipady=7)
    e.focus_set()
    e.selection_range(0, "end")
    row = tk.Frame(body, bg=C["bg"])
    row.pack(fill="x", pady=(22, 0))
    make_button(row, "OK", lambda: d.close(var.get().strip()), "success", 12, 24, 11).pack(side="right")
    make_button(row, "Cancel", lambda: d.close(None), "normal", 12, 22, 11).pack(side="right", padx=(0, 10))
    d.bind("<Return>", lambda ev: d.close(var.get().strip()))
    return d.run(560)


def choose_folder(app, start, parent=None, title="Choose a Folder"):
    d = ModalDialog(app, title, parent)
    d.minsize(700, 520)
    start_path = Path(start) if start and Path(start).is_dir() else Path.home()
    state = {"path": start_path}

    body = tk.Frame(d, bg=C["bg"], padx=20, pady=18)
    body.pack(fill="both", expand=True)
    lbl(body, title, 16, True, C["accent_light"]).pack(anchor="w", pady=(0, 10))

    # Path row
    prow = tk.Frame(body, bg=C["bg"])
    prow.pack(fill="x")
    path_var = tk.StringVar()
    path_entry = make_entry(prow, path_var, 10, 12)

    # Quick locations
    qrow = tk.Frame(body, bg=C["bg"])
    qrow.pack(fill="x", pady=(10, 8))
    lbl(qrow, "Go to:", 11, True, C["muted"]).pack(side="left", padx=(0, 8))

    # List
    lframe = tk.Frame(body, bg=C["border"], padx=1, pady=1)
    lframe.pack(fill="both", expand=True)
    inner = tk.Frame(lframe, bg=C["field"])
    inner.pack(fill="both", expand=True)
    lb = tk.Listbox(
        inner, font=F(12), bg=C["field"], fg=C["text"],
        selectbackground=C["accent"], selectforeground="white",
        activestyle="none", relief="flat", bd=0, highlightthickness=0,
    )
    sb = ttk.Scrollbar(inner, orient="vertical", command=lb.yview)
    lb.configure(yscrollcommand=sb.set)
    sb.pack(side="right", fill="y")
    lb.pack(side="left", fill="both", expand=True, padx=6, pady=4)

    hint = lbl(body, "Double-click a folder to open it, then press \"Use This Folder\".", 10, False, C["muted"])
    hint.pack(anchor="w", pady=(8, 0))

    def refresh():
        p = state["path"]
        path_var.set(str(p))
        lb.delete(0, "end")
        try:
            names = sorted(
                (c.name for c in p.iterdir() if c.is_dir() and not c.name.startswith(".")),
                key=str.lower,
            )
        except (PermissionError, OSError):
            lb.insert("end", "(cannot open this folder)")
            return
        for n in names:
            lb.insert("end", n + "/")
        if not names:
            lb.insert("end", "(no sub-folders)")

    def go(path):
        p = Path(path).expanduser()
        if p.is_dir():
            state["path"] = p
            refresh()
        else:
            show_error(app, "Folder Not Found", f"This folder does not exist:\n{p}", parent=d)

    def go_up():
        parent_p = state["path"].parent
        if parent_p != state["path"]:
            state["path"] = parent_p
            refresh()

    def open_selected(_e=None):
        sel = lb.curselection()
        if not sel:
            return
        name = lb.get(sel[0])
        if not name.endswith("/"):
            return
        state["path"] = state["path"] / name[:-1]
        refresh()

    def new_folder():
        name = ask_string(app, "New Folder", "Name for the new folder:", "COMET Logs", parent=d)
        if not name:
            return
        if any(ch in name for ch in '\\/:*?"<>|'):
            show_error(app, "Invalid Name", "A folder name can't contain  \\ / : * ? \" < > |", parent=d)
            return
        try:
            (state["path"] / name).mkdir(parents=True, exist_ok=True)
            state["path"] = state["path"] / name
            refresh()
        except Exception as e:
            show_error(app, "Could Not Create Folder", str(e), parent=d)

    make_button(prow, "Up", go_up, "normal", 11, 16, 7).pack(side="left", padx=(0, 8))
    path_entry.pack(side="left", fill="x", expand=True, ipady=6)
    make_button(prow, "Go", lambda: go(path_var.get()), "normal", 11, 16, 7).pack(side="left", padx=(8, 0))
    path_entry.bind("<Return>", lambda e: go(path_var.get()))

    home = Path.home()
    quick = [("Home", home), ("Documents", home / "Documents"),
             ("Desktop", home / "Desktop"), ("Downloads", home / "Downloads")]
    if sys.platform.startswith("win"):
        quick += [(f"{c}:", Path(f"{c}:\\")) for c in string.ascii_uppercase if os.path.exists(f"{c}:\\")]
    else:
        quick += [("/", Path("/"))]
        for mnt in (Path("/media"), Path("/mnt")):
            if mnt.exists():
                quick.append((str(mnt), mnt))
    for name, p in quick:
        if p.exists():
            make_button(qrow, name, lambda p=p: go(p), "normal", 10, 10, 5).pack(side="left", padx=(0, 6))

    lb.bind("<Double-Button-1>", open_selected)
    lb.bind("<Return>", open_selected)

    brow = tk.Frame(body, bg=C["bg"])
    brow.pack(fill="x", pady=(16, 0))
    make_button(brow, "New Folder", new_folder, "normal", 12, 18, 10).pack(side="left")
    make_button(brow, "Use This Folder", lambda: d.close(str(state["path"])), "success", 12, 24, 10).pack(side="right")
    make_button(brow, "Cancel", lambda: d.close(None), "normal", 12, 22, 10).pack(side="right", padx=(0, 10))

    refresh()
    return d.run(800, 600)


def csv_save_dialog(app, title, default_name):
    d = ModalDialog(app, title)
    body = tk.Frame(d, bg=C["bg"], padx=26, pady=22)
    body.pack(fill="both", expand=True)
    lbl(body, title, 16, True, C["accent_light"]).pack(anchor="w", pady=(0, 14))

    dir_var = tk.StringVar(value=app.csv_dir)
    name_var = tk.StringVar(value=default_name)

    lbl(body, "Save in folder", 11, True).pack(anchor="w")
    frow = tk.Frame(body, bg=C["bg"])
    frow.pack(fill="x", pady=(5, 14))
    make_entry(frow, dir_var, 10, 12).pack(side="left", fill="x", expand=True, ipady=7)

    def browse():
        chosen = choose_folder(app, dir_var.get(), parent=d)
        if chosen:
            dir_var.set(chosen)

    make_button(frow, "Browse...", browse, "normal", 12, 20, 8).pack(side="left", padx=(10, 0))

    lbl(body, "File name", 11, True).pack(anchor="w")
    name_entry = make_entry(body, name_var, 10, 12)
    name_entry.pack(fill="x", pady=(5, 10), ipady=7)
    name_entry.focus_set()

    preview_var = tk.StringVar()
    lbl(body, "", 10, False, C["muted"], textvariable=preview_var, wraplength=700, justify="left").pack(anchor="w", pady=(0, 6))

    def update_preview(*_):
        n = name_var.get().strip()
        if n and not n.lower().endswith(".csv"):
            n += ".csv"
        preview_var.set(str(Path(dir_var.get().strip()).expanduser() / n) if n else "")

    dir_var.trace_add("write", update_preview)
    name_var.trace_add("write", update_preview)
    update_preview()

    def save():
        folder = Path(dir_var.get().strip()).expanduser()
        name = name_var.get().strip()
        if not name:
            show_error(app, "Missing File Name", "Enter a file name.", parent=d)
            return
        if not name.lower().endswith(".csv"):
            name += ".csv"
        if any(ch in name for ch in '\\/:*?"<>|'):
            show_error(app, "Invalid File Name", "A file name can't contain  \\ / : * ? \" < > |", parent=d)
            return
        if not folder.exists():
            if not ask_yes_no(app, "Create Folder?", f"This folder does not exist:\n{folder}\n\nCreate it?",
                              yes="Create", parent=d):
                return
            try:
                folder.mkdir(parents=True, exist_ok=True)
            except Exception as e:
                show_error(app, "Folder Error", str(e), parent=d)
                return
        path = folder / name
        if path.exists() and not ask_yes_no(app, "Overwrite File?", f"This file already exists:\n{path}\n\nReplace it?",
                                            yes="Replace", danger=True, parent=d):
            return
        app.csv_dir = str(folder)
        d.close(str(path))

    row = tk.Frame(body, bg=C["bg"])
    row.pack(fill="x", pady=(14, 0))
    make_button(row, "Save and Download", save, "success", 12, 24, 11).pack(side="right")
    make_button(row, "Cancel", lambda: d.close(None), "normal", 12, 22, 11).pack(side="right", padx=(0, 10))
    d.bind("<Return>", lambda e: save())
    return d.run(780)


class ScrollFrame(tk.Frame):
    """Vertical scroll container so tall tabs still fit small laptop screens."""

    def __init__(self, parent, bg):
        super().__init__(parent, bg=bg)
        self.canvas = tk.Canvas(self, bg=bg, highlightthickness=0, bd=0)
        sb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.inner = tk.Frame(self.canvas, bg=bg)
        win = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.inner.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(win, width=e.width))
        self.bind("<Enter>", lambda e: self._wheel_binding(True))
        self.bind("<Leave>", lambda e: self._wheel_binding(False))

    def _wheel_binding(self, on):
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            if on:
                self.canvas.bind_all(seq, self._on_wheel)
            else:
                self.canvas.unbind_all(seq)

    def _on_wheel(self, e):
        if self.inner.winfo_reqheight() <= self.canvas.winfo_height():
            return
        step = -1 if (getattr(e, "num", 0) == 4 or getattr(e, "delta", 0) > 0) else 1
        self.canvas.yview_scroll(step * 3, "units")


# ============================================================
# LIVE PLOT
# ============================================================

class LivePlot(tk.Canvas):
    def __init__(self, parent, title, series_names, window_seconds=10.0, y_label="", show_noise=False, **kw):
        super().__init__(parent, bg=C["field"], highlightthickness=1,
                         highlightbackground=C["border"], **kw)
        self.title = title
        self.series_names = series_names
        self.window_seconds = float(window_seconds)
        self.y_label = y_label
        self.show_noise = show_noise
        self.colors = ["#ff5964", "#2ecc71", "#4da3ff", "#ffb020"]
        self.data = {n: [] for n in series_names}
        self.dirty = True
        self.bind("<Configure>", lambda e: self.redraw())

    def clear(self):
        for n in self.series_names:
            self.data[n].clear()
        self.redraw()

    def add_point(self, t, values):
        cutoff = t - self.window_seconds
        for n in self.series_names:
            try:
                v = float(values.get(n, 0.0))
            except Exception:
                v = 0.0
            self.data[n].append((t, v))
            self.data[n] = [(tt, vv) for tt, vv in self.data[n] if tt >= cutoff]
        self.dirty = True

    def trim(self):
        pts = [p for vals in self.data.values() for p in vals]
        if not pts:
            return
        cutoff = max(t for t, _ in pts) - self.window_seconds
        for n in self.series_names:
            self.data[n] = [(t, v) for t, v in self.data[n] if t >= cutoff]
        self.dirty = True

    def redraw_if_dirty(self):
        if self.dirty:
            self.redraw()

    def redraw(self):
        self.dirty = False
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 100 or h < 80:
            return
        x0, y0, x1, y1 = 64, 34, w - 18, h - 40

        title_id = self.create_text(12, 9, anchor="nw", text=self.title, fill=C["accent_light"], font=F(11, True))

        pts = [p for vals in self.data.values() for p in vals]
        if not pts:
            self.create_text(w / 2, h / 2, text="Waiting for telemetry...", fill=C["muted"], font=F(11))
            return

        max_t = max(t for t, _ in pts)
        min_t = max_t - self.window_seconds
        vis = [v for t, v in pts if t >= min_t]
        y_min, y_max = min(vis), max(vis)
        if self.show_noise and len(vis) >= 10:
            # Spread of the visible window: while the board sits still this is
            # the barometer noise, useful when tuning oversampling / filtering.
            mean = sum(vis) / len(vis)
            sd = (sum((v - mean) ** 2 for v in vis) / len(vis)) ** 0.5
            self.create_text(self.bbox(title_id)[2] + 16, 9, anchor="nw",
                             text=f"σ {sd:.2f}   span {y_max - y_min:.2f}", fill=C["muted"], font=FM(10))
        if abs(y_max - y_min) < 1e-6:
            y_min -= 1.0
            y_max += 1.0
        pad = (y_max - y_min) * 0.12
        y_min -= pad
        y_max += pad

        self.create_rectangle(x0, y0, x1, y1, outline=C["border"])
        for i in range(5):
            y = y0 + (y1 - y0) * i / 4
            val = y_max - (y_max - y_min) * i / 4
            self.create_line(x0, y, x1, y, fill="#242424")
            self.create_text(x0 - 8, y, anchor="e", text=f"{val:.1f}", fill=C["muted"], font=FM(9))

        self.create_text(x0, y1 + 8, anchor="n", text=f"-{self.window_seconds:.0f} s", fill=C["muted"], font=FM(9))
        self.create_text(x1, y1 + 8, anchor="n", text="now", fill=C["muted"], font=FM(9))
        if self.y_label:
            self.create_text(16, (y0 + y1) / 2, text=self.y_label, angle=90, fill=C["muted"], font=F(10))

        lx = w - 18
        for idx in reversed(range(len(self.series_names))):
            name = self.series_names[idx]
            color = self.colors[idx % len(self.colors)]
            self.create_text(lx, 10, anchor="ne", text=name, fill=C["text"], font=F(10, True))
            self.create_line(lx - 60, 17, lx - 40, 17, fill=color, width=3)
            lx -= 80

        for idx, name in enumerate(self.series_names):
            vals = [(t, v) for t, v in self.data[name] if t >= min_t]
            if len(vals) < 2:
                continue
            coords = []
            for t, v in vals:
                coords.append(x0 + (t - min_t) / max(1e-9, max_t - min_t) * (x1 - x0))
                coords.append(y1 - (v - y_min) / (y_max - y_min) * (y1 - y0))
            self.create_line(coords, fill=self.colors[idx % len(self.colors)], width=2)


# ============================================================
# MAIN APPLICATION
# ============================================================

class COMETApp:
    def __init__(self, root):
        self.root = root
        root.title(f"COMET Flight Computer {GUI_VERSION}")
        # Fit laptop screens (1366x768 and scaled 1080p): never taller than the screen.
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        w, h = min(1240, sw - 40), min(900, sh - 90)
        root.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 3)}")
        root.minsize(min(980, w), min(640, h))
        root.configure(bg=C["bg"])

        self.ser = None
        self.reader_running = False
        self.rx_queue = queue.Queue()
        self.port_map = {}
        self.port_scores = {}
        self.auto_var = tk.BooleanVar(value=True)
        self.auto_suppressed = None      # port the user manually disconnected
        self.auto_retry_at = {}          # port -> monotonic time of next attempt
        self.perm_hint_shown = False
        self.fw_version = None
        self.params_seen = False
        self.follow_active_once = False
        self.fw_busy = False
        self.fw_queue = queue.Queue()
        self.last_lock_prompt = 0.0
        self.last_cmd_ground = False     # was the most recent command a user ground action?
        self.last_cmd_at = 0.0
        self.storage_prompted = False

        self.device_verified = False
        self.connection_lost = False
        self.last_telemetry_wall = None
        self.telemetry_rate_hz = 0.0
        self.last_telemetry = {}
        self.plot_paused = False
        self.tick = 0

        self.unit_mode = tk.IntVar(value=0)
        self._last_units = 0

        docs = Path.home() / "Documents"
        self.csv_dir = str(docs if docs.is_dir() else Path.home())

        # Profile cache (SI units)
        self.profiles = {n: dict(DEFAULT_PROFILE, alt_m=float(a)) for n, a, _, _ in PROFILES}
        self.pyro_live = None  # None = unknown until the board reports it
        self.active_profile = None
        self.selected_profile = "VIOLET"
        self.profile_buttons = {}

        # Log table cache and download state
        self.log_rows = {}
        self.dl = None

        self.configure_theme()
        self.build_ui()
        self.refresh_ports()
        self.refresh_profile_buttons()
        self.load_profile_editor()

        root.after(50, self.process_serial_queue)
        root.after(100, self.refresh_plots)
        root.after(500, self.monitor)
        root.after(300, self.auto_connect)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    @property
    def imperial(self):
        return self.unit_mode.get() == 1

    # --------------------------------------------------------
    # Theme
    # --------------------------------------------------------
    def configure_theme(self):
        r = self.root
        r.option_add("*TCombobox*Listbox.background", C["field"])
        r.option_add("*TCombobox*Listbox.foreground", C["text"])
        r.option_add("*TCombobox*Listbox.selectBackground", C["accent"])
        r.option_add("*TCombobox*Listbox.selectForeground", "white")
        r.option_add("*TCombobox*Listbox.font", F(11))

        s = ttk.Style()
        s.theme_use("clam")
        s.configure(".", background=C["bg"], foreground=C["text"], bordercolor=C["border"],
                    lightcolor=C["border"], darkcolor=C["border"], troughcolor=C["panel"])

        s.configure("TCombobox", fieldbackground=C["field"], foreground=C["text"],
                    background=C["panel3"], arrowcolor=C["text"], bordercolor=C["border"], padding=6)
        s.map("TCombobox",
              fieldbackground=[("readonly", C["field"])],
              foreground=[("readonly", C["text"])],
              selectbackground=[("readonly", C["field"])],
              selectforeground=[("readonly", C["text"])],
              background=[("readonly", C["panel3"])])

        s.configure("TNotebook", background=C["bg"], borderwidth=0, tabmargins=(0, 0, 0, 0))
        s.configure("TNotebook.Tab", background=C["panel2"], foreground=C["muted"],
                    padding=(22, 10), font=F(12, True), borderwidth=0)
        s.map("TNotebook.Tab",
              background=[("selected", C["accent"]), ("active", C["panel3"])],
              foreground=[("selected", "white"), ("active", "white")])

        s.configure("TCheckbutton", background=C["panel"], foreground=C["text"], font=F(12))
        s.map("TCheckbutton",
              background=[("active", C["panel"])],
              indicatorbackground=[("selected", C["accent"]), ("!selected", C["field"])],
              indicatorforeground=[("selected", "white")])
        s.configure("Bar.TCheckbutton", background=C["bg"], foreground=C["text"], font=F(11))
        s.map("Bar.TCheckbutton", background=[("active", C["bg"])],
              indicatorbackground=[("selected", C["accent"]), ("!selected", C["field"])])

        s.configure("Unit.TRadiobutton", background=C["panel"], foreground=C["text"], font=F(11))
        s.configure("Bar.TRadiobutton", background=C["bg"], foreground=C["text"], font=F(11))
        s.map("Bar.TRadiobutton", background=[("active", C["bg"])],
              indicatorbackground=[("selected", C["accent"]), ("!selected", C["field"])])
        s.configure("Unit.TCheckbutton", background=C["panel"], foreground=C["text"], font=F(11))
        s.map("Unit.TCheckbutton", background=[("active", C["panel"])],
              indicatorbackground=[("selected", C["accent"]), ("!selected", C["field"])])
        s.map("Unit.TRadiobutton", background=[("active", C["panel"])],
              indicatorbackground=[("selected", C["accent"]), ("!selected", C["field"])])

        s.configure("Treeview", background=C["field"], fieldbackground=C["field"],
                    foreground=C["text"], rowheight=34, font=F(11), borderwidth=0)
        s.map("Treeview", background=[("selected", C["accent"])], foreground=[("selected", "white")])
        s.configure("Treeview.Heading", background=C["panel3"], foreground=C["text"],
                    font=F(11, True), padding=8, borderwidth=0)
        s.map("Treeview.Heading", background=[("active", C["panel2"])])

        s.configure("Vertical.TScrollbar", background=C["panel3"], troughcolor=C["field"],
                    arrowcolor=C["text"], bordercolor=C["field"])
        s.configure("Horizontal.TProgressbar", background=C["accent_light"], troughcolor=C["panel2"])

    # --------------------------------------------------------
    # UI construction
    # --------------------------------------------------------
    def build_ui(self):
        main = tk.Frame(self.root, bg=C["bg"], padx=14, pady=10)
        main.pack(fill="both", expand=True)

        # Title
        title = tk.Frame(main, bg=C["bg"])
        title.pack(fill="x", pady=(0, 8))
        lbl(title, "COMET", 24, True, C["accent_light"]).pack(side="left")
        lbl(title, "   Compact Onboard Management for Ejection Timing", 11, False, C["muted"]).pack(side="left", pady=(8, 0))
        self.fw_var = tk.StringVar(value=f"GUI {GUI_VERSION}  |  firmware: not connected")
        self.fw_label = lbl(title, "", 10, False, C["muted"], textvariable=self.fw_var)
        self.fw_label.pack(side="right", pady=(8, 0))
        units = tk.Frame(title, bg=C["bg"])
        units.pack(side="right", padx=(0, 24), pady=(8, 0))
        lbl(units, "Units", 11, True).pack(side="left", padx=(0, 8))
        ttk.Radiobutton(units, text="Metric", value=0, variable=self.unit_mode,
                        style="Bar.TRadiobutton", command=self.on_unit_change).pack(side="left", padx=(0, 10))
        ttk.Radiobutton(units, text="Imperial", value=1, variable=self.unit_mode,
                        style="Bar.TRadiobutton", command=self.on_unit_change).pack(side="left")

        self.build_connection_bar(main)
        self.build_status_cards(main)

        # Bottom message bar (pack before notebook so it stays visible)
        bar = tk.Frame(main, bg=C["panel"], highlightthickness=1, highlightbackground=C["border"])
        bar.pack(side="bottom", fill="x", pady=(8, 0))
        self.msg_var = tk.StringVar(value="Ready.")
        self.msg_label = tk.Label(bar, textvariable=self.msg_var, bg=C["panel"], fg=C["text"],
                                  font=F(11), anchor="w", padx=12, pady=7)
        self.msg_label.pack(fill="x")

        self.notebook = ttk.Notebook(main)
        self.notebook.pack(fill="both", expand=True)

        self.tab_dash = tk.Frame(self.notebook, bg=C["bg"], padx=0, pady=10)
        self.tab_setup = tk.Frame(self.notebook, bg=C["bg"], padx=0, pady=10)
        self.tab_logs = tk.Frame(self.notebook, bg=C["bg"], padx=0, pady=10)
        self.tab_console = tk.Frame(self.notebook, bg=C["bg"], padx=0, pady=10)

        self.notebook.add(self.tab_dash, text="Dashboard")
        self.notebook.add(self.tab_setup, text="Flight Setup")
        self.notebook.add(self.tab_logs, text="Flight Logs")
        self.notebook.add(self.tab_console, text="Console")

        self.build_dashboard()
        self.build_setup()
        self.build_logs()
        self.build_console()

    def build_connection_bar(self, parent):
        bar = tk.Frame(parent, bg=C["panel"], highlightthickness=1, highlightbackground=C["border"])
        bar.pack(fill="x", pady=(0, 8))
        inner = tk.Frame(bar, bg=C["panel"])
        inner.pack(fill="x", padx=12, pady=10)

        lbl(inner, "Port", 11, True).pack(side="left", padx=(0, 8))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var, width=34, state="readonly", font=F(11))
        self.port_combo.pack(side="left", padx=(0, 8))
        make_button(inner, "Refresh", self.refresh_ports, "normal", 11, 14, 6).pack(side="left", padx=(0, 12))
        self.connect_button = make_button(inner, "Connect", self.toggle_connection, "success", 11, 20, 6)
        self.connect_button.pack(side="left", padx=(0, 14))
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_label = tk.Label(inner, textvariable=self.status_var, bg=C["panel"], fg=C["warn"],
                                     font=F(11, True))
        ttk.Checkbutton(inner, text="Auto-connect", variable=self.auto_var,
                        style="Unit.TCheckbutton").pack(side="right")
        self.status_label.pack(side="left", fill="x", expand=True, anchor="w")

    def build_status_cards(self, parent):
        row = tk.Frame(parent, bg=C["bg"])
        row.pack(fill="x", pady=(0, 8))
        keys = [("STATE", "STATE"), ("ALT", "ALTITUDE"), ("MAXALT", "MAX ALT"), ("VZ", "SPEED"),
                ("BATT", "BATTERY"), ("PROFILE", "PROFILE"), ("DROGUE", "DROGUE"),
                ("MAIN", "MAIN"), ("LOG", "LOGGING"), ("OUTPUTS", "OUTPUTS")]
        self.card_vars = {}
        self.card_labels = {}
        for i, (key, title) in enumerate(keys):
            row.grid_columnconfigure(i, weight=1, uniform="cards")
            card = tk.Frame(row, bg=C["panel"], highlightthickness=1, highlightbackground=C["border"])
            card.grid(row=0, column=i, sticky="ew", padx=3)
            tk.Label(card, text=title, bg=C["panel"], fg=C["muted"], font=F(9, True), anchor="w"
                     ).pack(fill="x", padx=8, pady=(7, 0))
            var = tk.StringVar(value="--")
            lab = tk.Label(card, textvariable=var, bg=C["panel"], fg=C["accent_light"],
                           font=FM(11, True), anchor="w")
            lab.pack(fill="x", padx=8, pady=(0, 7))
            if key == "PROFILE":  # "VIOLET 200m" is the longest value
                lab.configure(font=FM(10, True))
            self.card_vars[key] = var
            self.card_labels[key] = lab

    # ---------------- Dashboard ----------------
    def build_dashboard(self):
        t = self.tab_dash
        t.grid_rowconfigure(0, weight=1)
        t.grid_columnconfigure(0, weight=1)

        area = tk.Frame(t, bg=C["bg"])
        area.grid(row=0, column=0, sticky="nsew")
        area.grid_columnconfigure(0, weight=1)
        for i in range(3):
            area.grid_rowconfigure(i, weight=1)

        self.accel_plot = LivePlot(area, "Acceleration", ["AX", "AY", "AZ"], y_label="m/s²", height=120)
        self.gyro_plot = LivePlot(area, "Gyroscope", ["GX", "GY", "GZ"], y_label="deg/s", height=120)
        self.baro_plot = LivePlot(area, "Altitude (barometer)", ["ALT"], y_label="m", show_noise=True, height=120)
        self.accel_plot.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        self.gyro_plot.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        self.baro_plot.grid(row=2, column=0, sticky="nsew")
        self.plots = [self.accel_plot, self.gyro_plot, self.baro_plot]

        ctl = tk.Frame(t, bg=C["bg"])
        ctl.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self.pause_btn = make_button(ctl, "Pause Plots", self.toggle_plot_pause, "normal", 11, 18, 8)
        self.pause_btn.pack(side="left", padx=(0, 8))
        make_button(ctl, "Clear Plots", self.clear_plots, "normal", 11, 18, 8).pack(side="left", padx=(0, 20))
        lbl(ctl, "Time window", 11).pack(side="left", padx=(0, 8))
        self.window_var = tk.StringVar(value="10")
        combo = ttk.Combobox(ctl, textvariable=self.window_var, values=["5", "10", "15", "30", "60"],
                             width=5, state="readonly", font=F(11))
        combo.pack(side="left")
        combo.bind("<<ComboboxSelected>>", lambda e: self.update_plot_window())
        lbl(ctl, "seconds", 11).pack(side="left", padx=(8, 0))

    # ---------------- Flight setup ----------------
    def build_setup(self):
        sf = ScrollFrame(self.tab_setup, C["bg"])
        sf.pack(fill="both", expand=True)
        t = sf.inner
        t.grid_columnconfigure(0, weight=0)
        t.grid_columnconfigure(1, weight=1)
        t.grid_rowconfigure(0, weight=1)

        # Left: profile list
        left_card, left = make_card(t, "1. Pick a flight profile")
        left_card.grid(row=0, column=0, rowspan=2, sticky="nsw", padx=(0, 12))
        lbl(left, "Each color is a saved main-parachute altitude.\nThe MODE button on the board cycles these.",
            10, False, C["muted"], justify="left").pack(anchor="w", pady=(0, 10))
        for name, _alt, color, fg in PROFILES:
            b = tk.Button(
                left, text=name, command=lambda n=name: self.select_profile(n),
                bg=color, fg=fg, activebackground=color, activeforeground=fg,
                relief="flat", bd=0, font=F(13, True), anchor="w", padx=16, pady=10,
                cursor="hand2", highlightthickness=4, highlightbackground=color, width=22,
            )
            b._color = color
            b.pack(fill="x", pady=3)
            self.profile_buttons[name] = b

        # Right top: editor
        right = tk.Frame(t, bg=C["bg"])
        right.grid(row=0, column=1, sticky="nsew")
        right.grid_columnconfigure(0, weight=1)

        ed_card, ed = make_card(right, "2. Edit the selected profile")
        ed_card.grid(row=0, column=0, sticky="ew")
        self.editor_title = tk.StringVar()
        lbl(ed, "", 15, True, textvariable=self.editor_title).pack(anchor="w", pady=(0, 10))

        lbl(ed, "Main parachute deploys at (height above launch pad)", 11, True).pack(anchor="w")
        arow = tk.Frame(ed, bg=C["panel"])
        arow.pack(anchor="w", pady=(6, 4))
        self.alt_var = tk.StringVar()
        make_entry(arow, self.alt_var, 10, 14).pack(side="left", ipady=7)
        self.alt_unit_var = tk.StringVar(value="m")
        lbl(arow, "", 13, True, textvariable=self.alt_unit_var).pack(side="left", padx=(10, 0))
        self.alt_hint_var = tk.StringVar()
        lbl(ed, "", 10, False, C["muted"], textvariable=self.alt_hint_var).pack(anchor="w", pady=(0, 12))

        self.accel_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ed, text="Detect launch with the accelerometer (barometer is always on)",
                        variable=self.accel_var).pack(anchor="w", pady=(0, 12))

        sg = tk.Frame(ed, bg=C["panel"])
        sg.pack(anchor="w", fill="x", pady=(0, 6))
        self.lockout_var = tk.StringVar()
        self.dtimer_var = tk.BooleanVar(value=False)
        self.dtimer_s_var = tk.StringVar()
        self.mtimer_var = tk.BooleanVar(value=False)
        self.mtimer_s_var = tk.StringVar()
        lbl(sg, "Safety lockout after launch", 11, True).grid(row=0, column=0, sticky="w", pady=4)
        make_entry(sg, self.lockout_var, 6, 12).grid(row=0, column=1, sticky="w", padx=(10, 6), ipady=4)
        lbl(sg, "s   no apogee / main detection or firing (Mach, bumped launch)", 10, False, C["muted"]
            ).grid(row=0, column=2, sticky="w")
        self.gap_var = tk.StringVar()
        lbl(sg, "Minimum drogue → main gap", 11, True).grid(row=3, column=0, sticky="w", pady=4)
        make_entry(sg, self.gap_var, 6, 12).grid(row=3, column=1, sticky="w", padx=(10, 6), ipady=4)
        lbl(sg, "s   whole seconds, 1 - 60", 10, False, C["muted"]).grid(row=3, column=2, sticky="w")
        self.apogee_delay_var = tk.StringVar()
        self.main_delay_var = tk.StringVar()
        lbl(sg, "Drogue delay (+T)", 11, True).grid(row=4, column=0, sticky="w", pady=4)
        make_entry(sg, self.apogee_delay_var, 6, 12).grid(row=4, column=1, sticky="w", padx=(10, 6), ipady=4)
        lbl(sg, "s   after any drogue trigger (apogee, timer, no-climb)", 10, False, C["muted"]
            ).grid(row=4, column=2, sticky="w")
        lbl(sg, "Main delay (+T)", 11, True).grid(row=5, column=0, sticky="w", pady=4)
        make_entry(sg, self.main_delay_var, 6, 12).grid(row=5, column=1, sticky="w", padx=(10, 6), ipady=4)
        lbl(sg, "s   after any main trigger; main is always >= gap after drogue", 10, False, C["muted"]
            ).grid(row=5, column=2, sticky="w")
        ttk.Checkbutton(sg, text="Drogue backup timer", variable=self.dtimer_var
                        ).grid(row=1, column=0, sticky="w", pady=4)
        make_entry(sg, self.dtimer_s_var, 6, 12).grid(row=1, column=1, sticky="w", padx=(10, 6), ipady=4)
        lbl(sg, "s after launch", 10, False, C["muted"]).grid(row=1, column=2, sticky="w")
        ttk.Checkbutton(sg, text="Main backup timer", variable=self.mtimer_var
                        ).grid(row=2, column=0, sticky="w", pady=4)
        make_entry(sg, self.mtimer_s_var, 6, 12).grid(row=2, column=1, sticky="w", padx=(10, 6), ipady=4)
        lbl(sg, "s after launch, only after a drogue command", 10, False, C["muted"]
            ).grid(row=2, column=2, sticky="w")
        lbl(ed, "Lockout: longer than the motor burn (+1 s), shorter than the time to apogee. On small motors\n"
                "(apogee in under ~7 s) use a short lockout (e.g. 2 s) and gap (1 s), or recovery will be late.\n"
                "Backup timers are OFF by default; with both on, main must be at least 2 s after drogue.\n"
                "Drogue / main delays (+T, default 0) apply to every trigger; main never fires before drogue + gap.",
            10, False, C["muted"], justify="left").pack(anchor="w", pady=(0, 12))

        brow = tk.Frame(ed, bg=C["panel"])
        brow.pack(anchor="w")
        make_button(brow, "Save to Board", self.save_profile, "primary", 12, 24, 11).pack(side="left", padx=(0, 10))
        make_button(brow, "Make This the Active Profile", self.activate_profile, "normal", 12, 20, 11
                    ).pack(side="left", padx=(0, 10))
        make_button(brow, "Read from Board", lambda: self.send_command("PROFILE LIST", ground=True), "normal", 12, 16, 11
                    ).pack(side="left")

        lbl(ed, "Changes are written to the board's flash a moment after saving, and are\n"
                "locked once launch has been detected.",
            10, False, C["muted"], justify="left").pack(anchor="w", pady=(12, 0))

        # Right bottom: pyro outputs + bench tests
        bench_card, bench = make_card(right, "Pyro outputs and bench testing")
        bench_card.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        prow = tk.Frame(bench, bg=C["panel"])
        prow.pack(fill="x", pady=(0, 10))
        lbl(prow, "MOSFET outputs:", 12, True).pack(side="left")
        self.pyro_state_var = tk.StringVar(value="unknown")
        self.pyro_state_label = lbl(prow, "", 13, True, C["muted"], textvariable=self.pyro_state_var)
        self.pyro_state_label.pack(side="left", padx=(8, 16))
        make_button(prow, "Make SAFE (bench)", self.disarm_pyro, "success", 11, 16, 7).pack(side="right")
        make_button(prow, "Enable Outputs (LIVE)", self.arm_pyro, "primary", 11, 16, 7
                    ).pack(side="right", padx=(0, 8))
        lbl(bench, "LIVE (default): drogue / main really fire. The board beeps 3 long tones at power-up when LIVE.\n"
                   "SAFE: the flight logic runs and logs, but the MOSFETs never switch. The choice is saved on the board.\n"
                   "Force Launch = pop test: drogue fires when the lockout ends, main after the gap. "
                   "Stand clear of installed charges.", 10, False, C["warn"],
            justify="left", wraplength=680).pack(anchor="w", pady=(0, 10))
        g = tk.Frame(bench, bg=C["panel"])
        g.pack(fill="x")
        for c in range(2):
            g.grid_columnconfigure(c, weight=1, uniform="bench")
        make_button(g, "Reset to IDLE", lambda: self.send_command("RESET"), "normal", 11, 10, 10
                    ).grid(row=0, column=0, sticky="ew", padx=(0, 5), pady=4)
        make_button(g, "Force Launch", lambda: self.confirm_command(
            "LAUNCH", "Force Launch",
            "This runs a complete flight sequence as if the rocket had launched (a pop test in flight "
            "configuration): after the safety lockout the DROGUE fires, then the MAIN after the "
            "drogue -> main gap. The flight is logged (at least 30 s)."), "primary", 11, 10, 10
                    ).grid(row=0, column=1, sticky="ew", padx=(5, 0), pady=4)
        make_button(g, "Test Drogue", lambda: self.confirm_command(
            "DROGUE", "Test Drogue", "This fires the DROGUE output immediately."), "primary", 11, 10, 10
                    ).grid(row=1, column=0, sticky="ew", padx=(0, 5), pady=4)
        make_button(g, "Test Main", lambda: self.confirm_command(
            "MAIN", "Test Main", "This fires the MAIN output immediately."), "primary", 11, 10, 10
                    ).grid(row=1, column=1, sticky="ew", padx=(5, 0), pady=4)
        make_button(g, "Update Firmware...", self.update_firmware, "normal", 11, 10, 10
                    ).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 4))

    # ---------------- Logs ----------------
    def build_logs(self):
        t = self.tab_logs
        t.grid_rowconfigure(1, weight=1)
        t.grid_columnconfigure(0, weight=1)

        top = tk.Frame(t, bg=C["bg"])
        top.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        make_button(top, "Refresh List", lambda: self.send_command("LIST", ground=True), "normal", 12, 20, 11).pack(side="left", padx=(0, 8))
        make_button(top, "Download Selected", self.download_selected, "success", 12, 22, 11).pack(side="left", padx=(0, 8))
        make_button(top, "Export All", self.export_all, "success", 12, 22, 11).pack(side="left", padx=(0, 8))
        make_button(top, "Erase All", self.erase_all, "primary", 12, 20, 11).pack(side="right")
        make_button(top, "Erase Selected", self.erase_selected, "primary", 12, 20, 11).pack(side="right", padx=(0, 8))

        tf = tk.Frame(t, bg=C["border"], padx=1, pady=1)
        tf.grid(row=1, column=0, sticky="nsew")
        cols = ("type", "slot", "id", "length", "records", "maxalt", "maxg", "maxv", "status", "latest")
        self.log_tree = ttk.Treeview(tf, columns=cols, show="headings", selectmode="browse")
        heads = [("type", "Type", 150), ("slot", "Slot", 60), ("id", "Log #", 70),
                 ("length", "Length", 100), ("records", "Records", 90),
                 ("maxalt", "Max Alt", 110), ("maxg", "Max G", 90), ("maxv", "Max Speed", 120),
                 ("status", "Status", 120), ("latest", "Newest", 80)]
        self.log_sort = ("id", True)
        for key, text, width in heads:
            self.log_tree.heading(key, text=text, command=lambda k=key: self.sort_logs_by(k))
            self.log_tree.column(key, width=width, anchor="w")
        sb = ttk.Scrollbar(tf, orient="vertical", command=self.log_tree.yview)
        self.log_tree.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_tree.pack(side="left", fill="both", expand=True)

        bottom = tk.Frame(t, bg=C["bg"])
        bottom.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        self.dl_status_var = tk.StringVar(value="Pick a log, then press Download Selected.")
        lbl(bottom, "", 11, False, C["muted"], textvariable=self.dl_status_var).pack(anchor="w")
        self.progress = ttk.Progressbar(bottom, mode="indeterminate", length=300)
        self.progress_slot = tk.Frame(bottom, bg=C["bg"], height=1)  # keeps the layout stable
        self.progress_slot.pack(anchor="w", fill="x")
        lbl(bottom, "Flight logs record at 10 Hz during a flight. Ground sessions record 1 Hz while powered on but not flying.\n"
                    "Max Alt / Max G / Max Speed help tell flights apart. Downloaded flights are marked \"Downloaded\".\n"
                    "Logs can be downloaded before launch, or after landing once the board shows LANDED (no reset needed).",
            10, False, C["muted"], justify="left").pack(anchor="w", pady=(8, 0))

    # ---------------- Console ----------------
    def build_console(self):
        t = self.tab_console
        t.grid_rowconfigure(1, weight=1)
        t.grid_columnconfigure(0, weight=1)

        tb = tk.Frame(t, bg=C["bg"])
        tb.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        make_button(tb, "Clear", self.clear_console, "normal", 11, 16, 7).pack(side="left", padx=(0, 8))
        make_button(tb, "Help", lambda: self.send_command("HELP"), "normal", 11, 16, 7).pack(side="left", padx=(0, 8))
        make_button(tb, "Get Settings", lambda: self.send_command("GETPARAMS"), "normal", 11, 16, 7).pack(side="left", padx=(0, 8))
        make_button(tb, "Status", lambda: self.send_command("STATUS"), "normal", 11, 16, 7).pack(side="left", padx=(0, 16))
        self.show_stream_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(tb, text="Show raw DATA stream", variable=self.show_stream_var,
                        style="Bar.TCheckbutton").pack(side="left")

        frame, self.console = make_text(t, FM(10))
        frame.grid(row=1, column=0, sticky="nsew")
        self.console.tag_configure("err", foreground="#ff6b6b")
        self.console.tag_configure("cmd", foreground="#7cc4ff")
        self.console.tag_configure("gui", foreground=C["warn"])
        self.console.tag_configure("data", foreground="#8fb8d9")

        mrow = tk.Frame(t, bg=C["bg"])
        mrow.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        lbl(mrow, "Command", 11, True).pack(side="left", padx=(0, 8))
        self.cmd_var = tk.StringVar()
        e = make_entry(mrow, self.cmd_var, 10, 12)
        e.pack(side="left", fill="x", expand=True, ipady=7)
        e.bind("<Return>", lambda ev: self.send_manual())
        make_button(mrow, "Send", self.send_manual, "primary", 11, 22, 7).pack(side="left", padx=(8, 0))

    # --------------------------------------------------------
    # Messages / console
    # --------------------------------------------------------
    def set_message(self, text, error=False):
        self.msg_var.set(text)
        self.msg_label.configure(fg="#ff6b6b" if error else C["text"])

    def console_write(self, text, tag=None):
        c = self.console
        c.insert("end", text, tag) if tag else c.insert("end", text)
        lines = int(c.index("end-1c").split(".")[0])
        if lines > 3000:
            c.delete("1.0", "600.0")
        c.see("end")

    def clear_console(self):
        self.console.delete("1.0", "end")

    # --------------------------------------------------------
    # Serial
    # --------------------------------------------------------
    def is_connected(self):
        return bool(self.ser and self.ser.is_open)

    def refresh_ports(self):
        """Refresh the port list; COMET boards are listed first and labelled."""
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            ports = []
        ports.sort(key=lambda p: -score_port(p))
        self.port_map, self.port_scores, names = {}, {}, []
        for p in ports:
            desc = (p.description or "").strip()
            sc = score_port(p)
            if sc >= 100:
                desc = "COMET flight computer (RP2040)"
            label = f"{p.device} - {desc}" if desc and desc != "n/a" else p.device
            self.port_map[label] = p.device
            self.port_scores[p.device] = sc
            names.append(label)
        self.port_combo["values"] = names
        if self.port_var.get() not in self.port_map:
            self.port_var.set(names[0] if names else "")
        return ports

    def best_comet_port(self):
        """Device name of the most likely COMET board, or None."""
        best = max(self.port_scores.items(), key=lambda kv: kv[1], default=(None, 0))
        return best[0] if best[1] >= AUTO_CONNECT_MIN_SCORE else None

    def auto_connect(self):
        """Connect to a COMET board without the user picking a port."""
        if self.is_connected() or self.fw_busy or not self.auto_var.get():
            return
        self.refresh_ports()
        present = set(self.port_scores)
        if self.auto_suppressed and self.auto_suppressed not in present:
            self.auto_suppressed = None  # unplugged since the manual disconnect
        port = self.best_comet_port()
        if not port or port == self.auto_suppressed:
            return
        if time.monotonic() < self.auto_retry_at.get(port, 0):
            return
        for label, dev in self.port_map.items():
            if dev == port:
                self.port_var.set(label)
        self.connect(auto=True)

    def toggle_connection(self):
        if self.is_connected():
            self.auto_suppressed = self.ser.port  # don't instantly reconnect what the user closed
            self.disconnect()
        else:
            self.auto_suppressed = None
            self.connect(auto=False)

    def serial_help(self, port, err):
        if sys.platform.startswith("linux") and (isinstance(err, PermissionError) or "ermission" in str(err)):
            return ("Linux needs permission to use the USB serial port. Fix it once (then unplug/replug):\n"
                    "  ./start_comet_gui_linux.sh --setup      (installs a udev rule)\n"
                    "or:  sudo usermod -aG dialout $USER   and log out / back in.")
        if "denied" in str(err).lower() or "busy" in str(err).lower():
            return (f"{port} is open in another program (Arduino IDE Serial Monitor, another COMET window?). "
                    "Close it and COMET will connect automatically.")
        return str(err)

    def connect(self, auto=False):
        label = self.port_var.get()
        port = self.port_map.get(label, label)
        if not port:
            if not auto:
                show_error(self, "No Port Found",
                           "No flight computer found. Plug it in with a data-capable USB cable; it is found "
                           "automatically (COM port on Windows, /dev/ttyACM0 on Linux).")
            return
        try:
            ser = serial.Serial(port, 115200, timeout=0.05)
        except Exception as e:
            self.ser = None
            help_text = self.serial_help(port, e)
            if auto:
                self.auto_retry_at[port] = time.monotonic() + 8.0  # back off, don't hammer the port
                self.set_message(f"Found {port} but could not open it: {help_text.splitlines()[0]}", True)
                if not self.perm_hint_shown:
                    self.perm_hint_shown = True
                    show_error(self, "Could Not Open the Flight Computer", help_text)
                return
            show_error(self, "Could Not Connect", help_text)
            return

        self.ser = ser
        self.reader_running = True
        threading.Thread(target=self.serial_reader, args=(ser,), daemon=True).start()
        self.connection_lost = False
        self.device_verified = False
        self.fw_version = None
        self.params_seen = False
        self.follow_active_once = True
        self.last_telemetry_wall = None
        self.telemetry_rate_hz = 0.0
        self.clear_plots()
        self.connect_button.configure(text="Disconnect", bg=C["accent"], activebackground=C["accent_dark"], fg="white",
                                      activeforeground="white")
        self.connect_button._base_bg = C["accent"]
        self.status_var.set(f"Connected: {port}")
        self.status_label.configure(fg=C["success"])
        self.fw_var.set(f"GUI {GUI_VERSION}  |  firmware: checking...")
        self.console_write(f"[GUI] Connected to {port}\n", "gui")
        self.set_message(f"Connected to {port}.")

        # Pull current settings from the board.
        self.root.after(300, lambda: self.send_command("GETPARAMS", echo=False, warn=False))
        self.root.after(550, lambda: self.send_command("PROFILE LIST", echo=False, warn=False))
        self.root.after(800, lambda: self.send_command("LIST", echo=False, warn=False))
        self.root.after(2500, self.check_firmware_version)

    @staticmethod
    def version_tuple(v):
        try:
            return tuple(int(x) for x in v.split("."))
        except (ValueError, AttributeError):
            return (0,)

    def firmware_at_least(self, v):
        return bool(self.fw_version) and self.version_tuple(self.fw_version) >= self.version_tuple(v)

    def check_firmware_version(self):
        if not self.is_connected():
            return
        if self.firmware_at_least(GUI_VERSION):
            self.fw_var.set(f"GUI {GUI_VERSION}  |  firmware {self.fw_version}")
            self.fw_label.configure(fg=C["muted"])
        elif self.fw_version or self.params_seen:
            shown = self.fw_version or "older than 7.2"
            self.fw_var.set(f"GUI {GUI_VERSION}  |  firmware {shown}: update recommended")
            self.fw_label.configure(fg=C["warn"])
            self.set_message(f"This board runs older firmware. Flight Setup -> Update Firmware installs {GUI_VERSION}.", True)

    def disconnect(self):
        self.reader_running = False
        ser, self.ser = self.ser, None
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        self.device_verified = False
        self.abort_download("Disconnected.", quiet=True)
        self.connect_button.configure(text="Connect", bg=C["success"], activebackground="#22a85a", fg="black",
                                      activeforeground="black")
        self.connect_button._base_bg = C["success"]
        self.status_var.set("Disconnected")
        self.status_label.configure(fg=C["warn"])
        self.fw_var.set(f"GUI {GUI_VERSION}  |  firmware: not connected")
        self.console_write("[GUI] Disconnected\n", "gui")

    def serial_reader(self, ser):
        buf = b""
        while self.reader_running and self.ser is ser:
            try:
                data = ser.read(1024)
            except Exception as e:
                if self.ser is ser:  # not a deliberate close
                    self.rx_queue.put(f"[GUI ERROR] Serial read failed: {e}")
                    self.rx_queue.put("__LOST__")
                return
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.rx_queue.put(line.decode(errors="replace").rstrip("\r"))

    def process_serial_queue(self):
        n = 0
        while n < 200:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if line == "__LOST__":
                self.handle_connection_lost()
            else:
                self.handle_incoming_line(line)
            n += 1
        self.root.after(40, self.process_serial_queue)

    def handle_connection_lost(self):
        if self.connection_lost:
            return
        self.connection_lost = True
        old = self.ser.port if self.ser else "device"
        self.disconnect()
        if self.fw_busy:
            return
        self.status_var.set("Disconnected - device removed")
        self.set_message(f"Lost connection to {old}. Plug it back in; COMET reconnects automatically.", True)

    def send_command(self, cmd, echo=True, warn=True, ground=False):
        """ground=True marks a user action that needs the board on the ground (IDLE or
        LANDED); only those may trigger the "board is in flight mode" prompt."""
        self.last_cmd_ground = ground
        self.last_cmd_at = time.monotonic()
        if not self.is_connected():
            if warn:
                show_info(self, "Not Connected", "Connect to the flight computer first.")
            return False
        try:
            clean = cmd.strip()
            self.ser.write((clean + "\n").encode())
            if echo:
                self.console_write(f">>> {clean}\n", "cmd")
            return True
        except Exception as e:
            show_error(self, "Send Failed", str(e))
            return False

    def send_many(self, cmds, gap_ms=130, ground=False):
        if not self.is_connected():
            show_info(self, "Not Connected", "Connect to the flight computer first.")
            return
        for i, c in enumerate(cmds):
            self.root.after(i * gap_ms, lambda c=c: self.send_command(c, warn=False, ground=ground))

    def send_manual(self):
        cmd = self.cmd_var.get().strip()
        if cmd:
            self.send_command(cmd)
            self.cmd_var.set("")

    def confirm_command(self, cmd, title, detail):
        live = ("\n\nPYRO OUTPUTS ARE LIVE: the MOSFETs will really switch."
                if self.pyro_live else "\n\nPyro outputs are SAFE: MOSFETs will not switch.")
        if ask_yes_no(self, title,
                      f"{detail}{live}\n\nIf igniters or charges are connected, make sure everyone is clear.",
                      yes=f"Send {cmd}", no="Cancel", danger=True):
            self.send_command(cmd)

    # --------------------------------------------------------
    # Incoming data
    # --------------------------------------------------------
    def handle_incoming_line(self, line):
        if line.startswith("DATA:"):
            self.device_verified = True
            self.handle_telemetry(line)
            if self.show_stream_var.get():
                self.console_write(line + "\n", "data")
            return

        if self.dl is not None and self.download_line(line):
            return

        self.handle_board_line(line)

    def handle_board_line(self, line):
        if "locked during flight" in line:
            self.offer_reset_from_flight()
        elif line.startswith("LOGGER ERROR: LittleFS mount failed"):
            self.root.after(200, self.offer_format_storage)
        if line.startswith("PARAM "):
            self.handle_param(line)
        elif line.startswith("PROFILE "):
            self.handle_profile_line(line)
        elif "COMET LOG ARCHIVE" in line:
            self.clear_log_table()
        else:
            m = LOG_ROW_RE.match(line)
            if m:
                self.add_log_row(m, line)

        if any(k in line.upper() for k in ("COMET", "BOOT", "PARAM ", "PROFILE ")):
            self.device_verified = True

        is_err = line.startswith("ERR") or "FAILED" in line.upper()
        self.console_write(line + "\n", "err" if is_err else None)
        if is_err or line.startswith(STATUS_PREFIXES):
            self.set_message(line.strip(), is_err)

    def handle_param(self, line):
        parts = line.split()
        if len(parts) < 3:
            return
        name, value = parts[1], parts[2]
        self.params_seen = True
        try:
            if name == "STORAGE_OK":
                if value.strip() == "0":
                    self.root.after(200, self.offer_format_storage)
                return
            if name == "FW_VERSION":
                self.fw_version = value.strip()
                self.check_firmware_version()
                return
            if name == "PYRO_ENABLE":
                self.set_pyro_state(value.strip() == "1")
                return
            if name == "MAIN_ALT_MODE":
                idx = int(value)
                if 0 <= idx < len(PROFILE_NAMES):
                    self.set_active_profile(PROFILE_NAMES[idx])
            elif name == "MAIN_ALT" and self.active_profile:
                self.profiles[self.active_profile]["alt_m"] = float(value)
                self.refresh_profile_buttons()
            elif name == "ACCEL_ENABLE" and self.active_profile:
                self.profiles[self.active_profile]["accel"] = value.strip() not in ("0", "OFF")
        except ValueError:
            return
        if self.active_profile == self.selected_profile:
            self.load_profile_editor()

    def handle_profile_line(self, line):
        # PROFILE <COLOR> MAIN_ALT <m> ACCEL_ENABLE <0|1>
        parts = line.split()
        if len(parts) < 6 or parts[1] not in self.profiles:
            return
        color = parts[1]
        vals = {parts[i]: parts[i + 1] for i in range(2, len(parts) - 1, 2)}
        p = self.profiles[color]
        try:
            if "MAIN_ALT" in vals:
                p["alt_m"] = float(vals["MAIN_ALT"])
            if "ACCEL_ENABLE" in vals:
                p["accel"] = vals["ACCEL_ENABLE"] not in ("0", "OFF")
            if "LOCKOUT_MS" in vals:
                p["lockout_s"] = int(vals["LOCKOUT_MS"]) / 1000.0
            if "DROGUE_MAIN_GAP_MS" in vals:
                p["gap_s"] = int(vals["DROGUE_MAIN_GAP_MS"]) / 1000.0
            if "APOGEE_DELAY_MS" in vals:
                p["apogee_delay_s"] = int(vals["APOGEE_DELAY_MS"]) / 1000.0
            if "MAIN_DELAY_MS" in vals:
                p["main_delay_s"] = int(vals["MAIN_DELAY_MS"]) / 1000.0
            if "TIMER_ENABLE" in vals:  # firmware before v7.1: one switch for both
                p["dtimer"] = p["mtimer"] = vals["TIMER_ENABLE"] not in ("0", "OFF")
            if "DROGUE_TIMER_ENABLE" in vals:
                p["dtimer"] = vals["DROGUE_TIMER_ENABLE"] not in ("0", "OFF")
            if "MAIN_TIMER_ENABLE" in vals:
                p["mtimer"] = vals["MAIN_TIMER_ENABLE"] not in ("0", "OFF")
            if "DROGUE_BACKUP_MS" in vals:
                p["dtimer_s"] = int(vals["DROGUE_BACKUP_MS"]) / 1000.0
            if "MAIN_BACKUP_MS" in vals:
                p["mtimer_s"] = int(vals["MAIN_BACKUP_MS"]) / 1000.0
        except ValueError:
            return
        self.refresh_profile_buttons()
        if color == self.selected_profile:
            self.load_profile_editor()

    # ---------------- Telemetry ----------------
    @staticmethod
    def parse_data_line(line):
        parts = line.strip().split(":")
        if len(parts) < 3 or parts[0] != "DATA":
            return {}
        out = {"T_MS": parts[1]}
        i = 2
        while i + 1 < len(parts):
            out[parts[i].strip()] = parts[i + 1].strip()
            i += 2
        return out

    @staticmethod
    def num(v):
        try:
            return float(v)
        except Exception:
            return 0.0

    def handle_telemetry(self, line):
        d = self.parse_data_line(line)
        if not d:
            return
        self.last_telemetry = d

        # Rate from the board's own timestamps: USB delivers lines in bursts, so
        # PC arrival times would show nonsense like 1000 Hz.
        t_ms = self.num(d.get("T_MS"))
        prev = getattr(self, "_last_t_ms", None)
        if prev is not None and 20 <= t_ms - prev <= 2000:
            inst = 1000.0 / (t_ms - prev)
            self.telemetry_rate_hz = 0.85 * self.telemetry_rate_hz + 0.15 * inst if self.telemetry_rate_hz > 0 else inst
        self._last_t_ms = t_ms
        self.last_telemetry_wall = time.monotonic()

        # Active profile follows the board (physical MODE button, etc.)
        try:
            idx = int(d.get("MODE", "-1"))
            if 0 <= idx < len(PROFILE_NAMES) and PROFILE_NAMES[idx] != self.active_profile:
                self.set_active_profile(PROFILE_NAMES[idx])
        except ValueError:
            pass

        self.update_cards(d)

        if not self.plot_paused:
            t = self.num(d.get("T_MS")) / 1000.0
            f = M_TO_FT if self.imperial else 1.0
            self.accel_plot.add_point(t, {k: self.num(d.get(k)) * f for k in ("AX", "AY", "AZ")})
            self.gyro_plot.add_point(t, {k: self.num(d.get(k)) for k in ("GX", "GY", "GZ")})
            self.baro_plot.add_point(t, {"ALT": self.num(d.get("ALT")) * f})

    def update_cards(self, d):
        imp = self.imperial
        f = M_TO_FT if imp else 1.0
        du, su = ("ft", "ft/s") if imp else ("m", "m/s")
        v = self.card_vars
        lab = self.card_labels

        state = d.get("STATE", "--")
        state_text = SHORT_STATE.get(state, state)
        left_ms = self.num(d.get("LOCKOUT_LEFT_MS"))
        if state == "ASCENT_LOCKOUT" and left_ms > 0:
            state_text = f"LOCKOUT {left_ms / 1000.0:.1f}s"
        v["STATE"].set(state_text)
        lab["STATE"].configure(fg=C["accent_light"] if state == "FAULT" else
                               (C["success"] if state == "IDLE" else C["warn"]))
        v["ALT"].set(f"{self.num(d.get('ALT')) * f:.1f} {du}")
        v["MAXALT"].set(f"{self.num(d.get('MAXALT')) * f:.1f} {du}")
        v["VZ"].set(f"{self.num(d.get('VZ')) * f:.1f} {su}")
        v["BATT"].set(f"{self.num(d.get('BATT')):.2f} V")

        if self.active_profile:
            alt = self.num(d.get("MAINALT")) * f
            v["PROFILE"].set(f"{self.active_profile} {alt:.0f}{du}")
        pyro = d.get("PYRO") == "1"
        for key in ("DROGUE", "MAIN"):
            fired = d.get(key) == "1"
            due_ms = self.num(d.get(f"{key}_IN_MS"))  # firmware 7.4+: queued, waiting on delay/gap
            if fired:
                v[key].set("FIRED")
            elif due_ms > 1:
                v[key].set(f"in {due_ms / 1000.0:.1f}s")
            else:
                v[key].set("ready")
            lab[key].configure(fg=C["warn"] if fired or due_ms > 1 else C["success"])
        if pyro:
            v["DROGUE" if d.get("DROGUE") == "1" and d.get("MAIN") != "1" else "MAIN"].set("FIRING")
        v["LOG"].set("ACTIVE" if d.get("LOG") == "1" else "off")
        lab["LOG"].configure(fg=C["warn"] if d.get("LOG") == "1" else C["muted"])
        if "PYRO_OUTPUTS" in d:
            self.set_pyro_state(d.get("PYRO_OUTPUTS") == "1")

        if self.is_connected():
            self.status_var.set(f"Connected: {self.ser.port}  |  {self.telemetry_rate_hz:.1f} Hz")

    # --------------------------------------------------------
    # Units / plots
    # --------------------------------------------------------
    def on_unit_change(self):
        new = self.unit_mode.get()
        if new == self._last_units:
            return
        self._last_units = new
        self.accel_plot.y_label = "ft/s²" if new else "m/s²"
        self.baro_plot.y_label = "ft" if new else "m"
        self.clear_plots()
        self.load_profile_editor()
        self.render_log_table()
        if self.last_telemetry:
            self.update_cards(self.last_telemetry)
        self.set_message(f"Units set to {'Imperial' if new else 'Metric'}. (The board always works in metric.)")

    def refresh_plots(self):
        for p in self.plots:
            p.redraw_if_dirty()
        self.root.after(100, self.refresh_plots)

    def clear_plots(self):
        for p in self.plots:
            p.clear()

    def update_plot_window(self):
        try:
            sec = float(self.window_var.get())
        except ValueError:
            sec = 10.0
        for p in self.plots:
            p.window_seconds = sec
            p.trim()

    def toggle_plot_pause(self):
        self.plot_paused = not self.plot_paused
        self.pause_btn.configure(text="Resume Plots" if self.plot_paused else "Pause Plots")

    # --------------------------------------------------------
    # Profiles
    # --------------------------------------------------------
    def fmt_alt(self, alt_m):
        v = alt_m * (M_TO_FT if self.imperial else 1.0)
        return f"{v:.0f}" if abs(v - round(v)) < 0.05 else f"{v:.1f}"

    def set_active_profile(self, name):
        self.active_profile = name
        if self.follow_active_once:  # first report after connecting: edit what the board will fly
            self.follow_active_once = False
            self.selected_profile = name
            self.load_profile_editor()
        self.refresh_profile_buttons()

    def select_profile(self, name):
        self.selected_profile = name
        self.refresh_profile_buttons()
        self.load_profile_editor()

    def refresh_profile_buttons(self):
        unit = "ft" if self.imperial else "m"
        for name, b in self.profile_buttons.items():
            p = self.profiles[name]
            active = "  (ACTIVE)" if name == self.active_profile else ""
            b.configure(text=f"{name}   {self.fmt_alt(p['alt_m'])} {unit}{active}")
            b.configure(highlightbackground="white" if name == self.selected_profile else b._color)

    def load_profile_editor(self):
        name = self.selected_profile
        p = self.profiles[name]
        unit = "ft" if self.imperial else "m"
        self.editor_title.set(f"{name} profile" + ("  (currently active)" if name == self.active_profile else ""))
        self.alt_var.set(self.fmt_alt(p["alt_m"]))
        self.alt_unit_var.set(unit)
        lo = MIN_MAIN_ALT_M * (M_TO_FT if self.imperial else 1.0)
        hi = MAX_MAIN_ALT_M * (M_TO_FT if self.imperial else 1.0)
        self.alt_hint_var.set(f"Allowed range: {lo:.0f} - {hi:.0f} {unit} (default 200 m). "
                              f"The 100 m minimum is a built-in safety floor.")
        self.accel_var.set(p["accel"])
        self.lockout_var.set(f"{p['lockout_s']:g}")
        self.gap_var.set(f"{p['gap_s']:g}")
        self.apogee_delay_var.set(f"{p['apogee_delay_s']:g}")
        self.main_delay_var.set(f"{p['main_delay_s']:g}")
        self.dtimer_var.set(p["dtimer"])
        self.dtimer_s_var.set(f"{p['dtimer_s']:g}")
        self.mtimer_var.set(p["mtimer"])
        self.mtimer_s_var.set(f"{p['mtimer_s']:g}")
        self.refresh_profile_buttons()

    def save_profile(self):
        text = self.alt_var.get().strip()
        try:
            val = float(text)
        except ValueError:
            show_error(self, "Invalid Altitude", "Enter the altitude as a number, for example 300.")
            return
        alt_m = val / M_TO_FT if self.imperial else val
        if MIN_MAIN_ALT_M - 1.0 <= alt_m < MIN_MAIN_ALT_M:
            alt_m = MIN_MAIN_ALT_M  # rounding from ft -> m
        if not (MIN_MAIN_ALT_M <= alt_m <= MAX_MAIN_ALT_M):
            unit = "ft" if self.imperial else "m"
            lo = MIN_MAIN_ALT_M * (M_TO_FT if self.imperial else 1.0)
            hi = MAX_MAIN_ALT_M * (M_TO_FT if self.imperial else 1.0)
            show_error(self, "Altitude Out of Range",
                       f"Main altitude must be between {lo:.0f} and {hi:.0f} {unit}.")
            return
        try:
            lockout_s = float(self.lockout_var.get().strip())
            gap_s = float(self.gap_var.get().strip())
            apogee_delay_s = float(self.apogee_delay_var.get().strip() or 0)
            main_delay_s = float(self.main_delay_var.get().strip() or 0)
            dtimer_s = float(self.dtimer_s_var.get().strip())
            mtimer_s = float(self.mtimer_s_var.get().strip())
        except ValueError:
            show_error(self, "Invalid Time", "Enter the lockout and timer delays as numbers of seconds.")
            return
        if not (0.0 <= lockout_s <= MAX_LOCKOUT_S):
            show_error(self, "Lockout Out of Range", f"Safety lockout must be 0 - {MAX_LOCKOUT_S:.0f} s.")
            return
        if gap_s != int(gap_s) or not (MIN_GAP_S <= gap_s <= MAX_GAP_S):
            show_error(self, "Gap Out of Range",
                       f"The drogue -> main gap must be a whole number of seconds, {MIN_GAP_S} - {MAX_GAP_S}.")
            return
        for label, secs in (("Drogue", apogee_delay_s), ("Main", main_delay_s)):
            if not (0.0 <= secs <= MAX_EVENT_DELAY_S):
                show_error(self, "Delay Out of Range",
                           f"{label} delay must be 0 - {MAX_EVENT_DELAY_S:.0f} s (0.1 s steps).")
                return
        fw_has_delays = self.firmware_at_least("7.4")
        if not fw_has_delays and (apogee_delay_s or main_delay_s):
            show_error(self, "Firmware Update Needed",
                       "Drogue / main delays need firmware 7.4 or newer on the board. "
                       "Use Update Firmware..., or set both delays to 0.")
            return
        for label, secs in (("Drogue", dtimer_s), ("Main", mtimer_s)):
            if not (MIN_TIMER_S <= secs <= MAX_TIMER_S):
                show_error(self, "Timer Out of Range",
                           f"{label} timer must be {MIN_TIMER_S:.0f} - {MAX_TIMER_S:.0f} s after launch.")
                return
        dtimer, mtimer = self.dtimer_var.get(), self.mtimer_var.get()
        if dtimer and mtimer and mtimer_s < dtimer_s + 2.0:
            show_error(self, "Timer Order",
                       "With both timers on, the main timer must be at least 2 s after the drogue timer.")
            return
        if lockout_s < 1.0 and not ask_yes_no(
                self, "Short Lockout",
                f"A {lockout_s:g} s lockout gives almost no protection against Mach effects or a bumped "
                "launch. Save anyway?", yes="Save", danger=True):
            return
        name = self.selected_profile
        # One command: the board applies and validates every field together.
        self.send_many([
            f"PROFILE SET {name} MAIN_ALT {alt_m:.2f} ACCEL_ENABLE {1 if self.accel_var.get() else 0} "
            f"LOCKOUT_MS {round(lockout_s * 1000)} DROGUE_MAIN_GAP_MS {int(gap_s) * 1000} "
            f"DROGUE_BACKUP_MS {round(dtimer_s * 1000)} MAIN_BACKUP_MS {round(mtimer_s * 1000)} "
            f"DROGUE_TIMER_ENABLE {1 if dtimer else 0} MAIN_TIMER_ENABLE {1 if mtimer else 0}"
            + (f" APOGEE_DELAY_MS {round(apogee_delay_s * 10) * 100} MAIN_DELAY_MS {round(main_delay_s * 10) * 100}"
               if fw_has_delays else ""),  # older boards reject unknown fields (all-or-nothing)
            f"PROFILE GET {name}",
        ], ground=True)
        self.set_message(f"Saving {name} profile...")

    def set_pyro_state(self, live):
        if live == self.pyro_live:
            return
        self.pyro_live = live
        self.pyro_state_var.set("LIVE" if live else "SAFE")
        self.pyro_state_label.configure(fg=C["accent_light"] if live else C["success"])
        self.card_vars["OUTPUTS"].set("LIVE" if live else "SAFE")
        self.card_labels["OUTPUTS"].configure(fg=C["accent_light"] if live else C["success"])

    def arm_pyro(self):
        if not self.is_connected():
            show_info(self, "Not Connected", "Connect to the flight computer first.")
            return
        if not ask_yes_no(self, "Enable Pyro Outputs",
                          "LIVE outputs fire the drogue and main charges for real, and stay LIVE after "
                          "power cycles (this is the normal flight setting).",
                          yes="Make LIVE", danger=True):
            return
        self.send_command("SET PYRO_ENABLE 1", ground=True)
        self.root.after(400, lambda: self.send_command("GETPARAMS", echo=False, warn=False))

    def disarm_pyro(self):
        if self.send_command("SET PYRO_ENABLE 0", ground=True):
            self.root.after(400, lambda: self.send_command("GETPARAMS", echo=False, warn=False))

    def offer_reset_from_flight(self):
        """The board refused a ground-only command because it is in flight mode.
        Only prompt when the refused command came from a user click: background
        commands (auto LIST on connect, marking downloads) must never pop up a
        "reset?" dialog right after the user started a flight."""
        # The board answers commands in order, so a refusal right after a user
        # click belongs to that click.
        if not self.last_cmd_ground or time.monotonic() - self.last_cmd_at > 3:
            self.set_message("Board is in flight mode: settings and logs unlock after landing or Reset to IDLE.", True)
            return
        if time.monotonic() - self.last_lock_prompt < 10:
            return
        self.last_lock_prompt = time.monotonic()
        self.root.after(50, self._ask_reset_from_flight)

    def _ask_reset_from_flight(self):
        if ask_yes_no(self, "Board Is in Flight Mode",
                      "The flight computer thinks it is still flying, so settings and logs are locked.\n\n"
                      "If the rocket is on the ground, reset it to IDLE now? (Any flight log is saved first.)",
                      yes="Reset to IDLE"):
            self.send_command("RESET")
            self.root.after(700, lambda: self.send_command("LIST", echo=False, warn=False))
            self.root.after(900, lambda: self.send_command("PROFILE LIST", echo=False, warn=False))

    def offer_format_storage(self):
        if self.storage_prompted or not self.is_connected():
            return
        self.storage_prompted = True
        if ask_yes_no(self, "Log Storage Not Ready",
                      "The board's log storage could not be opened, so no flights will be recorded.\n\n"
                      "Formatting fixes this but ERASES anything stored there (old logs from a different "
                      "firmware layout). Profiles are re-saved afterwards.",
                      yes="Format Storage", danger=True):
            self.send_command("FORMATFS CONFIRM")
            self.root.after(1500, lambda: self.send_command("LIST", echo=False, warn=False))

    def update_firmware(self):
        if self.fw_busy:
            return
        uf2 = UF2_DEFAULT if UF2_DEFAULT.is_file() else None
        if uf2 is None:
            path = ask_string(self, "Firmware File", "Path to the COMET .uf2 firmware file:", "")
            if not path:
                return
            uf2 = Path(path.strip('"')).expanduser()
            if not uf2.is_file() or uf2.suffix.lower() != ".uf2":
                show_error(self, "Not a Firmware File", f"{uf2} is not a .uf2 file.")
                return
        state = self.last_telemetry.get("STATE") if self.is_connected() else None
        if state and state not in ("IDLE", "LANDED", "FAULT", "BOOT"):
            show_error(self, "Board Is Flying", "Reset the board to IDLE before updating firmware.")
            return
        port = self.ser.port if self.is_connected() else self.best_comet_port()
        if not ask_yes_no(self, "Update Firmware",
                          f"Install {uf2.name} on the flight computer?\n\n"
                          "Saved profiles and flight logs are kept. The board restarts into update mode, "
                          "the file is copied, and COMET reconnects by itself (about 15 s).\n\n"
                          "If nothing happens: unplug the board, hold its BOOTSEL button while plugging it "
                          "back in, then press Update Firmware again.",
                          yes="Update"):
            return
        self.fw_busy = True
        self.disconnect()
        self.set_message("Updating firmware...")
        threading.Thread(target=self._firmware_worker, args=(port, uf2), daemon=True).start()
        self.root.after(200, self._poll_firmware_worker)

    def _firmware_worker(self, port, uf2):
        q = self.fw_queue
        try:
            drive = find_uf2_drive()
            if drive is None and port:
                q.put(("status", "Restarting the board into update mode..."))
                try:  # opening at 1200 baud with DTR low reboots an RP2040 into its UF2 bootloader
                    s = serial.Serial(port, 1200)
                    s.dtr = False
                    time.sleep(0.3)
                    s.close()
                except Exception:
                    pass  # the port disappears as the board reboots
            deadline = time.monotonic() + 25
            while drive is None and time.monotonic() < deadline:
                time.sleep(0.5)
                drive = find_uf2_drive()
            if drive is None:
                q.put(("nodrive", str(uf2)))
                return
            q.put(("status", f"Copying firmware to {drive} ..."))
            data = uf2.read_bytes()
            try:
                with open(Path(drive) / uf2.name, "wb") as fh:
                    fh.write(data)
                    fh.flush()
                    os.fsync(fh.fileno())
            except OSError:
                # The bootloader reboots as soon as the last block lands, so the
                # drive can vanish before the OS finishes closing the file.
                if Path(drive).exists():
                    raise
            q.put(("done", drive))
        except Exception as e:
            q.put(("error", str(e)))

    def _poll_firmware_worker(self):
        try:
            while True:
                kind, info = self.fw_queue.get_nowait()
                if kind == "status":
                    self.set_message(info)
                    continue
                self.fw_busy = False
                if kind == "done":
                    self.set_message("Firmware installed. The board is restarting; reconnecting automatically.")
                    self.auto_suppressed = None
                    show_info(self, "Firmware Updated",
                              "The new firmware is installed and the board is restarting. COMET reconnects "
                              "automatically in a few seconds.")
                elif kind == "nodrive":
                    self.set_message("Update mode drive (RPI-RP2) not found.", True)
                    if message_box(self, "Update Drive Not Found",
                                   "The board's update drive (RPI-RP2) did not appear.\n\n"
                                   "1. Unplug the board.\n2. Hold the BOOTSEL button and plug it back in.\n"
                                   "3. Press Update Firmware again, or copy this file onto the RPI-RP2 drive:\n"
                                   f"   {info}",
                                   [("Close", False, "normal"), ("Open Firmware Folder", True, "success")],
                                   default=False):
                        open_folder(str(Path(info).parent))
                else:
                    self.set_message(f"Firmware update failed: {info}", True)
                    show_error(self, "Firmware Update Failed", info)
                return
        except queue.Empty:
            pass
        self.root.after(200, self._poll_firmware_worker)

    def activate_profile(self):
        self.send_command(f"PROFILE APPLY {self.selected_profile}", ground=True)

    # --------------------------------------------------------
    # Log table
    # --------------------------------------------------------
    @staticmethod
    def fmt_duration(sec):
        sec = int(sec)
        return f"{sec // 60}m {sec % 60:02d}s" if sec >= 60 else f"{sec}s"

    def clear_log_table(self):
        self.log_rows = {}
        for item in self.log_tree.get_children():
            self.log_tree.delete(item)

    def add_log_row(self, m, line=""):
        star, kind, slot, ident, records, seconds, status = m.groups()
        slot, ident, records, seconds, status = int(slot), int(ident), int(records), int(seconds), int(status)
        iid = f"{kind}:{slot}"
        row = {"kind": kind, "slot": slot, "id": ident, "records": records, "seconds": seconds,
               "status": LOG_STATUS.get(status, str(status)), "status_code": status,
               "newest": bool(star), "max_alt": None, "max_g": None, "max_v": None}
        sm = LOG_STATS_RE.search(line) if kind == "FLIGHT" else None
        if sm:
            row["max_alt"], row["max_g"], row["max_v"] = (float(x) for x in sm.groups())
        self.log_rows[iid] = row
        if self.log_tree.exists(iid):
            self.log_tree.delete(iid)
        self.log_tree.insert("", "end", iid=iid, values=self.log_row_values(row))
        self.sort_logs_by(self.log_sort[0], keep_direction=True)

    def log_row_values(self, row):
        f = M_TO_FT if self.imperial else 1.0
        du, su = ("ft", "ft/s") if self.imperial else ("m", "m/s")
        dash = "—"
        return (
            "Flight" if row["kind"] == "FLIGHT" else "Ground session",
            row["slot"], row["id"], self.fmt_duration(row["seconds"]), row["records"],
            dash if row["max_alt"] is None else f"{row['max_alt'] * f:.0f} {du}",
            dash if row["max_g"] is None else f"{row['max_g']:.1f} g",
            dash if row["max_v"] is None else f"{row['max_v'] * f:.0f} {su}",
            row["status"], "yes" if row["newest"] else "")

    def render_log_table(self):
        for iid, row in self.log_rows.items():
            if self.log_tree.exists(iid):
                self.log_tree.item(iid, values=self.log_row_values(row))

    def sort_logs_by(self, key, keep_direction=False):
        prev_key, prev_desc = self.log_sort
        if keep_direction:
            desc = prev_desc
        elif key == prev_key:
            desc = not prev_desc  # second click on the same header flips it
        else:
            # Numbers default to biggest-first; text columns to A-Z.
            desc = key in ("id", "maxalt", "maxg", "maxv", "records", "length", "latest")
        self.log_sort = (key, desc)
        field = {"type": "kind", "slot": "slot", "id": "id", "length": "seconds",
                 "records": "records", "maxalt": "max_alt", "maxg": "max_g",
                 "maxv": "max_v", "status": "status", "latest": "newest"}[key]

        # Flights always above ground sessions; rows without stats go last.
        items = []
        for is_flight in (True, False):
            group = [i for i in self.log_rows if (self.log_rows[i]["kind"] == "FLIGHT") == is_flight]
            have = [i for i in group if self.log_rows[i][field] is not None]
            have.sort(key=lambda i: self.log_rows[i][field], reverse=desc)
            items += have + [i for i in group if self.log_rows[i][field] is None]
        for idx, iid in enumerate(items):
            if self.log_tree.exists(iid):
                self.log_tree.move(iid, "", idx)
        arrows = {k: "" for k in self.log_tree["columns"]}
        arrows[key] = " ▼" if desc else " ▲"
        titles = {"type": "Type", "slot": "Slot", "id": "Log #", "length": "Length", "records": "Records",
                  "maxalt": "Max Alt", "maxg": "Max G", "maxv": "Max Speed", "status": "Status", "latest": "Newest"}
        for k, t in titles.items():
            self.log_tree.heading(k, text=t + arrows[k])

    def selected_log(self):
        sel = self.log_tree.selection()
        if not sel:
            show_info(self, "Select a Log", "Click a log in the list first.")
            return None
        return self.log_rows.get(sel[0])

    def file_stub(self, kind, slot):
        row = self.log_rows.get(f"{kind}:{slot}")
        ident = f"_log{row['id']}" if row else ""
        return f"COMET_{'flight' if kind == 'FLIGHT' else 'ground'}_slot{int(slot):02d}{ident}"

    def download_selected(self):
        row = self.selected_log()
        if not row:
            return
        if row["kind"] == "FLIGHT" and row["status_code"] == 1:
            show_info(self, "Still Recording", "That flight is still being recorded. Press RESET (Flight Setup tab) "
                                               "or wait for landing, then refresh the list.")
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default = f"{self.file_stub(row['kind'], row['slot'])}_{stamp}.csv"
        path = csv_save_dialog(self, "Save Log as CSV", default)
        if not path:
            return
        cmd = f"DUMPCSV {row['slot']}" if row["kind"] == "FLIGHT" else f"STATICCSV {row['slot']}"
        self.start_download({"mode": "single", "path": path, "cmd": cmd})

    def export_all(self):
        if not self.log_rows:
            show_info(self, "No Logs", "No logs are listed yet. Press Refresh List first.")
            return
        folder = choose_folder(self, self.csv_dir, title="Choose where to save all logs")
        if not folder:
            return
        self.csv_dir = folder
        sub = Path(folder) / f"COMET_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        try:
            sub.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            show_error(self, "Folder Error", str(e))
            return
        self.start_download({"mode": "all", "folder": str(sub), "cmd": "EXPORTALLCSV"})

    def erase_selected(self):
        row = self.selected_log()
        if not row:
            return
        what = f"flight log in slot {row['slot']}" if row["kind"] == "FLIGHT" else f"ground session in slot {row['slot']}"
        if not ask_yes_no(self, "Erase Log", f"Permanently erase the {what}?\n\nThis cannot be undone.",
                          yes="Erase", danger=True):
            return
        cmd = f"ERASE {row['slot']}" if row["kind"] == "FLIGHT" else f"ERASESTATIC {row['slot']}"
        self.send_command(cmd, ground=True)
        self.root.after(500, lambda: self.send_command("LIST", echo=False, warn=False))

    def erase_all(self):
        if not ask_yes_no(self, "Erase ALL Logs",
                          "Permanently erase every flight log and ground-session log?\n\n"
                          "Your saved flight profiles are kept. This cannot be undone.",
                          yes="Erase Everything", danger=True):
            return
        self.send_command("FORMATLOG", ground=True)
        self.root.after(800, lambda: self.send_command("LIST", echo=False, warn=False))

    # --------------------------------------------------------
    # CSV download state machine
    # --------------------------------------------------------
    def start_download(self, dl):
        if self.dl is not None:
            show_info(self, "Busy", "A download is already in progress.")
            return
        dl.update({"cur": None, "saved": [], "saved_flights": [], "rows": 0, "last_rx": time.monotonic()})
        self.dl = dl
        self.progress.pack(in_=self.progress_slot, anchor="w", pady=(6, 0))
        self.progress.start(12)
        self.dl_status_var.set("Requesting data from the board...")
        if not self.send_command(dl["cmd"], ground=True):
            self.abort_download("Could not send command.", quiet=True)

    def download_line(self, line):
        """Return True if the line was consumed by the download."""
        dl = self.dl
        dl["last_rx"] = time.monotonic()

        if line.startswith("ERR"):
            self.abort_download(line)
            return True

        if line.startswith("BEGIN_CSV"):
            parts = line.split()
            kind = "FLIGHT" if len(parts) > 1 and parts[1] == "SLOT" else "STATIC"
            try:
                slot = int(parts[2])
            except (IndexError, ValueError):
                slot = 0
            dl["cur"] = {"kind": kind, "slot": slot, "lines": []}
            self.dl_status_var.set(f"Downloading {'flight' if kind == 'FLIGHT' else 'ground'} slot {slot}...")
            return True

        if line.startswith("END_CSV"):
            self.finish_current_csv()
            if dl["mode"] == "single":
                self.finish_download()
            return True

        if line.startswith("END_EXPORT_ALL"):
            self.finish_download()
            return True

        if dl["cur"] is not None:
            if line.strip():
                dl["cur"]["lines"].append(line)
                dl["rows"] += 1
                if dl["rows"] % 50 == 0:
                    self.dl_status_var.set(f"Downloading... {dl['rows']} rows received")
            return True

        return False

    def finish_current_csv(self):
        dl = self.dl
        cur = dl["cur"]
        dl["cur"] = None
        if not cur or not cur["lines"]:
            return
        if dl["mode"] == "single":
            path = dl["path"]
        else:
            path = str(Path(dl["folder"]) / f"{self.file_stub(cur['kind'], cur['slot'])}.csv")
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                fh.write("\n".join(cur["lines"]) + "\n")
            dl["saved"].append(path)
            if cur["kind"] == "FLIGHT":
                dl["saved_flights"].append(cur["slot"])
        except Exception as e:
            self.abort_download(f"Could not save file: {e}")

    def finish_download(self):
        dl = self.dl
        if dl is None:
            return
        self.dl = None
        self.progress.stop()
        self.progress.pack_forget()
        saved = dl["saved"]
        if not saved:
            self.dl_status_var.set("Download finished, but no data was received.")
            show_info(self, "No Data", "The board did not send any log data.")
            return
        folder = str(Path(saved[0]).parent)
        self.dl_status_var.set(f"Saved {len(saved)} file(s) to {folder}")
        # Mark saved flights as Downloaded on the board, then refresh the list.
        marks = [f"MARKDOWNLOADED {slot}" for slot in dl["saved_flights"]
                 if self.log_rows.get(f"FLIGHT:{slot}", {}).get("status_code") == 2]
        if marks and self.is_connected():
            self.send_many(marks + ["LIST"], gap_ms=80)
        self.set_message(f"Saved {len(saved)} log file(s).")
        msg = (f"Saved: {saved[0]}" if len(saved) == 1 else f"Saved {len(saved)} files to:\n{folder}")
        if message_box(self, "Download Complete", msg,
                       [("Close", False, "normal"), ("Open Folder", True, "success")], default=False):
            open_folder(folder)

    def abort_download(self, reason, quiet=False):
        if self.dl is None:
            return
        self.dl = None
        self.progress.stop()
        self.progress.pack_forget()
        self.dl_status_var.set(f"Download stopped: {reason}")
        if not quiet:
            show_error(self, "Download Failed", reason)

    # --------------------------------------------------------
    # Periodic checks
    # --------------------------------------------------------
    def monitor(self):
        try:
            self.tick += 1
            if self.dl is not None and time.monotonic() - self.dl["last_rx"] > 15:
                self.abort_download("The board stopped responding (timed out).")

            if not self.is_connected() and self.tick % 4 == 0:
                self.auto_connect()
            if self.is_connected() and self.tick % 4 == 0:
                port = self.ser.port
                if port not in {p.device for p in serial.tools.list_ports.comports()}:
                    self.handle_connection_lost()
                elif self.last_telemetry_wall is not None and \
                        time.monotonic() - self.last_telemetry_wall > 4.0:
                    self.status_var.set(f"Connected: {port}  |  no recent telemetry")
        finally:
            self.root.after(500, self.monitor)

    def on_close(self):
        self.disconnect()
        self.root.destroy()


# ============================================================
# FLIGHT-LOG PLOTTER
#   Flight Logs tab -> Plot Selected (or double-click a log), or Open CSV...
#   Altitude + vertical speed, acceleration, gyro; launch / apogee / drogue /
#   main / landed marked, MOSFET on-time shaded, flight summary side panel.
# ============================================================

G = None  # this module (theme, widgets, dialogs); set by install()

G0 = 9.80665

# ============================================================
# CSV PARSING
# ============================================================

# Header names are matched after upper-casing, turning punctuation into "_",
# and also with a unit suffix removed (ALT_M -> ALT, AX_MPS2 -> AX ...).
ALIASES = {
    "t": ["T_MS", "TIME_MS", "MS", "MILLIS", "TIMESTAMP_MS", "TIMESTAMP", "T", "TIME",
          "TIME_S", "T_S", "SECONDS", "ELAPSED", "US", "MICROS", "T_US"],
    "alt": ["ALT", "ALTITUDE", "BARO_ALT", "ALT_AGL", "AGL", "REL_ALT", "HEIGHT", "ALT_BARO",
            "BARO_ALTITUDE", "FILTERED_ALT", "ALT_FILT", "ALT_FILTERED"],
    "vz": ["VZ", "V_Z", "VEL", "VELOCITY", "SPEED", "VSPEED", "VERT_VEL", "VERT_SPEED",
           "VERTICAL_SPEED", "VERTICAL_VELOCITY", "CLIMB_RATE", "VEL_Z"],
    "ax": ["AX", "A_X", "ACC_X", "ACCEL_X", "ACCX", "ACCELX"],
    "ay": ["AY", "A_Y", "ACC_Y", "ACCEL_Y", "ACCY", "ACCELY"],
    "az": ["AZ", "A_Z", "ACC_Z", "ACCEL_Z", "ACCZ", "ACCELZ"],
    "gx": ["GX", "G_X", "GYRO_X", "GYROX", "GYR_X", "WX"],
    "gy": ["GY", "G_Y", "GYRO_Y", "GYROY", "GYR_Y", "WY"],
    "gz": ["GZ", "G_Z", "GYRO_Z", "GYROZ", "GYR_Z", "WZ"],
    "state": ["STATE", "FLIGHT_STATE", "STATE_NAME", "PHASE"],
    "drogue": ["DROGUE", "DROGUE_FIRED", "DROGUE_FIRE", "DROGUE_DEPLOYED", "DROGUE_CMD"],
    "main": ["MAIN", "MAIN_FIRED", "MAIN_FIRE", "MAIN_DEPLOYED", "MAIN_CMD"],
    "pyro": ["PYRO", "PYRO_ON", "PYRO_ACTIVE", "PYRO_FIRING", "FIRING", "MOSFET", "MOSFET_ON"],
    "pyro_live": ["PYRO_OUTPUTS", "PYRO_ENABLE", "PYRO_LIVE", "OUTPUTS_LIVE"],
    "flags": ["FLAGS", "STATUS_FLAGS", "STATE_FLAGS"],
    "note": ["EVENT_NOTE", "NOTE", "EVENT_TEXT", "MESSAGE", "EVENT_MSG"],
}
# Bits of the COMET log "flags" column (firmware 7.3): drogue fired (latched),
# main fired (latched), pyro MOSFET switched on right now.
COMET_FLAG_BITS = {"drogue": 0, "main": 1, "pyro": 3}
# Log values stuck at one of these mean the field overflowed (int16 / 100 etc.).
CLIP_LIMITS = (327.67, 3276.7, 32767.0)
SUFFIX_UNITS = [("_MPS2", "mps2"), ("_MS2", "mps2"), ("_MSS", "mps2"), ("_M_S2", "mps2"),
                ("_DPS", "deg/s"), ("_DEG_S", "deg/s"), ("_RAD_S", "rad/s"), ("_RADS", "rad/s"),
                ("_MPS", "mps"), ("_M_S", "mps"), ("_MS", "ms"), ("_US", "us"), ("_FT", "ft"),
                ("_M", "m"), ("_S", "s"), ("_G", "g")]
NUMERIC_KEYS = ("alt", "vz", "ax", "ay", "az", "gx", "gy", "gz", "drogue", "main", "pyro", "pyro_live", "flags")


def _f(x):
    s = x.strip().upper()
    if s in ("TRUE", "YES", "ON", "Y"):
        return 1.0
    if s in ("FALSE", "NO", "OFF", "N"):
        return 0.0
    try:
        return float(s)
    except ValueError:
        return math.nan


def _isnum(x):
    try:
        float(x)
        return True
    except ValueError:
        return False


def _match(header):
    raw = header.strip()
    unit = ""
    m = re.search(r"[\(\[]\s*([^\)\]]*?)\s*[\)\]]", raw)
    if m:
        unit = m.group(1).lower().replace(" ", "")
        raw = raw[:m.start()] + raw[m.end():]
    n = re.sub(r"[^A-Za-z0-9]+", "_", raw).strip("_").upper()
    cands = [(n, unit)]
    for suf, u in SUFFIX_UNITS:
        if n.endswith(suf) and len(n) > len(suf):
            cands.append((n[:-len(suf)], unit or u))
    for name, u in cands:
        for key, names in ALIASES.items():
            if name in names:
                return key, u
    return None, ""


class FlightLog:
    def __init__(self):
        self.t = []            # seconds, increasing
        self.cols = {}         # key -> list of float (NaN = missing)
        self.state = None      # list of upper-case strings, or None
        self.state_numeric = False
        self.accel_unit = "mps2"
        self.gyro_unit = "deg/s"
        self.vz_derived = False
        self.found = {}        # key -> header text
        self.header = []
        self.from_flags = []   # keys decoded from the flags bit-field
        self.notes_text = []   # (row index, board event message)
        self.clipped = []      # (key, limit) for columns stuck at the format limit


def parse_csv_lines(lines, default_rate_hz=10.0):
    clean = [ln for ln in lines if ln.strip() and not ln.lstrip().startswith(("#", "//"))]
    rows = []
    for r in csv.reader(clean):
        r = [x.strip() for x in r]
        while r and r[-1] == "":
            r.pop()
        if r:
            rows.append(r)

    # Header = the mostly-text row followed by the most rows of the same width.
    best, best_count = None, 0
    for i, r in enumerate(rows):
        if len(r) < 3 or sum(_isnum(x) for x in r) > len(r) // 2:
            continue
        count = sum(1 for rr in rows[i + 1:] if abs(len(rr) - len(r)) <= 1)
        if count > best_count:
            best, best_count = i, count
    if best is None:
        raise ValueError("No column header row was found in this log.")
    header = rows[best]
    width = len(header)
    body = []
    for r in rows[best + 1:]:
        if abs(len(r) - width) > 1 or sum(_isnum(x) for x in r) < width // 2:
            continue
        body.append((r + [""] * width)[:width])
    if len(body) < 2:
        raise ValueError("This log has fewer than 2 data rows -- nothing to plot.")

    log = FlightLog()
    log.header = header
    cands = {}
    for ci, h in enumerate(header):
        key, unit = _match(h)
        if key:
            cands.setdefault(key, []).append((ci, unit, h))
    mapping = {}
    for key, lst in cands.items():
        pick = lst[0]
        if key == "state":  # e.g. "state" = 3 and "state_name" = ASCENT: use the names
            for c in lst:
                vals = [r[c[0]] for r in body[:200] if r[c[0]].strip()]
                if vals and not all(_isnum(v) for v in vals):
                    pick = c
                    break
        mapping[key] = (pick[0], pick[1])
        log.found[key] = pick[2]

    # Time base
    if "t" in mapping:
        ci = mapping["t"][0]
        tv = [_f(r[ci]) for r in body]
    else:
        tv = [i / default_rate_hz for i in range(len(body))]
    order = sorted((i for i, x in enumerate(tv) if x == x), key=lambda i: tv[i])
    keep, last = [], None
    for i in order:
        if last is None or tv[i] > last:
            keep.append(i)
            last = tv[i]
    if len(keep) < 2:
        raise ValueError("The time column has no usable values.")
    diffs = [tv[keep[k + 1]] - tv[keep[k]] for k in range(len(keep) - 1)]
    med = statistics.median(diffs)
    scale = 1e-6 if med >= 20000 else (1e-3 if med >= 20 else 1.0)
    log.t = [tv[i] * scale for i in keep]

    for key in NUMERIC_KEYS:
        if key in mapping:
            ci = mapping[key][0]
            log.cols[key] = [_f(body[i][ci]) for i in keep]
    if "state" in mapping:
        ci = mapping["state"][0]
        log.state = [body[i][ci].strip().upper() for i in keep]
        vals = [s for s in log.state if s]
        log.state_numeric = bool(vals) and all(_isnum(s) for s in vals)
    if "note" in mapping:
        ci = mapping["note"][0]
        log.notes_text = [(k, body[i][ci].strip()) for k, i in enumerate(keep) if body[i][ci].strip()]
    if "flags" in log.cols:
        fl = log.cols["flags"]
        for key, bit in COMET_FLAG_BITS.items():
            if key not in log.cols:
                log.cols[key] = [float((int(v) >> bit) & 1) if v == v else math.nan for v in fl]
                log.from_flags.append(key)
    for key in ("alt", "vz", "gx", "gy", "gz"):
        vals = [abs(v) for v in log.cols.get(key, []) if v == v]
        if vals:
            top = max(vals)
            if any(abs(top - lim) < 1e-6 for lim in CLIP_LIMITS) and vals.count(top) >= 3:
                log.clipped.append((key, top))

    # Accelerometer units: header first, else magnitude at the start of the log.
    acc = [k for k in ("ax", "ay", "az") if k in log.cols]
    if acc:
        unit = mapping[acc[0]][1]
        if unit == "g":
            log.accel_unit = "g"
        elif unit in ("mps2", "m/s2", "m/s^2", "m/s²", "ms-2"):
            log.accel_unit = "mps2"
        else:
            mags = []
            for i in range(min(50, len(log.t))):
                vs = [log.cols[k][i] for k in acc]
                if all(v == v for v in vs):
                    mags.append(math.sqrt(sum(v * v for v in vs)))
            if mags and 0.3 < statistics.median(mags) < 3.0:
                log.accel_unit = "g"
        if len(acc) == 3:
            log.cols["amag"] = [math.sqrt(x * x + y * y + z * z)
                                for x, y, z in zip(log.cols["ax"], log.cols["ay"], log.cols["az"])]
    gyr = [k for k in ("gx", "gy", "gz") if k in log.cols]
    if gyr and "rad" in mapping[gyr[0]][1]:
        log.gyro_unit = "rad/s"

    # No speed column: derive it from altitude over ~0.5 s.
    if "vz" not in log.cols and "alt" in log.cols:
        t, alt, n = log.t, log.cols["alt"], len(log.t)
        dt = statistics.median(t[i + 1] - t[i] for i in range(n - 1))
        k = max(1, int(round(0.25 / max(dt, 1e-6))))
        vz = []
        for i in range(n):
            a, b = max(0, i - k), min(n - 1, i + k)
            if b > a and alt[a] == alt[a] and alt[b] == alt[b]:
                vz.append((alt[b] - alt[a]) / (t[b] - t[a]))
            else:
                vz.append(math.nan)
        log.cols["vz"] = vz
        log.vz_derived = True
    return log


# ============================================================
# EVENT DETECTION
# ============================================================

GROUND_STATES = {"BOOT", "IDLE", "READY", "ARMED", "PAD", "GROUND", "CAL", "CALIBRATING", "INIT", ""}

EVENT_STYLE = {
    # key: (table label, plot label, color, minor)
    "launch": ("Launch", "Launch", "#4da3ff", False),
    "lockout": ("Lockout ended", "Lockout end", "#8a8a8a", True),
    "apogee": ("Apogee detected", "Apogee", "#ffb020", False),
    "noclimb": ("No-climb trigger", "No climb", "#ffb020", True),  # pop test (firmware 7.3+)
    "drogue": ("Drogue fired", "Drogue", "#ff5964", False),
    "main": ("Main fired", "Main", "#2ecc71", False),
    "landed": ("Landed", "Landed", "#b8b8b8", False),
    "fault": ("FAULT", "FAULT", "#ff6b6b", False),
    "touchdown": ("Touchdown (est.)", "Touchdown", "#b8b8b8", True),
}
# Board event messages -> event. The keyword nearest the start of the message wins
# ("Safety lockout ended; apogee/main detection active" is a lockout event).
NOTE_KEYWORDS = (("NO CLIMB", "noclimb"),  # "No climb after launch: ... end of lockout" (7.3+)
                 ("LAUNCH", "launch"), ("LIFTOFF", "launch"), ("LOCKOUT", "lockout"),
                 ("APOGEE", "apogee"), ("DROGUE", "drogue"), ("MAIN", "main"),
                 ("LANDED", "landed"), ("LANDING", "landed"), ("TOUCHDOWN", "landed"), ("FAULT", "fault"))


def _note_event(text):
    u = text.upper()
    hits = [(u.find(kw), key) for kw, key in NOTE_KEYWORDS if kw in u]
    if not hits:
        return None
    key = min(hits)[1]
    if key in ("drogue", "main") and "FIRE" not in u:
        return None  # "main backup timer armed" etc. is not a firing
    return key


class Analysis:
    pass


def _nanmax(vals, i0=0, i1=None):
    best, bi = -math.inf, None
    for i in range(i0, len(vals) if i1 is None else i1):
        v = vals[i]
        if v == v and v > best:
            best, bi = v, i
    return bi


def analyse(log):
    a = Analysis()
    t, n, cols = log.t, len(log.t), log.cols
    st = log.state if (log.state and not log.state_numeric) else None
    found = {}

    def first(key, i):
        if key not in found or key == "fault":
            found.setdefault(key, i)

    launch_i = trans_drogue = trans_main = None
    if st:
        if st[0] not in GROUND_STATES and st[0] not in ("LANDED", "FAULT"):
            launch_i = 0  # log starts at launch detection
        for i in range(1, n):
            p, s = st[i - 1], st[i]
            if p == s:
                continue
            if launch_i is None and p in GROUND_STATES and s not in GROUND_STATES \
                    and s not in ("FAULT", "LANDED"):
                launch_i = i
            if p in ("ASCENT_LOCKOUT", "LOCKOUT"):
                first("lockout", i)
            if s.startswith("APOGEE"):
                first("apogee", i)
            if s.startswith("DROGUE") and trans_drogue is None:
                trans_drogue = i
            if s.startswith("MAIN") and trans_main is None:
                trans_main = i
            if s == "LANDED":
                first("landed", i)
            if s == "FAULT":
                first("fault", i)
    # Board event messages are the firmware's own word: they win over state guesses.
    # The board writes one note per row, so notes from the same moment (lockout
    # end + no-climb + drogue, apogee + drogue) land on consecutive rows and the
    # later ones trail their event by a row or two. Date each note to the start
    # of its run of consecutive note rows.
    note_rows = {i for i, _ in log.notes_text}

    def burst_start(i):
        while i - 1 in note_rows:
            i -= 1
        return i

    note_found = {}
    for i, text in log.notes_text:
        key = _note_event(text)
        if key and key not in note_found:
            note_found[key] = burst_start(i)
    for key in ("launch", "lockout", "apogee", "noclimb", "landed", "fault"):
        if key in note_found:
            found[key] = note_found[key]
    if "launch" in note_found:
        launch_i = note_found["launch"]
    if launch_i is not None:
        found["launch"] = launch_i

    def rising(col):
        for i in range(n):
            if col[i] > 0.5 and (i == 0 or not col[i - 1] > 0.5):
                return i
        return None

    d_i = rising(cols["drogue"]) if "drogue" in cols else None
    m_i = rising(cols["main"]) if "main" in cols else None

    # MOSFET on-time: runs of PYRO = 1, credited to whichever channel fired last.
    runs = []
    if "pyro" in cols:
        p, i = cols["pyro"], 0
        while i < n:
            if p[i] > 0.5:
                j = i
                while j + 1 < n and p[j + 1] > 0.5:
                    j += 1
                runs.append((i, j))
                i = j + 1
            else:
                i += 1
    spans = []
    for k, (i, j) in enumerate(runs):
        who, best = None, -1
        for fi, name in ((d_i, "drogue"), (m_i, "main")):
            if fi is not None and fi <= i + 1 and fi > best:
                who, best = name, fi
        if who is None:
            who = "drogue" if k == 0 else "main"
        dt = (t[j + 1] - t[j]) if j + 1 < n else (t[j] - t[j - 1] if j > 0 else 0.1)
        spans.append({"i": i, "j": j, "who": who, "dur": t[j] - t[i] + dt})
    if d_i is None:
        d_i = note_found.get("drogue")
    if d_i is None:
        d_i = next((s["i"] for s in spans if s["who"] == "drogue"), trans_drogue)
    if m_i is None:
        m_i = note_found.get("main")
    if m_i is None:
        m_i = next((s["i"] for s in spans if s["who"] == "main"), trans_main)
    if d_i is not None:
        found["drogue"] = d_i
    if m_i is not None:
        found["main"] = m_i

    alt, vz0 = cols.get("alt"), cols.get("vz")
    if "landed" not in found and alt and m_i is not None:
        # near the pad AND nearly stopped (a baro glitch can dip below 2 m at speed)
        td = next((i for i in range(m_i, n) if alt[i] == alt[i] and alt[i] < 2.0
                   and (vz0 is None or (vz0[i] == vz0[i] and abs(vz0[i]) < 3.0))), None)
        if td is not None:
            found["touchdown"] = td

    tz = t[launch_i] if launch_i is not None else t[0]
    a.tz = tz
    a.has_launch = launch_i is not None
    a.t = [x - tz for x in t]

    live = cols.get("pyro_live")
    a.events = []
    for key, i in found.items():
        table, short, color, minor = EVENT_STYLE[key]
        safe = key in ("drogue", "main") and live is not None and live[i] == 0
        if safe:
            table += " (SAFE: no fire)"
        a.events.append({"key": key, "i": i, "t": a.t[i], "label": table,
                         "short": f"{short} {a.t[i]:.1f}s", "color": color, "minor": minor})
    a.events.sort(key=lambda e: e["t"])
    a.spans = [{"t0": a.t[s["i"]], "t1": a.t[s["i"]] + s["dur"], "dur": s["dur"],
                "color": EVENT_STYLE[s["who"]][2], "who": s["who"]} for s in spans]

    # Stats (SI units)
    alt, vz = cols.get("alt"), cols.get("vz")
    s = a.stats = {}
    if alt:
        pi = _nanmax(alt)
        if pi is not None:
            s["peak_i"], s["peak_t"], s["peak_alt"] = pi, a.t[pi], alt[pi]
            if "apogee" in found:
                s["detect_delay"] = a.t[found["apogee"]] - a.t[pi]
    if vz:
        vi = _nanmax(vz)
        if vi is not None:
            s["max_v"], s["max_v_t"] = vz[vi], a.t[vi]
    if "amag" in cols:
        gi = _nanmax(cols["amag"])
        if gi is not None:
            v = cols["amag"][gi]
            s["max_g"] = v if log.accel_unit == "g" else v / G0
            s["max_g_t"] = a.t[gi]
    end_i = found.get("landed", found.get("touchdown"))

    def descent(i_from, t_after, i_to):
        if vz is None or i_from is None:
            return None
        t_lo = a.t[i_from] + t_after
        t_hi = a.t[i_to] - 0.5 if i_to is not None else a.t[-1]
        vals = [vz[i] for i in range(i_from, n) if t_lo <= a.t[i] <= t_hi and vz[i] == vz[i]]
        return abs(statistics.median(vals)) if len(vals) >= 3 else None

    # Measured timing, to check the drogue / main +T delays and the gap (firmware 7.4).
    # The drogue / main indices come from the flags column, so they are exact to
    # one log row; a board note can trail its event by up to ~0.2 s.
    trig_i = found.get("apogee", found.get("noclimb"))
    if trig_i is not None and d_i is not None and d_i >= trig_i:
        s["trigger_to_drogue"] = a.t[d_i] - a.t[trig_i]
        s["trigger_kind"] = "Apogee" if "apogee" in found else "No-climb trigger"
    if d_i is not None and m_i is not None:
        s["drogue_to_main"] = a.t[m_i] - a.t[d_i]
    s["drogue_descent"] = descent(d_i, 2.0, m_i if m_i is not None else end_i)
    s["main_descent"] = descent(m_i, 3.0, end_i)
    if launch_i is not None and end_i is not None:
        s["flight_time"] = a.t[end_i]
        s["flight_time_est"] = "landed" not in found
    s["duration"] = t[-1] - t[0]
    s["samples"] = n
    s["rate"] = (n - 1) / s["duration"] if s["duration"] > 0 else 0

    a.board_log = [(a.t[i], text) for i, text in log.notes_text]
    a.clipped = dict(log.clipped)
    a.notes = []
    if log.from_flags:
        a.notes.append("Drogue / main / MOSFET times decoded from the flags column.")
    for key, lim in log.clipped:
        name = {"alt": "Altitude", "vz": "Vertical speed"}.get(key, key.upper())
        a.notes.append(f"{name} is stuck at {lim:g} in this log: that is the largest value the log "
                       "format can store, so the real value was higher there (firmware log field "
                       "overflow).")
    if not st and log.state and log.state_numeric:
        a.notes.append("STATE column is numeric, so launch / apogee / landing could not be named.")
    elif not log.state:
        a.notes.append("No STATE column: launch, apogee and landing events are not available.")
    if "pyro" not in cols:
        a.notes.append("No PYRO or flags column: MOSFET on-time is not shown.")
    if log.vz_derived:
        a.notes.append("No speed column: speed is derived from altitude.")
    if live is not None and any(v == 0 for v in live if v == v) and found.get("drogue") is not None:
        if live[found["drogue"]] == 0:
            a.notes.append("Outputs were SAFE during this log: the MOSFETs did not switch.")
    return a


# ============================================================
# DRAWING HELPERS
# ============================================================

def _blend(c, bg, alpha):
    c, bg = c.lstrip("#"), bg.lstrip("#")
    out = []
    for k in (0, 2, 4):
        a, b = int(c[k:k + 2], 16), int(bg[k:k + 2], 16)
        out.append(round(a * alpha + b * (1 - alpha)))
    return "#%02x%02x%02x" % tuple(out)


def _nice_step(span, n):
    raw = span / max(1, n)
    if raw <= 0:
        return 1.0
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            return m * mag
    return 10 * mag


def _ticks(lo, hi, n):
    step = _nice_step(hi - lo, n)
    v = math.ceil(lo / step) * step
    out = []
    while v <= hi + step * 1e-9 and len(out) < 60:
        out.append(v)
        v += step
    return out, step


def _fmt_tick(v, step):
    if abs(v) < step * 1e-6:
        v = 0.0
    d = max(0, -int(math.floor(math.log10(step))))
    if abs(step * 10 ** d - round(step * 10 ** d)) > 1e-6:
        d += 1
    return f"{v:.{d}f}"


def _value_at(t, v, x):
    if not t:
        return None
    i = bisect.bisect_left(t, x)
    if i >= len(t):
        i = len(t) - 1
    elif i > 0 and x - t[i - 1] < t[i] - x:
        i -= 1
    val = v[i]
    return None if val != val else val


def _nz(v, digits=0):
    """Round, without ever printing -0."""
    return round(v, digits) + 0.0


def _overlap(a, b):
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


# ============================================================
# PLOT CANVAS
# ============================================================

class TracePlot(tk.Canvas):
    PAD_L, PAD_R1, PAD_R2, PAD_T, PAD_B = 70, 18, 70, 36, 28

    def __init__(self, parent, win, title, left_label, right_label=None, labels=False, height=180):
        super().__init__(parent, bg=G.C["field"], highlightthickness=1,
                         highlightbackground=G.C["border"], height=height, cursor="crosshair")
        self.win = win
        self.title = title
        self.left_label = left_label
        self.right_label = right_label
        self.show_event_labels = labels
        self.series = []
        self.markers = []
        self.box = None
        self.yr = {}
        self._drag = None
        self._pan = None
        self.bind("<Configure>", lambda e: self.redraw())
        self.bind("<Enter>", lambda e: self.focus_set())
        self.bind("<Motion>", self._on_motion)
        self.bind("<Leave>", lambda e: self.win.set_cursor(None))
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Double-Button-1>", lambda e: self.win.reset_view())
        for b in ("2", "3"):
            self.bind(f"<ButtonPress-{b}>", self._on_pan_start)
            self.bind(f"<B{b}-Motion>", self._on_pan)
            self.bind(f"<ButtonRelease-{b}>", lambda e: setattr(self, "_pan", None))
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.bind(seq, self._on_wheel)

    def add_series(self, name, t, v, color, axis="left", width=2, unit=""):
        self.series.append({"name": name, "t": t, "v": v, "color": color,
                            "axis": axis, "width": width, "unit": unit})

    def add_marker(self, t, v, text, color, axis="left"):
        self.markers.append((t, v, text, color, axis))

    # ---- coordinate helpers ----
    def tx(self, t):
        x0, _, x1, _ = self.box
        v0, v1 = self.win.view
        return x0 + (t - v0) / max(1e-9, v1 - v0) * (x1 - x0)

    def xt(self, x):
        x0, _, x1, _ = self.box
        v0, v1 = self.win.view
        return v0 + (x - x0) / max(1, x1 - x0) * (v1 - v0)

    def ty(self, val, axis):
        _, y0, _, y1 = self.box
        lo, hi = self.yr[axis]
        return y1 - (val - lo) / (hi - lo) * (y1 - y0)

    @staticmethod
    def _slice(t, v0, v1, extra=1):
        return (max(0, bisect.bisect_left(t, v0) - extra),
                min(len(t), bisect.bisect_right(t, v1) + extra))

    def _segments(self, s, v0, v1):
        t, v, ax = s["t"], s["v"], s["axis"]
        i0, i1 = self._slice(t, v0, v1)
        x0, _, x1, _ = self.box
        pw = max(1, int(x1 - x0))
        span = max(1e-9, v1 - v0)
        segs, cur = [], []
        if i1 - i0 <= pw * 2:
            for i in range(i0, i1):
                y = v[i]
                if y != y:
                    if len(cur) >= 4:
                        segs.append(cur)
                    cur = []
                    continue
                cur += (self.tx(t[i]), self.ty(y, ax))
        else:  # more samples than pixels: keep min and max of each pixel column
            b = lo_i = hi_i = None
            for i in range(i0, i1 + 1):
                y = v[i] if i < i1 else None
                if y is not None and y != y:
                    continue
                bi = int((t[i] - v0) / span * pw) if i < i1 else None
                if bi != b:
                    if b is not None:
                        xx = x0 + b + 0.5
                        for j in sorted({lo_i, hi_i}):
                            cur += (xx, self.ty(v[j], ax))
                    b, lo_i, hi_i = bi, i, i
                else:
                    if y < v[lo_i]:
                        lo_i = i
                    if y > v[hi_i]:
                        hi_i = i
        if len(cur) >= 4:
            segs.append(cur)
        return segs

    # ---- drawing ----
    def redraw(self):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 160 or h < 90 or not self.series:
            self.box = None
            return
        C = G.C
        # Same right margin on every plot in the window so event lines line up.
        x0, y0 = self.PAD_L, self.PAD_T
        x1, y1 = w - (self.PAD_R2 if self.win.any_right else self.PAD_R1), h - self.PAD_B
        self.box = (x0, y0, x1, y1)
        v0, v1 = self.win.view
        label_font = G.F(10, True)

        # MOSFET on-time spans
        for sp in self.win.spans:
            if sp["t1"] < v0 or sp["t0"] > v1:
                continue
            xa, xb = max(x0, self.tx(sp["t0"])), min(x1, self.tx(sp["t1"]))
            self.create_rectangle(xa, y0, max(xb, xa + 3), y1, outline="",
                                  fill=_blend(sp["color"], C["field"], 0.30))

        # Y ranges from what is visible
        self.yr = {}
        for axis in ("left", "right"):
            lo, hi = math.inf, -math.inf
            for s in self.series:
                if s["axis"] != axis:
                    continue
                i0, i1 = self._slice(s["t"], v0, v1, extra=0)
                for y in s["v"][i0:i1]:
                    if y == y:
                        lo, hi = min(lo, y), max(hi, y)
            if lo > hi:
                continue
            if hi - lo < 1e-6:
                lo, hi = lo - 1, hi + 1
            pad = (hi - lo) * 0.08
            top = (hi - lo) * 0.30 if self.show_event_labels else pad  # room for labels
            self.yr[axis] = (lo - pad, hi + top)

        # grid + left axis ticks
        nyt = max(2, int((y1 - y0) / 38))
        if "left" in self.yr:
            lo, hi = self.yr["left"]
            vals, step = _ticks(lo, hi, nyt)
            for val in vals:
                y = self.ty(val, "left")
                self.create_line(x0, y, x1, y, fill="#242424")
        vals_t, step_t = _ticks(v0, v1, max(2, int((x1 - x0) / 90)))
        for val in vals_t:
            x = self.tx(val)
            self.create_line(x, y0, x, y1, fill="#1e1e1e")
        if "right" in self.yr and self.yr["right"][0] < 0 < self.yr["right"][1]:
            y = self.ty(0, "right")
            rc = next(s["color"] for s in self.series if s["axis"] == "right")
            self.create_line(x0, y, x1, y, fill=_blend(rc, C["field"], 0.45), dash=(6, 4))

        # series
        for s in self.series:
            if s["axis"] not in self.yr:
                continue
            for seg in self._segments(s, v0, v1):
                self.create_line(seg, fill=s["color"], width=s["width"])

        # mask overdraw outside the plot box
        bg = C["field"]
        self.create_rectangle(0, 0, x0, h, fill=bg, outline="")
        self.create_rectangle(x1, 0, w, h, fill=bg, outline="")
        self.create_rectangle(0, 0, w, y0, fill=bg, outline="")
        self.create_rectangle(0, y1, w, h, fill=bg, outline="")
        self.create_rectangle(x0, y0, x1, y1, outline=C["border"])

        # tick labels
        if "left" in self.yr:
            lo, hi = self.yr["left"]
            vals, step = _ticks(lo, hi, nyt)
            for val in vals:
                self.create_text(x0 - 8, self.ty(val, "left"), anchor="e",
                                 text=_fmt_tick(val, step), fill=C["muted"], font=G.FM(9))
        if "right" in self.yr:
            lo, hi = self.yr["right"]
            rc = next(s["color"] for s in self.series if s["axis"] == "right")
            vals, step = _ticks(lo, hi, nyt)
            for val in vals:
                self.create_text(x1 + 8, self.ty(val, "right"), anchor="w",
                                 text=_fmt_tick(val, step), fill=rc, font=G.FM(9))
            if self.right_label:
                self.create_text(w - 12, (y0 + y1) / 2, text=self.right_label, angle=270,
                                 fill=rc, font=G.F(10))
        for val in vals_t:
            x = self.tx(val)
            if x0 - 1 <= x <= x1 + 1:
                self.create_text(x, y1 + 5, anchor="n", text=_fmt_tick(val, step_t) + " s",
                                 fill=C["muted"], font=G.FM(9))
        if self.left_label:
            self.create_text(16, (y0 + y1) / 2, text=self.left_label, angle=90,
                             fill=C["muted"], font=G.F(10))

        # event lines (+ labels on the labelled plot)
        placed = []
        for ev in self.win.events:
            if not v0 <= ev["t"] <= v1:
                continue
            x = self.tx(ev["t"])
            self.create_line(x, y0 + 1, x, y1 - 1, fill=ev["color"], dash=(5, 3),
                             width=1 if ev["minor"] else 2)
            if not self.show_event_labels:
                continue
            tid, bb = None, None
            for row in range(6):
                ty = y0 + 5 + row * 19
                if tid:
                    self.delete(tid)
                tid = self.create_text(x + 5, ty, anchor="nw", text=ev["short"],
                                       fill=ev["color"], font=label_font)
                bb = self.bbox(tid)
                if bb[2] > x1 - 3:
                    self.itemconfigure(tid, anchor="ne")
                    self.coords(tid, x - 5, ty)
                    bb = self.bbox(tid)
                if not any(_overlap(bb, p) for p in placed):
                    break
            placed.append(bb)
            r = self.create_rectangle(bb[0] - 3, bb[1] - 1, bb[2] + 3, bb[3] + 1,
                                      fill=C["field"], outline=_blend(ev["color"], C["field"], 0.6))
            self.tag_raise(tid, r)

        # markers
        for t, v, text, color, axis in self.markers:
            if not (v0 <= t <= v1) or axis not in self.yr:
                continue
            x, y = self.tx(t), self.ty(v, axis)
            self.create_oval(x - 5, y - 5, x + 5, y + 5, fill=color, outline=C["field"], width=2)
            if text:
                tid = self.create_text(x + 9, y - 4, anchor="sw", text=text, fill=color, font=label_font)
                bb = self.bbox(tid)
                if bb[2] > x1 - 3:
                    self.itemconfigure(tid, anchor="se")
                    self.coords(tid, x - 9, y - 4)

        # title + legend
        self.create_text(12, 10, anchor="nw", text=self.title, fill=C["accent_light"], font=G.F(11, True))
        lx = w - 14
        for s in reversed(self.series):
            tid = self.create_text(lx, 11, anchor="ne", text=s["name"], fill=C["text"], font=G.F(10, True))
            bb = self.bbox(tid)
            self.create_line(bb[0] - 26, 18, bb[0] - 6, 18, fill=s["color"], width=3)
            lx = bb[0] - 40
        self.draw_cursor(self.win.cursor)

    def draw_cursor(self, t):
        self.delete("cur")
        if t is None or not self.box:
            return
        C = G.C
        x0, y0, x1, y1 = self.box
        v0, v1 = self.win.view
        if not v0 <= t <= v1:
            return
        x = self.tx(t)
        self.create_line(x, y0, x, y1, fill="#6a6a6a", tags="cur")
        rows = [(f"T{t:+.2f} s", C["text"])]
        near = [ev for ev in self.win.events if abs(self.tx(ev["t"]) - x) <= 5]
        rows += [(ev["label"], ev["color"]) for ev in near]
        for s in self.series:
            val = _value_at(s["t"], s["v"], t)
            if val is None or s["axis"] not in self.yr:
                continue
            rows.append((f"{s['name']}: {val:.2f} {s['unit']}", s["color"]))
            y = self.ty(val, s["axis"])
            self.create_oval(x - 3, y - 3, x + 3, y + 3, fill=s["color"], outline="", tags="cur")
        ids = []
        for k, (text, color) in enumerate(rows):
            ids.append(self.create_text(x + 12, y0 + 8 + k * 16, anchor="nw", text=text, fill=color,
                                        font=G.FM(10, k == 0), tags=("cur", "curtxt")))
        bb = self.bbox("curtxt")
        if bb[2] > x1 - 4:
            self.move("curtxt", -(bb[2] - bb[0]) - 24, 0)
            bb = self.bbox("curtxt")
        r = self.create_rectangle(bb[0] - 6, bb[1] - 4, bb[2] + 6, bb[3] + 4, fill="#141414",
                                  outline=C["border"], tags="cur")
        for i in ids:
            self.tag_raise(i, r)

    # ---- mouse ----
    def _clamp_x(self, x):
        x0, _, x1, _ = self.box
        return min(max(x, x0), x1)

    def _on_motion(self, e):
        if not self.box:
            return
        x0, y0, x1, y1 = self.box
        self.win.set_cursor(self.xt(e.x) if x0 <= e.x <= x1 else None)

    def _on_press(self, e):
        if self.box and self.box[0] <= e.x <= self.box[2]:
            self._drag = e.x

    def _on_drag(self, e):
        self._on_motion(e)
        if self._drag is None:
            return
        self.delete("sel")
        _, y0, _, y1 = self.box
        xa, xb = sorted((self._drag, self._clamp_x(e.x)))
        self.create_rectangle(xa, y0, xb, y1, outline=G.C["accent_light"], dash=(3, 2), tags="sel")

    def _on_release(self, e):
        if self._drag is None or not self.box:
            return
        xa, xb = sorted((self._drag, self._clamp_x(e.x)))
        self._drag = None
        self.delete("sel")
        if xb - xa >= 8:
            self.win.set_view(self.xt(xa), self.xt(xb))

    def _on_wheel(self, e):
        if not self.box:
            return
        up = getattr(e, "num", 0) == 4 or getattr(e, "delta", 0) > 0
        self.win.zoom_at(self.xt(self._clamp_x(e.x)), 0.8 if up else 1.25)

    def _on_pan_start(self, e):
        if self.box:
            self._pan = (e.x, self.win.view)

    def _on_pan(self, e):
        if not self._pan or not self.box:
            return
        sx, (v0, v1) = self._pan
        x0, _, x1, _ = self.box
        dt = (e.x - sx) / max(1, x1 - x0) * (v1 - v0)
        self.win.set_view(v0 - dt, v1 - dt, keep_span=True)


# ============================================================
# PLOT WINDOW
# ============================================================

class FlightPlotWindow(tk.Toplevel):
    def __init__(self, app, log, name, lines=None, row=None, default_name=None):
        super().__init__(app.root)
        C = G.C
        self.app, self.log, self.lines, self.row = app, log, lines, row
        self.default_name = default_name or "COMET_log.csv"
        self.configure(bg=C["bg"])
        self.title(f"COMET - {name}")
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        w, h = min(1500, sw - 60), min(940, sh - 90)
        self.geometry(f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 4)}")
        self.minsize(min(900, w), min(600, h))

        a = self.a = analyse(log)
        self.events, self.spans = a.events, a.spans
        self.full = (a.t[0], a.t[-1])
        self.view = self.full
        self.cursor = None
        self.any_right = False

        imp = app.imperial
        f = G.M_TO_FT if imp else 1.0
        self.f = f
        self.du, self.su = ("ft", "ft/s") if imp else ("m", "m/s")

        # toolbar
        top = tk.Frame(self, bg=C["bg"], padx=14, pady=10)
        top.pack(fill="x")
        G.lbl(top, name, 16, True, C["accent_light"]).pack(side="left")
        zero = "time 0 = launch" if a.has_launch else "time 0 = start of log"
        G.lbl(top, f"   {zero}   |   drag = zoom   wheel = zoom   right-drag = pan   "
                   f"double-click = full log", 10, False, C["muted"]).pack(side="left", pady=(4, 0))
        G.make_button(top, "Full Log", self.reset_view, "normal", 11, 16, 7).pack(side="right")
        if lines:
            G.make_button(top, "Save CSV...", self.save_csv, "success", 11, 16, 7
                          ).pack(side="right", padx=(0, 8))

        body = tk.Frame(self, bg=C["bg"], padx=14)
        body.pack(fill="both", expand=True, pady=(0, 14))
        body.grid_columnconfigure(0, weight=1)
        body.grid_rowconfigure(0, weight=1)
        pf = tk.Frame(body, bg=C["bg"])
        pf.grid(row=0, column=0, sticky="nsew")
        pf.grid_columnconfigure(0, weight=1)

        self.plots = []
        cols, t = log.cols, a.t
        du, su = self.du, self.su
        if "alt" in cols:
            p = TracePlot(pf, self, "Altitude and vertical speed" if "vz" in cols else "Altitude",
                          f"Altitude ({du})", f"Vertical speed ({su})", labels=True, height=260)
            p.add_series("Altitude", t, [x * f for x in cols["alt"]], "#7cc4ff", unit=du)
            if "vz" in cols:
                p.add_series("Speed" + (" (derived)" if log.vz_derived else ""), t,
                             [x * f for x in cols["vz"]], "#c084fc", axis="right", unit=su)
            st = a.stats
            if "peak_i" in st:
                p.add_marker(st["peak_t"], st["peak_alt"] * f,
                             f"Max {st['peak_alt'] * f:.0f} {du}", "#ffb020")
            for ev in a.events:
                v = cols["alt"][ev["i"]]
                if ev["key"] in ("apogee", "drogue", "main") and v == v:
                    p.add_marker(ev["t"], v * f, "", ev["color"])
            self.plots.append((p, 3))
        acc = [k for k in ("ax", "ay", "az") if k in cols]
        if acc:
            if log.accel_unit == "g":
                af, au = 1.0, "g"
            else:
                af, au = f, ("ft/s²" if imp else "m/s²")
            p = TracePlot(pf, self, "Acceleration", f"Acceleration ({au})")
            for k, color in zip(("ax", "ay", "az"), ("#ff5964", "#2ecc71", "#4da3ff")):
                if k in cols:
                    p.add_series(k.upper(), t, [x * af for x in cols[k]], color, unit=au)
            if "amag" in cols:
                p.add_series("|A|", t, [x * af for x in cols["amag"]], "#e5e5e5", width=1, unit=au)
            self.plots.append((p, 2))
        gyr = [k for k in ("gx", "gy", "gz") if k in cols]
        if gyr:
            gu = log.gyro_unit
            p = TracePlot(pf, self, "Gyroscope", f"Rotation rate ({gu})")
            for k, color in zip(("gx", "gy", "gz"), ("#ff5964", "#2ecc71", "#4da3ff")):
                if k in cols:
                    p.add_series(k.upper(), t, cols[k], color, unit=gu)
            self.plots.append((p, 2))
        for r, (p, weight) in enumerate(self.plots):
            pf.grid_rowconfigure(r, weight=weight)
            p.grid(row=r, column=0, sticky="nsew", pady=(0 if r == 0 else 8, 0))
        self.plots = [p for p, _ in self.plots]
        self.any_right = any(s["axis"] == "right" for p in self.plots for s in p.series)

        self.build_side(body)

    # ---- side panel ----
    def build_side(self, body):
        C = G.C
        holder = tk.Frame(body, bg=C["bg"], width=450)
        holder.grid(row=0, column=1, sticky="ns", padx=(12, 0))
        holder.pack_propagate(False)
        sf = G.ScrollFrame(holder, C["bg"])
        sf.pack(fill="both", expand=True)
        side = sf.inner
        a, f, du, su = self.a, self.f, self.du, self.su
        alt, vz = self.log.cols.get("alt"), self.log.cols.get("vz")

        card, inner = G.make_card(side, "Key events")
        card.pack(fill="x", padx=(0, 4))
        if a.events:
            tv = ttk.Treeview(inner, columns=("ev", "t", "alt", "v"), show="headings",
                              selectmode="browse", height=min(10, len(a.events)))
            for key, text, width in (("ev", "Event", 150), ("t", "Time", 74),
                                     ("alt", "Alt", 74), ("v", "Speed", 84)):
                tv.heading(key, text=text)
                tv.column(key, width=width, anchor="w", stretch=key == "ev")
            for k, ev in enumerate(a.events):
                i = ev["i"]
                av = f"{_nz(alt[i] * f):.0f} {du}" if alt and alt[i] == alt[i] else "—"
                vv = f"{_nz(vz[i] * f, 1):.1f}" if vz and vz[i] == vz[i] else "—"
                tv.insert("", "end", iid=str(k), values=(ev["label"], f"{ev['t']:.2f} s", av, vv))
            tv.pack(fill="x")
            tv.bind("<Double-1>", lambda e: self._zoom_event(tv))
            G.lbl(inner, "Double-click an event to zoom to it.", 9, False, C["muted"]
                  ).pack(anchor="w", pady=(6, 0))
        else:
            G.lbl(inner, "No events found in this log.", 11, False, C["muted"]).pack(anchor="w")

        card, inner = G.make_card(side, "Flight summary")
        card.pack(fill="x", padx=(0, 4), pady=(12, 0))
        s = a.stats
        rows = []
        if "peak_alt" in s:
            rows.append(("Apogee", f"{s['peak_alt'] * f:.1f} {du}  at {s['peak_t']:.2f} s"))
        if "detect_delay" in s:
            rows.append(("Apogee detected", f"{s['detect_delay']:+.2f} s after peak"))
        if "max_v" in s:
            clip = " (log limit; real peak higher)" if "vz" in a.clipped else ""
            rows.append(("Max speed", f"{s['max_v'] * f:.1f} {su}{clip}  at {s['max_v_t']:.2f} s"))
        if "max_g" in s:
            rows.append(("Max acceleration", f"{s['max_g']:.1f} g  at {s['max_g_t']:.2f} s"))
        if "trigger_to_drogue" in s:
            rows.append((f"{s['trigger_kind']} → drogue", f"{s['trigger_to_drogue']:.1f} s  (drogue delay)"))
        if "drogue_to_main" in s:
            rows.append(("Drogue → main", f"{s['drogue_to_main']:.1f} s"))
        for sp in a.spans:
            rows.append((f"{sp['who'].capitalize()} MOSFET on", f"{sp['dur']:.2f} s  from {sp['t0']:.2f} s"))
        if s.get("drogue_descent") is not None:
            rows.append(("Descent under drogue", f"{s['drogue_descent'] * f:.1f} {su}"))
        if s.get("main_descent") is not None:
            rows.append(("Descent under main", f"{s['main_descent'] * f:.1f} {su}"))
        if "flight_time" in s:
            est = " (est. touchdown)" if s.get("flight_time_est") else ""
            rows.append(("Flight time", f"{s['flight_time']:.1f} s{est}"))
        rows.append(("Log length", f"{s['duration']:.1f} s  ({s['samples']} rows, {s['rate']:.0f} Hz)"))
        grid = tk.Frame(inner, bg=C["panel"])
        grid.pack(fill="x")
        for r, (k, v) in enumerate(rows):
            G.lbl(grid, k, 10, True, C["muted"]).grid(row=r, column=0, sticky="nw", pady=2, padx=(0, 10))
            G.lbl(grid, v, 10, False, wraplength=250, justify="left").grid(row=r, column=1, sticky="nw", pady=2)

        notes = list(a.notes)
        used = ", ".join(self.log.found[k] for k in ("t", "alt", "vz", "ax", "ay", "az", "gx", "gy", "gz",
                                                     "state", "drogue", "main", "pyro", "pyro_live",
                                                     "flags", "note")
                         if k in self.log.found)
        notes.append(f"Columns used: {used or 'none recognised'}")
        if a.board_log:
            card, inner = G.make_card(side, "Board messages")
            card.pack(fill="x", padx=(0, 4), pady=(12, 0))
            for tt, text in a.board_log:
                m = G.lbl(inner, f"{tt:7.2f} s   {text}", 10, False, C["text"], wraplength=390, justify="left")
                m.configure(font=G.FM(10))
                m.pack(anchor="w", pady=1)
        card, inner = G.make_card(side, "Notes")
        card.pack(fill="x", padx=(0, 4), pady=(12, 0))
        for n in notes:
            G.lbl(inner, "• " + n, 10, False, C["muted"], wraplength=390, justify="left"
                  ).pack(anchor="w", pady=1)

    def _zoom_event(self, tv):
        sel = tv.selection()
        if not sel:
            return
        t = self.events[int(sel[0])]["t"]
        half = max(3.0, (self.full[1] - self.full[0]) * 0.06)
        self.set_view(t - half, t + half)
        self.set_cursor(t)

    # ---- view control ----
    def redraw_all(self):
        for p in self.plots:
            p.redraw()

    def set_view(self, a, b, keep_span=False):
        f0, f1 = self.full
        span = b - a
        if keep_span:
            span = min(span, f1 - f0)
            if a < f0:
                a, b = f0, f0 + span
            if b > f1:
                a, b = f1 - span, f1
        else:
            a, b = max(f0, a), min(f1, b)
            if b - a < 0.3:
                mid = (a + b) / 2
                a, b = mid - 0.15, mid + 0.15
        self.view = (a, b)
        self.redraw_all()

    def reset_view(self):
        self.view = self.full
        self.redraw_all()

    def zoom_at(self, t, factor):
        v0, v1 = self.view
        a, b = t - (t - v0) * factor, t + (v1 - t) * factor
        if b - a >= self.full[1] - self.full[0]:
            self.reset_view()
        else:
            self.set_view(a, b, keep_span=True)

    def set_cursor(self, t):
        self.cursor = t
        for p in self.plots:
            p.draw_cursor(t)

    # ---- save ----
    def save_csv(self):
        app = self.app
        folder = G.choose_folder(app, app.csv_dir, parent=self, title="Choose where to save the CSV")
        if not folder:
            return
        name = G.ask_string(app, "File Name", "Save the log as:", self.default_name, parent=self)
        if not name:
            return
        if not name.lower().endswith(".csv"):
            name += ".csv"
        if any(ch in name for ch in '\\/:*?"<>|'):
            G.show_error(app, "Invalid File Name", "A file name can't contain  \\ / : * ? \" < > |", parent=self)
            return
        path = Path(folder) / name
        if path.exists() and not G.ask_yes_no(app, "Overwrite File?", f"This file already exists:\n{path}\n\n"
                                              "Replace it?", yes="Replace", danger=True, parent=self):
            return
        try:
            path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")
        except Exception as e:
            G.show_error(app, "Could Not Save", str(e), parent=self)
            return
        app.csv_dir = folder
        row = self.row
        if row and row.get("kind") == "FLIGHT" and row.get("status_code") == 2 and app.is_connected():
            app.send_many([f"MARKDOWNLOADED {row['slot']}", "LIST"], gap_ms=80)
        app.set_message(f"Saved {path}")
        G.show_info(app, "Saved", f"Saved: {path}", parent=self)


# ============================================================
# CSV FILE PICKER (dark, like the rest of the GUI)
# ============================================================

def choose_csv(app, start, parent=None):
    C = G.C
    d = G.ModalDialog(app, "Open a Saved Log", parent)
    d.minsize(720, 520)
    state = {"path": Path(start) if start and Path(start).is_dir() else Path.home(), "items": []}
    body = tk.Frame(d, bg=C["bg"], padx=20, pady=18)
    body.pack(fill="both", expand=True)
    G.lbl(body, "Open a Saved Log", 16, True, C["accent_light"]).pack(anchor="w", pady=(0, 10))
    prow = tk.Frame(body, bg=C["bg"])
    prow.pack(fill="x")
    path_var = tk.StringVar()
    qrow = tk.Frame(body, bg=C["bg"])
    qrow.pack(fill="x", pady=(10, 8))
    lframe = tk.Frame(body, bg=C["border"], padx=1, pady=1)
    lframe.pack(fill="both", expand=True)
    inner = tk.Frame(lframe, bg=C["field"])
    inner.pack(fill="both", expand=True)
    lb = tk.Listbox(inner, font=G.F(12), bg=C["field"], fg=C["text"], selectbackground=C["accent"],
                    selectforeground="white", activestyle="none", relief="flat", bd=0, highlightthickness=0)
    sb = ttk.Scrollbar(inner, orient="vertical", command=lb.yview)
    lb.configure(yscrollcommand=sb.set)
    sb.pack(side="right", fill="y")
    lb.pack(side="left", fill="both", expand=True, padx=6, pady=4)
    G.lbl(body, "Folders end in /. CSV files are newest first. Double-click to open.", 10, False,
          C["muted"]).pack(anchor="w", pady=(8, 0))

    def refresh():
        p = state["path"]
        path_var.set(str(p))
        lb.delete(0, "end")
        state["items"] = []
        try:
            kids = [c for c in p.iterdir() if not c.name.startswith(".")]
        except OSError:
            lb.insert("end", "(cannot open this folder)")
            return
        dirs = sorted((c for c in kids if c.is_dir()), key=lambda c: c.name.lower())
        files = sorted((c for c in kids if c.is_file() and c.suffix.lower() == ".csv"),
                       key=lambda c: -c.stat().st_mtime)
        for c in dirs:
            lb.insert("end", c.name + "/")
            state["items"].append(c)
        for c in files:
            stamp = datetime.fromtimestamp(c.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
            lb.insert("end", f"{c.name}      {stamp}")
            state["items"].append(c)
        if not state["items"]:
            lb.insert("end", "(no folders or CSV files here)")

    def go(p):
        p = Path(p).expanduser()
        if p.is_dir():
            state["path"] = p
            refresh()

    def activate(_e=None):
        sel = lb.curselection()
        if not sel or sel[0] >= len(state["items"]):
            return
        item = state["items"][sel[0]]
        if item.is_dir():
            go(item)
        else:
            d.close(str(item))

    G.make_button(prow, "Up", lambda: go(state["path"].parent), "normal", 11, 16, 7).pack(side="left", padx=(0, 8))
    e = G.make_entry(prow, path_var, 10, 12)
    e.pack(side="left", fill="x", expand=True, ipady=6)
    e.bind("<Return>", lambda ev: go(path_var.get()))
    G.make_button(prow, "Go", lambda: go(path_var.get()), "normal", 11, 16, 7).pack(side="left", padx=(8, 0))
    home = Path.home()
    quick = [("Last used", Path(app.csv_dir)), ("Home", home), ("Documents", home / "Documents"),
             ("Desktop", home / "Desktop"), ("Downloads", home / "Downloads")]
    for name, p in quick:
        if p.is_dir():
            G.make_button(qrow, name, lambda p=p: go(p), "normal", 10, 10, 5).pack(side="left", padx=(0, 6))
    lb.bind("<Double-Button-1>", activate)
    lb.bind("<Return>", activate)
    brow = tk.Frame(body, bg=C["bg"])
    brow.pack(fill="x", pady=(16, 0))
    G.make_button(brow, "Open", activate, "success", 12, 24, 10).pack(side="right")
    G.make_button(brow, "Cancel", lambda: d.close(None), "normal", 12, 22, 10).pack(side="right", padx=(0, 10))
    refresh()
    return d.run(820, 600)


# ============================================================
# HOOK-UP
# ============================================================

def open_plot(app, lines, name, row=None, kind="FLIGHT", default_name=None, keep_lines=True):
    try:
        log = parse_csv_lines(lines, 1.0 if kind == "STATIC" else 10.0)
    except ValueError as e:
        G.show_error(app, "Cannot Plot This Log", str(e))
        return None
    if not any(k in log.cols for k in ("alt", "ax", "ay", "az", "gx", "gy", "gz")):
        G.show_error(app, "Cannot Plot This Log",
                     "No altitude, accelerometer or gyroscope columns were recognised.\n\n"
                     f"Columns in the file: {', '.join(log.header)}")
        return None
    return FlightPlotWindow(app, log, name, lines if keep_lines else None, row, default_name)


def plot_selected(app):
    row = app.selected_log()
    if not row:
        return
    if row["kind"] == "FLIGHT" and row["status_code"] == 1:
        G.show_info(app, "Still Recording", "That flight is still being recorded. Press RESET (Flight Setup tab) "
                                            "or wait for landing, then refresh the list.")
        return
    if not app.is_connected():
        G.show_info(app, "Not Connected", "Connect to the flight computer first, or use Open CSV... "
                                          "to plot a log you saved earlier.")
        return
    cmd = f"DUMPCSV {row['slot']}" if row["kind"] == "FLIGHT" else f"STATICCSV {row['slot']}"
    app.start_download({"mode": "plot", "cmd": cmd, "row": dict(row)})


def open_saved_csv(app):
    path = choose_csv(app, app.csv_dir)
    if not path:
        return
    p = Path(path)
    try:
        lines = p.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as e:
        G.show_error(app, "Could Not Open", str(e))
        return
    app.csv_dir = str(p.parent)
    kind = "STATIC" if "ground" in p.name.lower() else "FLIGHT"
    open_plot(app, lines, p.name, kind=kind, keep_lines=False)


def install(app):
    """Add the plot buttons to a running COMETApp (call once, right after creating it)."""
    global G
    if getattr(app, "_logplot_installed", False):
        return
    app._logplot_installed = True
    G = sys.modules[type(app).__module__]

    top = app.tab_logs.grid_slaves(row=0, column=0)[0]
    G.make_button(top, "Plot Selected", lambda: plot_selected(app), "normal", 12, 20, 11
                  ).pack(side="left", padx=(0, 8))
    G.make_button(top, "Open CSV...", lambda: open_saved_csv(app), "normal", 12, 20, 11
                  ).pack(side="left", padx=(0, 8))
    app.dl_status_var.set("Pick a log, then press Download Selected -- or Plot Selected (or double-click) to view it.")

    def on_double(e):
        if app.log_tree.identify_region(e.x, e.y) == "cell":
            plot_selected(app)
    app.log_tree.bind("<Double-1>", on_double)

    # A "plot" download keeps the CSV in memory and opens the window instead of saving.
    orig_finish_current = app.finish_current_csv
    orig_finish_download = app.finish_download

    def finish_current_csv():
        dl = app.dl
        if dl is None or dl.get("mode") != "plot":
            return orig_finish_current()
        cur, dl["cur"] = dl["cur"], None
        if cur and cur["lines"]:
            dl["plot_lines"], dl["plot_kind"] = cur["lines"], cur["kind"]
        finish_download()

    def finish_download():
        dl = app.dl
        if dl is None or dl.get("mode") != "plot":
            return orig_finish_download()
        app.dl = None
        app.progress.stop()
        app.progress.pack_forget()
        lines, row = dl.get("plot_lines"), dl["row"]
        if not lines:
            app.dl_status_var.set("The board did not send any log data.")
            G.show_info(app, "No Data", "The board did not send any log data.")
            return
        kind = dl.get("plot_kind", row["kind"])
        what = "Flight" if kind == "FLIGHT" else "Ground session"
        name = f"{what} log #{row['id']}  (slot {row['slot']})"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default = f"{app.file_stub(row['kind'], row['slot'])}_{stamp}.csv"
        app.dl_status_var.set(f"Plotted {what.lower()} slot {row['slot']} ({len(lines) - 1} rows).")

        def show():
            try:
                open_plot(app, lines, name, row, kind, default)
            except Exception as e:  # never let a bad log break the serial loop
                G.show_error(app, "Plot Failed", f"{type(e).__name__}: {e}")
        app.root.after(10, show)

    app.finish_current_csv = finish_current_csv
    app.finish_download = finish_download


def main():
    root = tk.Tk()
    pick_fonts(root)
    app = COMETApp(root)
    install(app)  # flight-log plotter
    root.mainloop()


if __name__ == "__main__":
    main()
