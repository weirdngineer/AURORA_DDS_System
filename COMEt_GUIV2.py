#!/usr/bin/env python3
"""
COMET Flight Computer Interface  (Windows + Linux)

Matches the SimpleRecovery firmware:
  * Profiles only store MAIN_ALT and ACCEL_ENABLE (everything else is fixed in firmware)
  * 20 flight logs + 3 ground-session logs, downloadable as CSV
  * EXPORTALLCSV support (one click saves every log into a new folder)

Requires:  pip install pyserial
All pop-ups and the folder browser are built in-app (dark, large text) so they
look the same on every OS -- no white-on-white native dialogs.
"""

import os
import re
import string
import subprocess
import sys
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports

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
MIN_MAIN_ALT_M = 200.0
MAX_MAIN_ALT_M = 10000.0

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


# ============================================================
# LIVE PLOT
# ============================================================

class LivePlot(tk.Canvas):
    def __init__(self, parent, title, series_names, window_seconds=10.0, y_label="", **kw):
        super().__init__(parent, bg=C["field"], highlightthickness=1,
                         highlightbackground=C["border"], **kw)
        self.title = title
        self.series_names = series_names
        self.window_seconds = float(window_seconds)
        self.y_label = y_label
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

        self.create_text(12, 9, anchor="nw", text=self.title, fill=C["accent_light"], font=F(11, True))

        pts = [p for vals in self.data.values() for p in vals]
        if not pts:
            self.create_text(w / 2, h / 2, text="Waiting for telemetry...", fill=C["muted"], font=F(11))
            return

        max_t = max(t for t, _ in pts)
        min_t = max_t - self.window_seconds
        vis = [v for t, v in pts if t >= min_t]
        y_min, y_max = min(vis), max(vis)
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
        root.title("COMET Flight Computer")
        root.geometry("1200x840")
        root.minsize(1020, 700)
        root.configure(bg=C["bg"])

        self.ser = None
        self.reader_running = False
        self.rx_queue = queue.Queue()
        self.port_map = {}

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
        self.profiles = {n: {"alt_m": float(a), "accel": True} for n, a, _, _ in PROFILES}
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
        root.after(800, self.auto_connect)
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
        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var, width=36, state="readonly", font=F(11))
        self.port_combo.pack(side="left", padx=(0, 8))
        make_button(inner, "Refresh", self.refresh_ports, "normal", 11, 14, 6).pack(side="left", padx=(0, 12))
        self.connect_button = make_button(inner, "Connect", self.toggle_connection, "success", 11, 20, 6)
        self.connect_button.pack(side="left", padx=(0, 14))
        self.status_var = tk.StringVar(value="Disconnected")
        self.status_label = tk.Label(inner, textvariable=self.status_var, bg=C["panel"], fg=C["warn"],
                                     font=F(11, True))
        self.status_label.pack(side="left")

        units = tk.Frame(inner, bg=C["panel"])
        units.pack(side="right")
        lbl(units, "Units", 11, True).pack(side="left", padx=(0, 8))
        ttk.Radiobutton(units, text="Metric", value=0, variable=self.unit_mode,
                        style="Unit.TRadiobutton", command=self.on_unit_change).pack(side="left", padx=(0, 10))
        ttk.Radiobutton(units, text="Imperial", value=1, variable=self.unit_mode,
                        style="Unit.TRadiobutton", command=self.on_unit_change).pack(side="left")

    def build_status_cards(self, parent):
        row = tk.Frame(parent, bg=C["bg"])
        row.pack(fill="x", pady=(0, 8))
        keys = [("STATE", "STATE"), ("ALT", "ALTITUDE"), ("MAXALT", "MAX ALT"), ("VZ", "SPEED"),
                ("BATT", "BATTERY"), ("PROFILE", "PROFILE"), ("DROGUE", "DROGUE"),
                ("MAIN", "MAIN"), ("LOG", "LOGGING")]
        self.card_vars = {}
        self.card_labels = {}
        for i, (key, title) in enumerate(keys):
            row.grid_columnconfigure(i, weight=1, uniform="cards")
            card = tk.Frame(row, bg=C["panel"], highlightthickness=1, highlightbackground=C["border"])
            card.grid(row=0, column=i, sticky="ew", padx=3)
            tk.Label(card, text=title, bg=C["panel"], fg=C["muted"], font=F(9, True), anchor="w"
                     ).pack(fill="x", padx=10, pady=(7, 0))
            var = tk.StringVar(value="--")
            lab = tk.Label(card, textvariable=var, bg=C["panel"], fg=C["accent_light"],
                           font=FM(12, True), anchor="w")
            lab.pack(fill="x", padx=10, pady=(0, 7))
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
        self.baro_plot = LivePlot(area, "Altitude (barometer)", ["ALT"], y_label="m", height=120)
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
        t = self.tab_setup
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
                        variable=self.accel_var).pack(anchor="w", pady=(0, 14))

        brow = tk.Frame(ed, bg=C["panel"])
        brow.pack(anchor="w")
        make_button(brow, "Save Changes", self.save_profile, "primary", 12, 24, 11).pack(side="left", padx=(0, 10))
        make_button(brow, "Use This Profile", self.activate_profile, "normal", 12, 24, 11).pack(side="left")

        lbl(ed, "Changes are written to the board's flash a moment after saving, and are\n"
                "locked once launch has been detected.",
            10, False, C["muted"], justify="left").pack(anchor="w", pady=(12, 0))

        # Right bottom: bench tests
        bench_card, bench = make_card(right, "Bench testing")
        bench_card.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        lbl(bench, "Disconnect ALL igniters and charges before using these.", 11, True, C["warn"]
            ).pack(anchor="w", pady=(0, 10))
        g = tk.Frame(bench, bg=C["panel"])
        g.pack(fill="x")
        for c in range(2):
            g.grid_columnconfigure(c, weight=1, uniform="bench")
        make_button(g, "Reset to IDLE", lambda: self.send_command("RESET"), "normal", 11, 10, 10
                    ).grid(row=0, column=0, sticky="ew", padx=(0, 5), pady=4)
        make_button(g, "Force Launch", lambda: self.confirm_command(
            "LAUNCH", "Force Launch",
            "This starts the flight logic as if the rocket had launched. If pyro outputs are "
            "enabled, the drogue and main can fire."), "primary", 11, 10, 10
                    ).grid(row=0, column=1, sticky="ew", padx=(5, 0), pady=4)
        make_button(g, "Test Drogue", lambda: self.confirm_command(
            "DROGUE", "Test Drogue", "This fires the DROGUE output."), "primary", 11, 10, 10
                    ).grid(row=1, column=0, sticky="ew", padx=(0, 5), pady=4)
        make_button(g, "Test Main", lambda: self.confirm_command(
            "MAIN", "Test Main", "This fires the MAIN output."), "primary", 11, 10, 10
                    ).grid(row=1, column=1, sticky="ew", padx=(5, 0), pady=4)

    # ---------------- Logs ----------------
    def build_logs(self):
        t = self.tab_logs
        t.grid_rowconfigure(1, weight=1)
        t.grid_columnconfigure(0, weight=1)

        top = tk.Frame(t, bg=C["bg"])
        top.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        make_button(top, "Refresh List", lambda: self.send_command("LIST"), "normal", 12, 20, 11).pack(side="left", padx=(0, 8))
        make_button(top, "Download Selected", self.download_selected, "success", 12, 22, 11).pack(side="left", padx=(0, 8))
        make_button(top, "Export All", self.export_all, "success", 12, 22, 11).pack(side="left", padx=(0, 8))
        make_button(top, "Erase All", self.erase_all, "primary", 12, 20, 11).pack(side="right")
        make_button(top, "Erase Selected", self.erase_selected, "primary", 12, 20, 11).pack(side="right", padx=(0, 8))

        tf = tk.Frame(t, bg=C["border"], padx=1, pady=1)
        tf.grid(row=1, column=0, sticky="nsew")
        cols = ("type", "slot", "id", "length", "records", "status", "latest")
        self.log_tree = ttk.Treeview(tf, columns=cols, show="headings", selectmode="browse")
        heads = [("type", "Type", 160), ("slot", "Slot", 70), ("id", "Log #", 80),
                 ("length", "Length", 120), ("records", "Records", 110),
                 ("status", "Status", 140), ("latest", "Newest", 90)]
        for key, text, width in heads:
            self.log_tree.heading(key, text=text)
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
        self.progress.pack(anchor="w", pady=(6, 0))
        lbl(bottom, "Flight logs record at 10 Hz during a flight. Ground sessions record 1 Hz while powered on but not flying.\n"
                    "Logs can only be listed or downloaded on the ground (not after launch is detected, until you press RESET).",
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
        ports = list(serial.tools.list_ports.comports())
        self.port_map = {}
        names = []
        for p in ports:
            desc = (p.description or "").strip()
            label = f"{p.device} - {desc}" if desc and desc != "n/a" else p.device
            self.port_map[label] = p.device
            names.append(label)
        self.port_combo["values"] = names

        current = self.port_var.get()
        if current in self.port_map:
            return
        keywords = ["COMET", "RP2040", "PICO", "RPI", "USB SERIAL", "CDC", "ACM"]
        for p, label in zip(ports, names):
            blob = f"{p.device} {p.description} {p.manufacturer} {p.hwid}".upper()
            if any(k in blob for k in keywords):
                self.port_var.set(label)
                return
        self.port_var.set(names[0] if names else "")

    def auto_connect(self):
        if not self.is_connected() and self.port_var.get():
            self.connect(auto=True)

    def toggle_connection(self):
        if self.is_connected():
            self.disconnect()
        else:
            self.connect(auto=False)

    def connect(self, auto=False):
        label = self.port_var.get()
        port = self.port_map.get(label, label)
        if not port:
            if not auto:
                show_error(self, "No Port Selected", "Plug in the flight computer, press Refresh, and pick its port.")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.05)
        except Exception as e:
            self.ser = None
            if auto:
                self.set_message(f"Auto-connect to {port} failed: {e}", True)
                return
            hint = ""
            if isinstance(e, PermissionError) or "Permission" in str(e):
                if sys.platform.startswith("linux"):
                    hint = ("\n\nOn Linux you usually need serial permission. Run this once, "
                            "then log out and back in:\n  sudo usermod -aG dialout $USER")
                else:
                    hint = "\n\nThe port may be open in another program (Arduino IDE serial monitor?)."
            show_error(self, "Could Not Connect", f"{e}{hint}")
            return

        time.sleep(0.5)
        self.reader_running = True
        threading.Thread(target=self.serial_reader, daemon=True).start()
        self.connection_lost = False
        self.device_verified = False
        self.last_telemetry_wall = None
        self.telemetry_rate_hz = 0.0
        self.clear_plots()
        self.connect_button.configure(text="Disconnect", bg=C["accent"], activebackground=C["accent_dark"], fg="white",
                                      activeforeground="white")
        self.connect_button._base_bg = C["accent"]
        self.status_var.set(f"Connected: {port}")
        self.status_label.configure(fg=C["success"])
        self.console_write(f"[GUI] Connected to {port}\n", "gui")
        self.set_message(f"Connected to {port}.")

        # Pull current settings from the board.
        self.root.after(500, lambda: self.send_command("GETPARAMS", echo=False, warn=False))
        self.root.after(750, lambda: self.send_command("PROFILE LIST", echo=False, warn=False))
        self.root.after(1000, lambda: self.send_command("LIST", echo=False, warn=False))

    def disconnect(self):
        self.reader_running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.device_verified = False
        self.abort_download("Disconnected.", quiet=True)
        self.connect_button.configure(text="Connect", bg=C["success"], activebackground="#22a85a", fg="black",
                                      activeforeground="black")
        self.connect_button._base_bg = C["success"]
        self.status_var.set("Disconnected")
        self.status_label.configure(fg=C["warn"])
        self.console_write("[GUI] Disconnected\n", "gui")

    def serial_reader(self):
        buf = b""
        while self.reader_running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(1024)
            except Exception as e:
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
        self.status_var.set("Disconnected - device removed")
        self.set_message(f"Lost connection to {old}. Plug it back in and press Connect.", True)

    def send_command(self, cmd, echo=True, warn=True):
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

    def send_many(self, cmds, gap_ms=130):
        if not self.is_connected():
            show_info(self, "Not Connected", "Connect to the flight computer first.")
            return
        for i, c in enumerate(cmds):
            self.root.after(i * gap_ms, lambda c=c: self.send_command(c, warn=False))

    def send_manual(self):
        cmd = self.cmd_var.get().strip()
        if cmd:
            self.send_command(cmd)
            self.cmd_var.set("")

    def confirm_command(self, cmd, title, detail):
        if ask_yes_no(self, title,
                      f"{detail}\n\nOnly continue if no igniters or charges are connected.",
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
        if line.startswith("PARAM "):
            self.handle_param(line)
        elif line.startswith("PROFILE "):
            self.handle_profile_line(line)
        elif "COMET LOG ARCHIVE" in line:
            self.clear_log_table()
        else:
            m = LOG_ROW_RE.match(line)
            if m:
                self.add_log_row(m)

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
        try:
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
        try:
            if "MAIN_ALT" in vals:
                self.profiles[color]["alt_m"] = float(vals["MAIN_ALT"])
            if "ACCEL_ENABLE" in vals:
                self.profiles[color]["accel"] = vals["ACCEL_ENABLE"] not in ("0", "OFF")
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

        now = time.monotonic()
        if self.last_telemetry_wall is not None:
            dt = now - self.last_telemetry_wall
            if dt > 0:
                inst = 1.0 / dt
                self.telemetry_rate_hz = 0.85 * self.telemetry_rate_hz + 0.15 * inst if self.telemetry_rate_hz > 0 else inst
        self.last_telemetry_wall = now

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
        v["STATE"].set(SHORT_STATE.get(state, state))
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
            v[key].set("FIRED" if fired else "ready")
            lab[key].configure(fg=C["warn"] if fired else C["success"])
        if pyro:
            v["DROGUE" if d.get("DROGUE") == "1" and d.get("MAIN") != "1" else "MAIN"].set("FIRING")
        v["LOG"].set("ACTIVE" if d.get("LOG") == "1" else "off")
        lab["LOG"].configure(fg=C["warn"] if d.get("LOG") == "1" else C["muted"])

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
        self.alt_hint_var.set(f"Allowed range: {lo:.0f} - {hi:.0f} {unit}. The 200 m minimum is a built-in safety floor.")
        self.accel_var.set(p["accel"])
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
        name = self.selected_profile
        self.send_many([
            f"PROFILE SET {name} MAIN_ALT {alt_m:.2f}",
            f"PROFILE SET {name} ACCEL_ENABLE {1 if self.accel_var.get() else 0}",
            f"PROFILE GET {name}",
        ])
        self.set_message(f"Saving {name} profile...")

    def activate_profile(self):
        self.send_command(f"PROFILE APPLY {self.selected_profile}")

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

    def add_log_row(self, m):
        star, kind, slot, ident, records, seconds, status = m.groups()
        slot, ident, records, seconds, status = int(slot), int(ident), int(records), int(seconds), int(status)
        iid = f"{kind}:{slot}"
        row = {"kind": kind, "slot": slot, "id": ident, "records": records,
               "status": LOG_STATUS.get(status, str(status)), "status_code": status}
        self.log_rows[iid] = row
        self.log_tree.insert("", "end", iid=iid, values=(
            "Flight" if kind == "FLIGHT" else "Ground session",
            slot, ident, self.fmt_duration(seconds), records, row["status"], "yes" if star else ""))

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
        self.send_command(cmd)
        self.root.after(500, lambda: self.send_command("LIST", echo=False, warn=False))

    def erase_all(self):
        if not ask_yes_no(self, "Erase ALL Logs",
                          "Permanently erase every flight log and ground-session log?\n\n"
                          "Your saved flight profiles are kept. This cannot be undone.",
                          yes="Erase Everything", danger=True):
            return
        self.send_command("FORMATLOG")
        self.root.after(800, lambda: self.send_command("LIST", echo=False, warn=False))

    # --------------------------------------------------------
    # CSV download state machine
    # --------------------------------------------------------
    def start_download(self, dl):
        if self.dl is not None:
            show_info(self, "Busy", "A download is already in progress.")
            return
        dl.update({"cur": None, "saved": [], "rows": 0, "last_rx": time.monotonic()})
        self.dl = dl
        self.progress.start(12)
        self.dl_status_var.set("Requesting data from the board...")
        if not self.send_command(dl["cmd"]):
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
        except Exception as e:
            self.abort_download(f"Could not save file: {e}")

    def finish_download(self):
        dl = self.dl
        if dl is None:
            return
        self.dl = None
        self.progress.stop()
        saved = dl["saved"]
        if not saved:
            self.dl_status_var.set("Download finished, but no data was received.")
            show_info(self, "No Data", "The board did not send any log data.")
            return
        folder = str(Path(saved[0]).parent)
        self.dl_status_var.set(f"Saved {len(saved)} file(s) to {folder}")
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


def main():
    root = tk.Tk()
    pick_fonts(root)
    COMETApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
