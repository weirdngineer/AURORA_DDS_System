import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText
import serial
import serial.tools.list_ports
import threading
import queue
import time
import sys
from datetime import datetime
from pathlib import Path


class LivePlot(tk.Canvas):
    """Simple dark-theme 10-second rolling plot using only Tkinter Canvas."""

    def __init__(self, parent, title, series_names, window_seconds=10.0, y_label="", **kwargs):
        super().__init__(
            parent,
            bg="#080808",
            highlightthickness=1,
            highlightbackground="#2a2a2a",
            **kwargs,
        )

        self.title = title
        self.series_names = series_names
        self.window_seconds = float(window_seconds)
        self.y_label = y_label
        self.colors = ["#ff4d5a", "#29c76f", "#4da3ff", "#ffb020"]
        self.data = {name: [] for name in series_names}
        self.dirty = True

        self.bind("<Configure>", lambda event: self.redraw())

    def clear(self):
        for name in self.series_names:
            self.data[name].clear()
        self.redraw()

    def add_point(self, t_sec, values):
        try:
            t_sec = float(t_sec)
        except Exception:
            t_sec = time.monotonic()

        cutoff = t_sec - self.window_seconds

        for name in self.series_names:
            try:
                value = float(values.get(name, 0.0))
            except Exception:
                value = 0.0

            self.data[name].append((t_sec, value))
            self.data[name] = [(t, v) for (t, v) in self.data[name] if t >= cutoff]

        self.dirty = True

    def redraw_if_dirty(self):
        if self.dirty:
            self.redraw()

    def redraw(self):
        self.dirty = False
        self.delete("all")

        w = self.winfo_width()
        h = self.winfo_height()

        if w < 80 or h < 70:
            return

        pad_l = 58
        pad_r = 16
        pad_t = 32
        pad_b = 36

        x0 = pad_l
        y0 = pad_t
        x1 = w - pad_r
        y1 = h - pad_b

        self.create_text(
            10,
            8,
            anchor="nw",
            text=self.title,
            fill="#ff4d5a",
            font=("Segoe UI", 10, "bold"),
        )

        all_points = []
        for name in self.series_names:
            all_points.extend(self.data[name])

        if not all_points:
            self.create_text(
                w / 2,
                h / 2,
                text="Waiting for telemetry...",
                fill="#777777",
                font=("Segoe UI", 10),
            )
            return

        latest_t = max(t for t, _ in all_points)
        min_t = latest_t - self.window_seconds
        max_t = latest_t

        visible_values = [v for t, v in all_points if min_t <= t <= max_t]
        if not visible_values:
            return

        y_min = min(visible_values)
        y_max = max(visible_values)

        if abs(y_max - y_min) < 1e-6:
            y_min -= 1.0
            y_max += 1.0

        pad = (y_max - y_min) * 0.12
        y_min -= pad
        y_max += pad

        self.create_rectangle(x0, y0, x1, y1, outline="#333333")

        for i in range(5):
            y = y0 + (y1 - y0) * i / 4
            val = y_max - (y_max - y_min) * i / 4
            self.create_line(x0, y, x1, y, fill="#1f1f1f")
            self.create_text(
                x0 - 6,
                y,
                anchor="e",
                text=f"{val:.1f}",
                fill="#888888",
                font=("Consolas", 8),
            )

        self.create_text(x0, y1 + 14, anchor="n", text=f"-{self.window_seconds:.0f}s", fill="#888888", font=("Consolas", 8))
        self.create_text(x1, y1 + 14, anchor="n", text="now", fill="#888888", font=("Consolas", 8))

        if self.y_label:
            self.create_text(14, (y0 + y1) / 2, text=self.y_label, angle=90, fill="#777777", font=("Segoe UI", 8))

        legend_x = x0 + 8
        legend_y = y1 + 8

        for idx, name in enumerate(self.series_names):
            color = self.colors[idx % len(self.colors)]
            self.create_line(legend_x, legend_y + 6, legend_x + 18, legend_y + 6, fill=color, width=2)
            self.create_text(
                legend_x + 24,
                legend_y,
                anchor="nw",
                text=name,
                fill="#dddddd",
                font=("Segoe UI", 8),
            )
            legend_x += 82

        for idx, name in enumerate(self.series_names):
            vals = [(t, v) for (t, v) in self.data[name] if min_t <= t <= max_t]

            if len(vals) < 2:
                continue

            color = self.colors[idx % len(self.colors)]
            points = []

            for t, v in vals:
                x = x0 + (t - min_t) / max(1e-9, (max_t - min_t)) * (x1 - x0)
                y = y1 - (v - y_min) / (y_max - y_min) * (y1 - y0)
                points.extend([x, y])

            self.create_line(points, fill=color, width=2, smooth=True)


class COMETGUI:
    # Change this if your firmware uses a different buzzer command.
    # Set to None if the firmware should not receive a beep/click command after GUI button presses.
    BOARD_BEEP_COMMAND = "BEEP"

    def __init__(self, root):
        self.root = root
        self.root.title("COMET Flight Computer Interface")
        self.root.geometry("1280x820")
        self.root.minsize(1120, 720)

        self.ser = None
        self.reader_thread = None
        self.reader_running = False
        self.rx_queue = queue.Queue()

        self.downloading = False
        self.csv_capture_started = False
        self.download_lines = []
        self.download_slot = None
        self.csv_save_path = None
        self.csv_default_dir = str(Path.home() / "Documents")

        self.last_telemetry_wall_time = None
        self.telemetry_rate_hz = 0.0
        self.last_state = "--"
        self.device_verified = False
        self.auto_connect_attempted = False
        self.connection_lost = False
        self.plot_paused = False
        self.plot_window_seconds = tk.DoubleVar(value=10.0)

        # Unit system: 0 = Metric, 1 = Imperial. Firmware always remains in SI.
        self.unit_mode = tk.IntVar(value=0)
        self._last_unit_mode = 0
        self.last_telemetry_data = {}

        self.show_data_stream = True

        self.colors = {
            "bg": "#0d0d0d",
            "panel": "#161616",
            "panel2": "#1d1d1d",
            "panel3": "#242424",
            "accent": "#c1121f",
            "accent_dark": "#8f0d16",
            "accent_light": "#ff4d5a",
            "text": "#f3f3f3",
            "muted": "#aaaaaa",
            "success": "#29c76f",
            "warn": "#ffb020",
            "button_pressed": "#5c1016",
        }

        self.configure_theme()
        self.build_ui()
        self.refresh_ports()

        self.root.after(50, self.process_serial_queue)
        self.root.after(100, self.refresh_plots)
        self.root.after(1000, self.monitor_connection)
        self.root.after(700, self.auto_connect_standard_port)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ============================================================
    # THEME
    # ============================================================

    def configure_theme(self):
        self.root.configure(bg=self.colors["bg"])

        style = ttk.Style()
        style.theme_use("clam")

        style.configure(
            ".",
            background=self.colors["bg"],
            foreground=self.colors["text"],
            fieldbackground="#101010",
            bordercolor="#2a2a2a",
            lightcolor="#2a2a2a",
            darkcolor="#2a2a2a",
            troughcolor=self.colors["panel"],
        )

        style.configure("Main.TFrame", background=self.colors["bg"])
        style.configure("Panel.TFrame", background=self.colors["panel"])
        style.configure("Panel2.TFrame", background=self.colors["panel2"])
        style.configure("TabContent.TFrame", background=self.colors["bg"], padding=12)

        style.configure(
            "Card.TLabelframe",
            background=self.colors["panel"],
            foreground=self.colors["accent_light"],
            bordercolor=self.colors["accent"],
            relief="solid",
            borderwidth=1,
            padding=10,
        )

        style.configure(
            "Card.TLabelframe.Label",
            background=self.colors["panel"],
            foreground=self.colors["accent_light"],
            font=("Segoe UI", 10, "bold"),
        )

        style.configure(
            "Title.TLabel",
            background=self.colors["bg"],
            foreground=self.colors["accent_light"],
            font=("Segoe UI", 24, "bold"),
        )

        style.configure(
            "SubTitle.TLabel",
            background=self.colors["bg"],
            foreground=self.colors["muted"],
            font=("Segoe UI", 11),
        )

        style.configure("Normal.TLabel", background=self.colors["panel"], foreground=self.colors["text"], font=("Segoe UI", 10))
        style.configure("Bg.TLabel", background=self.colors["bg"], foreground=self.colors["text"], font=("Segoe UI", 10))
        style.configure("Value.TLabel", background=self.colors["panel"], foreground=self.colors["accent_light"], font=("Consolas", 12, "bold"))
        style.configure("Status.TLabel", background=self.colors["panel"], foreground=self.colors["success"], font=("Segoe UI", 10, "bold"))

        style.configure("Danger.TButton", background=self.colors["accent"], foreground="white", borderwidth=0, padding=8, font=("Segoe UI", 10, "bold"))
        style.map("Danger.TButton", background=[("active", self.colors["accent_dark"]), ("pressed", self.colors["accent_dark"])], foreground=[("active", "white")])

        style.configure("Dark.TButton", background=self.colors["panel2"], foreground=self.colors["text"], borderwidth=0, padding=8, font=("Segoe UI", 10))
        style.map("Dark.TButton", background=[("active", "#2a2a2a"), ("pressed", "#222222")], foreground=[("active", "white")])

        style.configure("Connect.TButton", background=self.colors["success"], foreground="black", borderwidth=0, padding=8, font=("Segoe UI", 10, "bold"))
        style.map("Connect.TButton", background=[("active", "#1ea95c"), ("pressed", "#1b8f4f")], foreground=[("active", "black")])

        style.configure("Disconnect.TButton", background=self.colors["accent"], foreground="white", borderwidth=0, padding=8, font=("Segoe UI", 10, "bold"))
        style.map("Disconnect.TButton", background=[("active", self.colors["accent_dark"]), ("pressed", self.colors["accent_dark"])], foreground=[("active", "white")])

        style.configure("TEntry", fieldbackground="#101010", foreground=self.colors["text"], bordercolor="#2a2a2a", padding=6)
        style.configure("TCombobox", fieldbackground="#101010", foreground=self.colors["text"], background=self.colors["panel2"], arrowcolor=self.colors["accent_light"], bordercolor="#2a2a2a", padding=5)
        style.map("TCombobox", fieldbackground=[("readonly", "#101010")], foreground=[("readonly", self.colors["text"])], background=[("readonly", self.colors["panel2"])])

        style.configure("Dark.TNotebook", background=self.colors["bg"], borderwidth=0)

        # Main notebook tabs. Selected tab is red, larger, and easier to identify.
        style.configure(
            "Dark.TNotebook.Tab",
            background=self.colors["panel2"],
            foreground=self.colors["muted"],
            padding=(18, 7),
            font=("Segoe UI", 10, "bold"),
        )
        style.map(
            "Dark.TNotebook.Tab",
            background=[
                ("selected", self.colors["accent"]),
                ("active", "#2a2a2a"),
            ],
            foreground=[
                ("selected", "white"),
                ("active", "white"),
            ],
            padding=[
                ("selected", (26, 11)),
                ("!selected", (18, 7)),
            ],
            font=[
                ("selected", ("Segoe UI", 12, "bold")),
                ("!selected", ("Segoe UI", 10, "bold")),
            ],
        )

        # Smaller nested control tabs, but still with a prominent red selected tab.
        style.configure(
            "Control.TNotebook",
            background=self.colors["panel"],
            borderwidth=0,
        )
        style.configure(
            "Control.TNotebook.Tab",
            background=self.colors["panel2"],
            foreground=self.colors["muted"],
            padding=(14, 6),
            font=("Segoe UI", 9, "bold"),
        )
        style.map(
            "Control.TNotebook.Tab",
            background=[
                ("selected", self.colors["accent"]),
                ("active", "#2a2a2a"),
            ],
            foreground=[
                ("selected", "white"),
                ("active", "white"),
            ],
            padding=[
                ("selected", (22, 10)),
                ("!selected", (14, 6)),
            ],
            font=[
                ("selected", ("Segoe UI", 11, "bold")),
                ("!selected", ("Segoe UI", 9, "bold")),
            ],
        )

    # ============================================================
    # UI
    # ============================================================

    def build_ui(self):
        main = ttk.Frame(self.root, style="Main.TFrame", padding=14)
        main.pack(fill="both", expand=True)

        header = ttk.Frame(main, style="Main.TFrame")
        header.pack(fill="x", pady=(0, 12))

        ttk.Label(header, text="COMET", style="Title.TLabel").pack(anchor="w")
        ttk.Label(header, text="Compact Onboard Management for Ejection Timing", style="SubTitle.TLabel").pack(anchor="w", pady=(2, 0))

        self.build_connection_bar(main)

        self.notebook = ttk.Notebook(main, style="Dark.TNotebook")
        self.notebook.pack(fill="both", expand=True)

        self.dashboard_tab = ttk.Frame(self.notebook, style="TabContent.TFrame")
        self.flight_tab = ttk.Frame(self.notebook, style="TabContent.TFrame")

        self.notebook.add(self.dashboard_tab, text="Comet Dashboard")
        self.notebook.add(self.flight_tab, text="Flight Control")

        self.build_dashboard_tab(self.dashboard_tab)
        self.build_flight_control_tab(self.flight_tab)

    def build_connection_bar(self, parent):
        topbar = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        topbar.pack(fill="x", pady=(0, 12))

        ttk.Label(topbar, text="Port", style="Normal.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 6))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(topbar, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w", padx=(0, 8))

        self.refresh_button = ttk.Button(topbar, text="Refresh", style="Dark.TButton", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=(0, 8))

        self.auto_button = ttk.Button(topbar, text="Auto Connect", style="Dark.TButton", command=self.auto_connect_standard_port)
        self.auto_button.grid(row=0, column=3, padx=(0, 14))

        ttk.Label(topbar, text="Baud", style="Normal.TLabel").grid(row=0, column=4, sticky="w", padx=(0, 6))

        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(topbar, textvariable=self.baud_var, values=["9600", "57600", "115200", "230400", "460800", "921600"], width=12, state="readonly")
        self.baud_combo.grid(row=0, column=5, sticky="w", padx=(0, 12))

        self.connect_button = ttk.Button(topbar, text="Connect", style="Connect.TButton", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=6, padx=(0, 12))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(topbar, textvariable=self.status_var, style="Status.TLabel").grid(row=0, column=7, sticky="w")

        topbar.grid_columnconfigure(8, weight=1)

        # Global Metric / Imperial switch. Only GUI presentation changes;
        # firmware commands and stored logs remain in canonical SI units.
        units = tk.Frame(topbar, bg=self.colors["panel"])
        units.grid(row=0, column=9, sticky="e", padx=(16, 0))

        tk.Label(units, text="Units", bg=self.colors["panel"], fg=self.colors["text"],
                 font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 8))
        tk.Label(units, text="Metric", bg=self.colors["panel"], fg=self.colors["muted"],
                 font=("Segoe UI", 9)).pack(side="left")

        self.unit_scale = tk.Scale(
            units,
            from_=0,
            to=1,
            resolution=1,
            orient="horizontal",
            showvalue=0,
            variable=self.unit_mode,
            command=self.on_unit_switch,
            length=72,
            sliderlength=24,
            width=10,
            bd=0,
            highlightthickness=0,
            troughcolor="#303030",
            bg=self.colors["panel"],
            activebackground=self.colors["accent_light"],
        )
        self.unit_scale.pack(side="left", padx=6)

        tk.Label(units, text="Imperial", bg=self.colors["panel"], fg=self.colors["muted"],
                 font=("Segoe UI", 9)).pack(side="left")

    def create_status_readout(self, parent):
        box = ttk.LabelFrame(parent, text="Live Status Readout", style="Card.TLabelframe")

        self.live_vars = getattr(self, "live_vars", None)
        if self.live_vars is None:
            self.live_vars = {
                "STATE": tk.StringVar(value="--"),
                "ALT": tk.StringVar(value="-- m"),
                "VZ": tk.StringVar(value="-- m/s"),
                "MAXALT": tk.StringVar(value="-- m"),
                "BATT": tk.StringVar(value="-- V"),
                "TEMP": tk.StringVar(value="-- C"),
                "LOG": tk.StringVar(value="--"),
                "SLOT": tk.StringVar(value="--"),
                "REC": tk.StringVar(value="--"),
                "RATE": tk.StringVar(value="-- Hz"),
            }

        row = ttk.Frame(box, style="Panel.TFrame")
        row.pack(fill="x")

        keys = ["STATE", "ALT", "VZ", "MAXALT", "BATT", "TEMP", "LOG", "SLOT", "REC", "RATE"]

        for i, key in enumerate(keys):
            card = tk.Frame(row, bg="#101010", bd=1, relief="solid", highlightthickness=1, highlightbackground="#2a2a2a")
            card.grid(row=0, column=i, sticky="ew", padx=3, pady=2)
            row.grid_columnconfigure(i, weight=1)

            card.grid_columnconfigure(0, weight=1)

            tk.Label(
                card,
                text=key,
                bg="#101010",
                fg="#aaaaaa",
                font=("Segoe UI", 8, "bold"),
                width=10,
                anchor="w",
            ).grid(row=0, column=0, sticky="ew", padx=8, pady=(5, 0))

            tk.Label(
                card,
                textvariable=self.live_vars[key],
                bg="#101010",
                fg="#ff4d5a",
                font=("Consolas", 11, "bold"),
                anchor="w",
                width=12,
            ).grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 5))

        return box

    def build_dashboard_tab(self, parent):
        parent.grid_rowconfigure(1, weight=1)
        parent.grid_columnconfigure(0, weight=1)

        status_box = self.create_status_readout(parent)
        status_box.grid(row=0, column=0, sticky="ew", pady=(0, 12))

        plot_area = ttk.Frame(parent, style="Main.TFrame")
        plot_area.grid(row=1, column=0, sticky="nsew")

        plot_area.grid_rowconfigure(0, weight=1)
        plot_area.grid_rowconfigure(1, weight=1)
        plot_area.grid_rowconfigure(2, weight=1)
        plot_area.grid_columnconfigure(0, weight=1)

        self.accel_plot = LivePlot(plot_area, title="Acceleration", series_names=["AX", "AY", "AZ"], window_seconds=10.0, y_label="m/s²")
        self.accel_plot.grid(row=0, column=0, sticky="nsew", pady=(0, 10))

        self.gyro_plot = LivePlot(plot_area, title="Gyroscope", series_names=["GX", "GY", "GZ"], window_seconds=10.0, y_label="deg/s")
        self.gyro_plot.grid(row=1, column=0, sticky="nsew", pady=(0, 10))

        self.baro_plot = LivePlot(plot_area, title="Barometric Altitude", series_names=["ALT"], window_seconds=10.0, y_label="m")
        self.baro_plot.grid(row=2, column=0, sticky="nsew")

        bottom = ttk.Frame(parent, style="Panel.TFrame", padding=8)
        bottom.grid(row=2, column=0, sticky="ew", pady=(10, 0))

        self.plot_pause_button = self.make_gui_button(bottom, "Pause Plots", self.toggle_plot_pause, danger=False)
        self.plot_pause_button.pack(side="left", padx=(0, 8))

        self.make_gui_button(bottom, "Clear Plots", self.clear_plots, danger=False).pack(side="left", padx=(0, 18))

        ttk.Label(bottom, text="Plot Window", style="Normal.TLabel").pack(side="left", padx=(0, 8))
        self.plot_window_combo = ttk.Combobox(
            bottom,
            textvariable=self.plot_window_seconds,
            values=[5, 10, 15, 30, 45, 60],
            width=8,
            state="readonly",
        )
        self.plot_window_combo.pack(side="left")
        self.plot_window_combo.bind("<<ComboboxSelected>>", lambda event: self.update_plot_window())

        ttk.Label(bottom, text="seconds", style="Normal.TLabel").pack(side="left", padx=(6, 0))

    def build_flight_control_tab(self, parent):
        parent.grid_rowconfigure(1, weight=4)
        parent.grid_rowconfigure(2, weight=2)
        parent.grid_columnconfigure(0, weight=1)

        status_box = self.create_status_readout(parent)
        status_box.grid(row=0, column=0, sticky="ew", pady=(0, 12))

        work_area = ttk.Frame(parent, style="Main.TFrame")
        work_area.grid(row=1, column=0, sticky="nsew")

        work_area.grid_columnconfigure(0, weight=0, minsize=390)
        work_area.grid_columnconfigure(1, weight=1)
        work_area.grid_rowconfigure(0, weight=1)

        left = ttk.Frame(work_area, style="Main.TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))

        terminal_panel = ttk.LabelFrame(work_area, text="Board Callouts / Command Responses", style="Card.TLabelframe")
        terminal_panel.grid(row=0, column=1, sticky="nsew")
        terminal_panel.grid_rowconfigure(1, weight=1)
        terminal_panel.grid_columnconfigure(0, weight=1)

        terminal_toolbar = ttk.Frame(terminal_panel, style="Panel.TFrame")
        terminal_toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        self.make_gui_button(terminal_toolbar, "Clear", self.clear_callouts).pack(side="left", padx=(0, 6))
        self.make_gui_button(terminal_toolbar, "HELP", lambda: self.send_button_command("HELP")).pack(side="left", padx=(0, 6))
        self.make_gui_button(terminal_toolbar, "LOGHELP", lambda: self.send_button_command("LOGHELP")).pack(side="left", padx=(0, 6))
        self.make_gui_button(terminal_toolbar, "GETPARAMS", lambda: self.send_button_command("GETPARAMS")).pack(side="left", padx=(0, 6))

        self.callout_terminal = ScrolledText(
            terminal_panel,
            wrap="word",
            bg="#080808",
            fg="#f2f2f2",
            insertbackground="#ffffff",
            relief="flat",
            borderwidth=0,
            font=("Consolas", 9),
        )
        self.callout_terminal.grid(row=1, column=0, sticky="nsew")

        manual_bar = ttk.Frame(terminal_panel, style="Panel.TFrame", padding=(0, 8, 0, 0))
        manual_bar.grid(row=2, column=0, sticky="ew")

        ttk.Label(manual_bar, text="Manual Command", style="Normal.TLabel").pack(side="left", padx=(0, 8))

        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(manual_bar, textvariable=self.command_var)
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", lambda event: self.send_manual_command())

        self.make_gui_button(manual_bar, "Send", self.send_manual_command, danger=True).pack(side="left", padx=(8, 0))

        self.build_control_panel(left)

        data_frame = ttk.LabelFrame(parent, text="Raw DATA Stream", style="Card.TLabelframe")
        data_frame.grid(row=2, column=0, sticky="nsew", pady=(12, 0))
        data_frame.grid_rowconfigure(1, weight=1)
        data_frame.grid_columnconfigure(0, weight=1)

        data_toolbar = ttk.Frame(data_frame, style="Panel.TFrame")
        data_toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        self.make_gui_button(data_toolbar, "Clear DATA", self.clear_data_stream).pack(side="left", padx=(0, 8))

        self.data_stream_enabled_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            data_toolbar,
            text="Show DATA stream",
            variable=self.data_stream_enabled_var,
            command=self.update_data_stream_state,
        ).pack(side="left")

        self.data_terminal = ScrolledText(
            data_frame,
            wrap="none",
            bg="#050505",
            fg="#9fd3ff",
            insertbackground="#ffffff",
            relief="flat",
            borderwidth=0,
            font=("Consolas", 8),
            height=8,
        )
        self.data_terminal.grid(row=1, column=0, sticky="nsew")

    def build_control_panel(self, parent):
        controls = ttk.LabelFrame(parent, text="Controls", style="Card.TLabelframe")
        controls.pack(fill="both", expand=True)

        # Mini notebook keeps the control area clean while still giving each
        # group enough room.
        self.control_notebook = ttk.Notebook(controls, style="Control.TNotebook")
        self.control_notebook.pack(fill="both", expand=True)

        flight_tab = ttk.Frame(self.control_notebook, style="Panel.TFrame", padding=10)
        logs_tab = ttk.Frame(self.control_notebook, style="Panel.TFrame", padding=10)
        params_tab = ttk.Frame(self.control_notebook, style="Panel.TFrame", padding=10)
        profiles_tab = ttk.Frame(self.control_notebook, style="Panel.TFrame", padding=10)

        self.control_notebook.add(flight_tab, text="Flight")
        self.control_notebook.add(logs_tab, text="Logs")
        self.control_notebook.add(params_tab, text="Parameters")
        self.control_notebook.add(profiles_tab, text="Profiles")

        # ------------------------------------------------------------
        # Flight tab
        # ------------------------------------------------------------
        flight_tab.grid_columnconfigure(0, weight=1)

        ttk.Label(
            flight_tab,
            text="Bench and flight-state commands",
            style="Normal.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=(0, 8))

        self.make_gui_button(flight_tab, "STATUS", lambda: self.send_button_command("STATUS")).grid(row=1, column=0, sticky="ew", pady=4)
        self.make_gui_button(flight_tab, "RESET Flight State", lambda: self.send_button_command("RESET")).grid(row=2, column=0, sticky="ew", pady=4)
        self.make_gui_button(flight_tab, "Force LAUNCH", lambda: self.send_button_command("LAUNCH"), danger=True).grid(row=3, column=0, sticky="ew", pady=(12, 4))

        pyro_row = ttk.Frame(flight_tab, style="Panel.TFrame")
        pyro_row.grid(row=4, column=0, sticky="ew", pady=4)
        pyro_row.grid_columnconfigure(0, weight=1)
        pyro_row.grid_columnconfigure(1, weight=1)

        self.make_gui_button(pyro_row, "Test DROGUE", lambda: self.confirm_and_send("DROGUE"), danger=True).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.make_gui_button(pyro_row, "Test MAIN", lambda: self.confirm_and_send("MAIN"), danger=True).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        safety = ttk.LabelFrame(flight_tab, text="Safety Note", style="Card.TLabelframe")
        safety.grid(row=5, column=0, sticky="ew", pady=(14, 0))
        ttk.Label(
            safety,
            text="Only use pyro test commands with no charges/igniters connected unless you intentionally want to test outputs.",
            style="Normal.TLabel",
            justify="left",
            wraplength=330,
        ).pack(anchor="w")

        # ------------------------------------------------------------
        # Logs tab
        # ------------------------------------------------------------
        logs_tab.grid_columnconfigure(0, weight=1)

        slotrow = ttk.Frame(logs_tab, style="Panel.TFrame")
        slotrow.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        slotrow.grid_columnconfigure(1, weight=1)

        ttk.Label(slotrow, text="Selected Slot", style="Normal.TLabel").grid(row=0, column=0, sticky="w")
        self.slot_var = tk.StringVar(value="0")
        self.slot_combo = ttk.Combobox(slotrow, textvariable=self.slot_var, values=["0", "1", "2"], width=8, state="readonly")
        self.slot_combo.grid(row=0, column=1, sticky="e")

        self.make_gui_button(logs_tab, "List Logs", lambda: self.send_button_command("LIST")).grid(row=1, column=0, sticky="ew", pady=4)
        self.make_gui_button(logs_tab, "Log Status", lambda: self.send_button_command("LOGSTATUS")).grid(row=2, column=0, sticky="ew", pady=4)
        self.make_gui_button(logs_tab, "Download Selected Log", self.download_csv, danger=True).grid(row=3, column=0, sticky="ew", pady=(12, 4))
        self.make_gui_button(logs_tab, "Mark Downloaded", self.mark_downloaded).grid(row=4, column=0, sticky="ew", pady=4)

        erase_row = ttk.Frame(logs_tab, style="Panel.TFrame")
        erase_row.grid(row=5, column=0, sticky="ew", pady=(12, 4))
        erase_row.grid_columnconfigure(0, weight=1)
        erase_row.grid_columnconfigure(1, weight=1)

        self.make_gui_button(erase_row, "Erase Slot", self.erase_slot, danger=True).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.make_gui_button(erase_row, "Erase All", self.format_logs, danger=True).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        advanced_logs = ttk.LabelFrame(logs_tab, text="Manual Logging", style="Card.TLabelframe")
        advanced_logs.grid(row=6, column=0, sticky="ew", pady=(14, 0))
        advanced_logs.grid_columnconfigure(0, weight=1)
        advanced_logs.grid_columnconfigure(1, weight=1)

        self.make_gui_button(advanced_logs, "STARTLOG", lambda: self.send_button_command("STARTLOG")).grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=4)
        self.make_gui_button(advanced_logs, "STOPLOG", lambda: self.send_button_command("STOPLOG")).grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=4)

        # ------------------------------------------------------------
        # Parameters tab
        # ------------------------------------------------------------
        params_tab.grid_columnconfigure(0, weight=1)
        params_tab.grid_columnconfigure(1, weight=1)

        ttk.Label(
            params_tab,
            text="Active flight profile parameters",
            style="Normal.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        self.param_entries = {}
        self.param_unit_kinds = {}
        self.param_unit_labels = {}

        self.add_param_entry(params_tab, "MAIN_ALT", "200", row=1, column=0,
                             label="Main Deploy Alt", unit_kind="distance")
        self.add_param_entry(params_tab, "MAIN_ARM_MARGIN", "20", row=1, column=1,
                             label="Main Arm Margin", unit_kind="distance")
        self.add_param_entry(params_tab, "APOGEE_VZ_NEG", "-1.5", row=2, column=0,
                             label="Apogee Vz Negative", unit_kind="speed")
        self.add_param_entry(params_tab, "DROGUE_BACKUP_S", "15.0", row=2, column=1,
                             label="Drogue Backup", unit_kind="seconds")
        self.add_param_entry(params_tab, "MAIN_BACKUP_S", "25.0", row=3, column=0,
                             label="Main Backup", unit_kind="seconds")
        self.add_param_entry(params_tab, "LOCKOUT_S", "10.0", row=3, column=1,
                             label="Flight Lockout", unit_kind="seconds")

        methods = ttk.LabelFrame(params_tab, text="Detection Sources", style="Card.TLabelframe")
        methods.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(12, 0))

        self.method_vars = {
            "BARO_ENABLE": tk.BooleanVar(value=True),
            "ACCEL_ENABLE": tk.BooleanVar(value=True),
            "TIMER_ENABLE": tk.BooleanVar(value=True),
        }

        ttk.Checkbutton(
            methods,
            text="Barometer - apogee Vz + main altitude/fall-rate logic",
            variable=self.method_vars["BARO_ENABLE"],
        ).pack(anchor="w", pady=2)

        ttk.Checkbutton(
            methods,
            text="Accelerometer - automatic launch detection",
            variable=self.method_vars["ACCEL_ENABLE"],
        ).pack(anchor="w", pady=2)

        ttk.Checkbutton(
            methods,
            text="Timer - drogue/main backup timers",
            variable=self.method_vars["TIMER_ENABLE"],
        ).pack(anchor="w", pady=2)

        param_buttons = ttk.Frame(params_tab, style="Panel.TFrame")
        param_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        param_buttons.grid_columnconfigure(0, weight=1)
        param_buttons.grid_columnconfigure(1, weight=1)

        self.make_gui_button(param_buttons, "Send Parameters", self.send_parameters, danger=True).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.make_gui_button(param_buttons, "Read Parameters", lambda: self.send_button_command("GETPARAMS")).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        help_box = ttk.LabelFrame(params_tab, text="Note", style="Card.TLabelframe")
        help_box.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        ttk.Label(
            help_box,
            text=(
                "Times are entered in seconds and may contain decimals. The board still stores "
                "timing internally in milliseconds. Metric/Imperial only changes the GUI; the "
                "firmware and CSV logs stay in SI units."
            ),
            style="Normal.TLabel",
            justify="left",
            wraplength=350,
        ).pack(anchor="w")

        # ------------------------------------------------------------
        # Flight Profiles tab
        # ------------------------------------------------------------
        profiles_tab.grid_columnconfigure(0, weight=1)
        profiles_tab.grid_columnconfigure(1, weight=1)

        profile_select = ttk.Frame(profiles_tab, style="Panel.TFrame")
        profile_select.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        profile_select.grid_columnconfigure(1, weight=1)

        ttk.Label(profile_select, text="Profile Color", style="Normal.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.profile_color_var = tk.StringVar(value="VIOLET")
        self.profile_color_combo = ttk.Combobox(
            profile_select,
            textvariable=self.profile_color_var,
            values=["RED", "ORANGE", "YELLOW", "GREEN", "BLUE", "INDIGO", "VIOLET"],
            state="readonly",
            width=14,
        )
        self.profile_color_combo.grid(row=0, column=1, sticky="ew")
        self.profile_color_combo.bind("<<ComboboxSelected>>", lambda event: self.read_selected_profile())

        self.profile_entries = {}
        self.profile_unit_kinds = {}
        self.profile_unit_labels = {}

        self.add_profile_entry(profiles_tab, "MAIN_ALT", "200", row=1, column=0,
                               label="Main Deploy Alt", unit_kind="distance")
        self.add_profile_entry(profiles_tab, "MAIN_ARM_MARGIN", "20", row=1, column=1,
                               label="Main Arm Margin", unit_kind="distance")
        self.add_profile_entry(profiles_tab, "APOGEE_VZ_NEG", "-1.5", row=2, column=0,
                               label="Apogee Vz Negative", unit_kind="speed")
        self.add_profile_entry(profiles_tab, "DROGUE_BACKUP_S", "15.0", row=2, column=1,
                               label="Drogue Backup", unit_kind="seconds")
        self.add_profile_entry(profiles_tab, "MAIN_BACKUP_S", "25.0", row=3, column=0,
                               label="Main Backup", unit_kind="seconds")
        self.add_profile_entry(profiles_tab, "LOCKOUT_S", "10.0", row=3, column=1,
                               label="Flight Lockout", unit_kind="seconds")

        profile_methods = ttk.LabelFrame(profiles_tab, text="Profile Detection Sources", style="Card.TLabelframe")
        profile_methods.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(12, 0))

        self.profile_method_vars = {
            "BARO_ENABLE": tk.BooleanVar(value=True),
            "ACCEL_ENABLE": tk.BooleanVar(value=True),
            "TIMER_ENABLE": tk.BooleanVar(value=True),
        }

        ttk.Checkbutton(profile_methods, text="Barometer", variable=self.profile_method_vars["BARO_ENABLE"]).pack(side="left", padx=(0, 12))
        ttk.Checkbutton(profile_methods, text="Accelerometer launch", variable=self.profile_method_vars["ACCEL_ENABLE"]).pack(side="left", padx=(0, 12))
        ttk.Checkbutton(profile_methods, text="Timer backups", variable=self.profile_method_vars["TIMER_ENABLE"]).pack(side="left")

        profile_buttons = ttk.Frame(profiles_tab, style="Panel.TFrame")
        profile_buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        for c in range(3):
            profile_buttons.grid_columnconfigure(c, weight=1)

        self.make_gui_button(profile_buttons, "Read Profile", self.read_selected_profile).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.make_gui_button(profile_buttons, "Save Profile", self.save_selected_profile, danger=True).grid(row=0, column=1, sticky="ew", padx=4)
        self.make_gui_button(profile_buttons, "Apply Profile", self.apply_selected_profile, danger=True).grid(row=0, column=2, sticky="ew", padx=(4, 0))

        profile_note = ttk.LabelFrame(profiles_tab, text="Profile Behavior", style="Card.TLabelframe")
        profile_note.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        ttk.Label(
            profile_note,
            text=(
                "Each ROYGBIV color stores its own complete parameter set. Applying a profile "
                "changes the active color and loads all of its values. The physical MODE button "
                "cycles through these same stored profiles."
            ),
            style="Normal.TLabel",
            justify="left",
            wraplength=350,
        ).pack(anchor="w")

    def _unit_text(self, unit_kind):
        imperial = self.unit_mode.get() == 1
        if unit_kind == "distance":
            return "ft" if imperial else "m"
        if unit_kind == "speed":
            return "ft/s" if imperial else "m/s"
        if unit_kind == "seconds":
            return "s"
        return ""

    def _add_value_entry(self, parent, store, unit_store, unit_label_store,
                         name, default, row, column, label, unit_kind):
        frame = ttk.Frame(parent, style="Panel.TFrame")
        parent.grid_columnconfigure(column, weight=1)
        frame.grid(
            row=row,
            column=column,
            sticky="ew",
            padx=(0 if column == 0 else 6, 6 if column == 0 else 0),
            pady=2,
        )

        ttk.Label(frame, text=label, style="Normal.TLabel", width=19).pack(side="left")
        unit_var = tk.StringVar(value=self._unit_text(unit_kind))
        ttk.Label(frame, textvariable=unit_var, style="Normal.TLabel", width=5).pack(side="right", padx=(4, 0))

        var = tk.StringVar(value=default)
        ttk.Entry(frame, textvariable=var, width=10).pack(side="right", fill="x", expand=True)

        store[name] = var
        unit_store[name] = unit_kind
        unit_label_store[name] = unit_var

    def add_param_entry(self, parent, name, default, row, column, label=None, unit_kind=None):
        self._add_value_entry(
            parent,
            self.param_entries,
            self.param_unit_kinds,
            self.param_unit_labels,
            name,
            default,
            row,
            column,
            label or name,
            unit_kind,
        )

    def add_profile_entry(self, parent, name, default, row, column, label=None, unit_kind=None):
        self._add_value_entry(
            parent,
            self.profile_entries,
            self.profile_unit_kinds,
            self.profile_unit_labels,
            name,
            default,
            row,
            column,
            label or name,
            unit_kind,
        )

    def make_gui_button(self, parent, text, command, danger=False):
        normal_bg = self.colors["accent"] if danger else self.colors["panel3"]
        active_bg = self.colors["accent_dark"] if danger else "#303030"

        btn = tk.Button(
            parent,
            text=text,
            command=lambda: self.button_press_feedback(btn, command),
            bg=normal_bg,
            fg="white",
            activebackground=active_bg,
            activeforeground="white",
            relief="raised",
            bd=2,
            padx=8,
            pady=5,
            font=("Segoe UI", 10, "bold" if danger else "normal"),
            cursor="hand2",
            highlightthickness=0,
        )

        btn._normal_bg = normal_bg
        return btn

    def button_press_feedback(self, button, command):
        try:
            self.root.bell()
        except Exception:
            pass

        button.configure(relief="sunken", bg=self.colors["button_pressed"])
        self.root.after(140, lambda: button.configure(relief="raised", bg=button._normal_bg))

        command()

    # ============================================================
    # SERIAL
    # ============================================================

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        port_names = [p.device for p in ports]
        self.port_combo["values"] = port_names

        preferred = self.select_preferred_port(ports)

        if preferred:
            self.port_var.set(preferred)
        elif port_names and (not self.port_var.get() or self.port_var.get() not in port_names):
            self.port_var.set(port_names[0])

    def select_preferred_port(self, ports):
        if not ports:
            return None

        priority_names = [
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "COM3",
            "COM4",
            "COM5",
            "COM6",
            "COM7",
            "COM8",
            "COM9",
            "COM10",
        ]

        port_by_name = {p.device: p.device for p in ports}
        for name in priority_names:
            if name in port_by_name:
                return port_by_name[name]

        keywords = ["COMET", "RP2040", "PICO", "RPI-RP2", "USB SERIAL", "CDC", "ACM"]
        for p in ports:
            blob = f"{p.device} {p.description} {p.manufacturer} {p.hwid}".upper()
            if any(k in blob for k in keywords):
                return p.device

        if sys.platform.startswith("win"):
            com_ports = [p.device for p in ports if p.device.upper().startswith("COM")]
            if com_ports:
                return sorted(com_ports, key=lambda x: int("".join(filter(str.isdigit, x)) or 999))[0]

        return None

    def auto_connect_standard_port(self):
        if self.ser and self.ser.is_open:
            return

        self.refresh_ports()
        port = self.port_var.get()

        if not port:
            self.status_var.set("Disconnected - no serial ports found")
            return

        self.auto_connect_attempted = True
        self.connect(auto=True)

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect(auto=False)

    def connect(self, auto=False):
        port = self.port_var.get()
        baud = int(self.baud_var.get())

        if not port:
            if not auto:
                messagebox.showerror("No Port", "Select a serial port first.")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(0.8)

            self.reader_running = True
            self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.reader_thread.start()

            self.connection_lost = False
            self.device_verified = False
            self.last_telemetry_wall_time = None
            self.telemetry_rate_hz = 0.0
            self.clear_plots()

            self.connect_button.configure(text="Disconnect", style="Disconnect.TButton")
            self.status_var.set(f"Connected: {port} @ {baud}")
            self.log(f"[GUI] Connected to {port} @ {baud}\n")

            # Ask the board to identify/state itself. If the firmware responds with COMET text
            # or DATA packets, device_verified is set in handle_incoming_line().
            self.root.after(250, lambda: self.send_command("STATUS", log_to_terminal=True, board_beep=False, warn_if_disconnected=False))
            self.root.after(450, lambda: self.send_command("GETPARAMS", log_to_terminal=True, board_beep=False, warn_if_disconnected=False))

        except Exception as e:
            self.ser = None
            self.reader_running = False
            if auto:
                self.status_var.set("Auto-connect failed")
                self.log(f"[GUI] Auto-connect failed on {port}: {e}\n")
            else:
                messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        self.reader_running = False

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

        self.ser = None
        self.device_verified = False
        self.connect_button.configure(text="Connect", style="Connect.TButton")
        self.status_var.set("Disconnected")
        self.log("[GUI] Disconnected\n")

    def serial_reader(self):
        buffer = b""

        while self.reader_running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(1024)

                if data:
                    buffer += data

                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        text = line.decode(errors="replace").rstrip("\r")
                        self.rx_queue.put(text)

            except Exception as e:
                self.rx_queue.put(f"[GUI ERROR] Serial read failed: {e}")
                self.rx_queue.put("__GUI_CONNECTION_LOST__")
                break

        if self.reader_running:
            self.rx_queue.put("__GUI_CONNECTION_LOST__")

    def process_serial_queue(self):
        processed = 0
        while not self.rx_queue.empty() and processed < 120:
            line = self.rx_queue.get()
            if line == "__GUI_CONNECTION_LOST__":
                self.handle_connection_lost()
            else:
                self.handle_incoming_line(line)
            processed += 1

        self.root.after(50, self.process_serial_queue)

    def refresh_plots(self):
        if hasattr(self, "accel_plot"):
            self.accel_plot.redraw_if_dirty()
            self.gyro_plot.redraw_if_dirty()
            self.baro_plot.redraw_if_dirty()
        self.root.after(100, self.refresh_plots)

    def handle_incoming_line(self, line):
        is_data = line.startswith("DATA:")

        if is_data:
            self.device_verified = True
            self.handle_telemetry_line(line)
            if self.show_data_stream and getattr(self, "data_stream_enabled_var", None) is not None and self.data_stream_enabled_var.get():
                self.data_log(line + "\n")
        else:
            if line.startswith("PARAM "):
                self.handle_parameter_response(line)
            elif line.startswith("PROFILE "):
                self.handle_profile_response(line)

            if "COMET" in line.upper() or "STATE" in line.upper() or "BOOT" in line.upper() or line.startswith("PARAM ") or line.startswith("PROFILE "):
                self.device_verified = True

        if self.device_verified and self.ser and self.ser.is_open:
            port = self.ser.port
            self.status_var.set(f"Connected / Verified: {port}")

        if self.downloading:
            if line.startswith("BEGIN_CSV"):
                self.csv_capture_started = True
                self.download_lines = []
                self.log("[GUI] CSV download started\n")
                return

            if line.startswith("END_CSV"):
                self.log("[GUI] CSV download complete\n")
                self.finish_csv_download()
                return

            if self.csv_capture_started and line.strip():
                self.download_lines.append(line)
                return

        if not is_data:
            self.log(line + "\n")

    def handle_telemetry_line(self, line):
        data = self.parse_data_line(line)
        if not data:
            return

        self.last_telemetry_data = dict(data)

        now_wall = time.monotonic()
        if self.last_telemetry_wall_time is not None:
            dt = now_wall - self.last_telemetry_wall_time
            if dt > 0:
                inst_rate = 1.0 / dt
                self.telemetry_rate_hz = 0.85 * self.telemetry_rate_hz + 0.15 * inst_rate if self.telemetry_rate_hz > 0 else inst_rate
        self.last_telemetry_wall_time = now_wall

        self.update_live_status_from_data(data)

        try:
            t_sec = float(data.get("T_MS", "0")) / 1000.0
        except Exception:
            t_sec = time.monotonic()

        if not self.plot_paused:
            accel_factor = 3.280839895 if self.unit_mode.get() == 1 else 1.0
            altitude_factor = 3.280839895 if self.unit_mode.get() == 1 else 1.0

            self.accel_plot.add_point(
                t_sec,
                {
                    "AX": self._float_or_zero(data.get("AX")) * accel_factor,
                    "AY": self._float_or_zero(data.get("AY")) * accel_factor,
                    "AZ": self._float_or_zero(data.get("AZ")) * accel_factor,
                },
            )
            self.gyro_plot.add_point(
                t_sec,
                {
                    "GX": self._float_or_zero(data.get("GX")),
                    "GY": self._float_or_zero(data.get("GY")),
                    "GZ": self._float_or_zero(data.get("GZ")),
                },
            )
            self.baro_plot.add_point(
                t_sec,
                {"ALT": self._float_or_zero(data.get("ALT")) * altitude_factor},
            )

    def _float_or_zero(self, value):
        try:
            return float(value)
        except Exception:
            return 0.0

    def update_live_status_from_data(self, data):
        imperial = self.unit_mode.get() == 1
        distance_factor = 3.280839895 if imperial else 1.0
        speed_factor = 3.280839895 if imperial else 1.0

        if "STATE" in data:
            self.live_vars["STATE"].set(data["STATE"])

        if "ALT" in data:
            v = self._float_or_zero(data["ALT"]) * distance_factor
            self.live_vars["ALT"].set(f"{v:.2f} {'ft' if imperial else 'm'}")

        if "VZ" in data:
            v = self._float_or_zero(data["VZ"]) * speed_factor
            self.live_vars["VZ"].set(f"{v:.2f} {'ft/s' if imperial else 'm/s'}")

        if "MAXALT" in data:
            v = self._float_or_zero(data["MAXALT"]) * distance_factor
            self.live_vars["MAXALT"].set(f"{v:.2f} {'ft' if imperial else 'm'}")

        if "BATT" in data:
            self.live_vars["BATT"].set(f"{self._float_or_zero(data['BATT']):.2f} V")

        if "TEMP" in data:
            c = self._float_or_zero(data["TEMP"])
            if imperial:
                self.live_vars["TEMP"].set(f"{(c * 9.0 / 5.0 + 32.0):.1f} F")
            else:
                self.live_vars["TEMP"].set(f"{c:.1f} C")

        for key in ["SLOT", "REC"]:
            if key in data and key in self.live_vars:
                self.live_vars[key].set(data[key])

        if "LOG" in data:
            self.live_vars["LOG"].set("ACTIVE" if data["LOG"] == "1" else "OFF")

        self.live_vars["RATE"].set(f"{self.telemetry_rate_hz:.1f} Hz")

    def parse_data_line(self, line):
        parts = line.strip().split(":")
        if len(parts) < 3 or parts[0] != "DATA":
            return {}

        out = {"T_MS": parts[1]}
        i = 2
        while i + 1 < len(parts):
            key = parts[i].strip()
            value = parts[i + 1].strip()
            out[key] = value
            i += 2

        return out

    def _board_to_display(self, key, value):
        v = float(value)
        imperial = self.unit_mode.get() == 1

        if key in ("MAIN_ALT", "MAIN_ARM_MARGIN"):
            return v * 3.280839895 if imperial else v
        if key == "APOGEE_VZ_NEG":
            return v * 3.280839895 if imperial else v
        if key in ("DROGUE_BACKUP_MS", "MAIN_BACKUP_MS", "LOCKOUT_MS"):
            return v / 1000.0
        return v

    def _display_to_board(self, key, text):
        v = float(text)
        imperial = self.unit_mode.get() == 1

        if key in ("MAIN_ALT", "MAIN_ARM_MARGIN"):
            return v / 3.280839895 if imperial else v
        if key == "APOGEE_VZ_NEG":
            return v / 3.280839895 if imperial else v
        return v

    def _set_display_entry_from_board(self, entry_store, display_key, board_key, value):
        if display_key not in entry_store:
            return
        try:
            converted = self._board_to_display(board_key, value)
        except Exception:
            return

        if display_key in ("DROGUE_BACKUP_S", "MAIN_BACKUP_S", "LOCKOUT_S"):
            entry_store[display_key].set(f"{converted:.3f}".rstrip("0").rstrip("."))
        else:
            entry_store[display_key].set(f"{converted:.3f}".rstrip("0").rstrip("."))

    def handle_parameter_response(self, line):
        parts = line.split()
        if len(parts) < 3:
            return

        name = parts[1]
        value = parts[2]

        mapping = {
            "MAIN_ALT": ("MAIN_ALT", "MAIN_ALT"),
            "MAIN_ARM_MARGIN": ("MAIN_ARM_MARGIN", "MAIN_ARM_MARGIN"),
            "APOGEE_VZ_NEG": ("APOGEE_VZ_NEG", "APOGEE_VZ_NEG"),
            "DROGUE_BACKUP_MS": ("DROGUE_BACKUP_S", "DROGUE_BACKUP_MS"),
            "MAIN_BACKUP_MS": ("MAIN_BACKUP_S", "MAIN_BACKUP_MS"),
            "LOCKOUT_MS": ("LOCKOUT_S", "LOCKOUT_MS"),
        }

        if name in mapping:
            display_key, board_key = mapping[name]
            self._set_display_entry_from_board(self.param_entries, display_key, board_key, value)
        elif name in self.method_vars:
            self.method_vars[name].set(value.strip() not in ("0", "OFF", "FALSE"))

        if name == "MAIN_ALT_MODE" and len(parts) >= 4:
            self.profile_color_var.set(parts[3].upper())

    def handle_profile_response(self, line):
        parts = line.split()
        if len(parts) < 4:
            return

        color = parts[1].upper()
        if color != self.profile_color_var.get().upper():
            # PROFILE LIST may be streaming all colors; only populate the selected editor.
            return

        values = {}
        i = 2
        while i + 1 < len(parts):
            values[parts[i]] = parts[i + 1]
            i += 2

        mapping = {
            "MAIN_ALT": ("MAIN_ALT", "MAIN_ALT"),
            "MAIN_ARM_MARGIN": ("MAIN_ARM_MARGIN", "MAIN_ARM_MARGIN"),
            "APOGEE_VZ_NEG": ("APOGEE_VZ_NEG", "APOGEE_VZ_NEG"),
            "DROGUE_BACKUP_MS": ("DROGUE_BACKUP_S", "DROGUE_BACKUP_MS"),
            "MAIN_BACKUP_MS": ("MAIN_BACKUP_S", "MAIN_BACKUP_MS"),
            "LOCKOUT_MS": ("LOCKOUT_S", "LOCKOUT_MS"),
        }

        for board_name, (display_key, board_key) in mapping.items():
            if board_name in values:
                self._set_display_entry_from_board(
                    self.profile_entries, display_key, board_key, values[board_name]
                )

        for name, var in self.profile_method_vars.items():
            if name in values:
                var.set(values[name] not in ("0", "OFF", "FALSE"))

    def on_unit_switch(self, _value=None):
        new_mode = int(self.unit_mode.get())
        old_mode = int(self._last_unit_mode)
        if new_mode == old_mode:
            return

        factor = 3.280839895
        if new_mode == 1 and old_mode == 0:
            scale = factor
        else:
            scale = 1.0 / factor

        def convert_entries(store, unit_kinds):
            for key, var in store.items():
                kind = unit_kinds.get(key)
                if kind not in ("distance", "speed"):
                    continue
                try:
                    var.set(f"{float(var.get()) * scale:.3f}".rstrip("0").rstrip("."))
                except Exception:
                    pass

        if hasattr(self, "param_entries"):
            convert_entries(self.param_entries, self.param_unit_kinds)
            for key, unit_var in self.param_unit_labels.items():
                unit_var.set(self._unit_text(self.param_unit_kinds.get(key)))

        if hasattr(self, "profile_entries"):
            convert_entries(self.profile_entries, self.profile_unit_kinds)
            for key, unit_var in self.profile_unit_labels.items():
                unit_var.set(self._unit_text(self.profile_unit_kinds.get(key)))

        self._last_unit_mode = new_mode

        if hasattr(self, "accel_plot"):
            self.accel_plot.y_label = "ft/s²" if new_mode == 1 else "m/s²"
            self.baro_plot.y_label = "ft" if new_mode == 1 else "m"
            # Existing plot points are in the old display units; clear them instead
            # of mixing two unit systems on one graph.
            self.clear_plots()

        if self.last_telemetry_data:
            self.update_live_status_from_data(self.last_telemetry_data)

        self.log(f"[GUI] Units changed to {'Imperial' if new_mode == 1 else 'Metric'}.\n")

    def send_button_command(self, cmd):
        self.send_command(cmd, log_to_terminal=True, board_beep=True)

    def send_command(self, cmd, log_to_terminal=True, board_beep=False, warn_if_disconnected=True):
        if not self.ser or not self.ser.is_open:
            if warn_if_disconnected:
                messagebox.showwarning("Not Connected", "Connect to COMET first.")
            return False

        try:
            clean_cmd = cmd.strip()
            self.ser.write((clean_cmd + "\n").encode())

            if log_to_terminal:
                self.log(f">>> {clean_cmd}\n")

            if board_beep and self.BOARD_BEEP_COMMAND and clean_cmd.upper() != self.BOARD_BEEP_COMMAND.upper():
                # Tiny delay keeps the click/beep command behind the main command.
                self.root.after(80, lambda: self.send_command(self.BOARD_BEEP_COMMAND, log_to_terminal=False, board_beep=False, warn_if_disconnected=False))

            return True
        except Exception as e:
            messagebox.showerror("Send Error", str(e))
            return False

    def send_manual_command(self):
        cmd = self.command_var.get().strip()
        if not cmd:
            return
        self.send_command(cmd, log_to_terminal=True, board_beep=False)
        self.command_var.set("")

    def confirm_and_send(self, cmd):
        ok = messagebox.askyesno(
            "Confirm Command",
            f"Send {cmd} command?\n\nOnly do this with no charges or igniters connected unless you intentionally want to test outputs.",
        )
        if ok:
            self.send_button_command(cmd)

    def monitor_connection(self):
        """Periodically check whether the serial device still looks alive."""
        try:
            if self.ser and self.ser.is_open:
                port = self.ser.port

                # Physical unplug on Linux/Windows usually makes the port disappear from list_ports().
                available = {p.device for p in serial.tools.list_ports.comports()}
                if port and port not in available:
                    self.handle_connection_lost()

                # If telemetry/callouts stop for a long time, warn but do not force disconnect.
                elif self.last_telemetry_wall_time is not None:
                    age = time.monotonic() - self.last_telemetry_wall_time
                    if age > 5.0 and not self.connection_lost:
                        self.status_var.set(f"Connected: {port} - no recent telemetry")
        finally:
            self.root.after(1000, self.monitor_connection)

    def handle_connection_lost(self):
        if self.connection_lost:
            return

        self.connection_lost = True
        old_port = self.ser.port if self.ser else "unknown port"

        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass

        self.ser = None
        self.reader_running = False
        self.device_verified = False
        self.connect_button.configure(text="Connect", style="Connect.TButton")
        self.status_var.set("Disconnected - device removed")
        self.log(f"[GUI] Connection lost on {old_port}. Plug COMET back in and press Auto Connect or Connect.\\n")

    def update_plot_window(self):
        try:
            seconds = float(self.plot_window_seconds.get())
        except Exception:
            seconds = 10.0

        seconds = max(1.0, min(60.0, seconds))
        self.plot_window_seconds.set(seconds)

        for plot in [self.accel_plot, self.gyro_plot, self.baro_plot]:
            plot.window_seconds = seconds

            # Trim old points immediately so the screen reflects the new setting.
            all_points = []
            for vals in plot.data.values():
                all_points.extend(vals)

            if all_points:
                latest_t = max(t for t, _ in all_points)
                cutoff = latest_t - seconds
                for name in plot.series_names:
                    plot.data[name] = [(t, v) for (t, v) in plot.data[name] if t >= cutoff]

            plot.dirty = True

    def toggle_plot_pause(self):
        self.plot_paused = not self.plot_paused
        if hasattr(self, "plot_pause_button"):
            self.plot_pause_button.configure(text="Resume Plots" if self.plot_paused else "Pause Plots")

        self.log("[GUI] Plot updates paused. Live status and terminals still update.\\n" if self.plot_paused else "[GUI] Plot updates resumed.\\n")

    # ============================================================
    # DOWNLOAD / SLOT MANAGEMENT
    # ============================================================

    def download_csv(self):
        slot = self.slot_var.get()
        default_name = f"COMET_slot_{slot}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        self.show_csv_save_dialog(slot, default_name)

    def show_csv_save_dialog(self, slot, default_name):
        """Dark, readable replacement for the native save dialog."""
        dialog = tk.Toplevel(self.root)
        dialog.title(f"Save COMET Slot {slot} CSV")
        dialog.geometry("820x280")
        dialog.minsize(720, 250)
        dialog.configure(bg=self.colors["bg"])
        dialog.transient(self.root)
        dialog.grab_set()

        # Center over main window.
        self.root.update_idletasks()
        x = self.root.winfo_x() + max(0, (self.root.winfo_width() - 820) // 2)
        y = self.root.winfo_y() + max(0, (self.root.winfo_height() - 280) // 2)
        dialog.geometry(f"+{x}+{y}")

        outer = tk.Frame(dialog, bg=self.colors["bg"], padx=18, pady=18)
        outer.pack(fill="both", expand=True)

        title = tk.Label(
            outer,
            text=f"Save Slot {slot} CSV",
            bg=self.colors["bg"],
            fg=self.colors["accent_light"],
            font=("Segoe UI", 18, "bold"),
            anchor="w",
        )
        title.pack(fill="x", pady=(0, 14))

        dir_var = tk.StringVar(value=self.csv_default_dir)
        name_var = tk.StringVar(value=default_name)

        def make_label(parent, text):
            return tk.Label(
                parent,
                text=text,
                bg=self.colors["bg"],
                fg=self.colors["text"],
                font=("Segoe UI", 10, "bold"),
                anchor="w",
            )

        make_label(outer, "Folder").pack(fill="x")
        folder_row = tk.Frame(outer, bg=self.colors["bg"])
        folder_row.pack(fill="x", pady=(4, 12))

        dir_entry = tk.Entry(
            folder_row,
            textvariable=dir_var,
            bg="#101010",
            fg="#ffffff",
            insertbackground="#ffffff",
            selectbackground=self.colors["accent"],
            relief="solid",
            bd=1,
            font=("Consolas", 11),
        )
        dir_entry.pack(side="left", fill="x", expand=True, ipady=6)

        def browse_folder():
            selected = filedialog.askdirectory(
                parent=dialog,
                title="Choose folder for COMET CSV",
                initialdir=dir_var.get() if Path(dir_var.get()).exists() else str(Path.home()),
            )
            if selected:
                dir_var.set(selected)

        browse_btn = tk.Button(
            folder_row,
            text="Browse...",
            command=browse_folder,
            bg=self.colors["panel3"],
            fg="white",
            activebackground="#303030",
            activeforeground="white",
            relief="raised",
            bd=2,
            padx=14,
            pady=6,
            font=("Segoe UI", 10, "bold"),
        )
        browse_btn.pack(side="left", padx=(10, 0))

        make_label(outer, "File Name").pack(fill="x")
        name_entry = tk.Entry(
            outer,
            textvariable=name_var,
            bg="#101010",
            fg="#ffffff",
            insertbackground="#ffffff",
            selectbackground=self.colors["accent"],
            relief="solid",
            bd=1,
            font=("Consolas", 11),
        )
        name_entry.pack(fill="x", pady=(4, 14), ipady=6)
        name_entry.focus_set()
        name_entry.selection_range(0, "end")

        preview_var = tk.StringVar()

        def update_preview(*_):
            filename = name_var.get().strip()
            if filename and not filename.lower().endswith(".csv"):
                filename += ".csv"
            preview_var.set(str(Path(dir_var.get().strip()).expanduser() / filename) if filename else "")

        dir_var.trace_add("write", update_preview)
        name_var.trace_add("write", update_preview)
        update_preview()

        preview = tk.Label(
            outer,
            textvariable=preview_var,
            bg=self.colors["bg"],
            fg=self.colors["muted"],
            font=("Consolas", 9),
            anchor="w",
            wraplength=760,
        )
        preview.pack(fill="x", pady=(0, 14))

        button_row = tk.Frame(outer, bg=self.colors["bg"])
        button_row.pack(fill="x")

        def cancel():
            dialog.grab_release()
            dialog.destroy()

        def save():
            folder = Path(dir_var.get().strip()).expanduser()
            filename = name_var.get().strip()

            if not filename:
                messagebox.showwarning("Missing File Name", "Enter a CSV file name.", parent=dialog)
                return

            if not filename.lower().endswith(".csv"):
                filename += ".csv"

            if not folder.exists():
                ok = messagebox.askyesno(
                    "Create Folder?",
                    f"The folder does not exist:\n{folder}\n\nCreate it?",
                    parent=dialog,
                )
                if not ok:
                    return
                try:
                    folder.mkdir(parents=True, exist_ok=True)
                except Exception as e:
                    messagebox.showerror("Folder Error", f"Could not create folder:\n{e}", parent=dialog)
                    return

            path = folder / filename

            if path.exists():
                ok = messagebox.askyesno(
                    "Overwrite CSV?",
                    f"This file already exists:\n{path}\n\nOverwrite it?",
                    parent=dialog,
                )
                if not ok:
                    return

            self.csv_default_dir = str(folder)
            self.csv_save_path = str(path)
            self.download_slot = slot
            self.downloading = True
            self.csv_capture_started = False
            self.download_lines = []

            dialog.grab_release()
            dialog.destroy()

            self.send_button_command(f"DUMPCSV {slot}")

        cancel_btn = tk.Button(
            button_row,
            text="Cancel",
            command=cancel,
            bg=self.colors["panel3"],
            fg="white",
            activebackground="#303030",
            activeforeground="white",
            relief="raised",
            bd=2,
            padx=18,
            pady=8,
            font=("Segoe UI", 10, "bold"),
        )
        cancel_btn.pack(side="right", padx=(8, 0))

        save_btn = tk.Button(
            button_row,
            text="Save and Download",
            command=save,
            bg=self.colors["accent"],
            fg="white",
            activebackground=self.colors["accent_dark"],
            activeforeground="white",
            relief="raised",
            bd=2,
            padx=18,
            pady=8,
            font=("Segoe UI", 10, "bold"),
        )
        save_btn.pack(side="right")

        dialog.bind("<Escape>", lambda event: cancel())
        dialog.bind("<Return>", lambda event: save())

    def finish_csv_download(self):
        self.downloading = False
        self.csv_capture_started = False

        if not self.download_lines:
            messagebox.showwarning("No Data", "No CSV data was received.")
            return

        try:
            cleaned = []
            header_seen = False

            for line in self.download_lines:
                if line.startswith("t_ms,"):
                    header_seen = True
                    cleaned.append(line)
                elif header_seen:
                    if "," in line and not line.startswith("[GUI]"):
                        cleaned.append(line)

            if not cleaned:
                cleaned = self.download_lines

            with open(self.csv_save_path, "w", newline="") as f:
                for line in cleaned:
                    f.write(line + "\n")

            messagebox.showinfo("Download Complete", f"Saved CSV:\n{self.csv_save_path}")
            self.log(f"[GUI] Saved CSV to {self.csv_save_path}\n")

        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def mark_downloaded(self):
        slot = self.slot_var.get()
        ok = messagebox.askyesno(
            "Mark Downloaded",
            f"Mark slot {slot} as DOWNLOADED?\n\nThis does not erase the data, but allows COMET to reuse it later.",
        )
        if ok:
            self.send_button_command(f"MARKDOWNLOADED {slot}")

    def erase_slot(self):
        slot = self.slot_var.get()
        ok = messagebox.askyesno(
            "Erase Slot",
            f"Erase slot {slot}?\n\nThis permanently clears that saved COMET flight.",
        )
        if ok:
            self.send_button_command(f"ERASE {slot}")

    def format_logs(self):
        ok = messagebox.askyesno("Erase All Logs", "Erase ALL COMET flight logs?\n\nThis permanently clears every slot.")
        if ok:
            self.send_button_command("FORMATLOG")

    # ============================================================
    # PARAMETER HANDLING
    # ============================================================

    def _validate_detection_sources(self, methods):
        # Accelerometer is launch detection only. At least barometer or timer must
        # remain enabled or the board would have no automatic drogue/main path.
        if not methods["BARO_ENABLE"].get() and not methods["TIMER_ENABLE"].get():
            return messagebox.askyesno(
                "No Automatic Deployment Source",
                "Both Barometer and Timer are disabled.\n\n"
                "That leaves no automatic drogue/main deployment method. "
                "Continue anyway?",
            )
        return True

    def _send_value_set(self, board_name, display_key, var):
        text = var.get().strip()
        if not text:
            return

        if display_key in ("DROGUE_BACKUP_S", "MAIN_BACKUP_S", "LOCKOUT_S"):
            seconds = float(text)
            if seconds < 0:
                raise ValueError(f"{display_key} cannot be negative")
            value = int(round(seconds * 1000.0))
        else:
            value = self._display_to_board(board_name, text)

        if isinstance(value, float):
            value_text = f"{value:.6f}".rstrip("0").rstrip(".")
        else:
            value_text = str(value)

        self.send_command(f"SET {board_name} {value_text}", log_to_terminal=True, board_beep=False)
        time.sleep(0.04)

    def send_parameters(self):
        if not self._validate_detection_sources(self.method_vars):
            return

        mapping = [
            ("MAIN_ALT", "MAIN_ALT"),
            ("MAIN_ARM_MARGIN", "MAIN_ARM_MARGIN"),
            ("APOGEE_VZ_NEG", "APOGEE_VZ_NEG"),
            ("DROGUE_BACKUP_MS", "DROGUE_BACKUP_S"),
            ("MAIN_BACKUP_MS", "MAIN_BACKUP_S"),
            ("LOCKOUT_MS", "LOCKOUT_S"),
        ]

        try:
            for board_name, display_key in mapping:
                self._send_value_set(board_name, display_key, self.param_entries[display_key])

            for name, var in self.method_vars.items():
                self.send_command(
                    f"SET {name} {1 if var.get() else 0}",
                    log_to_terminal=True,
                    board_beep=False,
                )
                time.sleep(0.04)

        except ValueError as e:
            messagebox.showerror("Parameter Error", str(e))
            return

        if self.BOARD_BEEP_COMMAND:
            self.send_command(self.BOARD_BEEP_COMMAND, log_to_terminal=False, board_beep=False, warn_if_disconnected=False)

    def read_selected_profile(self):
        color = self.profile_color_var.get().strip().upper()
        if color:
            self.send_command(f"PROFILE GET {color}", log_to_terminal=True, board_beep=False)

    def _profile_board_value(self, board_name, display_key, var):
        text = var.get().strip()
        if not text:
            raise ValueError(f"{display_key} is blank")

        if display_key in ("DROGUE_BACKUP_S", "MAIN_BACKUP_S", "LOCKOUT_S"):
            seconds = float(text)
            if seconds < 0:
                raise ValueError(f"{display_key} cannot be negative")
            return str(int(round(seconds * 1000.0)))

        value = self._display_to_board(board_name, text)
        return f"{value:.6f}".rstrip("0").rstrip(".")

    def save_selected_profile(self):
        if not self._validate_detection_sources(self.profile_method_vars):
            return

        color = self.profile_color_var.get().strip().upper()
        mapping = [
            ("MAIN_ALT", "MAIN_ALT"),
            ("MAIN_ARM_MARGIN", "MAIN_ARM_MARGIN"),
            ("APOGEE_VZ_NEG", "APOGEE_VZ_NEG"),
            ("DROGUE_BACKUP_MS", "DROGUE_BACKUP_S"),
            ("MAIN_BACKUP_MS", "MAIN_BACKUP_S"),
            ("LOCKOUT_MS", "LOCKOUT_S"),
        ]

        try:
            for board_name, display_key in mapping:
                value = self._profile_board_value(
                    board_name, display_key, self.profile_entries[display_key]
                )
                self.send_command(
                    f"PROFILE SET {color} {board_name} {value}",
                    log_to_terminal=True,
                    board_beep=False,
                )
                time.sleep(0.04)

            for name, var in self.profile_method_vars.items():
                self.send_command(
                    f"PROFILE SET {color} {name} {1 if var.get() else 0}",
                    log_to_terminal=True,
                    board_beep=False,
                )
                time.sleep(0.04)

            self.root.after(250, self.read_selected_profile)

        except ValueError as e:
            messagebox.showerror("Profile Error", str(e))

    def apply_selected_profile(self):
        color = self.profile_color_var.get().strip().upper()
        ok = messagebox.askyesno(
            "Apply Flight Profile",
            f"Apply the {color} flight profile to COMET?\n\n"
            "This changes the active deployment parameters and RGB profile.",
        )
        if ok:
            self.send_command(f"PROFILE APPLY {color}", log_to_terminal=True, board_beep=False)
            self.root.after(250, lambda: self.send_command("GETPARAMS", log_to_terminal=True, board_beep=False))

    # ============================================================
    # TERMINAL / MISC
    # ============================================================

    def _append_to_text_widget(self, widget, text, max_lines, trim_lines):
        widget.insert("end", text)
        line_count = int(widget.index("end-1c").split(".")[0])
        if line_count > max_lines:
            widget.delete("1.0", f"{trim_lines}.0")
        widget.see("end")

    def log(self, text):
        if hasattr(self, "callout_terminal"):
            self._append_to_text_widget(self.callout_terminal, text, max_lines=2500, trim_lines=500)

    def data_log(self, text):
        if hasattr(self, "data_terminal"):
            self._append_to_text_widget(self.data_terminal, text, max_lines=900, trim_lines=250)

    def clear_terminal(self):
        self.clear_callouts()
        self.clear_data_stream()

    def clear_callouts(self):
        if hasattr(self, "callout_terminal"):
            self.callout_terminal.delete("1.0", "end")

    def clear_data_stream(self):
        if hasattr(self, "data_terminal"):
            self.data_terminal.delete("1.0", "end")

    def update_data_stream_state(self):
        self.show_data_stream = bool(self.data_stream_enabled_var.get())
        if self.show_data_stream:
            self.data_log("[GUI] DATA stream display enabled\n")
        else:
            self.log("[GUI] DATA stream display disabled. Dashboard still updates.\n")

    def clear_plots(self):
        self.accel_plot.clear()
        self.gyro_plot.clear()
        self.baro_plot.clear()

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = COMETGUI(root)
    root.mainloop()
