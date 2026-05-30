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

        self.last_telemetry_wall_time = None
        self.telemetry_rate_hz = 0.0
        self.last_state = "--"
        self.device_verified = False
        self.auto_connect_attempted = False
        self.connection_lost = False
        self.plot_paused = False
        self.plot_window_seconds = tk.DoubleVar(value=10.0)

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

        self.control_notebook.add(flight_tab, text="Flight")
        self.control_notebook.add(logs_tab, text="Logs")
        self.control_notebook.add(params_tab, text="Parameters")

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
            text="Runtime parameters",
            style="Normal.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        self.param_entries = {}

        self.add_param_entry(params_tab, "MAIN_ALT", "200", row=1, column=0)
        self.add_param_entry(params_tab, "MAIN_ARM_MARGIN", "20", row=1, column=1)
        self.add_param_entry(params_tab, "DROGUE_BACKUP_MS", "15000", row=2, column=0)
        self.add_param_entry(params_tab, "MAIN_BACKUP_MS", "25000", row=2, column=1)
        self.add_param_entry(params_tab, "LOCKOUT_MS", "10000", row=3, column=0)

        param_buttons = ttk.Frame(params_tab, style="Panel.TFrame")
        param_buttons.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        param_buttons.grid_columnconfigure(0, weight=1)
        param_buttons.grid_columnconfigure(1, weight=1)

        self.make_gui_button(param_buttons, "Send Parameters", self.send_parameters, danger=True).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.make_gui_button(param_buttons, "Read Parameters", lambda: self.send_button_command("GETPARAMS")).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        help_box = ttk.LabelFrame(params_tab, text="Note", style="Card.TLabelframe")
        help_box.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        ttk.Label(
            help_box,
            text="These values are for bench tuning and firmware-supported runtime changes. Use Read Parameters first if you are not sure what is currently loaded.",
            style="Normal.TLabel",
            justify="left",
            wraplength=330,
        ).pack(anchor="w")
    def add_param_entry(self, parent, name, default, row=None, column=None):
        frame = ttk.Frame(parent, style="Panel.TFrame")

        if row is None or column is None:
            frame.pack(fill="x", pady=3)
        else:
            parent.grid_columnconfigure(column, weight=1)
            frame.grid(row=row, column=column, sticky="ew", padx=(0 if column == 0 else 6, 6 if column == 0 else 0), pady=2)

        ttk.Label(frame, text=name, style="Normal.TLabel", width=18).pack(side="left")
        var = tk.StringVar(value=default)
        ttk.Entry(frame, textvariable=var, width=10).pack(side="right", fill="x", expand=True)
        self.param_entries[name] = var

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
            if "COMET" in line.upper() or "STATE" in line.upper() or "BOOT" in line.upper():
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

        now_wall = time.monotonic()
        if self.last_telemetry_wall_time is not None:
            dt = now_wall - self.last_telemetry_wall_time
            if dt > 0:
                inst_rate = 1.0 / dt
                self.telemetry_rate_hz = 0.85 * self.telemetry_rate_hz + 0.15 * inst_rate if self.telemetry_rate_hz > 0 else inst_rate
        self.last_telemetry_wall_time = now_wall

        suffixes = {
            "ALT": " m",
            "VZ": " m/s",
            "MAXALT": " m",
            "BATT": " V",
            "TEMP": " C",
        }

        for key in ["STATE", "ALT", "VZ", "MAXALT", "BATT", "TEMP", "SLOT", "REC"]:
            if key in data and key in self.live_vars:
                self.live_vars[key].set(f"{data[key]}{suffixes.get(key, '')}")

        if "LOG" in data:
            self.live_vars["LOG"].set("ACTIVE" if data["LOG"] == "1" else "OFF")

        self.live_vars["RATE"].set(f"{self.telemetry_rate_hz:.1f} Hz")

        try:
            t_sec = float(data.get("T_MS", "0")) / 1000.0
        except Exception:
            t_sec = time.monotonic()

        if not self.plot_paused:
            self.accel_plot.add_point(t_sec, {"AX": data.get("AX", 0), "AY": data.get("AY", 0), "AZ": data.get("AZ", 0)})
            self.gyro_plot.add_point(t_sec, {"GX": data.get("GX", 0), "GY": data.get("GY", 0), "GZ": data.get("GZ", 0)})
            self.baro_plot.add_point(t_sec, {"ALT": data.get("ALT", 0)})

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
        path = filedialog.asksaveasfilename(
            title=f"Save COMET Slot {slot} CSV",
            defaultextension=".csv",
            initialfile=f"COMET_slot_{slot}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
        )
        if not path:
            return

        self.csv_save_path = path
        self.download_slot = slot
        self.downloading = True
        self.csv_capture_started = False
        self.download_lines = []
        self.send_button_command(f"DUMPCSV {slot}")

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

    def send_parameters(self):
        for name, var in self.param_entries.items():
            value = var.get().strip()
            if value:
                self.send_command(f"SET {name} {value}", log_to_terminal=True, board_beep=False)
                time.sleep(0.05)

        if self.BOARD_BEEP_COMMAND:
            self.send_command(self.BOARD_BEEP_COMMAND, log_to_terminal=False, board_beep=False, warn_if_disconnected=False)

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
