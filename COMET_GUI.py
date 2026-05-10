import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from tkinter.scrolledtext import ScrolledText
import serial
import serial.tools.list_ports
import threading
import queue
import time
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
        self.data = {name: [] for name in series_names}  # list of (t_sec, value)
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

        pad_l = 54
        pad_r = 16
        pad_t = 30
        pad_b = 34

        x0 = pad_l
        y0 = pad_t
        x1 = w - pad_r
        y1 = h - pad_b

        self.create_text(
            10,
            8,
            anchor="nw",
            text=f"{self.title}  |  last {self.window_seconds:.0f}s",
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

        # Plot border and grid
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

        # Time axis labels
        self.create_text(x0, y1 + 14, anchor="n", text=f"-{self.window_seconds:.0f}s", fill="#888888", font=("Consolas", 8))
        self.create_text(x1, y1 + 14, anchor="n", text="now", fill="#888888", font=("Consolas", 8))

        # Legend
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

        # Lines
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
    def __init__(self, root):
        self.root = root
        self.root.title("COMET Flight Computer Interface")
        self.root.geometry("1180x780")
        self.root.minsize(1050, 680)

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

        # Split serial output into two terminal panes:
        #   - data stream: raw DATA packets at telemetry rate
        #   - board callouts: commands, status, errors, LIST/LOGSTATUS, etc.
        self.show_data_stream = True

        self.colors = {
            "bg": "#0d0d0d",
            "panel": "#161616",
            "panel2": "#1d1d1d",
            "accent": "#c1121f",
            "accent_dark": "#8f0d16",
            "accent_light": "#ff4d5a",
            "text": "#f3f3f3",
            "muted": "#aaaaaa",
            "success": "#29c76f",
            "warn": "#ffb020",
        }

        self.configure_theme()
        self.build_ui()
        self.refresh_ports()

        self.root.after(50, self.process_serial_queue)
        self.root.after(100, self.refresh_plots)
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
        style.configure("Dark.TNotebook.Tab", background=self.colors["panel2"], foreground=self.colors["muted"], padding=(18, 8), font=("Segoe UI", 10, "bold"))
        style.map("Dark.TNotebook.Tab", background=[("selected", self.colors["accent"])], foreground=[("selected", "white")])

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
        self.terminal_tab = ttk.Frame(self.notebook, style="TabContent.TFrame")

        self.notebook.add(self.dashboard_tab, text="Dashboard")
        self.notebook.add(self.flight_tab, text="Flight Control")
        self.notebook.add(self.terminal_tab, text="Terminal")

        self.build_dashboard_tab(self.dashboard_tab)
        self.build_flight_tab(self.flight_tab)
        self.build_terminal_tab(self.terminal_tab)

    def build_connection_bar(self, parent):
        topbar = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        topbar.pack(fill="x", pady=(0, 12))

        ttk.Label(topbar, text="Port", style="Normal.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 6))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(topbar, textvariable=self.port_var, width=24, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w", padx=(0, 8))

        ttk.Button(topbar, text="Refresh", style="Dark.TButton", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 14))

        ttk.Label(topbar, text="Baud", style="Normal.TLabel").grid(row=0, column=3, sticky="w", padx=(0, 6))

        self.baud_var = tk.StringVar(value="115200")
        self.baud_combo = ttk.Combobox(topbar, textvariable=self.baud_var, values=["9600", "57600", "115200", "230400", "460800", "921600"], width=12, state="readonly")
        self.baud_combo.grid(row=0, column=4, sticky="w", padx=(0, 12))

        self.connect_button = ttk.Button(topbar, text="Connect", style="Connect.TButton", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=(0, 12))

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(topbar, textvariable=self.status_var, style="Status.TLabel").grid(row=0, column=6, sticky="w")

        topbar.grid_columnconfigure(7, weight=1)

    def build_dashboard_tab(self, parent):
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(0, weight=0, minsize=250)
        parent.grid_columnconfigure(1, weight=1, minsize=650)
        parent.grid_columnconfigure(2, weight=0, minsize=24)

        left = ttk.Frame(parent, style="Main.TFrame", width=250)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 12))
        left.grid_propagate(False)

        center = ttk.Frame(parent, style="Main.TFrame")
        center.grid(row=0, column=1, sticky="nsew")

        self.build_live_status_panel(left)

        self.accel_plot = LivePlot(center, title="Acceleration", series_names=["AX", "AY", "AZ"], window_seconds=10.0, y_label="m/s²", height=165)
        self.accel_plot.pack(fill="both", expand=True, pady=(0, 10))

        self.gyro_plot = LivePlot(center, title="Gyroscope", series_names=["GX", "GY", "GZ"], window_seconds=10.0, y_label="deg/s", height=165)
        self.gyro_plot.pack(fill="both", expand=True, pady=(0, 10))

        self.baro_plot = LivePlot(center, title="Barometric Altitude", series_names=["ALT"], window_seconds=10.0, y_label="m", height=165)
        self.baro_plot.pack(fill="both", expand=True)

    def build_live_status_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Live Status", style="Card.TLabelframe")
        box.pack(fill="x", pady=(0, 10))

        self.live_vars = {
            "STATE": tk.StringVar(value="--"),
            "ALT": tk.StringVar(value="-- m"),
            "VZ": tk.StringVar(value="-- m/s"),
            "BATT": tk.StringVar(value="-- V"),
            "TEMP": tk.StringVar(value="-- C"),
            "LOG": tk.StringVar(value="--"),
            "SLOT": tk.StringVar(value="--"),
            "REC": tk.StringVar(value="--"),
            "RATE": tk.StringVar(value="-- Hz"),
        }

        for label, var in self.live_vars.items():
            row = ttk.Frame(box, style="Panel.TFrame")
            row.pack(fill="x", pady=2)
            ttk.Label(row, text=label, style="Normal.TLabel", width=8).pack(side="left")
            value = ttk.Label(row, textvariable=var, style="Value.TLabel", anchor="e", width=14)
            value.pack(side="right")

        ttk.Button(box, text="Clear Plots", style="Dark.TButton", command=self.clear_plots).pack(fill="x", pady=(10, 0))

    def build_flight_tab(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)

        commands = ttk.LabelFrame(parent, text="Flight / Bench Commands", style="Card.TLabelframe")
        commands.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 10))

        ttk.Button(commands, text="STATUS", style="Dark.TButton", command=lambda: self.send_command("STATUS")).pack(fill="x", pady=4)
        ttk.Button(commands, text="Force LAUNCH", style="Danger.TButton", command=lambda: self.send_command("LAUNCH")).pack(fill="x", pady=4)
        ttk.Button(commands, text="RESET Flight State", style="Dark.TButton", command=lambda: self.send_command("RESET")).pack(fill="x", pady=4)
        ttk.Button(commands, text="Test DROGUE", style="Dark.TButton", command=lambda: self.confirm_and_send("DROGUE")).pack(fill="x", pady=4)
        ttk.Button(commands, text="Test MAIN", style="Dark.TButton", command=lambda: self.confirm_and_send("MAIN")).pack(fill="x", pady=4)

        logging = ttk.LabelFrame(parent, text="Logging / Data Recovery", style="Card.TLabelframe")
        logging.grid(row=0, column=1, sticky="nsew", padx=(8, 0), pady=(0, 10))

        ttk.Button(logging, text="LIST Slots", style="Dark.TButton", command=lambda: self.send_command("LIST")).pack(fill="x", pady=4)
        ttk.Button(logging, text="LOGSTATUS", style="Dark.TButton", command=lambda: self.send_command("LOGSTATUS")).pack(fill="x", pady=4)
        ttk.Button(logging, text="STARTLOG", style="Danger.TButton", command=lambda: self.send_command("STARTLOG")).pack(fill="x", pady=4)
        ttk.Button(logging, text="STOPLOG", style="Dark.TButton", command=lambda: self.send_command("STOPLOG")).pack(fill="x", pady=4)

        slotrow = ttk.Frame(logging, style="Panel.TFrame")
        slotrow.pack(fill="x", pady=(8, 6))
        ttk.Label(slotrow, text="Selected Slot", style="Normal.TLabel").pack(side="left")
        self.slot_var = tk.StringVar(value="0")
        self.slot_combo = ttk.Combobox(slotrow, textvariable=self.slot_var, values=["0", "1", "2"], width=8, state="readonly")
        self.slot_combo.pack(side="right")

        ttk.Button(logging, text="Download CSV", style="Danger.TButton", command=self.download_csv).pack(fill="x", pady=4)
        ttk.Button(logging, text="Mark Downloaded", style="Dark.TButton", command=self.mark_downloaded).pack(fill="x", pady=4)
        ttk.Button(logging, text="Erase Selected Slot", style="Dark.TButton", command=self.erase_slot).pack(fill="x", pady=4)
        ttk.Button(logging, text="FORMATLOG", style="Dark.TButton", command=self.format_logs).pack(fill="x", pady=4)

        params = ttk.LabelFrame(parent, text="Runtime Parameters", style="Card.TLabelframe")
        params.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        self.param_entries = {}
        grid = ttk.Frame(params, style="Panel.TFrame")
        grid.pack(fill="x")
        self.add_param_entry(grid, "MAIN_ALT", "200", 0, 0)
        self.add_param_entry(grid, "MAIN_ARM_MARGIN", "20", 0, 1)
        self.add_param_entry(grid, "DROGUE_BACKUP_MS", "15000", 1, 0)
        self.add_param_entry(grid, "MAIN_BACKUP_MS", "25000", 1, 1)
        self.add_param_entry(grid, "LOCKOUT_MS", "10000", 2, 0)

        row = ttk.Frame(params, style="Panel.TFrame")
        row.pack(fill="x", pady=(8, 0))
        ttk.Button(row, text="Send Parameters", style="Danger.TButton", command=self.send_parameters).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(row, text="GETPARAMS", style="Dark.TButton", command=lambda: self.send_command("GETPARAMS")).pack(side="left", fill="x", expand=True, padx=(4, 0))

        warning = ttk.LabelFrame(parent, text="Safety Note", style="Card.TLabelframe")
        warning.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Label(
            warning,
            text="Only use DROGUE or MAIN test commands with no charges/igniters connected unless you intentionally want to test outputs. Downloading does not erase data; MARKDOWNLOADED only makes a slot reusable later.",
            style="Normal.TLabel",
            justify="left",
            wraplength=1000,
        ).pack(anchor="w")

    def add_param_entry(self, parent, name, default, row, column):
        frame = ttk.Frame(parent, style="Panel.TFrame")
        frame.grid(row=row, column=column, sticky="ew", padx=(0 if column == 0 else 10, 10 if column == 0 else 0), pady=4)
        parent.grid_columnconfigure(column, weight=1)
        ttk.Label(frame, text=name, style="Normal.TLabel", width=22).pack(side="left")
        var = tk.StringVar(value=default)
        ttk.Entry(frame, textvariable=var, width=20).pack(side="left", fill="x", expand=True)
        self.param_entries[name] = var

    def build_terminal_tab(self, parent):
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(0, weight=1)

        box = ttk.LabelFrame(parent, text="Serial Monitor", style="Card.TLabelframe")
        box.grid(row=0, column=0, sticky="nsew")

        toolbar = ttk.Frame(box, style="Panel.TFrame")
        toolbar.pack(fill="x", pady=(0, 8))

        ttk.Button(toolbar, text="Clear Callouts", style="Dark.TButton", command=self.clear_callouts).pack(side="left")
        ttk.Button(toolbar, text="Clear Data", style="Dark.TButton", command=self.clear_data_stream).pack(side="left", padx=(6, 0))
        ttk.Button(toolbar, text="HELP", style="Dark.TButton", command=lambda: self.send_command("HELP")).pack(side="left", padx=(12, 0))
        ttk.Button(toolbar, text="LOGHELP", style="Dark.TButton", command=lambda: self.send_command("LOGHELP")).pack(side="left", padx=(6, 0))
        ttk.Button(toolbar, text="LIST", style="Dark.TButton", command=lambda: self.send_command("LIST")).pack(side="left", padx=(6, 0))
        ttk.Button(toolbar, text="GETPARAMS", style="Dark.TButton", command=lambda: self.send_command("GETPARAMS")).pack(side="left", padx=(6, 0))

        self.data_stream_enabled_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            toolbar,
            text="Show DATA stream",
            variable=self.data_stream_enabled_var,
            command=self.update_data_stream_state,
        ).pack(side="right")

        panes = ttk.PanedWindow(box, orient="vertical")
        panes.pack(fill="both", expand=True)

        callout_frame = ttk.LabelFrame(panes, text="Board Callouts / Command Responses", style="Card.TLabelframe")
        data_frame = ttk.LabelFrame(panes, text="Raw DATA Stream", style="Card.TLabelframe")

        self.callout_terminal = ScrolledText(
            callout_frame,
            wrap="none",
            height=13,
            bg="#080808",
            fg="#f2f2f2",
            insertbackground="#ffffff",
            relief="flat",
            borderwidth=0,
            font=("Consolas", 9),
        )
        self.callout_terminal.pack(fill="both", expand=True)

        self.data_terminal = ScrolledText(
            data_frame,
            wrap="none",
            height=10,
            bg="#050505",
            fg="#9fd3ff",
            insertbackground="#ffffff",
            relief="flat",
            borderwidth=0,
            font=("Consolas", 8),
        )
        self.data_terminal.pack(fill="both", expand=True)

        panes.add(callout_frame, weight=2)
        panes.add(data_frame, weight=1)

        bar = ttk.Frame(parent, style="Panel.TFrame", padding=10)
        bar.grid(row=1, column=0, sticky="ew", pady=(10, 0))

        ttk.Label(bar, text="Manual Command", style="Normal.TLabel").pack(side="left", padx=(0, 8))

        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(bar, textvariable=self.command_var)
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", lambda event: self.send_manual_command())

        ttk.Button(bar, text="Send", style="Danger.TButton", command=self.send_manual_command).pack(side="left", padx=(8, 0))

    # ============================================================
    # SERIAL
    # ============================================================

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        port_names = [p.device for p in ports]
        self.port_combo["values"] = port_names

        if port_names:
            if not self.port_var.get() or self.port_var.get() not in port_names:
                self.port_var.set(port_names[0])

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())

        if not port:
            messagebox.showerror("No Port", "Select a serial port first.")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(1.0)

            self.reader_running = True
            self.reader_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.reader_thread.start()

            self.connect_button.configure(text="Disconnect", style="Disconnect.TButton")
            self.status_var.set(f"Connected: {port} @ {baud}")
            self.log(f"[GUI] Connected to {port} @ {baud}\n")

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        self.reader_running = False

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

        self.ser = None
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
                break

    def process_serial_queue(self):
        processed = 0
        while not self.rx_queue.empty() and processed < 100:
            line = self.rx_queue.get()
            self.handle_incoming_line(line)
            processed += 1

        self.root.after(50, self.process_serial_queue)

    def refresh_plots(self):
        # Redraw plots on a fixed UI timer instead of redrawing all three plots
        # for every serial packet. This smooths the interface and prevents the
        # brief freezes that happened every few seconds.
        if hasattr(self, "accel_plot"):
            self.accel_plot.redraw_if_dirty()
            self.gyro_plot.redraw_if_dirty()
            self.baro_plot.redraw_if_dirty()
        self.root.after(100, self.refresh_plots)

    def handle_incoming_line(self, line):
        is_data = line.startswith("DATA:")

        if is_data:
            self.handle_telemetry_line(line)
            if self.show_data_stream and getattr(self, "data_stream_enabled_var", None) is not None and self.data_stream_enabled_var.get():
                self.data_log(line + "\n")

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

        for key in ["STATE", "ALT", "VZ", "BATT", "TEMP", "SLOT", "REC"]:
            if key in data and key in self.live_vars:
                suffix = {"ALT": " m", "VZ": " m/s", "BATT": " V", "TEMP": " C"}.get(key, "")
                self.live_vars[key].set(f"{data[key]}{suffix}")

        if "LOG" in data:
            self.live_vars["LOG"].set("ACTIVE" if data["LOG"] == "1" else "OFF")

        self.live_vars["RATE"].set(f"{self.telemetry_rate_hz:.1f} Hz")

        try:
            t_sec = float(data.get("T_MS", "0")) / 1000.0
        except Exception:
            t_sec = time.monotonic()

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

    def send_command(self, cmd):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Not Connected", "Connect to COMET first.")
            return

        try:
            self.ser.write((cmd.strip() + "\n").encode())
            self.log(f">>> {cmd.strip()}\n")
        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def send_manual_command(self):
        cmd = self.command_var.get().strip()
        if not cmd:
            return
        self.send_command(cmd)
        self.command_var.set("")

    def confirm_and_send(self, cmd):
        ok = messagebox.askyesno(
            "Confirm Command",
            f"Send {cmd} command?\n\nOnly do this with no charges or igniters connected unless you intentionally want to test outputs.",
        )
        if ok:
            self.send_command(cmd)

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
        self.send_command(f"DUMPCSV {slot}")

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
            self.send_command(f"MARKDOWNLOADED {slot}")

    def erase_slot(self):
        slot = self.slot_var.get()
        ok = messagebox.askyesno(
            "Erase Slot",
            f"Erase slot {slot}?\n\nThis permanently clears that saved COMET flight.",
        )
        if ok:
            self.send_command(f"ERASE {slot}")

    def format_logs(self):
        ok = messagebox.askyesno("Erase All Logs", "Erase ALL COMET flight logs?\n\nThis permanently clears every slot.")
        if ok:
            self.send_command("FORMATLOG")

    # ============================================================
    # PARAMETER HANDLING
    # ============================================================

    def send_parameters(self):
        for name, var in self.param_entries.items():
            value = var.get().strip()
            if value:
                self.send_command(f"SET {name} {value}")
                time.sleep(0.05)

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
        # Board callouts and command responses. DATA packets go to data_log().
        if hasattr(self, "callout_terminal"):
            self._append_to_text_widget(self.callout_terminal, text, max_lines=2500, trim_lines=500)

    def data_log(self, text):
        # Raw telemetry stream. Keep this shorter because DATA arrives at ~10 Hz.
        if hasattr(self, "data_terminal"):
            self._append_to_text_widget(self.data_terminal, text, max_lines=700, trim_lines=200)

    def clear_terminal(self):
        # Backward-compatible alias used by older callbacks.
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
