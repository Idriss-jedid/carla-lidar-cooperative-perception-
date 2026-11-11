"""
The scenario editor window.

    python main.py editor                     # last map used (Town03 the first time)
    python main.py editor --map Town05
    python main.py editor --plan config/spawn_plans/figure2_town03.json

Left: the map (roads drawn from OpenDRIVE, or an aerial photo taken from CARLA).
Right: vehicles, placement rules, CARLA tools, export / record (with the LiDAR and
simulation settings), and the BEV of the recordings.

Works without CARLA running: maps come from the OpenDRIVE files shipped with
CARLA. CARLA is only needed to preview, test-spawn, take aerial photos and record;
processing a recording up to the BEV and showing it need no CARLA.
"""
import math
import os
import queue
import threading
import time
import tkinter as tk
from dataclasses import replace
from tkinter import filedialog, messagebox, ttk

import numpy as np

from ..backend.actors.spawn_plan import DEFAULT_ROUTE_SPEED, MAX_SPEED, PROJECT_ROOT
from ..backend.geometry.transforms import box_corners_2d
from ..backend.maps import catalog, raster, routes
from ..backend.maps.road_map import RoadMap, wrap_degrees
from ..core.enums import Role, RouteEnd
from ..core.errors import RouteError
from . import export
from . import results
from . import settings
from . import simulation
from .map_view import MapView
from .settings import MAPS_CACHE, load_settings, save_settings
from .model import PlannedVehicle, ScenarioPlan, footprint
from .widgets import color_button, scrolling_frame, tip

PAPER_BBOX = [-100.0, 100.0, -200.0, 200.0]

COLORS = {Role.CAV: "#3d8bfd", Role.TRAFFIC: "#ff7043"}
SELECTED = "#ffd54f"
OK_GHOST, BAD_GHOST = "#7ee787", "#ff6b6b"
STOP_MARK = "#e53935"
# one colour per CAV for its BEV cells on the (dark) map
BEV_COLORS = ("#4f9dff", "#ff8a4c", "#33d19a", "#c38cff", "#ffd54f", "#ff6fa8", "#5ad7e8", "#d4a373", "#b0bec5",
              "#a5d6a7")
TRUTH_OUTLINE = "#c9d1d9"
# simulation bar
START_GREEN, PAUSE_AMBER, STOP_GREY, REC_RED = "#2e7d32", "#b26a00", "#455a64", "#c62828"
BUTTON_OFF = "#c3c7cc"       # a button that cannot be pressed now
LIVE_TRAFFIC_LABELS = 9.0    # px per metre from which traffic vehicles get a label in the live view
CARLA_ACTIONS = ("Show the plan in the CARLA window", "Test spawn (spawn, check, remove)",
                 "Record in a new console (fixed frames)...", "Take aerial photo of this map")
STARTUP_MAP = "Start CARLA on this map next time..."
WATCH_EVERY_MS = 3000      # how often a recording started from the editor is checked
WATCH_IDLE_S = 180         # stop watching when nothing changed for this long
FONT = ("Segoe UI", 10)


class CarlaWorker:
    """Runs CARLA calls on one background thread so the window never freezes."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.results = queue.Queue()
        self._jobs = queue.Queue()
        threading.Thread(target=self._loop, daemon=True).start()

    def submit(self, name, func, *args):
        self._jobs.put((name, func, args))

    def client(self, timeout=5.0):
        import carla
        client = carla.Client(self.cfg.server.host, self.cfg.server.port)
        client.set_timeout(timeout)
        client.get_server_version()
        return client

    def _loop(self):
        while True:
            name, func, args = self._jobs.get()
            try:
                self.results.put((name, True, func(self, *args)))
            except Exception as exc:  # report every failure in the window
                self.results.put((name, False, exc))


# ---------------------------------------------------------------------- CARLA jobs (worker thread)

def _job_status(worker):
    client = worker.client()
    world = client.get_world()
    town = os.path.basename(world.get_map().name)
    cached = catalog.cache_server_map(world, MAPS_CACHE)
    return town, cached


def _job_preview(worker, planned, paths):
    from ..backend.simulator import live
    live.draw_plan(worker.client().get_world(), planned, paths)
    return len(planned)


def _job_test_spawn(worker, planned, cav_blueprint):
    from ..backend.simulator import live
    client = worker.client(timeout=20.0)
    return live.test_spawn(client, client.get_world(), planned, cav_blueprint)


def _job_aerial(worker, town, bounds, ppm, progress):
    from ..backend.maps import aerial
    client = worker.client(timeout=60.0)
    world = client.get_world()
    if os.path.basename(world.get_map().name) != town:
        raise RuntimeError("CARLA has {} loaded, not {}".format(os.path.basename(world.get_map().name), town))
    img, georef = aerial.capture(client, world, bounds, ppm, worker.cfg.server.tm_port, progress)
    raster.save_image(img, georef, raster.aerial_path(MAPS_CACHE, town))
    return town


# ---------------------------------------------------------------------- offline jobs (no CARLA)

def _job_process(worker, run_dir, frames):
    from .. import api
    return run_dir, api.process_recording(run_dir, frames, preview="none")


def _job_figure(worker, run_dir, frame, cav):
    return results.make_figure(run_dir, frame, cav)


def _job_view(worker, run_dir, frame, which, cav):
    return results.make_view(run_dir, frame, which, cav)


class EditorApp:

    def __init__(self, root, cfg, town=None, plan_path=None):
        self.root, self.cfg = root, cfg
        self.settings = load_settings()
        self.worker = CarlaWorker(cfg)
        self.offline = CarlaWorker(cfg)   # processing and figures, so CARLA jobs never wait for them
        self.server_town = None
        self.maps = []
        self.road_map = None
        self.backgrounds = {}
        self.plan = ScenarioPlan(town or self.settings.get("last_map", cfg.server.town))
        self.selected = None
        self.dirty = False
        self.plan_path = None
        self._drag = None
        self._ghost = None
        self._route_ghost = None   # (x, y, yaw) where a route click would land
        self._routes = {}          # traced routes: key -> (Route or None, error text)

        self.add_role = tk.StringVar(value=Role.CAV.value)
        self.background = tk.StringVar(value="roads")
        self.use_bbox = tk.BooleanVar(value=True)
        self.bbox_vars = [tk.StringVar(value=str(v)) for v in PAPER_BBOX]
        self.allow_junction = tk.BooleanVar(value=True)
        self.snap_distance = tk.DoubleVar(value=6.0)
        self.route_mode = tk.BooleanVar(value=False)
        self.speed_on = tk.BooleanVar(value=False)
        self.speed_var = tk.StringVar(value=str(int(DEFAULT_ROUTE_SPEED)))
        self.route_end = tk.StringVar(value=RouteEnd.STOP.value)
        self.status = tk.StringVar()
        self.cursor = tk.StringVar()
        self.carla_status = tk.StringVar(value="CARLA: checking...")
        self.show_bev = tk.BooleanVar(value=False)
        self.show_truth = tk.BooleanVar(value=True)
        self.runs = []             # recorded runs (results.RunInfo), newest first
        self.bev = None            # results.BevOverlay of the selected frame, or None
        self._watch = None         # recording started from the editor, until it is done
        self._scroll_areas = []    # (tab, canvas) of the scrolling tabs
        self.sim = None            # simulation.LiveSimulation while one runs
        self._live = None          # its last simulation.LiveView
        self._setting_widgets = []  # disabled while a simulation runs

        self._build()
        self._find_maps()
        if plan_path:
            self.open_plan(plan_path)
        else:
            self.open_map(self.plan.town)
        self.check_carla()
        self.refresh_runs()
        self.root.after(100, self._poll_worker)
        # the window has its real size only once shown
        self.root.after(250, self.fit_vehicles)

    # ================================================================== layout
    def _build(self):
        root = self.root
        root.title("CARLA Scenario Editor")
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        w, h = int(sw * 0.9), int(sh * 0.8)
        root.geometry("{}x{}+{}+{}".format(w, h, (sw - w) // 2, 10))
        root.minsize(min(1000, w), min(620, h))
        if os.name == "nt":
            root.state("zoomed")  # maximised, above the taskbar
        style = ttk.Style(root)
        if "vista" in style.theme_names():
            style.theme_use("vista")
        style.configure("Treeview", rowheight=22, font=FONT)
        style.configure("Title.TLabel", font=("Segoe UI", 11, "bold"))
        style.configure("Hint.TLabel", foreground="#6b7480")
        style.configure("Accent.TButton", font=("Segoe UI", 10, "bold"))
        style.configure("Sim.TLabel", font=("Segoe UI", 10))
        style.configure("TLabelframe.Label", font=("Segoe UI", 10, "bold"))

        self._build_menu()
        bar = ttk.Frame(root, padding=(8, 6))
        bar.pack(side="top", fill="x")
        ttk.Label(bar, text="Map", style="Title.TLabel").pack(side="left")
        self.map_box = ttk.Combobox(bar, state="readonly", width=30)
        self.map_box.pack(side="left", padx=(6, 16))
        self.map_box.bind("<<ComboboxSelected>>", lambda e: self._on_map_chosen())
        ttk.Label(bar, text="Background", style="Title.TLabel").pack(side="left")
        ttk.Radiobutton(bar, text="Roads", value="roads", variable=self.background,
                        command=self._apply_background).pack(side="left", padx=4)
        self.aerial_radio = ttk.Radiobutton(bar, text="Aerial photo", value="aerial",
                                            variable=self.background, command=self._apply_background)
        self.aerial_radio.pack(side="left", padx=4)
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=12)
        ttk.Button(bar, text="Fit map", command=self.fit_map).pack(side="left")
        ttk.Button(bar, text="Fit vehicles", command=self.fit_vehicles).pack(side="left", padx=4)
        ttk.Button(bar, text="Undo", command=self.undo).pack(side="left", padx=(12, 2))
        ttk.Button(bar, text="Redo", command=self.redo).pack(side="left")
        ttk.Label(bar, textvariable=self.carla_status).pack(side="right")

        sim = ttk.Frame(root, padding=(8, 0, 8, 6))
        sim.pack(side="top", fill="x")
        self.btn_start = tip(color_button(sim, "\u25b6  Start", START_GREEN, self.start_simulation),
                             "Spawn the scene in CARLA with the Settings and let it drive in real time. "
                             "CARLA must be running the plan's map.")
        self.btn_pause = tip(color_button(sim, "\u275a\u275a  Pause", PAUSE_AMBER, self.toggle_pause),
                             "Freeze the simulation / let it continue.")
        self.btn_stop = tip(color_button(sim, "\u25a0  Stop", STOP_GREY, self.stop_simulation),
                            "End the simulation: vehicles and LiDARs removed, CARLA back to normal.")
        self.btn_record = tip(color_button(sim, "\u25cf  Record", REC_RED, self.toggle_record, width=17),
                              "Mount the LiDARs and save the sweeps and ground truth while it drives (Settings > "
                              "Recording); the simulation runs slower meanwhile. Press again to stop; the recording "
                              "is then processed up to the BEV.")
        for button in (self.btn_start, self.btn_pause, self.btn_stop, self.btn_record):
            button.pack(side="left", padx=(0, 6))
        self.sim_text = tk.StringVar()
        ttk.Label(sim, textvariable=self.sim_text, style="Sim.TLabel").pack(side="left", padx=12)

        status = ttk.Frame(root, padding=(8, 3))
        status.pack(side="bottom", fill="x")
        ttk.Label(status, textvariable=self.status).pack(side="left")
        ttk.Label(status, textvariable=self.cursor, style="Hint.TLabel").pack(side="right")

        body = ttk.PanedWindow(root, orient="horizontal")
        body.pack(fill="both", expand=True)
        self.view = MapView(body, on_overlay=self._draw_overlay, on_view_change=lambda v: None)
        body.add(self.view, weight=4)
        side = ttk.Notebook(body, width=380)
        body.add(side, weight=0)
        side.add(self._build_vehicles_tab(side), text="  Scene  ")
        side.add(self._build_settings_tab(side), text="  Settings  ")
        self.bev_tab = self._build_bev_tab(side)
        side.add(self.bev_tab, text="  BEV  ")
        self.side = side

        v = self.view
        v.bind("<ButtonPress-1>", self._on_press)
        v.bind("<B1-Motion>", self._on_drag)
        v.bind("<ButtonRelease-1>", self._on_release)
        v.bind("<Motion>", self._on_motion)
        v.bind("<Leave>", lambda e: self._clear_ghosts())
        for seq, func in (("<Delete>", self.delete_selected), ("<Control-z>", self.undo),
                          ("<Control-y>", self.redo), ("<Control-s>", self.save_plan),
                          ("<Control-o>", self.ask_open_plan), ("<Control-n>", self.new_plan),
                          ("<Escape>", self.escape), ("1", lambda: self.add_role.set("cav")),
                          ("2", lambda: self.add_role.set("traffic")), ("r", self.toggle_role),
                          ("f", self.fit_vehicles), ("t", self.toggle_route_mode),
                          ("<Return>", lambda: self.set_route_mode(False)),
                          ("<BackSpace>", self.remove_route_point), ("<space>", self.toggle_pause)):
            root.bind(seq, lambda e, f=func: None if isinstance(e.widget, (tk.Entry, ttk.Entry, ttk.Spinbox))
                      else f())
        root.protocol("WM_DELETE_WINDOW", self.quit)
        # mouse wheel over a scrolling tab scrolls it; it no longer changes spin boxes and lists by accident
        for cls in ("TSpinbox", "TCombobox"):
            root.unbind_class(cls, "<MouseWheel>")
        root.bind_all("<MouseWheel>", self._on_wheel_anywhere, add="+")
        self._update_sim_bar()

    def _build_menu(self):
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="New plan", accelerator="Ctrl+N", command=self.new_plan)
        file_menu.add_command(label="Open plan...", accelerator="Ctrl+O", command=self.ask_open_plan)
        file_menu.add_command(label="Save plan", accelerator="Ctrl+S", command=self.save_plan)
        file_menu.add_command(label="Save plan as...", command=lambda: self.save_plan(ask=True))
        file_menu.add_separator()
        file_menu.add_command(label="Export experiment (.yaml)", command=self.export_experiment)
        file_menu.add_command(label="Import OpenSCENARIO (.xosc)...", command=self.import_xosc)
        file_menu.add_command(label="Export OpenSCENARIO (.xosc)...", command=self.export_xosc)
        file_menu.add_separator()
        file_menu.add_command(label="CARLA folder...", command=self.ask_carla_root)
        file_menu.add_separator()
        file_menu.add_command(label="Quit", command=self.quit)
        menu.add_cascade(label="File", menu=file_menu)
        carla = tk.Menu(menu, tearoff=False)
        carla.add_command(label="Check connection", command=self.check_carla)
        carla.add_command(label=CARLA_ACTIONS[0], command=self.preview)
        carla.add_command(label=CARLA_ACTIONS[1], command=self.test_spawn)
        carla.add_separator()
        carla.add_command(label=CARLA_ACTIONS[2], command=self.record_in_console)
        carla.add_separator()
        carla.add_command(label=CARLA_ACTIONS[3], command=self.capture_aerial)
        carla.add_command(label=STARTUP_MAP, command=self.set_startup_map)
        menu.add_cascade(label="CARLA", menu=carla)
        self._carla_menu = carla
        help_menu = tk.Menu(menu, tearoff=False)
        help_menu.add_command(label="Controls", command=self.show_help)
        menu.add_cascade(label="Help", menu=help_menu)
        self.root.config(menu=menu)

    def _section(self, parent, text):
        ttk.Label(parent, text=text, style="Title.TLabel").pack(anchor="w", pady=(10, 4))

    @staticmethod
    def _hint(parent, text):
        ttk.Label(parent, style="Hint.TLabel", wraplength=330, justify="left", text=text).pack(anchor="w", pady=(2, 4))

    @staticmethod
    def _form(form, rows):
        """Label + widget per row; a third item is a tooltip shown on both."""
        for r, row in enumerate(rows):
            label = ttk.Label(form, text=row[0])
            label.grid(row=r, column=0, sticky="w", pady=2)
            row[1].grid(row=r, column=1, sticky="w", padx=8, pady=2)
            if len(row) > 2:
                tip(label, row[2])
                tip(row[1], row[2])

    def _scrollable(self, parent):
        """(tab, content frame): a tab whose content scrolls vertically when it is taller than the window."""
        outer, inner, canvas = scrolling_frame(parent)
        self._scroll_areas.append((outer, canvas))
        return outer, inner

    def _on_wheel_anywhere(self, event):
        widget = self.root.winfo_containing(event.x_root, event.y_root)
        path = str(widget) if widget is not None else ""
        for outer, canvas in self._scroll_areas:
            name = str(outer)
            if (path == name or path.startswith(name + ".")) and canvas.yview() != (0.0, 1.0):
                canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")
                return

    def _build_vehicles_tab(self, parent):
        tab = ttk.Frame(parent, padding=10)
        row = ttk.Frame(tab)
        row.pack(fill="x")
        ttk.Label(row, text="Add", style="Title.TLabel").pack(side="left", padx=(0, 8))
        ttk.Radiobutton(row, text="CAV (LiDAR)  [1]", value="cav", variable=self.add_role).pack(side="left")
        ttk.Radiobutton(row, text="Traffic  [2]", value="traffic", variable=self.add_role).pack(side="left", padx=8)
        self.count_label = ttk.Label(tab, font=("Segoe UI", 10, "bold"))
        self.count_label.pack(anchor="w", pady=(8, 4))
        frame = ttk.Frame(tab)
        frame.pack(fill="both", expand=True)
        cols = ("name", "x", "y", "yaw", "lane", "drive")
        self.table = ttk.Treeview(frame, columns=cols, show="headings", selectmode="browse")
        for col, width in zip(cols, (64, 52, 58, 40, 62, 84)):
            self.table.heading(col, text={"name": "Vehicle", "drive": "km/h, route"}.get(col, col.upper()))
            self.table.column(col, width=width, anchor="center")
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.table.yview)
        self.table.configure(yscrollcommand=scroll.set)
        self.table.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.table.bind("<<TreeviewSelect>>", self._on_table_select)
        buttons = ttk.Frame(tab)
        buttons.pack(fill="x", pady=6)
        ttk.Button(buttons, text="CAV <-> Traffic  [R]", command=self.toggle_role).pack(side="left", expand=True,
                                                                                     fill="x")
        ttk.Button(buttons, text="Delete  [Del]", command=self.delete_selected).pack(side="left", expand=True,
                                                                                  fill="x", padx=4)
        ttk.Button(buttons, text="Clear all", command=self.clear_all).pack(side="left", expand=True, fill="x")
        self._build_driving_box(tab)
        ttk.Label(tab, style="Hint.TLabel", justify="left", text=(
            "Click a road: add  |  drag: move  |  right drag: pan")).pack(anchor="w", pady=6)
        return tab

    def _build_driving_box(self, tab):
        box = ttk.LabelFrame(tab, text="Speed and route of the selected vehicle", padding=8)
        box.pack(fill="x", pady=(4, 0))
        row = ttk.Frame(box)
        row.pack(fill="x")
        self.chk_speed = ttk.Checkbutton(row, text="Own speed (km/h)", variable=self.speed_on,
                                         command=self._apply_speed)
        self.chk_speed.pack(side="left")
        self.spin_speed = ttk.Spinbox(row, from_=5, to=MAX_SPEED, increment=5, width=6,
                                      textvariable=self.speed_var, command=self._apply_speed)
        self.spin_speed.pack(side="left", padx=6)
        for seq in ("<Return>", "<FocusOut>"):
            self.spin_speed.bind(seq, lambda e: self._apply_speed())
        row = ttk.Frame(box)
        row.pack(fill="x", pady=(6, 0))
        self.btn_route = ttk.Checkbutton(row, text="Draw route  [T]", variable=self.route_mode,
                                         style="Toolbutton", command=self._on_route_button)
        self.btn_route.pack(side="left", expand=True, fill="x")
        self.btn_route_back = ttk.Button(row, text="Undo point", command=self.remove_route_point)
        self.btn_route_back.pack(side="left", padx=4)
        self.btn_route_clear = ttk.Button(row, text="Clear route", command=self.clear_route)
        self.btn_route_clear.pack(side="left")
        row = ttk.Frame(box)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="At the last point:").pack(side="left")
        self.end_radios = [ttk.Radiobutton(row, text=text, value=end.value, variable=self.route_end,
                                           command=self._apply_route_end)
                           for text, end in (("stop", RouteEnd.STOP), ("keep driving", RouteEnd.AUTOPILOT))]
        for radio in self.end_radios:
            radio.pack(side="left", padx=4)
        self.driving_info = ttk.Label(box, wraplength=330, justify="left")
        self.driving_info.pack(anchor="w", pady=(6, 0))
        self._driving_widgets = [self.chk_speed, self.spin_speed, self.btn_route, self.btn_route_back,
                                 self.btn_route_clear] + self.end_radios

    def _build_settings_tab(self, parent):
        outer, tab = self._scrollable(parent)
        sim, sc = self.cfg.simulation, self.cfg.scenario
        self.exp_name = tk.StringVar(value="my_scenario")
        self.extra_traffic = tk.IntVar(value=0)
        self.exp_autopilot = tk.BooleanVar(value=True)
        self.exp_frames = tk.IntVar(value=0)
        self.exp_every = tk.IntVar(value=20)
        self.lidar_vars = {key: tk.StringVar() for key in export.LIDAR_KEYS}
        self._default_lidar()
        self.seed_var = tk.StringVar(value=str(sim.seed))
        self.lights_vars = [tk.StringVar(value="{:g}".format(v)) for v in sc.ignore_lights_range]
        self.process_after = tk.BooleanVar(value=True)
        keep = self._setting_widgets.append

        def box(title):
            frame = ttk.LabelFrame(tab, text=title, padding=(10, 6))
            frame.pack(fill="x", pady=(0, 10))
            return frame

        def spin(parent, var, low, high, step, width=8):
            widget = ttk.Spinbox(parent, from_=low, to=high, increment=step, width=width, textvariable=var)
            keep(widget)
            return widget

        def combo(parent, var, values, width=10):
            widget = ttk.Combobox(parent, textvariable=var, values=values, width=width)
            keep(widget)
            return widget

        def check(parent, text, var, tooltip, command=None):
            widget = tip(ttk.Checkbutton(parent, text=text, variable=var, command=command), tooltip)
            widget.pack(anchor="w", pady=1)
            keep(widget)
            return widget

        ttk.Label(tab, style="Hint.TLabel", text="Hover a setting to see what it does.").pack(anchor="w",
                                                                                           pady=(0, 6))
        rules = box("Placement")
        check(rules, "Only inside the spawn area", self.use_bbox,
              "Vehicles may only be placed inside this world area (yellow box). Default on Town03: the paper's "
              "area, x -100..100, y -200..200.", self.redraw)
        grid = ttk.Frame(rules)
        grid.pack(anchor="w", pady=2)
        for i, label in enumerate(("x min", "x max", "y min", "y max")):
            ttk.Label(grid, text=label).grid(row=i // 2, column=(i % 2) * 2, sticky="e", padx=(0, 4), pady=2)
            entry = ttk.Entry(grid, textvariable=self.bbox_vars[i], width=8)
            entry.grid(row=i // 2, column=(i % 2) * 2 + 1, padx=(0, 12), pady=2)
            entry.bind("<FocusOut>", lambda e: self.redraw())
            entry.bind("<Return>", lambda e: self.redraw())
            keep(entry)
        keep(tip(ttk.Button(rules, text="Paper's area", command=self._paper_bbox),
                 "x -100..100, y -200..200 (paper, Table 1)."))
        self._setting_widgets[-1].pack(anchor="w", pady=(0, 4))
        check(rules, "Vehicles in junctions", self.allow_junction,
              "Off: a vehicle can only be placed on the legs, not inside a junction.")
        form = ttk.Frame(rules)
        form.pack(fill="x")
        self._form(form, (("Max distance to a lane (m)", spin(form, self.snap_distance, 1, 20, 1, 6),
                           "A click farther than this from a driving lane is refused."),))

        lidar = box("LiDAR (every CAV)")
        form = ttk.Frame(lidar)
        form.pack(fill="x")
        v = self.lidar_vars
        self._form(form, (
            ("Channels", combo(form, v["channels"], ("16", "32", "64")),
             "Laser rings, spread from +2 to -25 degrees. Paper: 32."),
            ("Range (m)", spin(form, v["range"], 10, 300, 10),
             "Farthest distance measured. Paper: 120 m."),
            ("Points per second", combo(form, v["points_per_second"], ("500000", "1000000", "2000000")),
             "Rays per second over all channels; one sweep (1/20 s) gets 1/20 of them. Paper: 1 000 000."),
            ("Mount height (m)", spin(form, v["mount_z"], 1.0, 4.0, 0.1),
             "Height of the LiDAR above the ground. Below about 2 m it also sees its own roof (the ego mask "
             "removes those points); 1.48 m or lower is inside a Tesla Model 3. Ours: 2.4 m."),
            ("Noise (m)", spin(form, v["noise_stddev"], 0.0, 0.5, 0.01),
             "Random error added to every range (standard deviation). 0 = perfect.")))
        keep(tip(ttk.Button(lidar, text="Paper values", command=self._default_lidar),
                 "Back to 32 channels, 120 m, 1 000 000 points/s, 2.4 m, no noise."))
        self._setting_widgets[-1].pack(anchor="w", pady=(4, 0))

        traffic = box("Traffic")
        check(traffic, "Vehicles drive (autopilot)", self.exp_autopilot,
              "Off: vehicles without a route or a speed stay parked (with the brakes on).")
        form = ttk.Frame(traffic)
        form.pack(fill="x")
        lights = ttk.Frame(form)
        spin(lights, self.lights_vars[0], 0, 100, 10, 5).pack(side="left")
        ttk.Label(lights, text=" to ").pack(side="left")
        spin(lights, self.lights_vars[1], 0, 100, 10, 5).pack(side="left")
        ttk.Label(lights, text=" %").pack(side="left")
        self._form(form, (
            ("Extra random traffic", spin(form, self.extra_traffic, 0, 100, 1),
             "Vehicles added at random CARLA spawn points (inside the spawn area if it is on), on top of yours."),
            ("Seed", spin(form, self.seed_var, 0, 999999, 1),
             "Chooses the random extra traffic, the vehicle models and the Traffic Manager's draws. Same seed "
             "and a parked scene give the same recording."),
            ("Red lights ignored", lights,
             "Each Traffic Manager vehicle draws its chance of running a red light in this range. Paper: "
             "0-100 %. Vehicles with a route always stop.")))
        note = tip(ttk.Label(traffic, style="Hint.TLabel", text="Time step: 0.05 s (fixed)"),
                   "One tick = 1 / 20 Hz, so every tick is one full 360-degree LiDAR sweep. Another value would "
                   "give partial sweeps.")
        note.pack(anchor="w", pady=(4, 0))

        rec = box("Recording")
        form = ttk.Frame(rec)
        form.pack(fill="x")
        name = ttk.Entry(form, textvariable=self.exp_name, width=20)
        keep(name)
        self._form(form, (
            ("Name", name, "Run folder data/runs/<date>_<time>_<name>, and the name of the exported plan."),
            ("Save every N ticks", spin(form, self.exp_every, 1, 1000, 1),
             "20 ticks = 1 simulated second, so 20 saves one frame per second."),
            ("Frames (0 = until you stop)", spin(form, self.exp_frames, 0, 10000, 1),
             "A recording ends by itself after this many frames; 0 = when you press Record again.")))
        check(rec, "Process up to the BEV when it ends", self.process_after,
              "Every frame: ego mask, drivable-road filter, ground removal, BEV, sectors and metadata. "
              "The BEV tab then shows it on the map.")
        return outer

    def _default_lidar(self):
        li = self.cfg.lidar
        for key in export.LIDAR_KEYS:
            value = getattr(li, key)
            self.lidar_vars[key].set(str(int(value)) if key in ("channels", "points_per_second")
                                     else "{:g}".format(value))

    def _build_bev_tab(self, parent):
        outer, tab = self._scrollable(parent)
        self._section(tab, "Recordings")
        row = ttk.Frame(tab)
        row.pack(fill="x")
        self.run_box = ttk.Combobox(row, state="readonly", width=38)
        self.run_box.pack(side="left", fill="x", expand=True)
        self.run_box.bind("<<ComboboxSelected>>", lambda e: self._on_run_chosen())
        ttk.Button(row, text="Refresh", command=self.refresh_runs).pack(side="left", padx=(4, 0))
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="Frame").pack(side="left")
        self.frame_box = ttk.Combobox(row, state="readonly", width=14)
        self.frame_box.pack(side="left", padx=6)
        self.frame_box.bind("<<ComboboxSelected>>", lambda e: self._on_frame_chosen())
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=6)
        process_tip = ("Ego mask, drivable-road filter, ground removal, BEV (0.5 m cells, +-60 m), 4 sectors and "
                       "their metadata. About 0.1 s per CAV and frame; no CARLA needed.")
        self.btn_process = tip(ttk.Button(row, text="Process this frame", command=lambda: self.process_run(False)),
                               process_tip)
        self.btn_process.pack(side="left", expand=True, fill="x")
        self.btn_process_all = tip(ttk.Button(row, text="Process all frames",
                                              command=lambda: self.process_run(True)), process_tip)
        self.btn_process_all.pack(side="left", expand=True, fill="x", padx=(4, 0))

        self._section(tab, "On the map")
        tip(ttk.Checkbutton(tab, text="BEV of every CAV", variable=self.show_bev, command=self._on_show_bev),
            "Each coloured square = an occupied 0.5 m cell of one CAV (at least one non-ground return), placed "
            "with the CAV's recorded pose; the dot is the LiDAR. Empty = no return, not free.").pack(anchor="w")
        tip(ttk.Checkbutton(tab, text="Recorded vehicles", variable=self.show_truth, command=self.redraw),
            "Ground-truth boxes of that frame (grey outlines).").pack(anchor="w")
        self.bev_legend = ttk.Frame(tab)
        self.bev_legend.pack(fill="x", pady=4)

        self._section(tab, "One CAV, stage by stage")
        row = ttk.Frame(tab)
        row.pack(fill="x")
        ttk.Label(row, text="CAV").pack(side="left")
        self.cav_box = ttk.Combobox(row, state="readonly", width=8)
        self.cav_box.pack(side="left", padx=6)
        self.btn_figure = tip(ttk.Button(row, text="Open figure", command=self.open_figure),
                              "Raw sweep, drivable road, ground removed, BEV with the 4 sectors and their size and "
                              "quality. Saved as processed/<frame>/<cav>_pipeline.png.")
        self.btn_figure.pack(side="left", fill="x", expand=True)
        self._section(tab, "All CAVs, before and after filtering")
        grid = ttk.Frame(tab)
        grid.pack(fill="x")
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        views = (("world_raw", "Raw, all CAVs",
                  "The raw point clouds of every CAV superposed in the world frame, one colour per CAV; grey "
                  "rectangles = CARLA's ground-truth boxes (drawn for every car, seen or not)."),
                 ("world_filtered", "After filtering",
                  "The points left after ego mask, road filter and ground removal, every CAV in the world frame. "
                  "A box is solid when a CAV has >= 10 points on that car, dashed when no CAV does."),
                 ("world_bev", "BEV, all CAVs",
                  "The BEV of all CAVs together: non-ground points per 0.5 m cell, with a zoom on the car with the "
                  "most points."),
                 ("vehicles", "Points per car",
                  "For each car: points on its body before -> after filtering (all CAVs together), and which CAVs "
                  "see it."))
        self.view_buttons = []
        for n, (which, text, tooltip) in enumerate(views):
            button = tip(ttk.Button(grid, text=text, command=lambda w=which: self.open_view(w)), tooltip)
            button.grid(row=n // 2, column=n % 2, sticky="ew", padx=(0 if n % 2 == 0 else 4, 0), pady=2)
            self.view_buttons.append(button)
        row = ttk.Frame(tab)
        row.pack(fill="x", pady=(2, 0))
        ttk.Label(row, text="CAV").pack(side="left")
        self.cav_view_box = ttk.Combobox(row, state="readonly", width=8)
        self.cav_view_box.pack(side="left", padx=6)
        button = tip(ttk.Button(row, text="This CAV: before / after",
                                command=lambda: self.open_view("cav", self.cav_view_box.get())),
                     "One CAV in its own frame: raw sweep | after filtering | BEV (points per cell).")
        button.pack(side="left", fill="x", expand=True)
        self.view_buttons.append(button)
        self._hint(tab, "Each opens on its own; vehicles.csv and cavs.csv (the numbers) are written next to them.")
        self.bev_info = ttk.Label(tab, wraplength=330, justify="left")
        self.bev_info.pack(anchor="w", pady=8)
        return outer

    # ================================================================== maps
    def _find_maps(self):
        root = settings.carla_root(self.cfg, self.settings)
        self.carla_root = root
        self.maps = catalog.list_maps(root, MAPS_CACHE)
        if root is None and not self.maps:
            self.root.after(300, self.ask_carla_root)
        self.map_box["values"] = [m.label for m in self.maps]

    def ask_carla_root(self):
        path = filedialog.askdirectory(title="CARLA folder (the one with CarlaUE4.exe)")
        if not path:
            return
        if not catalog.is_carla_root(path):
            messagebox.showerror("Not a CARLA folder", "No CarlaUE4/Content/Carla/Maps/OpenDrive in\n" + path)
            return
        self.settings["carla_root"] = path
        save_settings(self.settings)
        self._find_maps()
        self.open_map(self.plan.town)

    def _on_map_chosen(self):
        if self._locked():
            self._show_map_in_box()
            return
        info = self.maps[self.map_box.current()]
        if info.name == self.plan.town:
            return
        if self.plan.vehicles and not self._confirm_discard():
            self._show_map_in_box()
            return
        self.plan = ScenarioPlan(info.name)
        self.plan_path = None
        self.dirty = False
        self.open_map(info.name)

    def open_map(self, name):
        info = catalog.find_map(self.maps, name)
        if info is None:
            self.status.set("Map {} not found. Set the CARLA folder (File menu) or start CARLA.".format(name))
            self.view.redraw()
            return
        self.root.config(cursor="watch")
        self.root.update_idletasks()
        try:
            self.road_map = RoadMap.from_file(info.name, info.xodr_path)
            self._routes = {}
            roads = raster.road_image(self.road_map, info.xodr_path, MAPS_CACHE)
        finally:
            self.root.config(cursor="")
        self.plan.town = info.name
        self.backgrounds = {"roads": roads}
        aerial = raster.load_image(raster.aerial_path(MAPS_CACHE, info.name))
        if aerial:
            self.backgrounds["aerial"] = aerial
        if self.background.get() not in self.backgrounds:
            self.background.set("roads")
        self.use_bbox.set(info.name == "Town03")
        self.settings["last_map"] = info.name
        save_settings(self.settings)
        self._show_map_in_box()
        self._apply_background(fit=True)
        self.refresh()
        self._update_carla_widgets()
        self.status.set("{} loaded: {:.0f} m of lanes. Click on a road to place a vehicle.".format(
            info.name, len(self.road_map.lanes) * 1.0))

    def _show_map_in_box(self):
        names = [m.name for m in self.maps]
        if self.plan.town in names:
            self.map_box.current(names.index(self.plan.town))

    def _apply_background(self, fit=False):
        kind = self.background.get()
        if kind not in self.backgrounds:
            self.background.set("roads")
            kind = "roads"
            self.status.set("No aerial photo of {} yet: CARLA tab > Take aerial photo.".format(self.plan.town))
        img, georef = self.backgrounds[kind]
        self.view.set_background(img, georef, fit=fit)
        self.aerial_radio.state(["!disabled"] if "aerial" in self.backgrounds else ["disabled"])

    def fit_map(self):
        if self.road_map:
            self.view.fit(self.road_map.bounds(margin=5.0))

    def fit_vehicles(self):
        if not self.plan.vehicles:
            self.fit_map()
            return
        xs = [v.pose["x"] for v in self.plan.vehicles]
        ys = [v.pose["y"] for v in self.plan.vehicles]
        self.view.fit((min(xs) - 25, max(xs) + 25, min(ys) - 25, max(ys) + 25))

    # ================================================================== drawing
    def redraw(self):
        self.view.request_redraw()

    def bbox(self):
        if not self.use_bbox.get():
            return None
        try:
            box = [float(v.get()) for v in self.bbox_vars]
        except ValueError:
            return None
        return box if box[0] < box[1] and box[2] < box[3] else None

    def _paper_bbox(self):
        for var, value in zip(self.bbox_vars, PAPER_BBOX):
            var.set(str(value))
        self.use_bbox.set(True)
        self.redraw()

    def _draw_overlay(self, view):
        box = self.bbox()
        if box:
            x0, y0 = view.world_to_screen(box[0], box[2])
            x1, y1 = view.world_to_screen(box[1], box[3])
            view.create_rectangle(x0, y0, x1, y1, outline="#e2b93b", dash=(8, 5), width=2)
            view.create_text(x0 + 6, y0 + 6, text="spawn area", anchor="nw", fill="#e2b93b", font=FONT)
        outside = set(self.plan.outside(box))
        live = self._live if self.sim is not None and self._live is not None else None
        bev = self.bev if self.show_bev.get() and self.bev is not None and self.bev.town == self.plan.town else None
        if bev is not None:
            self._draw_bev(view, bev)
        ghost = bev is not None or live is not None
        for i, v in enumerate(self.plan.vehicles):
            if v.route and not ghost:
                self._draw_route(view, i, v)
        for i, v in enumerate(self.plan.vehicles):
            self._draw_vehicle(view, v.pose, COLORS[v.role], self.plan.name(i), i == self.selected, i in outside,
                               ghost=ghost)
        if live is not None:
            self._draw_live(view, live)
        if self._ghost is not None and live is None:
            pose, ok = self._ghost
            pts = [view.world_to_screen(x, y) for x, y in footprint(pose)]
            view.create_polygon(*[c for p in pts for c in p], outline=OK_GHOST if ok else BAD_GHOST,
                                fill="", dash=(4, 3), width=2)
        if self._route_ghost is not None and live is None:  # where a route click lands, and the lane's direction
            x, y, yaw = self._route_ghost
            sx, sy = view.world_to_screen(x, y)
            ex, ey = view.world_to_screen(x + 4 * math.cos(math.radians(yaw)), y + 4 * math.sin(math.radians(yaw)))
            view.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, outline=OK_GHOST, width=2)
            view.create_line(sx, sy, ex, ey, fill=OK_GHOST, width=2, arrow="last")
        # scale bar
        metres = next((m for m in (1, 2, 5, 10, 20, 50, 100, 200, 500) if m * view.scale >= 60), 1000)
        h = view.winfo_height()
        view.create_line(16, h - 18, 16 + metres * view.scale, h - 18, fill="white", width=3)
        view.create_text(16, h - 24, text="{} m".format(metres), anchor="sw", fill="white", font=FONT)

    def _draw_live(self, view, live):
        """Every vehicle of the running simulation at its current pose, with its real box and speed."""
        for v in live.vehicles:
            corners = box_corners_2d(v.pose, v.extent, v.center_offset)
            pts = [view.world_to_screen(x, y) for x, y in corners]
            color = COLORS[Role.parse(v.role)]
            view.create_polygon(*[c for p in pts for c in p], fill=color, outline="white", width=1)
            view.create_line(*pts[0], *pts[1], fill="white", width=3)   # front bumper
            # CAVs are always named; traffic only when zoomed in enough for the labels not to overlap
            if view.scale >= (1.5 if v.role == Role.CAV.value else LIVE_TRAFFIC_LABELS):
                cx = sum(p[0] for p in pts) / 4.0
                cy = min(p[1] for p in pts) - 9
                text = "{}  {:.0f} km/h".format(v.name, v.speed_kmh)
                view.create_text(cx + 1, cy + 1, text=text, fill="black", font=("Segoe UI", 8, "bold"))
                view.create_text(cx, cy, text=text, fill="white", font=("Segoe UI", 8, "bold"))

    def _draw_bev(self, view, bev):
        """Recorded vehicles (outlines) and every CAV's occupied BEV cells, world frame."""
        w2, h2, s = view.winfo_width() / 2.0, view.winfo_height() / 2.0, view.scale

        def screen(xy):
            out = np.empty_like(xy)
            out[..., 0] = w2 + (xy[..., 0] - view.cx) * s
            out[..., 1] = h2 + (xy[..., 1] - view.cy) * s
            return out

        if self.show_truth.get():
            for _, _, corners in bev.vehicles:
                view.create_polygon(*screen(np.asarray(corners)).ravel().tolist(), fill="", outline=TRUTH_OUTLINE)
        for i, cav in enumerate(bev.cavs):
            color = BEV_COLORS[i % len(BEV_COLORS)]
            if len(cav.cells):
                for poly in screen(cav.cells).reshape(len(cav.cells), 8).tolist():
                    view.create_polygon(*poly, fill=color, outline="")
            sx, sy = screen(np.array([cav.sensor_pose["x"], cav.sensor_pose["y"]]))
            view.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill=color, outline="white", width=2)
            view.create_text(sx + 8, sy - 6, text=cav.name, anchor="sw", fill=color, font=("Segoe UI", 9, "bold"))

    def _draw_vehicle(self, view, pose, color, label, selected, outside, ghost=False):
        pts = [view.world_to_screen(x, y) for x, y in footprint(pose)]
        flat = [c for p in pts for c in p]
        if ghost:  # the plan while the BEV is shown
            view.create_polygon(*flat, fill="", outline=color, dash=(3, 2), width=2 if selected else 1)
            return
        view.create_polygon(*flat, fill=color, outline=SELECTED if selected else "white",
                            width=3 if selected else 1)
        # windscreen line near the front, so the heading is visible at any zoom
        front = [(pts[0][0] * 0.7 + pts[3][0] * 0.3, pts[0][1] * 0.7 + pts[3][1] * 0.3),
                 (pts[1][0] * 0.7 + pts[2][0] * 0.3, pts[1][1] * 0.7 + pts[2][1] * 0.3)]
        view.create_line(*front[0], *front[1], fill="white", width=2)
        if view.scale >= 1.5:
            cx = sum(p[0] for p in pts) / 4
            cy = sum(p[1] for p in pts) / 4
            view.create_text(cx + 1, cy - 14 + 1, text=label, fill="black", font=("Segoe UI", 9, "bold"))
            view.create_text(cx, cy - 14, text=label, fill="#ff6b6b" if outside else "white",
                             font=("Segoe UI", 9, "bold"))

    def _draw_route(self, view, index, v):
        """The traced path (or, if it cannot be driven, dashed red lines) and the clicked points."""
        selected = index == self.selected
        route, error = self._route_of(v)
        if route is not None:
            flat = [c for x, y, _ in route.points for c in view.world_to_screen(x, y)]
            if len(flat) >= 4:
                view.create_line(*flat, fill=COLORS[v.role], width=4 if selected else 2, capstyle="round",
                                 arrow="last" if v.route_end == RouteEnd.AUTOPILOT else None,
                                 arrowshape=(12, 14, 5))
        else:
            pts = [(v.pose["x"], v.pose["y"])] + [tuple(p[:2]) for p in v.route]
            flat = [c for x, y in pts for c in view.world_to_screen(x, y)]
            view.create_line(*flat, fill=BAD_GHOST, width=2, dash=(6, 4))
        for n, p in enumerate(v.route, 1):
            sx, sy = view.world_to_screen(p[0], p[1])
            last = n == len(v.route)
            r = 6 if selected else 4
            fill = STOP_MARK if last and v.route_end == RouteEnd.STOP else "white"
            view.create_oval(sx - r, sy - r, sx + r, sy + r, fill=fill, outline=COLORS[v.role], width=2)
            if selected:
                view.create_text(sx + 9, sy - 9, text=str(n), fill="white", font=("Segoe UI", 9, "bold"))

    def refresh(self):
        """Update the table, counts and map after any change of the plan."""
        self.table.delete(*self.table.get_children())
        for i, v in enumerate(self.plan.vehicles):
            p = v.pose
            self.table.insert("", "end", iid=str(i), values=(
                self.plan.name(i), "{:.1f}".format(p["x"]), "{:.1f}".format(p["y"]),
                "{:.0f}".format(wrap_degrees(p["yaw"])),
                "{}/{}".format(v.road_id, v.lane_id), self._drive_text(v)))
        if self.selected is not None and self.selected < len(self.plan.vehicles):
            self.table.selection_set(str(self.selected))
            self.table.see(str(self.selected))
        else:
            self.selected = None
        counts = self.plan.counts()
        outside = len(self.plan.outside(self.bbox()))
        text = "{} CAVs  ·  {} traffic".format(counts[Role.CAV], counts[Role.TRAFFIC])
        if outside:
            text += "  ·  {} outside the spawn area".format(outside)
        self.count_label.configure(text=text, foreground="#c62828" if outside else "")
        name = os.path.basename(self.plan_path) if self.plan_path else "unsaved plan"
        self.root.title("CARLA Scenario Editor - {} - {}{}".format(self.plan.town, name, " *" if self.dirty else ""))
        self._sync_driving_box()
        self.redraw()

    # ================================================================== editing
    def _snap(self, sx, sy, ignore=None):
        """(LanePose or None, problem text or '') for the screen point."""
        x, y = self.view.screen_to_world(sx, sy)
        lane = self.road_map.snap(x, y, float(self.snap_distance.get())) if self.road_map else None
        if lane is None:
            return None, "No driving lane here"
        if lane.in_junction and not self.allow_junction.get():
            return lane, "Inside a junction (allowed in the Rules tab)"
        box = self.bbox()
        if box and not (box[0] <= lane.pose["x"] <= box[1] and box[2] <= lane.pose["y"] <= box[3]):
            return lane, "Outside the spawn area"
        hits = self.plan.collisions(lane.pose, ignore)
        if hits:
            return lane, "Too close to {}".format(self.plan.name(hits[0]))
        return lane, ""

    def _vehicle_from(self, lane, role):
        return PlannedVehicle(role, lane.pose, lane.road_id, lane.lane_id, lane.in_junction)

    def _on_press(self, event):
        if self._locked():
            return
        self.view.focus_set()
        if self.route_mode.get() and self.selected is not None:
            self._add_route_point(event.x, event.y)
            return
        x, y = self.view.screen_to_world(event.x, event.y)
        index = self.plan.hit(x, y)
        if index is not None:
            self.select(index)
            self._drag = {"index": index, "started": False}
            return
        lane, problem = self._snap(event.x, event.y)
        if problem:
            self.status.set(problem)
            self.root.bell()
            return
        index = self.plan.add(self._vehicle_from(lane, Role.parse(self.add_role.get())))
        self.dirty = True
        self.selected = index
        self.status.set("Added {} at ({:.1f}, {:.1f}), heading {:.0f} deg".format(
            self.plan.name(index), lane.pose["x"], lane.pose["y"], lane.pose["yaw"]))
        self._set_ghost(None)
        self.refresh()

    def _on_drag(self, event):
        if not self._drag:
            return
        index = self._drag["index"]
        lane, problem = self._snap(event.x, event.y, ignore=index)
        if lane is None or problem:
            self.status.set(problem)
            return
        if not self._drag["started"]:
            self.plan.begin_change()
            self._drag["started"] = True
        moved = replace(self.plan.vehicles[index], pose=lane.pose, road_id=lane.road_id, lane_id=lane.lane_id,
                        in_junction=lane.in_junction)  # keeps its speed and route
        self.plan.replace(index, moved, record=False)
        self.dirty = True
        self.status.set("{} -> ({:.1f}, {:.1f})".format(self.plan.name(index), lane.pose["x"], lane.pose["y"]))
        self.redraw()

    def _on_release(self, event):
        if self._drag and self._drag["started"]:
            self.refresh()
        self._drag = None

    def _on_motion(self, event):
        x, y = self.view.screen_to_world(event.x, event.y)
        text = "x {:.1f}   y {:.1f}".format(x, y)
        if self.road_map:
            text += "   z {:.1f}".format(self.road_map.road_height(x, y))
        self.cursor.set(text + "   |   {:.2f} px/m".format(self.view.scale))
        if self.sim is not None:
            return
        if self.route_mode.get() and self.selected is not None:
            self.view.configure(cursor="crosshair")
            lane = self.road_map.snap(x, y, float(self.snap_distance.get())) if self.road_map else None
            ghost = (lane.pose["x"], lane.pose["y"], lane.pose["yaw"]) if lane else None
            if ghost != self._route_ghost:
                self._route_ghost = ghost
                self.redraw()
            return
        if self.plan.hit(x, y) is not None:
            self.view.configure(cursor="hand2")
            self._set_ghost(None)
            return
        self.view.configure(cursor="crosshair")
        lane, problem = self._snap(event.x, event.y)
        self._set_ghost((lane.pose, not problem) if lane else None)

    def _clear_ghosts(self):
        self._route_ghost = None
        self._set_ghost(None)
        self.redraw()

    def _set_ghost(self, ghost):
        if ghost != self._ghost:
            self._ghost = ghost
            self.redraw()

    def _on_table_select(self, event):
        sel = self.table.selection()
        index = int(sel[0]) if sel else None
        if index != self.selected:
            self.selected = index
            if index is not None:
                p = self.plan.vehicles[index].pose
                self.view.centre_on(p["x"], p["y"])
            self._sync_driving_box()
            self.redraw()

    def select(self, index):
        self.selected = index
        if index is None:
            self.table.selection_remove(self.table.selection())
        else:
            self.table.selection_set(str(index))
            self.table.see(str(index))
        self._sync_driving_box()
        self.redraw()

    def toggle_role(self):
        if self._locked():
            return
        if self.selected is None:
            return
        v = self.plan.vehicles[self.selected]
        self.plan.set_role(self.selected, Role.TRAFFIC if v.role == Role.CAV else Role.CAV)
        self.dirty = True
        self.refresh()

    def delete_selected(self):
        if self._locked():
            return
        if self.selected is None:
            return
        name = self.plan.name(self.selected)
        self.plan.remove(self.selected)
        self.selected = None
        self.dirty = True
        self.status.set("Deleted {} (Ctrl+Z to undo); the others were renumbered".format(name))
        self.refresh()

    def clear_all(self):
        if self._locked():
            return
        if self.plan.vehicles and messagebox.askyesno("Clear all", "Remove every vehicle? (Ctrl+Z undoes it)"):
            self.plan.clear()
            self.selected = None
            self.dirty = True
            self.refresh()

    def undo(self):
        if self._locked():
            return
        if self.plan.undo():
            self.dirty = True
            self.selected = None
            self.refresh()

    def redo(self):
        if self._locked():
            return
        if self.plan.redo():
            self.dirty = True
            self.selected = None
            self.refresh()

    # ================================================================== speed and route
    def _route_of(self, v):
        """(Route, '') or (None, why it cannot be driven) for a vehicle with a route; cached."""
        key = (round(v.pose["x"], 2), round(v.pose["y"], 2), round(v.pose["yaw"], 1),
               tuple(tuple(p) for p in v.route))
        if key not in self._routes:
            try:
                self._routes[key] = (routes.trace_route(self.road_map.map, v.pose, v.route), "")
            except RouteError as exc:
                self._routes[key] = (None, str(exc))
        return self._routes[key]

    def _route_problems(self):
        return ["{}: {}".format(self.plan.name(i), self._route_of(v)[1])
                for i, v in enumerate(self.plan.vehicles) if v.route and self._route_of(v)[0] is None]

    def _drive_text(self, v):
        if v.route:
            route, _ = self._route_of(v)
            length = "{:.0f} m".format(route.length) if route else "error"
            return "{:.0f}, {}".format(v.speed_kmh or DEFAULT_ROUTE_SPEED, length)
        return "{:.0f}".format(v.speed_kmh) if v.speed_kmh else ""

    def _sync_driving_box(self):
        """Show the selected vehicle's speed and route in the box (or disable it)."""
        index = self.selected
        if index is None or index >= len(self.plan.vehicles):
            for widget in self._driving_widgets:
                widget.state(["disabled"])
            self.set_route_mode(False)
            self.driving_info.configure(text="Select a vehicle to give it a speed or a route.", foreground="")
            return
        v = self.plan.vehicles[index]
        for widget in self._driving_widgets:
            widget.state(["!disabled"])
        self.speed_on.set(bool(v.speed_kmh or v.route))
        self.speed_var.set("{:g}".format(v.speed_kmh or DEFAULT_ROUTE_SPEED))
        self.route_end.set(v.route_end.value)
        if v.route:
            self.chk_speed.state(["disabled"])  # a route always has a speed
        else:
            for widget in [self.btn_route_back, self.btn_route_clear] + self.end_radios:
                widget.state(["disabled"])
        if not self.speed_on.get():
            self.spin_speed.state(["disabled"])
        problem = bool(v.route) and self._route_of(v)[0] is None
        self.driving_info.configure(text=self._driving_text(v), foreground="#c62828" if problem else "")

    def _driving_text(self, v):
        name = self.plan.name(self.selected)
        if v.route:
            route, error = self._route_of(v)
            if route is None:
                return "Route problem: {}".format(error)
            speed = v.speed_kmh or DEFAULT_ROUTE_SPEED
            profile = routes.speed_profile(route, speed / 3.6, v.route_end == RouteEnd.STOP)
            ds = route.distances[1:] - route.distances[:-1]
            seconds = float(sum(d / max(0.5, (a + b) / 2) for d, a, b in zip(ds, profile[:-1], profile[1:])))
            end = "then stops" if v.route_end == RouteEnd.STOP else "then the Traffic Manager drives on"
            return ("{}: {} point(s), {:.0f} m, about {:.0f} s without stops. Drives at {:g} km/h (slower in "
                    "turns, waits at red lights and behind vehicles), {}. It starts when the recording "
                    "starts.").format(name, len(v.route), route.length, seconds, speed, end)
        if v.speed_kmh:
            return ("{}: no route. The Traffic Manager drives it at about {:g} km/h, choosing its own way."
                    ).format(name, v.speed_kmh)
        return ("{}: no route, no own speed. It drives with the Traffic Manager if autopilot is on "
                "(Export tab), else it stays parked. Press T to draw a route.").format(name)

    def _apply_speed(self):
        if self._locked():
            return
        index = self.selected
        if index is None:
            return
        v = self.plan.vehicles[index]
        speed = None
        if self.speed_on.get() or v.route:
            try:
                speed = round(min(MAX_SPEED, max(1.0, float(self.speed_var.get()))), 1)
            except ValueError:
                self.root.bell()
                self.status.set("Speed must be a number (km/h)")
                return
        if speed != v.speed_kmh:
            self.plan.set_driving(index, speed_kmh=speed)
            self.dirty = True
        self.refresh()

    def _apply_route_end(self):
        if self._locked():
            return
        index = self.selected
        if index is None:
            return
        end = RouteEnd.parse(self.route_end.get())
        if end != self.plan.vehicles[index].route_end:
            self.plan.set_driving(index, route_end=end)
            self.dirty = True
            self.refresh()

    def set_route_mode(self, on):
        on = bool(on) and self.selected is not None and self.road_map is not None
        self.route_mode.set(on)
        if not on and self._route_ghost is not None:
            self._route_ghost = None
            self.redraw()
        if on:
            self.status.set("Route of {}: click points on the roads in driving order. Enter or Esc ends.".format(
                self.plan.name(self.selected)))

    def toggle_route_mode(self):
        if self._locked():
            return
        self.set_route_mode(not self.route_mode.get())

    def _on_route_button(self):
        if self._locked():
            self.route_mode.set(False)
            return
        self.set_route_mode(self.route_mode.get())

    def escape(self):
        if self.route_mode.get():
            self.set_route_mode(False)
        else:
            self.select(None)

    def _add_route_point(self, sx, sy):
        index = self.selected
        v = self.plan.vehicles[index]
        x, y = self.view.screen_to_world(sx, sy)
        lane = self.road_map.snap(x, y, float(self.snap_distance.get()))
        if lane is None:
            self.status.set("No driving lane here")
            self.root.bell()
            return
        route = [list(p) for p in v.route] + [[lane.pose["x"], lane.pose["y"]]]
        traced, error = self._route_of(replace(v, route=route))
        if traced is None:
            self.status.set(error)
            self.root.bell()
            return
        self.plan.set_driving(index, route=route, speed_kmh=v.speed_kmh or DEFAULT_ROUTE_SPEED)
        self.dirty = True
        self.status.set("{}: point {} added, route {:.0f} m".format(self.plan.name(index), len(route), traced.length))
        self.refresh()

    def remove_route_point(self):
        if self._locked():
            return
        index = self.selected
        if index is None or not self.plan.vehicles[index].route:
            return
        self.plan.set_driving(index, route=self.plan.vehicles[index].route[:-1])
        self.dirty = True
        self.refresh()

    def clear_route(self):
        if self._locked():
            return
        index = self.selected
        if index is None or not self.plan.vehicles[index].route:
            return
        self.plan.set_driving(index, route=[])
        self.dirty = True
        self.set_route_mode(False)
        self.status.set("Route of {} removed (Ctrl+Z to undo)".format(self.plan.name(index)))
        self.refresh()

    # ================================================================== files
    def _confirm_discard(self):
        if not self.dirty:
            return True
        answer = messagebox.askyesnocancel("Unsaved plan", "Save the current plan first?")
        if answer is None:
            return False
        return self.save_plan() if answer else True

    def new_plan(self):
        if self._locked():
            return
        if self._confirm_discard():
            self.plan = ScenarioPlan(self.plan.town)
            self.plan_path, self.selected, self.dirty = None, None, False
            self.refresh()

    def ask_open_plan(self):
        if self._locked():
            return
        if not self._confirm_discard():
            return
        path = filedialog.askopenfilename(title="Open spawn plan", filetypes=[("Spawn plan", "*.json")],
                                          initialdir=os.path.join(PROJECT_ROOT, export.PLANS_DIR))
        if path:
            self.open_plan(path)

    def open_plan(self, path):
        try:
            plan = ScenarioPlan.load(path)
        except (OSError, ValueError, KeyError) as exc:
            messagebox.showerror("Cannot open plan", str(exc))
            return
        if catalog.find_map(self.maps, plan.town) is None:
            messagebox.showerror("Unknown map", "This plan is for {}, which is not available.".format(plan.town))
            return
        self.plan, self.plan_path, self.selected, self.dirty = plan, path, None, False
        self.open_map(plan.town)
        self.fit_vehicles()
        self.status.set("Opened {} ({} vehicles)".format(os.path.basename(path), len(plan.vehicles)))

    def save_plan(self, ask=False):
        if not self.plan.vehicles:
            messagebox.showinfo("Empty plan", "Place at least one vehicle first.")
            return False
        path = self.plan_path
        if ask or not path:
            path = filedialog.asksaveasfilename(
                title="Save spawn plan", defaultextension=".json", filetypes=[("Spawn plan", "*.json")],
                initialdir=os.path.join(PROJECT_ROOT, export.PLANS_DIR),
                initialfile="{}_plan.json".format(self.plan.town.lower()))
            if not path:
                return False
        self.plan.save(path)
        self.plan_path, self.dirty = path, False
        self.status.set("Saved {}".format(path))
        self.refresh()
        return True

    def import_xosc(self):
        """Open an OpenSCENARIO file (e.g. from CARLA's QGIS Traffic Generation Editor) as a plan."""
        if self._locked():
            return
        from ..backend.interop.openscenario import read_xosc
        from .openscenario_bridge import plan_from_xosc

        if not self._confirm_discard():
            return
        path = filedialog.askopenfilename(title="Import OpenSCENARIO", filetypes=[("OpenSCENARIO", "*.xosc")])
        if not path:
            return
        try:
            town = read_xosc(path).town
            info = catalog.find_map(self.maps, town)
            if info is None:
                raise ValueError("The file is for '{}', which is not in the map list.".format(town))
            plan, warnings = plan_from_xosc(path, RoadMap.from_file(info.name, info.xodr_path))
        except (OSError, ValueError, KeyError, SyntaxError) as exc:  # SyntaxError: XML ParseError
            messagebox.showerror("Cannot import", str(exc))
            return
        self.plan, self.plan_path, self.selected, self.dirty = plan, None, None, True
        self.open_map(plan.town)
        self.fit_vehicles()
        counts = plan.counts()
        text = "{} CAVs (ego vehicles) and {} traffic vehicles imported from {}.".format(
            counts[Role.CAV], counts[Role.TRAFFIC], os.path.basename(path))
        if warnings:
            text += "\n\nNot imported / to check:\n- " + "\n- ".join(warnings)
        messagebox.showinfo("OpenSCENARIO import", text)

    def export_xosc(self):
        from .openscenario_bridge import xosc_from_plan

        if not self.plan.vehicles:
            messagebox.showinfo("Empty plan", "Place at least one vehicle first.")
            return
        path = filedialog.asksaveasfilename(title="Export OpenSCENARIO", defaultextension=".xosc",
                                            filetypes=[("OpenSCENARIO", "*.xosc")],
                                            initialfile="{}_scenario.xosc".format(self.plan.town.lower()))
        if path:
            xosc_from_plan(self.plan, path, self.cfg.scenario.cav_blueprint, self.cfg.scenario.traffic_blueprint)
            note = " Routes are not written (speeds are, as initial speeds)." if any(
                v.route for v in self.plan.vehicles) else ""
            self.status.set("Exported {} (CAVs as Ego_1..., traffic as Vehicle_1...).{}".format(path, note))

    def quit(self):
        if self.sim is not None:
            if not messagebox.askyesno("Quit", "The simulation is running. Stop it and quit?"):
                return
            self.sim.stop()
            self.sim.join(60)
        if self._confirm_discard():
            self.root.destroy()

    # ================================================================== export / record
    def _export_files(self):
        """Write the plan and its experiment from the window's settings: (options, plan, yaml) or None."""
        try:
            v = self.lidar_vars
            lidar = {"channels": _number(v["channels"], "Channels", int),
                     "range": _number(v["range"], "Range"),
                     "points_per_second": _number(v["points_per_second"], "Points per second", int),
                     "mount_z": _number(v["mount_z"], "Mount height"),
                     "noise_stddev": _number(v["noise_stddev"], "Noise")}
            frames = _number(self.exp_frames, "Frames", int)
            if frames < 0:
                raise ValueError("Frames: 0 or more")
            options = export.ExportOptions(
                name=self.exp_name.get(), extra_traffic=_number(self.extra_traffic, "Extra random traffic", int),
                autopilot=self.exp_autopilot.get(), num_frames=frames or 10,
                save_every=_number(self.exp_every, "Save every N ticks", int), spawn_bbox=self.bbox(),
                lidar=lidar, seed=_number(self.seed_var, "Seed", int),
                ignore_lights=[_number(var, "Red lights ignored") for var in self.lights_vars],
                process=self.process_after.get())
            problems = self._route_problems()
            if problems:
                raise ValueError("These routes cannot be driven:\n" + "\n".join(problems))
            outside = self.plan.outside(options.spawn_bbox)
            if outside:
                raise ValueError("{} vehicles are outside the spawn area. Move them or turn the area off "
                                 "(Settings).".format(len(outside)))
            plan_rel, yaml_rel = export.export(self.plan, options)
            from .. import api
            api.load_config(os.path.join(PROJECT_ROOT, yaml_rel))  # same checks as the recorder
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("Cannot use these settings", str(exc))
            return None
        except Exception as exc:  # ConfigError and friends: show, don't crash the window
            messagebox.showerror("Invalid experiment", str(exc))
            return None
        return options, plan_rel, yaml_rel

    def export_experiment(self):
        out = self._export_files()
        if out is None:
            return
        options, plan_rel, yaml_rel = out
        name = export.safe_name(options.name)
        self.status.set("Exported {}".format(yaml_rel))
        messagebox.showinfo("Experiment exported", "Saved:\n  {}\n  {}\n\nRecord it with:\n  python main.py record "
                            "--config {} --name {} --process".format(plan_rel, yaml_rel,
                                                                    yaml_rel.replace(os.sep, "/"), name))

    def _carla_ready(self):
        if self.server_town is None:
            messagebox.showwarning("CARLA not running", "Start CARLA, then CARLA > Check connection.")
            return False
        if self.server_town != self.plan.town:
            messagebox.showwarning("Wrong map in CARLA", (
                "CARLA has {} loaded, this plan is for {}.\n\nCARLA > 'Start CARLA on this map next time', "
                "then close and reopen CARLA.").format(self.server_town, self.plan.town))
            return False
        return True

    def record_in_console(self):
        """The repeatable way: warm-up, then a fixed number of frames, in a console of its own."""
        if self.sim is not None or not self._carla_ready():
            return
        out = self._export_files()
        if out is None:
            return
        options, plan_rel, yaml_rel = out
        name = export.safe_name(options.name)
        export.launch_recording(yaml_rel, name, process=options.process)
        now = time.time()
        self._watch = {"name": name, "since": now, "frames": options.num_frames, "process": options.process,
                       "state": None, "changed": now}
        self.root.after(WATCH_EVERY_MS, self._watch_recording)
        self.status.set("Recording {} frames in a new window: {}. The BEV tab opens when it is done.".format(
            options.num_frames, name))

    def _watch_recording(self):
        """Follow a recording started from the editor; when it is done, show its BEV."""
        watch = self._watch
        if not watch:
            return
        run = next((r for r in results.list_runs(results.runs_dir(self.cfg))
                    if r.name.endswith("_" + watch["name"]) and _started(r.name) >= watch["since"] - 5), None)
        state = None if run is None else (len(run.frames), len(run.processed))
        now = time.time()
        if state != watch["state"]:
            watch["state"], watch["changed"] = state, now
        if run is not None and len(run.frames) >= watch["frames"] and (
                not watch["process"] or run.frames[-1] in run.processed):
            self._watch = None
            self.refresh_runs(select=run.path, frame=run.frames[-1])
            self.side.select(self.bev_tab)
            if run.processed:
                self.show_bev.set(True)
                self._on_show_bev()
            self.status.set("{} recorded{}.".format(run.name, ", processed: its BEV is on the map" if run.processed
                                                    else "; process it in the BEV tab"))
            return
        if now - watch["changed"] > WATCH_IDLE_S:
            self._watch = None
            self.status.set("Nothing new from the recording '{}' for {} s: see its console window.".format(
                watch["name"], WATCH_IDLE_S))
            return
        if run is not None:
            busy = "processing" if len(run.frames) >= watch["frames"] else "recording"
            self.status.set("{}: {} ({}/{} frames, {} processed)".format(run.name, busy, len(run.frames),
                                                                         watch["frames"], len(run.processed)))
        self.root.after(WATCH_EVERY_MS, self._watch_recording)

    # ================================================================== live simulation
    def _locked(self):
        """True (and says why) while a simulation runs: the scene and the settings cannot change then."""
        if self.sim is not None:
            self.status.set("Stop the simulation first (Stop) to change the scene or the settings.")
            return True
        return False

    def start_simulation(self):
        if self.sim is not None or not self._carla_ready():
            return
        if self.route_mode.get():
            self.set_route_mode(False)
        out = self._export_files()
        if out is None:
            return
        options, _, yaml_rel = out
        from .. import api
        cfg = api.load_config(os.path.join(PROJECT_ROOT, yaml_rel))
        self.sim = simulation.LiveSimulation(cfg, results.runs_dir(self.cfg), max_frames=int(self.exp_frames.get()))
        self._live = None
        self.sim.start()
        self._set_editing(False)
        self.status.set("Starting: spawning {} vehicles...".format(cfg.scenario.num_cavs + cfg.scenario.num_traffic))

    def toggle_pause(self):
        if self.sim is None or self._live is None:
            return
        if self._live.state == simulation.PAUSED:
            self.sim.resume()
        elif self._live.state == simulation.RUNNING:
            self.sim.pause()

    def stop_simulation(self):
        if self.sim is not None:
            self.sim.stop()
            self.status.set("Stopping: removing the vehicles and LiDARs...")

    def toggle_record(self):
        if self.sim is None or self._live is None:
            return
        if self._live.recording:
            self.sim.stop_recording()
            return
        try:
            name = export.safe_name(self.exp_name.get())
        except ValueError as exc:
            messagebox.showerror("Recording", str(exc))
            return
        self.sim.start_recording(name)

    def _poll_live(self):
        sim = self.sim
        if sim is None:
            return
        view = sim.view()
        old = self._live
        self._live = view
        if view.state == simulation.RUNNING and (old is None or old.state == simulation.STARTING):
            self.status.set("Running. Record saves LiDAR frames; Pause (or Space) freezes; Stop ends it.")
        if view.recording and (old is None or not old.recording):
            self.status.set("Recording into {} ...".format(view.recording))
        for run_dir, frames in sim.pop_finished():
            self._recording_finished(run_dir, frames)
        if view.state in (simulation.STOPPED, simulation.FAILED):
            self.sim = None
            sim.join(1.0)
            self._set_editing(True)
            self.check_carla()
            if view.state == simulation.FAILED:
                self.status.set("Simulation failed: " + view.error)
                messagebox.showerror("Simulation", view.error)
            else:
                self.status.set("Simulation stopped; CARLA is back to normal.")
            self.redraw()
        if old is None or (view.frame, view.state, view.recording, view.frames_saved) != (
                old.frame, old.state, old.recording, old.frames_saved):
            self._update_sim_bar()
            self.redraw()

    def _recording_finished(self, run_dir, frames):
        name = os.path.basename(run_dir)
        if frames == 0:
            self.status.set("Recording {} ended without any frame.".format(name))
        elif self.process_after.get():
            self.status.set("Recorded {} frames ({}); processing up to the BEV...".format(frames, name))
            self.offline.submit("process", _job_process, run_dir, None)
            return
        else:
            self.status.set("Recorded {} frames ({}). Process them in the BEV tab.".format(frames, name))
        self.refresh_runs(select=run_dir)

    def _set_editing(self, on):
        for widget in self._setting_widgets + self._driving_widgets:
            widget.state(["!disabled"] if on else ["disabled"])
        self.map_box.state(["!disabled"] if on else ["disabled"])
        if on:
            self._sync_driving_box()
        self._update_sim_bar()
        self._update_carla_widgets()

    def _update_sim_bar(self):
        view = self._live if self.sim is not None else None
        state = None if self.sim is None else (view.state if view is not None else simulation.STARTING)
        running = state in (simulation.RUNNING, simulation.PAUSED)
        recording = bool(view is not None and view.recording)
        for button, on in ((self.btn_start, self.sim is None), (self.btn_pause, running),
                           (self.btn_stop, self.sim is not None and state != simulation.STOPPING),
                           (self.btn_record, running)):
            color = button.color if on else BUTTON_OFF
            button.configure(state="normal" if on else "disabled", bg=color, activebackground=color,
                             cursor="hand2" if on else "arrow")
        self.btn_pause.configure(text="\u25b6  Resume" if state == simulation.PAUSED else "\u275a\u275a  Pause")
        self.btn_record.configure(text="\u25a0  Stop recording" if recording else "\u25cf  Record")
        if state is None:
            text = "Stopped. Place the vehicles, check the Settings, then Start (CARLA must be running)."
        elif state == simulation.STARTING:
            text = "Starting: spawning the vehicles..."
        elif state == simulation.STOPPING:
            text = "Stopping..."
        else:
            text = "{}   {:.1f} s simulated   {:.2f} x real time".format(
                "PAUSED" if state == simulation.PAUSED else "Running", view.sim_time, view.realtime)
            if recording:
                text += "      \u25cf REC  {} frames  ->  {}".format(view.frames_saved,
                                                                     os.path.basename(view.recording))
        self.sim_text.set(text)

    # ================================================================== BEV of the recordings
    def _selected_run(self):
        i = self.run_box.current()
        return self.runs[i] if 0 <= i < len(self.runs) else None

    def _selected_frame(self):
        run, i = self._selected_run(), self.frame_box.current()
        return run.frames[i] if run is not None and 0 <= i < len(run.frames) else None

    def refresh_runs(self, select=None, frame=None):
        """Read data/runs again; keep the selected run and frame unless `select` / `frame` say otherwise."""
        current = self._selected_run()
        select = select or (current.path if current else None)
        frame = frame if frame is not None else self._selected_frame()
        self.runs = results.list_runs(results.runs_dir(self.cfg))
        self.run_box["values"] = [r.label for r in self.runs]
        paths = [r.path for r in self.runs]
        if not self.runs:
            self.run_box.set("")
            self.frame_box["values"] = []
            self.frame_box.set("")
            self._set_bev(None)
            self.bev_info.configure(text="No recording yet in {}.".format(results.runs_dir(self.cfg)))
            return
        self.run_box.current(paths.index(select) if select in paths else 0)
        self._on_run_chosen(frame if select in paths else None)

    def _on_run_chosen(self, frame=None):
        run = self._selected_run()
        if run is None:
            return
        self.frame_box["values"] = ["{}{}".format(f, " (BEV)" if f in run.processed else "") for f in run.frames]
        if run.frames:
            self.frame_box.current(run.frames.index(frame) if frame in run.frames else len(run.frames) - 1)
        self._on_frame_chosen()

    def _on_frame_chosen(self):
        run, frame = self._selected_run(), self._selected_frame()
        overlay, problem = None, ""
        if run is not None and frame is not None and frame in run.processed:
            try:
                overlay = results.bev_overlay(run.path, frame)
            except (OSError, ValueError, KeyError) as exc:
                problem = "Cannot read the BEV of frame {}: {}".format(frame, exc)
        self._set_bev(overlay)
        if run is None or frame is None:
            text = "No frame in this run."
        elif problem:
            text = problem
        elif overlay is None:
            text = "Frame {} is not processed yet: press 'Process this frame'.".format(frame)
        else:
            text = "{}, frame {}: {} CAVs.".format(overlay.town or "?", frame, len(overlay.cavs))
            if overlay.town != self.plan.town:
                text += " Recorded on {}: open that map to see the BEV on it.".format(overlay.town)
        self.bev_info.configure(text=text)

    def _set_bev(self, overlay):
        self.bev = overlay
        for child in self.bev_legend.winfo_children():
            child.destroy()
        names = [c.name for c in overlay.cavs] if overlay else []
        chosen = self.cav_box.get()
        self.cav_box["values"] = names
        if names:
            self.cav_box.current(names.index(chosen) if chosen in names else 0)
        else:
            self.cav_box.set("")
        for button in [self.btn_figure] + self.view_buttons:
            button.state(["!disabled"] if names else ["disabled"])
        chosen = self.cav_view_box.get()
        self.cav_view_box["values"] = names
        if names:
            self.cav_view_box.current(names.index(chosen) if chosen in names else 0)
        else:
            self.cav_view_box.set("")
        for i, cav in enumerate(overlay.cavs if overlay else []):
            row = ttk.Frame(self.bev_legend)
            row.pack(anchor="w")
            ttk.Label(row, text="\u25a0", foreground=BEV_COLORS[i % len(BEV_COLORS)],
                      font=("Segoe UI", 12)).pack(side="left")
            ttk.Label(row, text=" {}: {} cells, {} points".format(cav.name, len(cav.cells),
                                                                  cav.non_ground_points)).pack(side="left")
        self.redraw()

    def _on_show_bev(self):
        if self.show_bev.get() and self.bev is not None:
            if self.bev.town == self.plan.town:
                self.view.fit(self.bev.bounds())
            else:
                self.status.set("This BEV was recorded on {}: open that map.".format(self.bev.town))
        elif self.show_bev.get():
            self.status.set("Choose a processed frame in the BEV tab first.")
        self.redraw()

    def process_run(self, all_frames):
        run, frame = self._selected_run(), self._selected_frame()
        if run is None or (frame is None and not all_frames):
            return
        for button in (self.btn_process, self.btn_process_all):
            button.state(["disabled"])
        self.bev_info.configure(text="Processing {} of {}...".format(
            "every frame" if all_frames else "frame {}".format(frame), run.name))
        self.offline.submit("process", _job_process, run.path, None if all_frames else [frame])

    def open_figure(self):
        run, frame, cav = self._selected_run(), self._selected_frame(), self.cav_box.get()
        if run is None or frame is None or not cav:
            return
        self.bev_info.configure(text="Drawing {} of frame {}...".format(cav, frame))
        self.offline.submit("figure", _job_figure, run.path, frame, cav)

    def open_view(self, which, cav=None):
        run, frame = self._selected_run(), self._selected_frame()
        if run is None or frame is None or self.bev is None or (which == "cav" and not cav):
            return
        self.bev_info.configure(text="Drawing {} of frame {}...".format(cav or which.replace("_", " "), frame))
        self.offline.submit("view", _job_view, run.path, frame, which, cav)

    def _show_image(self, path):
        from PIL import Image, ImageTk
        img = Image.open(path)
        scale = min(1.0, self.root.winfo_screenheight() * 0.85 / img.height,
                    self.root.winfo_screenwidth() * 0.9 / img.width)
        if scale < 1.0:
            img = img.resize((int(img.width * scale), int(img.height * scale)), Image.LANCZOS)
        top = tk.Toplevel(self.root)
        top.title(os.path.basename(path))
        photo = ImageTk.PhotoImage(img)
        label = ttk.Label(top, image=photo)
        label.image = photo  # keep a reference, or Tk shows nothing
        label.pack()
        ttk.Label(top, text=path, style="Hint.TLabel").pack(anchor="w", padx=8, pady=4)

    def _on_offline_result(self, name, ok, result):
        for button in (self.btn_process, self.btn_process_all):
            button.state(["!disabled"])
        if not ok:
            what = "Processing" if name == "process" else "Figure"
            self.bev_info.configure(text="{} failed: {}".format(what, result))
            messagebox.showerror(what, str(result))
            return
        if name == "process":
            run_dir, outputs = result
            frames = [int(os.path.basename(os.path.normpath(o))) for o in outputs]
            self.refresh_runs(select=run_dir, frame=frames[-1])
            self.side.select(self.bev_tab)
            self.show_bev.set(True)
            self._on_show_bev()
            self.status.set("Processed {} frame(s) of {}.".format(len(frames), os.path.basename(run_dir)))
        else:
            self.bev_info.configure(text="Figure saved: {}".format(result))
            self._show_image(result)

    # ================================================================== CARLA
    def check_carla(self):
        self.carla_status.set("CARLA: checking...")
        self.worker.submit("status", _job_status)

    def _update_carla_widgets(self):
        online = self.server_town is not None
        same = online and self.server_town == self.plan.town
        idle = self.sim is None
        info = catalog.find_map(self.maps, self.plan.town)
        for label in CARLA_ACTIONS:
            self._carla_menu.entryconfigure(label, state="normal" if same and idle else "disabled")
        can_switch = bool(self.carla_root) and info is not None and info.installed and not same and idle
        self._carla_menu.entryconfigure(STARTUP_MAP, state="normal" if can_switch else "disabled")
        if not online:
            text = "CARLA: not running"
        elif same:
            text = "CARLA: running {}  \u2713".format(self.server_town)
        elif info is not None and not info.installed:
            text = "CARLA: running {} ({} is edit only)".format(self.server_town, self.plan.town)
        else:
            text = "CARLA: running {} (plan: {})".format(self.server_town, self.plan.town)
        self.carla_status.set(text)

    def preview(self):
        paths = [(v.role, route.points.tolist()) for v in self.plan.vehicles if v.route
                 for route in [self._route_of(v)[0]] if route is not None]
        self.worker.submit("preview", _job_preview, self.plan.planned(), paths)
        self.status.set("Drawing the plan in the CARLA window...")

    def test_spawn(self):
        if not self.plan.vehicles:
            return
        self.worker.submit("test", _job_test_spawn, self.plan.planned(), self.cfg.scenario.cav_blueprint)
        self.status.set("Test spawn running...")

    def capture_aerial(self):
        self._carla_menu.entryconfigure(CARLA_ACTIONS[3], state="disabled")
        progress = lambda done, total: self.worker.results.put(("progress", True, (done, total)))  # noqa: E731
        georef = self.backgrounds["roads"][1]
        self.worker.submit("aerial", _job_aerial, self.plan.town, self.road_map.bounds(), georef.ppm, progress)
        self.status.set("Taking the aerial photo (the CARLA window will move)...")

    def set_startup_map(self):
        from ..backend.simulator import startup_map
        if not messagebox.askyesno("Start CARLA on {}".format(self.plan.town), (
                "CARLA will open {} the next time it starts.\n\nClose CARLA and start it again after this. "
                "Continue?").format(self.plan.town)):
            return
        try:
            path = startup_map.set_startup_map(self.carla_root, self.plan.town)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Cannot change the startup map", str(exc))
            return
        messagebox.showinfo("Done", "Startup map set to {} in\n{}\n\nNow close and reopen CARLA, then press "
                                    "'Check connection'.".format(self.plan.town, path))

    def _poll_worker(self):
        for worker in (self.worker, self.offline):
            try:
                while True:
                    name, ok, result = worker.results.get_nowait()
                    self._on_worker_result(name, ok, result)
            except queue.Empty:
                pass
        self._poll_live()
        self.root.after(100, self._poll_worker)

    def _on_worker_result(self, name, ok, result):
        if name in ("process", "figure", "view"):
            self._on_offline_result(name, ok, result)
            return
        if name == "progress":
            done, total = result
            self.status.set("Aerial photo: tile {} of {}...".format(done, total))
            return
        if name == "status":
            if ok:
                self.server_town, cached = result
                info = catalog.find_map(self.maps, cached.name)
                if info is None or not info.installed:  # no CARLA folder: use the server's map
                    self._find_maps()
                    # the map the server is running is installed, whatever the cache says
                    self.maps = [replace(m, installed=True) if m.name == cached.name else m for m in self.maps]
                    self.map_box["values"] = [m.label for m in self.maps]
                    self._show_map_in_box()
                    if self.road_map is None:
                        self.open_map(cached.name)
            else:
                self.server_town = None
            self._update_carla_widgets()
            return
        if not ok:
            self.status.set("CARLA error: {}".format(result))
            messagebox.showerror("CARLA", str(result))
            if name == "aerial":
                self._update_carla_widgets()
            return
        if name == "preview":
            self.status.set("{} vehicles drawn in the CARLA window for 60 s.".format(result))
        elif name == "test":
            if result:
                messagebox.showwarning("Blocked positions", "These vehicles cannot be spawned (something is "
                                                            "in the way):\n" + ", ".join(result))
                self.status.set("Test spawn: {} blocked".format(len(result)))
            else:
                self.status.set("Test spawn: all {} vehicles fit.".format(len(self.plan.vehicles)))
        elif name == "aerial":
            self.backgrounds["aerial"] = raster.load_image(raster.aerial_path(MAPS_CACHE, result))
            self.background.set("aerial")
            self._apply_background()
            self._update_carla_widgets()
            self.status.set("Aerial photo of {} saved.".format(result))

    def show_help(self):
        messagebox.showinfo("Controls", (
            "SCENE\n"
            "  Left click on a road: add a vehicle (1 = CAV, 2 = traffic)\n"
            "  Click a vehicle: select it; drag it: move it along the roads\n"
            "  R: CAV <-> traffic   Del: delete   T: draw a route (Enter ends)\n"
            "  Wheel: zoom   Right drag: pan   F: fit vehicles   Ctrl+Z / Ctrl+Y: undo / redo\n\n"
            "SIMULATION (CARLA running the plan's map)\n"
            "  Start: spawn the scene and drive it in real time\n"
            "  Pause / Resume (Space): freeze or continue\n"
            "  Record: save LiDAR frames while it drives; press again to stop, then the BEV is computed\n"
            "  Stop: remove everything from CARLA\n\n"
            "SETTINGS: hover any setting to see what it does.\n"
            "BEV: every CAV's BEV of a recorded frame, drawn on the map."))


def _number(var, label, kind=float):
    """The number in a Tk variable, or ValueError naming the field."""
    text = str(var.get()).strip()
    try:
        value = float(text)
    except ValueError:
        raise ValueError("{}: '{}' is not a number".format(label, text))
    if kind is int:
        if value != int(value):
            raise ValueError("{}: '{}' must be a whole number".format(label, text))
        return int(value)
    return value


def _started(run_name):
    """Start time (epoch seconds) from a run folder name '<YYYY-mm-dd>_<HHMMSS>_<name>', or 0."""
    try:
        return time.mktime(time.strptime(run_name[:17], "%Y-%m-%d_%H%M%S"))
    except ValueError:
        return 0.0


def _sharp_on_windows():
    """Tell Windows the window handles display scaling itself, so text and map are not blurred."""
    if os.name != "nt":
        return
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass


def run(cfg, town=None, plan_path=None):
    _sharp_on_windows()
    root = tk.Tk()
    EditorApp(root, cfg, town, plan_path)
    root.mainloop()
