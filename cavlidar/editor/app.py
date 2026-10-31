"""
The scenario editor window.

    python main.py editor                     # last map used (Town03 the first time)
    python main.py editor --map Town05
    python main.py editor --plan config/spawn_plans/figure2_town03.json

Left: the map (roads drawn from OpenDRIVE, or an aerial photo taken from CARLA).
Right: vehicles, placement rules, CARLA tools, export / record.

Works without CARLA running: maps come from the OpenDRIVE files shipped with
CARLA. CARLA is only needed to preview, test-spawn, take aerial photos and record.
"""
import math
import os
import queue
import threading
import tkinter as tk
from dataclasses import replace
from tkinter import filedialog, messagebox, ttk

from ..backend.actors.spawn_plan import DEFAULT_ROUTE_SPEED, MAX_SPEED, PROJECT_ROOT
from ..backend.maps import catalog, raster, routes
from ..backend.maps.road_map import RoadMap, wrap_degrees
from ..core.enums import Role, RouteEnd
from ..core.errors import RouteError
from . import export
from . import settings
from .map_view import MapView
from .settings import MAPS_CACHE, load_settings, save_settings
from .model import PlannedVehicle, ScenarioPlan, footprint

PAPER_BBOX = [-100.0, 100.0, -200.0, 200.0]

COLORS = {Role.CAV: "#3d8bfd", Role.TRAFFIC: "#ff7043"}
SELECTED = "#ffd54f"
OK_GHOST, BAD_GHOST = "#7ee787", "#ff6b6b"
STOP_MARK = "#e53935"
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


class EditorApp:

    def __init__(self, root, cfg, town=None, plan_path=None):
        self.root, self.cfg = root, cfg
        self.settings = load_settings()
        self.worker = CarlaWorker(cfg)
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

        self._build()
        self._find_maps()
        if plan_path:
            self.open_plan(plan_path)
        else:
            self.open_map(self.plan.town)
        self.check_carla()
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

        self._build_menu()
        bar = ttk.Frame(root, padding=(8, 6))
        bar.pack(side="top", fill="x")
        ttk.Label(bar, text="Map", style="Title.TLabel").pack(side="left")
        self.map_box = ttk.Combobox(bar, state="readonly", width=34)
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
        side.add(self._build_vehicles_tab(side), text="Vehicles")
        side.add(self._build_rules_tab(side), text="Rules")
        side.add(self._build_carla_tab(side), text="CARLA")
        side.add(self._build_record_tab(side), text="Export / Record")

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
                          ("<BackSpace>", self.remove_route_point)):
            root.bind(seq, lambda e, f=func: None if isinstance(e.widget, (tk.Entry, ttk.Entry, ttk.Spinbox))
                      else f())
        root.protocol("WM_DELETE_WINDOW", self.quit)

    def _build_menu(self):
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="New plan", accelerator="Ctrl+N", command=self.new_plan)
        file_menu.add_command(label="Open plan...", accelerator="Ctrl+O", command=self.ask_open_plan)
        file_menu.add_command(label="Save plan", accelerator="Ctrl+S", command=self.save_plan)
        file_menu.add_command(label="Save plan as...", command=lambda: self.save_plan(ask=True))
        file_menu.add_separator()
        file_menu.add_command(label="Import OpenSCENARIO (.xosc)...", command=self.import_xosc)
        file_menu.add_command(label="Export OpenSCENARIO (.xosc)...", command=self.export_xosc)
        file_menu.add_separator()
        file_menu.add_command(label="CARLA folder...", command=self.ask_carla_root)
        file_menu.add_separator()
        file_menu.add_command(label="Quit", command=self.quit)
        menu.add_cascade(label="File", menu=file_menu)
        help_menu = tk.Menu(menu, tearoff=False)
        help_menu.add_command(label="Controls", command=self.show_help)
        menu.add_cascade(label="Help", menu=help_menu)
        self.root.config(menu=menu)

    def _section(self, parent, text):
        ttk.Label(parent, text=text, style="Title.TLabel").pack(anchor="w", pady=(10, 4))

    def _build_vehicles_tab(self, parent):
        tab = ttk.Frame(parent, padding=10)
        self._section(tab, "Click on a road to add a")
        row = ttk.Frame(tab)
        row.pack(fill="x")
        ttk.Radiobutton(row, text="CAV (blue, has a LiDAR)   [1]", value="cav",
                        variable=self.add_role).pack(anchor="w")
        ttk.Radiobutton(row, text="Traffic vehicle (orange)   [2]", value="traffic",
                        variable=self.add_role).pack(anchor="w")
        self.count_label = ttk.Label(tab, font=("Segoe UI", 10, "bold"))
        self.count_label.pack(anchor="w", pady=(10, 4))
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
        ttk.Label(tab, style="Hint.TLabel", wraplength=340, justify="left", text=(
            "Left click: add or select   |   drag: move along the roads\n"
            "Wheel: zoom   |   right drag: pan   |   Ctrl+Z / Ctrl+Y: undo / redo\n"
            "Vehicles always snap to the nearest driving lane and face its direction.")).pack(anchor="w", pady=6)
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

    def _build_rules_tab(self, parent):
        tab = ttk.Frame(parent, padding=10)
        self._section(tab, "Spawn area")
        ttk.Checkbutton(tab, text="Only allow vehicles inside this box", variable=self.use_bbox,
                        command=self.redraw).pack(anchor="w")
        grid = ttk.Frame(tab)
        grid.pack(anchor="w", pady=4)
        for i, label in enumerate(("x min", "x max", "y min", "y max")):
            ttk.Label(grid, text=label).grid(row=i // 2, column=(i % 2) * 2, sticky="e", padx=(0, 4), pady=2)
            entry = ttk.Entry(grid, textvariable=self.bbox_vars[i], width=9)
            entry.grid(row=i // 2, column=(i % 2) * 2 + 1, padx=(0, 12), pady=2)
            entry.bind("<FocusOut>", lambda e: self.redraw())
            entry.bind("<Return>", lambda e: self.redraw())
        ttk.Button(tab, text="Use the paper's box (Table 1)", command=self._paper_bbox).pack(anchor="w", pady=2)
        ttk.Label(tab, style="Hint.TLabel", wraplength=340, justify="left", text=(
            "Paper: x in [-100, 100], y in [-200, 200] (Town03, world coordinates). "
            "Exported with the experiment as scenario.spawn_bbox.")).pack(anchor="w")
        self._section(tab, "Placement")
        ttk.Checkbutton(tab, text="Allow vehicles inside junctions", variable=self.allow_junction).pack(anchor="w")
        row = ttk.Frame(tab)
        row.pack(anchor="w", pady=4)
        ttk.Label(row, text="Max distance to a lane (m)").pack(side="left")
        ttk.Spinbox(row, from_=1, to=20, increment=1, width=5, textvariable=self.snap_distance).pack(
            side="left", padx=6)
        ttk.Label(tab, style="Hint.TLabel", wraplength=340, justify="left", text=(
            "Two vehicles may not overlap (4.8 x 2.1 m footprint + 0.6 m gap). "
            "Vehicles on different levels (bridges) are allowed to cross.")).pack(anchor="w", pady=4)
        return tab

    def _build_carla_tab(self, parent):
        tab = ttk.Frame(parent, padding=10)
        self._section(tab, "Simulator")
        self.carla_info = ttk.Label(tab, wraplength=340, justify="left")
        self.carla_info.pack(anchor="w")
        ttk.Button(tab, text="Check connection", command=self.check_carla).pack(fill="x", pady=(6, 2))
        self._section(tab, "Check the plan")
        self.btn_preview = ttk.Button(tab, text="Show the plan in the CARLA window", command=self.preview)
        self.btn_preview.pack(fill="x", pady=2)
        self.btn_test = ttk.Button(tab, text="Test spawn (spawn, check, remove)", command=self.test_spawn)
        self.btn_test.pack(fill="x", pady=2)
        self._section(tab, "Aerial photo")
        self.btn_aerial = ttk.Button(tab, text="Take aerial photo of this map", command=self.capture_aerial)
        self.btn_aerial.pack(fill="x", pady=2)
        self.aerial_progress = ttk.Progressbar(tab, mode="determinate")
        self.aerial_progress.pack(fill="x", pady=2)
        ttk.Label(tab, style="Hint.TLabel", wraplength=340, justify="left", text=(
            "Needs this map loaded in CARLA. Saved in data/maps/, so it is taken only once.")).pack(anchor="w")
        self._section(tab, "Switch CARLA to another map")
        self.btn_startup = ttk.Button(tab, text="Start CARLA on this map next time", command=self.set_startup_map)
        self.btn_startup.pack(fill="x", pady=2)
        ttk.Label(tab, style="Hint.TLabel", wraplength=340, justify="left", text=(
            "Writes the map into CARLA's DefaultEngine.ini (backup kept). Close and reopen CARLA "
            "afterwards. Safer than load_world(), which crashed CARLA on a 4 GB GPU.")).pack(anchor="w")
        return tab

    def _build_record_tab(self, parent):
        tab = ttk.Frame(parent, padding=10)
        self.exp_name = tk.StringVar(value="my_scenario")
        self.extra_traffic = tk.IntVar(value=0)
        self.exp_autopilot = tk.BooleanVar(value=True)
        self.exp_frames = tk.IntVar(value=10)
        self.exp_every = tk.IntVar(value=20)
        self._section(tab, "Experiment")
        form = ttk.Frame(tab)
        form.pack(fill="x")
        rows = (("Name", ttk.Entry(form, textvariable=self.exp_name, width=22)),
                ("Extra random traffic", ttk.Spinbox(form, from_=0, to=100, width=6, textvariable=self.extra_traffic)),
                ("Frames to save", ttk.Spinbox(form, from_=1, to=10000, width=6, textvariable=self.exp_frames)),
                ("Save every N ticks", ttk.Spinbox(form, from_=1, to=1000, width=6, textvariable=self.exp_every)))
        for r, (label, widget) in enumerate(rows):
            ttk.Label(form, text=label).grid(row=r, column=0, sticky="w", pady=3)
            widget.grid(row=r, column=1, sticky="w", padx=8, pady=3)
        ttk.Checkbutton(tab, text="Vehicles drive (autopilot)", variable=self.exp_autopilot).pack(anchor="w",
                                                                                                  pady=4)
        ttk.Label(tab, style="Hint.TLabel", wraplength=340, justify="left", text=(
            "Extra traffic is spawned at random CARLA spawn points (inside the spawn area if it is "
            "enabled). 20 ticks = 1 simulated second.")).pack(anchor="w")
        ttk.Button(tab, text="Export experiment", command=lambda: self.export(False)).pack(fill="x", pady=(12, 2))
        ttk.Button(tab, text="Export and record now", style="Accent.TButton",
                   command=lambda: self.export(True)).pack(fill="x", pady=2)
        self.export_info = ttk.Label(tab, wraplength=340, justify="left")
        self.export_info.pack(anchor="w", pady=8)
        return tab

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
        for i, v in enumerate(self.plan.vehicles):
            if v.route:
                self._draw_route(view, i, v)
        for i, v in enumerate(self.plan.vehicles):
            self._draw_vehicle(view, v.pose, COLORS[v.role], self.plan.name(i), i == self.selected, i in outside)
        if self._ghost is not None:
            pose, ok = self._ghost
            pts = [view.world_to_screen(x, y) for x, y in footprint(pose)]
            view.create_polygon(*[c for p in pts for c in p], outline=OK_GHOST if ok else BAD_GHOST,
                                fill="", dash=(4, 3), width=2)
        if self._route_ghost is not None:  # where a route click lands, and the lane's direction
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

    def _draw_vehicle(self, view, pose, color, label, selected, outside):
        pts = [view.world_to_screen(x, y) for x, y in footprint(pose)]
        flat = [c for p in pts for c in p]
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
        if self.selected is None:
            return
        v = self.plan.vehicles[self.selected]
        self.plan.set_role(self.selected, Role.TRAFFIC if v.role == Role.CAV else Role.CAV)
        self.dirty = True
        self.refresh()

    def delete_selected(self):
        if self.selected is None:
            return
        name = self.plan.name(self.selected)
        self.plan.remove(self.selected)
        self.selected = None
        self.dirty = True
        self.status.set("Deleted {} (Ctrl+Z to undo); the others were renumbered".format(name))
        self.refresh()

    def clear_all(self):
        if self.plan.vehicles and messagebox.askyesno("Clear all", "Remove every vehicle? (Ctrl+Z undoes it)"):
            self.plan.clear()
            self.selected = None
            self.dirty = True
            self.refresh()

    def undo(self):
        if self.plan.undo():
            self.dirty = True
            self.selected = None
            self.refresh()

    def redo(self):
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
        self.set_route_mode(not self.route_mode.get())

    def _on_route_button(self):
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
        index = self.selected
        if index is None or not self.plan.vehicles[index].route:
            return
        self.plan.set_driving(index, route=self.plan.vehicles[index].route[:-1])
        self.dirty = True
        self.refresh()

    def clear_route(self):
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
        if self._confirm_discard():
            self.plan = ScenarioPlan(self.plan.town)
            self.plan_path, self.selected, self.dirty = None, None, False
            self.refresh()

    def ask_open_plan(self):
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
        if self._confirm_discard():
            self.root.destroy()

    # ================================================================== export / record
    def export(self, record):
        try:
            options = export.ExportOptions(
                name=self.exp_name.get(), extra_traffic=int(self.extra_traffic.get()),
                autopilot=self.exp_autopilot.get(), num_frames=int(self.exp_frames.get()),
                save_every=int(self.exp_every.get()), spawn_bbox=self.bbox())
            problems = self._route_problems()
            if problems:
                raise ValueError("These routes cannot be driven:\n" + "\n".join(problems))
            outside = self.plan.outside(options.spawn_bbox)
            if outside:
                raise ValueError("{} vehicles are outside the spawn area. Move them or turn the area off "
                                 "(Rules tab).".format(len(outside)))
            plan_rel, yaml_rel = export.export(self.plan, options)
            from .. import api
            api.load_config(os.path.join(PROJECT_ROOT, yaml_rel))  # same checks as the recorder
        except (ValueError, tk.TclError) as exc:
            messagebox.showerror("Cannot export", str(exc))
            return
        except Exception as exc:  # ConfigError and friends: show, don't crash the window
            messagebox.showerror("Invalid experiment", str(exc))
            return
        name = export.safe_name(options.name)
        text = "Saved:\n  {}\n  {}\n\nRecord later with:\n  python main.py record --config {} --name {}".format(
            plan_rel, yaml_rel, yaml_rel.replace(os.sep, "/"), name)
        self.export_info.configure(text=text)
        self.status.set("Exported {}".format(yaml_rel))
        if not record:
            return
        if self.server_town is None:
            messagebox.showwarning("CARLA not running", "Start CARLA first, then press record again.")
            return
        if self.server_town != self.plan.town:
            messagebox.showwarning("Wrong map in CARLA", (
                "CARLA has {} loaded, this plan is for {}.\n\nCARLA tab > 'Start CARLA on this map next "
                "time', then close and reopen CARLA.").format(self.server_town, self.plan.town))
            return
        export.launch_recording(yaml_rel, name)
        self.status.set("Recording started in a new window: {}".format(name))

    # ================================================================== CARLA
    def check_carla(self):
        self.carla_status.set("CARLA: checking...")
        self.worker.submit("status", _job_status)

    def _update_carla_widgets(self):
        online = self.server_town is not None
        same = online and self.server_town == self.plan.town
        info = catalog.find_map(self.maps, self.plan.town)
        for button in (self.btn_preview, self.btn_test, self.btn_aerial):
            button.state(["!disabled"] if same else ["disabled"])
        can_switch = bool(self.carla_root) and info is not None and info.installed and not same
        self.btn_startup.state(["!disabled"] if can_switch else ["disabled"])
        if not online:
            text = "CARLA is not running. Editing works offline; start CARLA to preview, test or record."
        elif same:
            text = "CARLA is running {}: preview, test spawn, aerial photo and recording are available.".format(
                self.server_town)
        elif info is not None and not info.installed:
            text = "CARLA is running {}. {} is not installed in this CARLA (only its road network is), " \
                   "so it can be edited but not simulated.".format(self.server_town, self.plan.town)
        else:
            text = "CARLA is running {}, not {}. Use the button below and restart CARLA to record this " \
                   "map.".format(self.server_town, self.plan.town)
        self.carla_info.configure(text=text)

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
        self.aerial_progress["value"] = 0
        self.btn_aerial.state(["disabled"])
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
        try:
            while True:
                name, ok, result = self.worker.results.get_nowait()
                self._on_worker_result(name, ok, result)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_worker)

    def _on_worker_result(self, name, ok, result):
        if name == "progress":
            done, total = result
            self.aerial_progress["value"] = 100.0 * done / total
            return
        if name == "status":
            if ok:
                self.server_town, cached = result
                self.carla_status.set("CARLA: running {}".format(self.server_town))
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
                self.carla_status.set("CARLA: not running")
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
            "Left click on a road: add a vehicle (role chosen in the Vehicles tab, keys 1 / 2)\n"
            "Left click on a vehicle: select it; drag it to move it along the roads\n"
            "R: switch the selected vehicle between CAV and traffic\n"
            "T: draw a route for the selected vehicle (click points on the roads, Enter / Esc ends,\n"
            "    Backspace removes the last point)\n"
            "Delete: remove the selected vehicle\n"
            "Mouse wheel: zoom   |   right or middle drag: pan   |   F: fit vehicles\n"
            "Ctrl+Z / Ctrl+Y: undo / redo   |   Ctrl+S: save   |   Ctrl+O: open\n\n"
            "Dashed outline under the cursor: where a click would place a vehicle\n"
            "(green = allowed, red = not allowed, the reason is in the status bar)."))


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
