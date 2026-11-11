"""
Record the narrated walkthrough of the scenario editor, without touching the user's screen.

    python tools/demo/tts.py data/demo        # first, in an environment with edge-tts: the voice clips
    python tools/demo/record.py data/demo     # CARLA 0.9.12 running Town03 (e.g. CarlaUE4.exe -RenderOffScreen)
    python tools/demo/edit.py data/demo data/demo/editor_walkthrough.mp4

The editor runs as usual, but transparent and off-screen; a script drives it live (an invisible mouse moving,
clicking the map and the buttons, keys) against a running CARLA, and every frame is taken from the window itself with
PrintWindow, 15 times per second, with the pointer, click ripples, highlights and CARLA's own camera view drawn on
top. No slides: the explanations are spoken while the app works.

Writes  <out>/raw.mkv          the video, real time
        <out>/segments.json    the waits (spawning, processing), to speed up in the edit
        <out>/narration.json   every spoken line: time, title, sentence clips (file, duration)
        <out>/steps.log        every step with its time
"""
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
import traceback

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import cards  # noqa: E402
import winshot  # noqa: E402
from narration import BY_KEY  # noqa: E402

from cavlidar.api import DEFAULT_CONFIG, load_config  # noqa: E402
from cavlidar.editor import app as editor_app  # noqa: E402
from cavlidar.editor import simulation  # noqa: E402
from cavlidar.editor.model import ScenarioPlan  # noqa: E402

OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "data/demo")
W, H, FPS = cards.W, cards.H, 15
GAP = 0.25                     # silence between two sentences of one line
NAME = "paper_figure2"         # experiment / run name used in the video
FIGURE2 = "config/spawn_plans/figure2_town03.json"
HL = (255, 213, 79)            # highlight colour
DRY = os.environ.get("DEMO_DRY") == "1"   # rehearsal: sentence lengths estimated, no voice needed
os.makedirs(OUT, exist_ok=True)
try:
    with open(os.path.join(OUT, "tts", "manifest.json"), encoding="utf-8") as fh:
        MANIFEST = json.load(fh)["sentences"]
except OSError:
    if not DRY:
        raise
    MANIFEST = {}


def sentences(text):
    return [s.strip() for s in re.split(r"(?<=[.!?])\s+(?=[A-Z])", text.strip()) if s.strip()]


# ====================================================================== the picture: window + overlays
class Studio:
    """Takes the frames on its own thread and draws the overlays the director sets."""

    def __init__(self, root):
        self.root = root
        self.hwnd = winshot.hwnd_of(root)
        self.lock = threading.Lock()
        self.pointer = None          # (x, y) in frame pixels
        self.ripples = []            # (x, y, time)
        self.highlight = None        # (x0, y0, x1, y1, label)
        self.card = None             # full-frame slide
        self.picture = None          # (image, caption) shown over the window (e.g. a figure)
        self.pip = None              # (image, caption): CARLA's camera
        self.key = None              # (text, time) of a key press
        self.badge = None            # text of the "sped up" badge
        self.t0 = None
        self.stop = False
        self.frames = 0
        self.measure()

    def measure(self):
        win, self.box, self.origin = winshot.layout(self.hwnd)
        self.win_size = (win[2] - win[0], win[3] - win[1])
        cw, ch = self.box[2] - self.box[0], self.box[3] - self.box[1]
        self.scale = (W / float(cw), H / float(ch))

    def to_frame(self, sx, sy):
        """Screen pixel -> frame pixel."""
        return (sx - self.origin[0]) * self.scale[0], (sy - self.origin[1]) * self.scale[1]

    def widget_rect(self, w, pad=4):
        x0, y0 = self.to_frame(w.winfo_rootx(), w.winfo_rooty())
        x1, y1 = self.to_frame(w.winfo_rootx() + w.winfo_width(), w.winfo_rooty() + w.winfo_height())
        return x0 - pad, y0 - pad, x1 + pad, y1 + pad

    # ------------------------------------------------------------------ drawing
    def window_image(self):
        img = winshot.capture(self.hwnd, self.win_size).crop(self.box)
        return img if img.size == (W, H) else img.resize((W, H), Image.LANCZOS)

    def compose(self, t):
        with self.lock:
            card, picture, pip, highlight = self.card, self.picture, self.pip, self.highlight
            pointer, ripples, key, badge = self.pointer, list(self.ripples), self.key, self.badge
        img = card.copy() if card is not None else self.window_image()
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        d = ImageDraw.Draw(layer)
        if card is None and pip is not None:
            image, caption = pip
            x0, y0 = 22, H - image.height - 70
            img.paste(image, (x0, y0))
            d.rectangle((x0 - 3, y0 - 3, x0 + image.width + 2, y0 + image.height + 2), outline=(255, 255, 255, 230),
                        width=3)
            d.rounded_rectangle((x0, y0 - 38, x0 + 330, y0 - 6), radius=8, fill=(16, 20, 25, 220))
            d.text((x0 + 10, y0 - 34), caption, font=cards.font(20, True), fill=(255, 255, 255, 255))
        if card is None and picture is not None:
            image, caption = picture
            d.rectangle((0, 0, W, H), fill=(10, 12, 15, 200))
            x0, y0 = (W - image.width) // 2, (H - image.height) // 2 + 10
            layer.alpha_composite(image.convert("RGBA"), (x0, y0))
            d.text((W / 2, y0 - 22), caption, font=cards.font(24, True), fill=(255, 255, 255, 255), anchor="mm")
        if highlight is not None:
            x0, y0, x1, y1, label = highlight
            d.rounded_rectangle((x0, y0, x1, y1), radius=10, outline=HL + (255,), width=4)
            if label:
                f = cards.font(22, True)
                tw = d.textlength(label, font=f)
                ly = y1 + 8 if y1 + 44 < H - 130 else y0 - 42
                lx = min(max(x0, 8), W - tw - 30)
                d.rounded_rectangle((lx, ly, lx + tw + 22, ly + 34), radius=8, fill=HL + (245,))
                d.text((lx + 11, ly + 4), label, font=f, fill=(20, 20, 20, 255))
        if key is not None and t - key[1] < 1.6:
            f = cards.font(30, True)
            text = key[0]
            tw = d.textlength(text, font=f)
            x0 = W / 2 - tw / 2 - 26
            d.rounded_rectangle((x0, 58, x0 + tw + 52, 112), radius=12, fill=(16, 20, 25, 225),
                                outline=(255, 255, 255, 200), width=2)
            d.text((W / 2, 85), text, font=f, fill=(255, 255, 255, 255), anchor="mm")
        if badge:
            f = cards.font(22, True)
            tw = d.textlength(badge, font=f)
            d.rounded_rectangle((W - tw - 52, 14, W - 14, 52), radius=10, fill=(235, 104, 52, 235))
            d.text((W - tw - 33, 18), badge, font=f, fill=(255, 255, 255, 255))
        for x, y, t_click in ripples:
            age = t - t_click
            if 0 <= age < 0.5:
                r = 10 + 46 * age / 0.5
                a = int(230 * (1 - age / 0.5))
                d.ellipse((x - r, y - r, x + r, y + r), outline=(255, 255, 255, a), width=4)
        if pointer is not None and card is None:
            x, y = pointer
            arrow = [(0, 0), (0, 26), (7, 20), (12, 31), (17, 29), (12, 18), (21, 18)]
            pts = [(x + px, y + py) for px, py in arrow]
            d.polygon(pts, fill=(255, 255, 255, 255), outline=(0, 0, 0, 255))
        img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
        return img

    # ------------------------------------------------------------------ the recording thread
    def start(self, path):
        self.proc = subprocess.Popen(
            ["ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "{}x{}".format(W, H),
             "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "ultrafast", "-crf", "16", "-pix_fmt",
             "yuv420p", path], stdin=subprocess.PIPE)
        self.t0 = time.perf_counter()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def now(self):
        return time.perf_counter() - self.t0

    def _run(self):
        next_t = self.t0
        while not self.stop:
            wait = next_t - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
                continue
            try:
                data = self.compose(self.now()).tobytes()
            except Exception:
                traceback.print_exc()
                continue
            while next_t <= time.perf_counter():      # late: repeat the frame, so the timeline stays real time
                self.proc.stdin.write(data)
                self.frames += 1
                next_t = self.t0 + self.frames / float(FPS)

    def finish(self):
        self.stop = True
        self.thread.join(5)
        self.proc.stdin.close()
        self.proc.wait()


# ====================================================================== the director
class Director:
    """Runs the walkthrough as a generator: yields seconds to wait, or a condition to wait for."""

    def __init__(self, root, ed, studio):
        self.root, self.ed, self.studio = root, ed, studio
        self.narration, self.segments, self.log = [], [], []
        self.speaking_until = 0.0
        self.wait_start = None
        self.failed = None
        self.camera = None

    def note(self, text):
        line = "{:7.2f}  {}".format(self.studio.now() if self.studio.t0 else 0.0, text)
        self.log.append(line)
        print(line, flush=True)

    # ------------------------------------------------------------------ running the script
    def run(self, script):
        gen = script()

        def step():
            try:
                item = next(gen)
            except StopIteration:
                self.done()
                return
            except Exception:
                self.failed = traceback.format_exc()
                self.note("FAILED\n" + self.failed)
                self.done()
                return
            if callable(item):
                poll(item)
            else:
                self.root.after(max(1, int(1000 * float(item or 0))), step)

        def poll(cond):
            try:
                ready = cond()
            except Exception:
                self.failed = traceback.format_exc()
                self.note("FAILED\n" + self.failed)
                self.done()
                return
            if ready:
                step()
            else:
                self.root.after(80, lambda: poll(cond))

        self.root.after(10, step)

    def done(self):
        try:
            self.stop_camera()
            if self.ed.sim is not None:
                self.ed.sim.stop()
                self.ed.sim.join(60)
        finally:
            self.root.after(300, self.root.destroy)

    # ------------------------------------------------------------------ speech
    def say(self, key, wait=True):
        yield from self.finish()
        title, text = BY_KEY[key]
        clips = []
        for s in sentences(text):
            m = MANIFEST.get(s) if DRY else MANIFEST[s]
            if m is None:
                m = {"file": None, "dur": round(0.4 + len(s.split()) / 2.7, 3)}   # ~2.7 words per second
            clips.append({"file": m["file"], "dur": m["dur"], "text": s})
        total = sum(c["dur"] for c in clips) + GAP * (len(clips) - 1)
        self.narration.append({"t": round(self.studio.now(), 3), "title": title, "clips": clips})
        self.speaking_until = self.studio.now() + total
        self.note("say {} ({:.1f} s)".format(key, total))
        if wait:
            yield from self.finish()

    def finish(self):
        left = self.speaking_until - self.studio.now() + 0.15
        if left > 0:
            yield left

    # ------------------------------------------------------------------ waits that the edit speeds up
    def begin_wait(self, label):
        self.wait_start = (self.studio.now(), label)
        with self.studio.lock:
            self.studio.badge = u"⏩ sped up · " + label

    def end_wait(self, speed=4.0):
        start, label = self.wait_start
        self.segments.append({"start": round(start, 3), "end": round(self.studio.now(), 3), "speed": speed,
                              "label": label})
        self.wait_start = None
        with self.studio.lock:
            self.studio.badge = None

    def until(self, cond, timeout, label):
        end = time.time() + timeout

        def check():
            if cond():
                return True
            if time.time() > end:
                raise RuntimeError("timeout: " + label)
            return False
        yield check

    # ------------------------------------------------------------------ pointer, clicks, keys, highlights
    def move(self, x, y, dur=0.6, motion=None):
        """Animate the pointer to frame pixel (x, y); `motion(t)` is called along the way (map hover)."""
        x0, y0 = self.studio.pointer or (W * 0.55, H * 0.55)
        n = max(1, int(dur / 0.04))
        for i in range(1, n + 1):
            u = i / float(n)
            e = u * u * (3 - 2 * u)
            px, py = x0 + (x - x0) * e, y0 + (y - y0) * e
            with self.studio.lock:
                self.studio.pointer = (px, py)
            if motion is not None:
                motion(px, py)
            yield 0.04

    def ripple(self):
        with self.studio.lock:
            x, y = self.studio.pointer
            self.studio.ripples = [r for r in self.studio.ripples if self.studio.now() - r[2] < 1] + \
                [(x, y, self.studio.now())]

    def widget_center(self, w, fx=0.5, fy=0.5):
        x0, y0, x1, y1 = self.studio.widget_rect(w, pad=0)
        return x0 + (x1 - x0) * fx, y0 + (y1 - y0) * fy

    def click(self, w, action=None, dur=0.6, after=0.5):
        x, y = self.widget_center(w)
        yield from self.move(x, y, dur)
        yield 0.15
        self.ripple()
        (action or w.invoke)()
        self.note("click {}".format(w))
        yield after

    def key(self, text, action=None):
        with self.studio.lock:
            self.studio.key = (text, self.studio.now())
        if action is not None:
            action()
        self.note("key {}".format(text))
        yield 0.5

    def highlight(self, target, label="", pad=6):
        rect = target if isinstance(target, tuple) else self.studio.widget_rect(target, pad)
        with self.studio.lock:
            self.studio.highlight = tuple(rect) + (label,)

    def unhighlight(self):
        with self.studio.lock:
            self.studio.highlight = None

    def slide(self, image):
        with self.studio.lock:
            self.studio.card = image

    # ------------------------------------------------------------------ the map
    def canvas_to_frame(self, cx, cy):
        v = self.ed.view
        return self.studio.to_frame(v.winfo_rootx() + cx, v.winfo_rooty() + cy)

    def world_to_frame(self, x, y):
        return self.canvas_to_frame(*self.ed.view.world_to_screen(x, y))

    def frame_to_canvas(self, fx, fy):
        v = self.ed.view
        sx = self.studio.origin[0] + fx / self.studio.scale[0]
        sy = self.studio.origin[1] + fy / self.studio.scale[1]
        return int(round(sx - v.winfo_rootx())), int(round(sy - v.winfo_rooty()))

    def hover_map(self, fx, fy):
        cx, cy = self.frame_to_canvas(fx, fy)
        self.ed.view.event_generate("<Motion>", x=cx, y=cy)

    def move_map(self, x, y, dur=0.7):
        fx, fy = self.world_to_frame(x, y)
        yield from self.move(fx, fy, dur, motion=self.hover_map)

    def click_map(self, x, y, dur=0.7):
        yield from self.move_map(x, y, dur)
        yield 0.2
        cx, cy = self.frame_to_canvas(*self.studio.pointer)
        self.ripple()
        self.ed.view.event_generate("<ButtonPress-1>", x=cx, y=cy)
        self.ed.view.event_generate("<ButtonRelease-1>", x=cx, y=cy)
        self.note("map click ({:.1f}, {:.1f}): {}".format(x, y, self.ed.status.get()))
        yield 0.35

    def drag_map(self, x0, y0, x1, y1, dur=1.2):
        yield from self.move_map(x0, y0, 0.6)
        cx, cy = self.frame_to_canvas(*self.studio.pointer)
        self.ripple()
        self.ed.view.event_generate("<ButtonPress-1>", x=cx, y=cy)
        fx, fy = self.world_to_frame(x1, y1)

        def drag(px, py):
            ccx, ccy = self.frame_to_canvas(px, py)
            self.ed.view.event_generate("<B1-Motion>", x=ccx, y=ccy)
        yield from self.move(fx, fy, dur, motion=drag)
        cx, cy = self.frame_to_canvas(fx, fy)
        self.ed.view.event_generate("<ButtonRelease-1>", x=cx, y=cy)
        yield 0.3

    def tab(self, index):
        """Click the header of notebook tab `index`."""
        nb = self.ed.side
        xs = []
        for x in range(2, nb.winfo_width(), 3):
            try:
                if nb.index("@{},{}".format(x, 12)) == index:
                    xs.append(x)
            except tk.TclError:
                pass
        cx = (xs[0] + xs[-1]) / 2.0 if xs else 30
        fx, fy = self.studio.to_frame(nb.winfo_rootx() + cx, nb.winfo_rooty() + 12)
        yield from self.move(fx, fy, 0.6)
        yield 0.15
        self.ripple()
        nb.select(index)
        yield 0.5

    # ------------------------------------------------------------------ CARLA's own view of a CAV
    def start_camera(self):
        import carla
        cfg = self.ed.cfg
        client = carla.Client(cfg.server.host, cfg.server.port)
        client.set_timeout(10.0)
        world = client.get_world()
        world.wait_for_tick(10.0)   # a new client sees the actors only after a tick of the simulation
        cav = next((a for a in world.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "cav_0"),
                   None)
        if cav is None:
            self.note("camera: cav_0 not found")
            return
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        for k, v in (("image_size_x", "480"), ("image_size_y", "270"), ("fov", "90"), ("sensor_tick", "0.1")):
            bp.set_attribute(k, v)
        self.camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=-7.5, z=3.4), carla.Rotation(pitch=-14)),
                                        attach_to=cav)

        def on_image(image):
            a = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
            with self.studio.lock:
                self.studio.pip = (Image.fromarray(a[:, :, 2::-1].copy()), "CARLA 3D view  ·  behind cav_0")
        self.camera.listen(on_image)
        self.note("camera on cav_0")

    def stop_camera(self):
        if self.camera is not None:
            try:
                self.camera.stop()
                self.camera.destroy()
            except Exception:
                pass
            self.camera = None
        with self.studio.lock:
            self.studio.pip = None


def busiest_car(bev):
    """(x, y, cells) of the recorded vehicle box holding the most BEV cells of the other CAVs."""
    from matplotlib.path import Path
    best = (0.0, 0.0, -1)
    for name, _, corners in bev.vehicles:
        c = np.asarray(corners)
        centre = c.mean(axis=0)
        box = Path(centre + (c - centre) + np.sign(c - centre) * 0.5)     # the box + 0.5 m
        n = sum(int(box.contains_points(cav.cells.mean(axis=1)).sum()) for cav in bev.cavs
                if cav.name != name.replace("_", "") and len(cav.cells))
        if n > best[2]:
            best = (float(centre[0]), float(centre[1]), n)
    return best


def find(widget, test):
    """First descendant of `widget` for which test(w) is true."""
    for child in widget.winfo_children():
        try:
            if test(child):
                return child
        except tk.TclError:
            pass
        hit = find(child, test)
        if hit is not None:
            return hit
    return None


def by_var(ed, var):
    return find(ed.root, lambda w: str(w.cget("textvariable")) == str(var))


def by_text(ed, text):
    return find(ed.root, lambda w: str(w.cget("text")) == text)


# ====================================================================== the walkthrough
def walkthrough(d, ed, studio):
    paper = ScenarioPlan.load(FIGURE2)
    cavs = [v for v in paper.vehicles if v.role.value == "cav"]
    traffic = [v for v in paper.vehicles if v.role.value != "cav"]
    figure = {}

    def show_figure(path):
        figure["path"] = path

    ed._show_image = show_figure          # the figure goes into the video, not into a window

    def connected():
        if ed.server_town == "Town03":
            return True
        if time.time() - connected.last > 3:      # ask CARLA again every 3 s
            connected.last = time.time()
            ed.check_carla()
        return False
    connected.last = time.time()

    def script():
        ed.view.centre_on(7.0, -132.0, 6.6)
        yield from d.until(connected, 120, "CARLA connection")
        yield 1.5

        # ---------------------------------------------------------- the window
        yield from d.say("intro")
        d.highlight(by_var(ed, ed.carla_status), "CARLA status")
        yield from d.say("win1")
        d.highlight(ed.map_box.master, "map, background, view, undo / redo", pad=2)
        yield from d.say("win2")
        d.highlight(ed.btn_start.master, "simulation bar", pad=2)
        yield from d.say("win3")
        d.highlight(ed.view, "the map (OpenDRIVE roads)", pad=-6)
        yield from d.say("win4", wait=False)
        yield from d.move(*d.world_to_frame(2.9, -136.5), dur=0.8)
        yield from d.finish()
        d.unhighlight()
        yield from d.say("win5", wait=False)
        yield from d.click(ed.aerial_radio)
        yield from d.finish()
        yield 1.2
        yield from d.say("win6", wait=False)
        yield from d.click(by_text(ed, "Roads"))
        yield from d.finish()
        d.highlight(ed.side, "Scene  ·  Settings  ·  BEV", pad=2)
        yield from d.say("win7")
        d.highlight((0, 0, 230, studio.to_frame(0, ed.root.winfo_rooty())[1]), "File  ·  CARLA  ·  Help menus", pad=0)
        yield from d.say("win8")
        d.unhighlight()

        # ---------------------------------------------------------- the paper's scene
        jx, jy = d.world_to_frame(2.9, -136.5)
        yield from d.say("paper1", wait=False)
        yield from d.move(jx, jy, 0.8)
        yield from d.finish()
        r = 26 * ed.view.scale
        d.highlight((jx - r, jy - r, jx + r, jy + r), "junction 1221: the paper's Figure 2", pad=0)
        yield from d.say("paper2")
        d.unhighlight()
        yield from d.tab(0)
        d.highlight(by_text(ed, "CAV (LiDAR)  [1]"), "key 1: CAV")
        yield from d.say("place1", wait=False)
        yield from d.key("1", lambda: ed.add_role.set("cav"))
        yield from d.finish()
        d.unhighlight()
        first = cavs[0].pose
        yield from d.say("place2", wait=False)
        yield from d.move_map(first["x"] + 14, first["y"] + 9, 1.0)
        yield from d.move_map(first["x"], first["y"], 1.0)
        yield from d.finish()
        yield from d.say("place3", wait=False)
        yield from d.click_map(first["x"], first["y"])
        yield from d.finish()
        yaw = math.radians(first["yaw"])          # red: right in front of the car just placed
        yield from d.move_map(first["x"] + 3.5 * math.cos(yaw), first["y"] + 3.5 * math.sin(yaw), 0.8)
        yield 1.6
        yield from d.say("place4", wait=False)
        for v in cavs[1:]:
            yield from d.click_map(v.pose["x"], v.pose["y"])
        yield from d.finish()
        d.highlight(by_text(ed, "Traffic  [2]"), "key 2: traffic")
        yield from d.say("place5", wait=False)
        yield from d.key("2", lambda: ed.add_role.set("traffic"))
        yield from d.finish()
        d.unhighlight()
        yield from d.say("place6", wait=False)
        for v in traffic:
            yield from d.click_map(v.pose["x"], v.pose["y"], dur=0.55)
        yield from d.finish()
        d.highlight(ed.table, "cars: position, heading, lane")
        yield from d.say("place7")
        d.unhighlight()
        d.note("placed {} vehicles".format(len(ed.plan.vehicles)))

        # ---------------------------------------------------------- editing
        t1 = traffic[1].pose
        yield from d.say("edit1", wait=False)
        yield from d.click_map(t1["x"], t1["y"])
        yield from d.key("R", ed.toggle_role)
        yield 0.8
        yield from d.key("R", ed.toggle_role)
        yield from d.finish()
        t5 = traffic[5].pose
        yaw = math.radians(t5["yaw"])
        yield from d.say("edit2", wait=False)
        yield from d.drag_map(t5["x"], t5["y"], t5["x"] + 7 * math.cos(yaw), t5["y"] + 7 * math.sin(yaw))
        yield 0.6
        yield from d.key("Ctrl + Z", ed.undo)
        yield from d.finish()
        c1 = cavs[1].pose
        yield from d.say("edit3", wait=False)
        yield from d.click_map(c1["x"], c1["y"])
        yield from d.click(ed.chk_speed)
        yield from d.key("T", ed.toggle_route_mode)
        yield from d.click_map(c1["x"] + 0.4, -165.0)
        yield from d.key("Enter", lambda: ed.set_route_mode(False))
        yield from d.finish()
        yield 1.0
        yield from d.say("edit4", wait=False)
        yield from d.click(ed.btn_route_clear)
        yield from d.click(ed.chk_speed)
        yield from d.finish()
        d.note("plan after editing: {} vehicles, routes {}".format(
            len(ed.plan.vehicles), [v.route for v in ed.plan.vehicles if v.route]))

        # ---------------------------------------------------------- the paper's settings
        yield from d.tab(1)
        d.highlight(by_text(ed, "LiDAR (every CAV)"), "the paper's LiDAR")
        yield from d.say("set1")
        lights = by_var(ed, ed.lights_vars[1])
        d.highlight(lights.master, "red lights ignored: 0 to 100 %")
        yield from d.say("set2", wait=False)
        yield from d.click(lights, action=lambda: None)
        ed.lights_vars[1].set("")
        for ch in "100":
            ed.lights_vars[1].set(ed.lights_vars[1].get() + ch)
            yield 0.15
        yield from d.finish()
        extra = by_var(ed, ed.extra_traffic)
        d.highlight(extra, "9 random cars: 11 + 9 = 20")
        yield from d.say("set3", wait=False)
        yield from d.click(extra, action=lambda: ed.extra_traffic.set(9))
        yield from d.finish()
        settings_canvas = ed._scroll_areas[0][1]
        settings_canvas.yview_moveto(1.0)
        yield 0.5
        name = by_var(ed, ed.exp_name)
        every = by_var(ed, ed.exp_every)
        d.highlight(every.master, "one frame every 10 ticks, until stopped")
        yield from d.say("set4", wait=False)
        yield from d.click(name, action=lambda: ed.exp_name.set(""))
        for ch in NAME:
            ed.exp_name.set(ed.exp_name.get() + ch)
            yield 0.05
        yield from d.click(every, action=lambda: ed.exp_every.set(10))
        yield from d.finish()
        d.unhighlight()
        settings_canvas.yview_moveto(0.0)

        # ---------------------------------------------------------- start
        yield from d.tab(0)
        yield from d.say("start1", wait=False)
        yield from d.click(ed.btn_start)
        yield from d.finish()
        d.begin_wait("spawning in CARLA")
        yield from d.say("carla1", wait=False)
        yield from d.until(lambda: ed._live is not None and ed._live.state == simulation.RUNNING, 120, "running")
        yield from d.finish()
        d.end_wait(speed=3.0)
        d.note("running: {}".format(ed.sim_text.get()))
        yield 1.0
        d.highlight(ed.view, "live: every car, every tick", pad=-6)
        yield from d.say("start2")
        d.highlight(by_var(ed, ed.sim_text), "simulated time, speed vs real time")
        yield from d.say("start3")
        d.unhighlight()

        # ---------------------------------------------------------- pause
        yield from d.say("pause1", wait=False)
        yield from d.click(ed.btn_pause)
        frame = ed._live.frame
        yield from d.finish()
        yield 2.0
        d.note("paused: frame {} -> {}".format(frame, ed._live.frame))
        yield from d.say("pause2", wait=False)
        yield from d.click(ed.btn_pause)
        yield from d.finish()
        yield 1.0

        # ---------------------------------------------------------- record (while the CAVs are still near the junction)
        yield from d.say("rec1", wait=False)
        yield from d.click(ed.btn_record)
        yield from d.finish()
        d.start_camera()
        yield 1.0
        yield from d.say("start4")
        d.highlight(by_var(ed, ed.sim_text), "REC: frames saved")
        yield from d.say("rec2")
        d.unhighlight()
        yield from d.say("carla2")
        yield from d.say("carla3")
        yield from d.say("carla4")
        yield from d.until(lambda: ed._live.frames_saved >= 16, 180, "16 frames")
        d.note("recorded: {}".format(ed.sim_text.get()))
        yield from d.say("rec4", wait=False)
        yield from d.click(ed.btn_record)
        d.begin_wait("processing up to the BEV")
        yield from d.until(lambda: ed.bev is not None and ed.bev.run.endswith("_" + NAME) and ed.show_bev.get(),
                           240, "BEV")
        yield from d.finish()
        d.end_wait(speed=3.0)
        d.note("BEV: " + ", ".join("{} {} cells {} points".format(c.name, len(c.cells), c.non_ground_points)
                                   for c in ed.bev.cavs))

        # ---------------------------------------------------------- stop the simulation, then the result alone
        d.stop_camera()
        yield from d.say("stop1", wait=False)
        yield from d.click(ed.btn_stop)
        d.begin_wait("removing the cars")
        yield from d.until(lambda: ed.sim is None, 120, "stopped")
        yield from d.finish()
        d.end_wait(speed=2.0)
        yield 0.6

        # ---------------------------------------------------------- the BEV
        d.highlight(ed.run_box, "the new recording")
        yield from d.say("bev1", wait=False)
        yield 1.5
        d.highlight(ed.frame_box, "first frame")
        yield from d.click(ed.frame_box, action=lambda: (ed.frame_box.current(0), ed._on_frame_chosen()))
        yield from d.finish()
        cx, cy, n = busiest_car(ed.bev)       # the car with the most BEV cells on it, from any CAV
        d.note("view on the car with {} cells at ({:.1f}, {:.1f})".format(n, cx, cy))
        ed.view.centre_on(cx, cy, 8.0)
        d.highlight(ed.view, "BEV cells of each CAV", pad=-6)
        yield from d.say("bev2")
        yield from d.say("bev3")
        d.highlight(ed.bev_legend, "cells and obstacle points per CAV")
        yield from d.say("bev4")
        d.unhighlight()
        best = max(ed.bev.cavs, key=lambda c: c.non_ground_points).name
        ed.cav_box.set(best)
        yield from d.say("bev5", wait=False)
        yield from d.click(ed.btn_figure)
        d.begin_wait("drawing the figure")
        yield from d.until(lambda: "path" in figure, 120, "figure")
        yield from d.finish()
        d.end_wait(speed=2.0)
        image = Image.open(figure["path"]).convert("RGB")
        image.thumbnail((W - 120, H - 110))
        with studio.lock:
            studio.picture = (image, "Open figure: {} stage by stage".format(best))
        yield from d.say("bev6")
        yield from d.say("bev7")
        yield from d.say("bev8")
        yield 0.8
        with studio.lock:
            studio.picture = None
        yield 0.6

        # ---------------------------------------------------------- every CAV, before and after filtering
        def show(path, caption):
            image = Image.open(path).convert("RGB")
            image.thumbnail((W - 120, H - 110))
            with studio.lock:
                studio.picture = (image, caption)

        def hide():
            with studio.lock:
                studio.picture = None

        d.highlight(ed.view_buttons[0].master, "All CAVs, before and after filtering", pad=4)
        yield 1.2
        views = (("world_raw", "cmp1", "Raw point clouds of the 4 CAVs, world frame"),
                 ("world_filtered", "cmp2", "After filtering, all CAVs"),
                 ("world_bev", "cmp3", "BEV of all CAVs together"),
                 ("vehicles", "cmp4", "Points per car, before -> after filtering"))
        for index, (which, key, caption) in enumerate(views):
            figure.clear()
            d.highlight(ed.view_buttons[index], caption)
            yield from d.click(ed.view_buttons[index])
            d.unhighlight()
            d.begin_wait("drawing the figure")
            yield from d.until(lambda: "path" in figure, 120, which)
            d.end_wait(speed=2.0)
            d.note("{}: {}".format(which, figure["path"]))
            show(figure["path"], caption)
            yield from d.say(key)
            yield 0.8
            hide()
            yield 0.4
        figure.clear()
        d.highlight(ed.cav_view_box, "one CAV: " + best)
        yield from d.click(ed.cav_view_box, action=lambda: ed.cav_view_box.set(best))
        yield from d.click(ed.view_buttons[4])
        d.unhighlight()
        d.begin_wait("drawing the figure")
        yield from d.until(lambda: "path" in figure, 120, "cav view")
        d.end_wait(speed=2.0)
        show(figure["path"], "{} on its own, before and after filtering".format(best))
        yield from d.say("cmp5")
        yield 0.8
        hide()
        yield 0.6

        # ---------------------------------------------------------- end
        yield from d.say("stop2")
        yield from d.say("outro")
        yield 2.0

    return script


def main():
    editor_app._sharp_on_windows()
    root = tk.Tk()
    root.withdraw()                       # never on the user's screen, not even for a moment
    root.state = lambda *a: None          # the editor maximises and centres its window: not this time
    root.geometry = lambda *a: ""
    for name in ("showerror", "showwarning", "showinfo"):
        setattr(editor_app.messagebox, name, lambda title, msg, **kw: print("MESSAGE BOX:", title, msg))
    editor_app.messagebox.askyesno = lambda *a, **kw: True
    cfg = load_config(DEFAULT_CONFIG)
    ed = editor_app.EditorApp(root, cfg, "Town03")
    del root.state, root.geometry
    winshot.hide_offscreen(root, W, H - 22)
    root.update()
    studio = Studio(root)
    # make the menu bar + client exactly W x H
    cw, ch = studio.box[2] - studio.box[0], studio.box[3] - studio.box[1]
    root.geometry("{}x{}".format(W - (cw - W), H - 22 - (ch - H)))
    root.update()
    studio.measure()
    print("frame area {} -> {}x{}".format(studio.box, W, H))
    director = Director(root, ed, studio)
    studio.start(os.path.join(OUT, "raw.mkv"))
    director.run(walkthrough(director, ed, studio))
    root.mainloop()
    studio.finish()
    with open(os.path.join(OUT, "narration.json"), "w") as fh:
        json.dump(director.narration, fh, indent=1)
    with open(os.path.join(OUT, "segments.json"), "w") as fh:
        json.dump(director.segments, fh, indent=1)
    with open(os.path.join(OUT, "steps.log"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(director.log))
    print("{} frames ({:.1f} min) -> {}".format(studio.frames, studio.frames / FPS / 60.0, OUT))
    if director.failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
