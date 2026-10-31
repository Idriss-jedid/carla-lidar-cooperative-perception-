"""
MapView: a Tk canvas showing a georeferenced map image with pan and zoom.

Screen axes follow the map image: world x to the right, world y downwards.
The owner draws its own items (vehicles, boxes...) in `on_overlay(view)`,
called after every redraw, using world_to_screen().

Mouse: wheel = zoom around the cursor, right or middle drag = pan.
"""
import tkinter as tk

from PIL import Image, ImageTk

MIN_SCALE, MAX_SCALE = 0.2, 40.0   # screen pixels per metre


class MapView(tk.Canvas):

    def __init__(self, master, on_overlay=None, on_view_change=None, **kw):
        kw.setdefault("background", "#1e2328")
        kw.setdefault("highlightthickness", 0)
        super().__init__(master, **kw)
        self.on_overlay = on_overlay
        self.on_view_change = on_view_change
        self.image = None
        self.georef = None
        self.cx, self.cy, self.scale = 0.0, 0.0, 2.0
        self._photo = None
        self._bg_item = None
        self._bg_key = None
        self._pan_from = None
        self._pending = False
        self.bind("<Configure>", lambda e: self.request_redraw())
        self.bind("<MouseWheel>", self._on_wheel)
        for button in ("2", "3"):
            self.bind("<ButtonPress-{}>".format(button), self._pan_start)
            self.bind("<B{}-Motion>".format(button), self._pan_move)
            self.bind("<ButtonRelease-{}>".format(button), self._pan_end)

    # ------------------------------------------------------------------ geometry
    def world_to_screen(self, x, y):
        return (self.winfo_width() / 2.0 + (x - self.cx) * self.scale,
                self.winfo_height() / 2.0 + (y - self.cy) * self.scale)

    def screen_to_world(self, sx, sy):
        return (self.cx + (sx - self.winfo_width() / 2.0) / self.scale,
                self.cy + (sy - self.winfo_height() / 2.0) / self.scale)

    def set_background(self, image, georef, fit=False):
        self.image, self.georef = image, georef
        if fit:
            self.fit((georef.x_min, georef.x_min + georef.width / georef.ppm,
                      georef.y_min, georef.y_min + georef.height / georef.ppm))
        else:
            self.request_redraw()

    def fit(self, bounds, margin=1.05):
        x_min, x_max, y_min, y_max = bounds
        w, h = max(self.winfo_width(), 200), max(self.winfo_height(), 200)
        self.cx, self.cy = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
        span_x, span_y = max(x_max - x_min, 1.0), max(y_max - y_min, 1.0)
        self.scale = _clamp(min(w / (span_x * margin), h / (span_y * margin)))
        self.request_redraw()

    def centre_on(self, x, y, scale=None):
        self.cx, self.cy = x, y
        if scale:
            self.scale = _clamp(scale)
        self.request_redraw()

    # ------------------------------------------------------------------ drawing
    def request_redraw(self):
        """Coalesce many events (drag, wheel) into one redraw."""
        if not self._pending:
            self._pending = True
            self.after_idle(self.redraw)

    def redraw(self):
        self._pending = False
        w, h = self.winfo_width(), self.winfo_height()
        if w < 2 or h < 2:
            return
        # The background image is rebuilt only when the view changed; overlays every time
        key = (self.cx, self.cy, self.scale, w, h, id(self.image))
        if key != self._bg_key:
            self._bg_key = key
            self.delete("all")
            self._bg_item = None
            if self.image is not None:
                self._draw_background(w, h)
        else:
            for item in self.find_all():
                if item != self._bg_item:
                    self.delete(item)
        if self.on_overlay:
            self.on_overlay(self)
        if self.on_view_change:
            self.on_view_change(self)

    def _draw_background(self, w, h):
        g = self.georef
        x0, y0 = self.screen_to_world(0, 0)
        x1, y1 = self.screen_to_world(w, h)
        px0, py0 = g.to_px(x0, y0)
        px1, py1 = g.to_px(x1, y1)
        box = tuple(int(round(v)) for v in (px0, py0, px1, py1))
        if box[2] <= box[0] or box[3] <= box[1]:
            return
        resample = Image.NEAREST if self.scale > 2 * g.ppm else Image.BILINEAR
        crop = self.image.crop(box).resize((w, h), resample, reducing_gap=2.0)
        self._photo = ImageTk.PhotoImage(crop)
        self._bg_item = self.create_image(0, 0, image=self._photo, anchor="nw")

    # ------------------------------------------------------------------ mouse
    def zoom_at(self, sx, sy, factor):
        wx, wy = self.screen_to_world(sx, sy)
        self.scale = _clamp(self.scale * factor)
        # keep the point under the cursor fixed
        self.cx = wx - (sx - self.winfo_width() / 2.0) / self.scale
        self.cy = wy - (sy - self.winfo_height() / 2.0) / self.scale
        self.request_redraw()

    def _on_wheel(self, event):
        self.zoom_at(event.x, event.y, 1.2 if event.delta > 0 else 1 / 1.2)

    def _pan_start(self, event):
        self._pan_from = (event.x, event.y)
        self.configure(cursor="fleur")

    def _pan_move(self, event):
        if self._pan_from is None:
            return
        dx, dy = event.x - self._pan_from[0], event.y - self._pan_from[1]
        self.cx -= dx / self.scale
        self.cy -= dy / self.scale
        self._pan_from = (event.x, event.y)
        self.request_redraw()

    def _pan_end(self, event):
        self._pan_from = None
        self.configure(cursor="")


def _clamp(scale):
    return max(MIN_SCALE, min(MAX_SCALE, scale))
