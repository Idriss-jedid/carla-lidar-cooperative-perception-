"""Small Tk helpers for the editor: tooltips, a vertically scrolling frame and coloured buttons."""
import tkinter as tk
from tkinter import ttk

TIP_BG, TIP_FG = "#ffffe6", "#202124"


class Tooltip:
    """Shows `text` in a small box under `widget` while the mouse rests on it."""

    def __init__(self, widget, text, delay_ms=350):
        self.widget, self.text, self.delay = widget, text, delay_ms
        self._after = None
        self._window = None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _event=None):
        self._cancel()
        self._after = self.widget.after(self.delay, self._show)

    def _cancel(self):
        if self._after is not None:
            self.widget.after_cancel(self._after)
            self._after = None

    def _show(self):
        self._after = None
        if self._window is not None or not self.text:
            return
        x = self.widget.winfo_rootx() + 10
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        self._window = tk.Toplevel(self.widget)
        self._window.wm_overrideredirect(True)
        self._window.wm_geometry("+{}+{}".format(x, y))
        tk.Label(self._window, text=self.text, justify="left", background=TIP_BG, foreground=TIP_FG,
                 relief="solid", borderwidth=1, wraplength=340, font=("Segoe UI", 9), padx=8, pady=5).pack()

    def _hide(self, _event=None):
        self._cancel()
        if self._window is not None:
            self._window.destroy()
            self._window = None


def tip(widget, text):
    """Attach a tooltip and return the widget (handy inside layout code)."""
    Tooltip(widget, text)
    return widget


def scrolling_frame(parent, padding=10):
    """(outer, inner, canvas): `inner` scrolls vertically inside `outer` when it is taller than the window."""
    outer = ttk.Frame(parent)
    background = ttk.Style().lookup("TFrame", "background") or "#f0f0f0"
    canvas = tk.Canvas(outer, highlightthickness=0, borderwidth=0, background=background)
    bar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
    inner = ttk.Frame(canvas, padding=padding)
    window = canvas.create_window((0, 0), window=inner, anchor="nw")
    inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
    canvas.configure(yscrollcommand=bar.set)
    bar.pack(side="right", fill="y")
    canvas.pack(side="left", fill="both", expand=True)
    return outer, inner, canvas


def color_button(parent, text, color, command, width=11):
    """A flat button with a coloured background (ttk buttons cannot be coloured on Windows); `.color` keeps it."""
    button = tk.Button(parent, text=text, command=command, width=width, bg=color, fg="white",
                       activebackground=color, activeforeground="white", disabledforeground="#f4f5f6",
                       relief="flat", bd=0, padx=8, pady=4, font=("Segoe UI", 10, "bold"), cursor="hand2")
    button.color = color
    return button
