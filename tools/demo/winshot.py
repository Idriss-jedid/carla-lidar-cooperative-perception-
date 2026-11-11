"""
Capture a Tk window's own pixels with PrintWindow (Windows): works while the window is transparent and off-screen,
so a recording never shows, or covers, the user's desktop.
"""
import ctypes
from ctypes import wintypes

from PIL import Image

user32, gdi32 = ctypes.windll.user32, ctypes.windll.gdi32
HANDLE = ctypes.c_void_p


class BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [("biSize", wintypes.DWORD), ("biWidth", wintypes.LONG), ("biHeight", wintypes.LONG),
                ("biPlanes", wintypes.WORD), ("biBitCount", wintypes.WORD), ("biCompression", wintypes.DWORD),
                ("biSizeImage", wintypes.DWORD), ("biXPelsPerMeter", wintypes.LONG),
                ("biYPelsPerMeter", wintypes.LONG), ("biClrUsed", wintypes.DWORD), ("biClrImportant", wintypes.DWORD)]


class MENUBARINFO(ctypes.Structure):
    _fields_ = [("cbSize", wintypes.DWORD), ("rcBar", wintypes.RECT), ("hMenu", HANDLE), ("hwndMenu", HANDLE),
                ("flags", wintypes.DWORD)]


user32.GetParent.restype = HANDLE
user32.GetParent.argtypes = [HANDLE]
user32.GetWindowRect.argtypes = [HANDLE, ctypes.POINTER(wintypes.RECT)]
user32.GetClientRect.argtypes = [HANDLE, ctypes.POINTER(wintypes.RECT)]
user32.ClientToScreen.argtypes = [HANDLE, ctypes.POINTER(wintypes.POINT)]
user32.GetMenuBarInfo.argtypes = [HANDLE, wintypes.LONG, wintypes.LONG, ctypes.POINTER(MENUBARINFO)]
user32.GetWindowDC.restype = HANDLE
user32.GetWindowDC.argtypes = [HANDLE]
user32.ReleaseDC.argtypes = [HANDLE, HANDLE]
user32.PrintWindow.argtypes = [HANDLE, HANDLE, wintypes.UINT]
gdi32.CreateCompatibleDC.restype = HANDLE
gdi32.CreateCompatibleDC.argtypes = [HANDLE]
gdi32.CreateCompatibleBitmap.restype = HANDLE
gdi32.CreateCompatibleBitmap.argtypes = [HANDLE, ctypes.c_int, ctypes.c_int]
gdi32.SelectObject.restype = HANDLE
gdi32.SelectObject.argtypes = [HANDLE, HANDLE]
gdi32.DeleteObject.argtypes = [HANDLE]
gdi32.DeleteDC.argtypes = [HANDLE]
gdi32.GetDIBits.argtypes = [HANDLE, HANDLE, wintypes.UINT, wintypes.UINT, ctypes.c_void_p,
                            ctypes.POINTER(BITMAPINFOHEADER), wintypes.UINT]
OBJID_MENU = -3


def hwnd_of(root):
    """The Win32 frame window of a Tk toplevel (the parent of Tk's own child window)."""
    return user32.GetParent(root.winfo_id())


def hide_offscreen(root, width, height):
    """Rendered but out of the user's way: transparent, off-screen, not in the taskbar. width x height = client."""
    root.attributes("-alpha", 0.0)
    root.attributes("-toolwindow", True)
    root.geometry("{}x{}+-7000+-7000".format(width, height))
    root.state("normal")          # shown (PrintWindow needs a mapped window), but already off-screen


def layout(hwnd):
    """(window rect, crop box in window pixels from the menu bar's top to the client's bottom, frame origin on screen)."""
    win = wintypes.RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(win))
    client = wintypes.RECT()
    user32.GetClientRect(hwnd, ctypes.byref(client))
    origin = wintypes.POINT(0, 0)
    user32.ClientToScreen(hwnd, ctypes.byref(origin))
    top = origin.y
    info = MENUBARINFO()
    info.cbSize = ctypes.sizeof(MENUBARINFO)
    if user32.GetMenuBarInfo(hwnd, OBJID_MENU, 0, ctypes.byref(info)):
        top = info.rcBar.top
    box = (origin.x - win.left, top - win.top, origin.x - win.left + client.right, origin.y - win.top + client.bottom)
    return (win.left, win.top, win.right, win.bottom), box, (origin.x, top)


def capture(hwnd, win_size):
    """The whole window as a PIL image (window pixels)."""
    w, h = win_size
    window_dc = user32.GetWindowDC(hwnd)
    dc = gdi32.CreateCompatibleDC(window_dc)
    bmp = gdi32.CreateCompatibleBitmap(window_dc, w, h)
    gdi32.SelectObject(dc, bmp)
    user32.PrintWindow(hwnd, dc, 2)  # PW_RENDERFULLCONTENT
    info = BITMAPINFOHEADER()
    info.biSize, info.biWidth, info.biHeight = ctypes.sizeof(BITMAPINFOHEADER), w, -h
    info.biPlanes, info.biBitCount = 1, 32
    buf = ctypes.create_string_buffer(w * h * 4)
    gdi32.GetDIBits(dc, bmp, 0, h, buf, ctypes.byref(info), 0)
    gdi32.DeleteObject(bmp)
    gdi32.DeleteDC(dc)
    user32.ReleaseDC(hwnd, window_dc)
    return Image.frombuffer("RGB", (w, h), buf, "raw", "BGRX", 0, 1)
