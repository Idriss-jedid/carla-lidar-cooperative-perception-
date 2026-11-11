"""Frame size and fonts of the walkthrough video (the overlays drawn by record.py)."""
from PIL import ImageFont

W, H = 1600, 900
FONT_DIR = "C:/Windows/Fonts/"


def font(size, bold=False):
    return ImageFont.truetype(FONT_DIR + ("segoeuib.ttf" if bold else "segoeui.ttf"), size)
