#!/usr/bin/env python3
"""Turn a recording from record_run.py into finished video frames.

    python3 compose_frames.py RUN_DIR OUT_DIR

Each output frame shows the top-down map with the robot's camera, the
explorer's state, progress and a legend. Square maps (warehouse) go on the
left with everything else in a column on the right; wide maps (watonomous)
go across the top with the rest underneath. Needs Pillow and the DejaVu fonts
(apt install python3-pil fonts-dejavu-core). The frames are then joined into a
GIF/MP4 with ffmpeg, see tools/demo/README.md.
"""
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

BG = (15, 18, 24)
PANEL = (24, 28, 36)
INK = (232, 235, 241)
MUTED = (152, 161, 177)
FAINT = (90, 98, 112)
STATES = {
    "EXPLORING": ("EXPLORING", (28, 126, 214)),
    "RETURNING_HOME": ("RETURNING HOME", (240, 140, 0)),
    "COMPLETE": ("MAP COMPLETE", (47, 158, 68)),
    "IDLE": ("EXPLORATION OFF", (73, 80, 87)),
}
LEGEND = [
    ((29, 33, 41), "unexplored"), ((233, 236, 239), "seen, free"),
    ((24, 24, 28), "obstacle"), ((240, 194, 152), "safety halo"),
    ((21, 170, 191), "frontier"), ((47, 158, 68), "planned path"),
    ((34, 139, 230), "robot"), ((230, 73, 128), "current goal"),
]
FONT_DIR = "/usr/share/fonts/truetype/dejavu"


def font(size, bold=False):
    return ImageFont.truetype(os.path.join(FONT_DIR, "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"), size)


TITLE, SUB, LABEL, SMALL = font(26, True), font(15), font(17, True), font(14)


def draw_camera(img, d, cam, x, y, w=360, h=270):
    d.rectangle((x - 1, y - 1, x + w, y + h), outline=(58, 64, 78))
    if cam:
        img.paste(cam.resize((w, h), Image.BILINEAR), (x, y))
    d.text((x + 8, y + 6), "robot's camera", font=SMALL, fill=(255, 255, 255))


def draw_status(d, fr, x, y, w=360):
    """State badge, time, coverage bar. Returns the y just below it."""
    text, color = STATES.get(fr["status"], (fr["status"], (73, 80, 87)))
    d.rounded_rectangle((x, y, x + w, y + 38), radius=8, fill=color)
    d.text((x + w / 2 - d.textlength(text, font=LABEL) / 2, y + 8), text, font=LABEL, fill=(255, 255, 255))
    t = int(fr["t"])
    d.text((x, y + 54), f"time  {t // 60}:{t % 60:02d}", font=SUB, fill=INK)
    cov = f"map explored  {fr['coverage']:.0%}"
    d.text((x + w - d.textlength(cov, font=SUB), y + 54), cov, font=SUB, fill=INK)
    d.rounded_rectangle((x, y + 80, x + w, y + 90), radius=5, fill=PANEL)
    d.rounded_rectangle((x, y + 80, x + max(10, int(w * fr["coverage"])), y + 90), radius=5, fill=(47, 158, 68))
    return y + 90


def draw_legend(d, x, y, cols=2, col_w=180, row_h=34):
    for k, (col, what) in enumerate(LEGEND):
        lx, ly = x + (k % cols) * col_w, y + (k // cols) * row_h
        d.rounded_rectangle((lx, ly, lx + 20, ly + 20), radius=4, fill=col, outline=(90, 96, 110))
        d.text((lx + 30, ly + 1), what, font=SMALL, fill=MUTED)


def square_frame(fr, m, cam):
    """Map on the left, everything else in a column on the right (1040 x 660)."""
    img = Image.new("RGB", (1040, 660), BG)
    d = ImageDraw.Draw(img)
    img.paste(m.resize((620, 620), Image.NEAREST), (20, 20))
    x = 660
    d.text((x, 20), "Frontier exploration", font=TITLE, fill=INK)
    d.text((x, 54), f"WATonomous ASD  ·  {fr['world']} world", font=SUB, fill=MUTED)
    draw_camera(img, d, cam, x, 90)
    draw_status(d, fr, x, 378)
    draw_legend(d, x, 492, row_h=36)
    d.text((x, 640), "lidar-only mapping · A* planning · pure pursuit", font=SMALL, fill=FAINT)
    return img


def wide_frame(fr, m, cam):
    """Title, then the map across the full width, then camera | status + legend."""
    W = 1040
    map_w = W - 40
    map_h = round(m.height * map_w / m.width)
    H = 64 + map_h + 20 + 270 + 24
    H += H % 2  # video encoders need even dimensions
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((20, 16), "Frontier exploration", font=TITLE, fill=INK)
    sub = f"WATonomous ASD  ·  {fr['world']} world"
    d.text((W - 20 - d.textlength(sub, font=SUB), 26), sub, font=SUB, fill=MUTED)
    img.paste(m.resize((map_w, map_h), Image.NEAREST), (20, 64))
    y = 64 + map_h + 20
    draw_camera(img, d, cam, 20, y)
    x = 420
    bottom = draw_status(d, fr, x, y, w=600)
    draw_legend(d, x, bottom + 22, cols=4, col_w=150, row_h=32)
    d.text((x, y + 250), "lidar-only mapping · A* planning · pure pursuit", font=SMALL, fill=FAINT)
    return img


def main():
    run_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    frames = [json.loads(line) for line in open(os.path.join(run_dir, "frames.jsonl"))]

    cam = None
    for fr in frames:
        name = f"{fr['frame']:04d}"
        m = Image.open(os.path.join(run_dir, f"map_{name}.png"))
        cam_path = os.path.join(run_dir, f"cam_{name}.png")
        if os.path.exists(cam_path):
            cam = Image.open(cam_path).convert("RGB")
        layout = wide_frame if m.width > 1.6 * m.height else square_frame
        layout(fr, m, cam).save(os.path.join(out_dir, f"frame_{name}.png"))
    print(f"wrote {len(frames)} frames to {out_dir}")


if __name__ == "__main__":
    main()
