#!/usr/bin/env python3
"""Turn a recording from record_run.py into finished video frames.

    python3 compose_frames.py RUN_DIR OUT_DIR

Each output frame shows the top-down map on the left and the robot's camera,
the explorer's state, progress and a legend on the right. Needs Pillow and the
DejaVu fonts (apt install python3-pil fonts-dejavu-core). The frames are then
joined into a GIF/MP4 with ffmpeg, see tools/demo/README.md.
"""
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

W, H = 1040, 660
BG = (15, 18, 24)
PANEL = (24, 28, 36)
INK = (232, 235, 241)
MUTED = (152, 161, 177)
STATES = {
    "EXPLORING": ("EXPLORING", (28, 126, 214)),
    "RETURNING_HOME": ("RETURNING HOME", (240, 140, 0)),
    "COMPLETE": ("MAP COMPLETE", (47, 158, 68)),
    "IDLE": ("EXPLORATION OFF", (73, 80, 87)),
}
LEGEND = [
    ((29, 33, 41), "unexplored"), ((233, 236, 239), "seen, free"),
    ((201, 42, 42), "obstacle"), ((239, 152, 90), "safety halo"),
    ((21, 170, 191), "frontier"), ((47, 158, 68), "planned path"),
    ((34, 139, 230), "robot"), ((230, 73, 128), "current goal"),
]

FONT_DIR = "/usr/share/fonts/truetype/dejavu"


def font(size, bold=False):
    return ImageFont.truetype(os.path.join(FONT_DIR, "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"), size)


def main():
    run_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    frames = [json.loads(line) for line in open(os.path.join(run_dir, "frames.jsonl"))]

    title, sub, label, small = font(26, True), font(15), font(17, True), font(14)
    last_cam = None
    for fr in frames:
        name = f"{fr['frame']:04d}"
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)

        # Map, scaled up 2x without smoothing so each cell stays a crisp square
        m = Image.open(os.path.join(run_dir, f"map_{name}.png"))
        img.paste(m.resize((620, 620), Image.NEAREST), (20, 20))

        # Right column
        x = 660
        d.text((x, 20), "Frontier exploration", font=title, fill=INK)
        d.text((x, 54), "WATonomous ASD  ·  warehouse world", font=sub, fill=MUTED)

        cam_path = os.path.join(run_dir, f"cam_{name}.png")
        if os.path.exists(cam_path):
            last_cam = Image.open(cam_path).convert("RGB").resize((360, 270), Image.BILINEAR)
        d.rectangle((x - 1, 89, x + 360, 360), outline=(58, 64, 78))
        if last_cam:
            img.paste(last_cam, (x, 90))
        d.text((x + 8, 96), "robot's camera", font=small, fill=(255, 255, 255))

        text, color = STATES.get(fr["status"], (fr["status"], (73, 80, 87)))
        d.rounded_rectangle((x, 378, x + 360, 416), radius=8, fill=color)
        tw = d.textlength(text, font=label)
        d.text((x + 180 - tw / 2, 386), text, font=label, fill=(255, 255, 255))

        t = int(fr["t"])
        d.text((x, 432), f"time  {t // 60}:{t % 60:02d}", font=sub, fill=INK)
        pct = fr["coverage"]
        cov = f"map explored  {pct:.0%}"
        d.text((x + 360 - d.textlength(cov, font=sub), 432), cov, font=sub, fill=INK)
        d.rounded_rectangle((x, 458, x + 360, 468), radius=5, fill=PANEL)
        d.rounded_rectangle((x, 458, x + max(10, int(360 * pct)), 468), radius=5, fill=(47, 158, 68))

        for k, (col, what) in enumerate(LEGEND):
            lx, ly = x + (k % 2) * 180, 492 + (k // 2) * 36
            d.rounded_rectangle((lx, ly, lx + 20, ly + 20), radius=4, fill=col, outline=(90, 96, 110))
            d.text((lx + 30, ly + 1), what, font=small, fill=MUTED)

        d.text((x, 640), "lidar-only mapping · A* planning · pure pursuit", font=small, fill=(90, 98, 112))
        img.save(os.path.join(out_dir, f"frame_{name}.png"))
    print(f"wrote {len(frames)} frames to {out_dir}")


if __name__ == "__main__":
    main()
