"""Deterministic procedural lumber/OSB, slip-disk and acuity-target textures; no downloads."""

from pathlib import Path

import numpy as np


def write_textures(directory: Path):
    from PIL import Image, ImageDraw

    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(2026)
    for name, base in (("osb", (169, 128, 76)), ("wood", (176, 130, 78))):
        noise = rng.normal(0, 4, (512, 512, 1))
        pixels = np.uint8(np.clip(np.array(base)[None, None, :] + noise, 0, 255))
        picture = Image.fromarray(pixels, "RGB")
        draw = ImageDraw.Draw(picture)
        for _ in range(1900 if name == "osb" else 800):
            x, y = rng.uniform(-40, 552, 2)
            angle = rng.uniform(0, np.pi) if name == "osb" else rng.normal(0, 0.008)
            length = rng.uniform(8, 60) if name == "osb" else rng.uniform(70, 360)
            width = rng.uniform(1, 5) if name == "osb" else rng.uniform(0.2, 1.5)
            axis = np.array([np.cos(angle), np.sin(angle)]) * length / 2
            normal = np.array([-np.sin(angle), np.cos(angle)]) * width / 2
            center = np.array([x, y])
            polygon = [
                tuple(center + axis + normal),
                tuple(center - axis + normal),
                tuple(center - axis - normal),
                tuple(center + axis - normal),
            ]
            shade = rng.uniform(0.70, 1.25)
            draw.polygon(polygon, fill=tuple(int(min(255, c * shade)) for c in base))
        picture.save(directory / f"{name}.png")
    from .geometry import ACUITY_TARGETS, COLORS

    for name, (color, gaps) in ACUITY_TARGETS.items():
        acuity_target(directory / f"target_{name}.png", color, name, gaps)
    slip_disk(directory / "slip_disk.png", COLORS["slip_disk"])


def slip_disk(path, color, size=256):
    """A slip disk's face, spanning the texture: the disk's colour with the thick marker line
    from the centre to the rim that shows it has turned (p. 40)."""
    from PIL import Image, ImageDraw

    image = Image.new("RGB", (size, size), tuple(int(round(v * 255)) for v in color))
    draw = ImageDraw.Draw(image)
    c = size / 2
    draw.rectangle((c, c - 5, size - 1, c + 5), fill=(15, 15, 15))
    image.save(path)


def acuity_target(path, color, label, gaps, size=256):
    """Visual/colour acuity target, printed p. 72: coloured ring, white ring, black disc
    with the label and three nested white Landolt Cs. `gaps` are the Cs' gap directions,
    counter-clockwise from the right; the image's up is the world's up on the pipe."""
    from PIL import Image, ImageDraw, ImageFont

    image = Image.new("RGB", (size, size), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    c = size / 2
    draw.ellipse((0, 0, size - 1, size - 1), fill=tuple(int(round(v * 255)) for v in color))
    draw.ellipse((22, 22, size - 23, size - 23), fill=(255, 255, 255))
    draw.ellipse((30, 30, size - 31, size - 31), fill=(0, 0, 0))
    for radius, width, gap in zip((70, 46, 24), (14, 11, 8), gaps):
        draw.ellipse(
            (c - radius, c - radius, c + radius, c + radius),
            outline=(255, 255, 255),
            width=width,
        )
        # PIL angles run clockwise because y points down; cut a 40 degree gap.
        draw.pieslice(
            (c - radius - 2, c - radius - 2, c + radius + 2, c + radius + 2),
            start=-gap - 20,
            end=-gap + 20,
            fill=(0, 0, 0),
        )
    try:
        font = ImageFont.load_default(size=20)
    except TypeError:  # Pillow before 10.1: bitmap default font only
        font = ImageFont.load_default()
    left, top, right, bottom = draw.textbbox((0, 0), label, font=font)
    draw.text(
        (c - (right - left) / 2 - left, 43 - (bottom - top) / 2 - top),
        label,
        fill=(255, 255, 255),
        font=font,
    )
    image.save(path)
