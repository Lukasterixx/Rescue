"""Search & Map Maze, printed pp. 63–67: tall "L" walls of 10 mm x 122 cm x 220 cm OSB on a
grid, under a blackout tarp, with mapping fiducials on the walls, diagonal 2x4 rails in
the hallways and two room terrains with an omni task.

The layout is the guide's example (pp. 65–67), transcribed from the drawing onto a 9 x 5
grid of 1.22 m cells: column c runs east, row r runs south from the top of the drawing.
The guide says any layout will do and should change during the competition.
"""

from __future__ import annotations

import math

import numpy as np

from .geometry import Lane, add, box, omni_inspect_task, orient3, prism, ring, yaw_quaternion

PITCH = 1.22
WALL = (0.010, 2.20)  # thickness, height
COLUMNS, ROWS = 9, 5

# Walls as (c0, r0, c1, r1) in cell units; the fractional ones are the entrance vestibules
# and the baffle east of the pocket at the right entrance.
WALLS = (
    # east-west
    (1, 0, 7, 0), (0, 1, 1, 1), (6, 1, 8.45, 1), (1, 2, 3, 2), (4, 2, 5, 2), (7, 2, 8, 2),
    (2, 3, 3, 3), (8, 3, 9, 3), (-0.65, 3, 0.33, 3), (0, 4, 1, 4), (3, 4, 4, 4), (5, 4, 8, 4),
    (1, 5, 4, 5), (7, 5, 8, 5), (4, 5.7, 5, 5.7),
    # north-south
    (0, 1, 0, 2), (0, 3, 0, 4), (1, 0, 1, 1), (1, 2, 1, 3), (1, 4, 1, 5), (2, 0, 2, 1),
    (2, 3, 2, 4), (3, 1, 3, 3), (3, 4, 3, 5), (4, 2, 4, 4), (5, 0, 5, 1), (5, 4, 5, 5.7),
    (6, 1, 6, 2), (6, 3, 6, 4), (7, 0, 7, 1), (7, 2, 7, 3), (8, 1, 8, 2), (8, 3, 8, 4),
    (8.45, 1, 8.45, 2), (9, 2, 9, 3), (-0.65, 2, -0.65, 3),
)
# Split-cylinder mapping fiducials (p. 67): (c, r, high). High ones are centred 2 m up,
# low ones 1 m up; each is a 30 cm tube 60 cm long straddling the wall so both sides see it.
FIDUCIALS = (
    (2, 0.5, True), (2, 2.0, True), (4.5, 2.0, True), (4, 3.5, True), (7, 4.0, True),
    (5, 0.5, False), (6, 1.5, False), (6, 3.5, False), (8, 3.5, False), (2, 3.5, False),
)
# Hallway diagonals (p. 66): (r, c, kind), kind "\\" from the cell's NW corner to its SE
# corner, "/" from SW to NE. Nineteen read off the drawing; the guide cuts twenty.
DIAGONALS = (
    (0, 1, "\\"), (0, 2, "/"), (0, 5, "\\"),
    (1, 0, "\\"), (1, 1, "/"), (1, 2, "\\"), (1, 5, "/"),
    (2, 1, "/"), (2, 2, "\\"), (2, 6, "\\"), (2, 7, "/"),
    (3, 0, "/"), (3, 1, "\\"), (3, 2, "/"), (3, 3, "\\"), (3, 6, "/"), (3, 7, "\\"),
    (4, 1, "/"), (4, 2, "\\"),
)
# Room terrains (p. 66): a cross of four 130 cm 2x4s on edge with an omni task at the
# centre, in the two open 2 x 2 rooms; (c, r) of the room centre and the acuity set.
ROOMS = (((4.0, 1.0), ("1", "1A", "1B", "1C", "1D")), ((5.0, 3.0), ("2", "2A", "2B", "2C", "2D")))
ENTRY = (2.9, 5.55)  # start pad, west of the south entrance corridor, facing east
RETURN = (9.8, 1.5)  # return pad outside the east entrance


def cell(c, r):
    """Cell units to lane-local metres, the grid centred on the lane origin, row 0 north."""
    return ((c - 4.5) * PITCH, (2.85 - r) * PITCH)


def maze_lane(options, origin):
    lane = Lane(
        "maze",
        "Search & Map Maze",
        origin,
        [],
        (origin[0] + cell(*ENTRY)[0], cell(*ENTRY)[1], 0.015 + 0.42),
        spawn_rotation=yaw_quaternion(0.0),
        subtitle="9 × 5 cells of 1.22 m tall L-walls · blackout tarp · example layout p. 67",
    )
    for i, (c0, r0, c1, r1) in enumerate(WALLS):
        (x0, y0), (x1, y1) = cell(c0, r0), cell(c1, r1)
        length = math.hypot(x1 - x0, y1 - y0)
        size = (length, WALL[0], WALL[1]) if r0 == r1 else (WALL[0], length, WALL[1])
        add(lane, box(f"wall_{i:02d}", ((x0 + x1) / 2, (y0 + y1) / 2, WALL[1] / 2), size, "osb"))
    for i, (c, r, high) in enumerate(FIDUCIALS):
        x, y = cell(c, r)
        tube = prism(f"fiducial_{i}", ring(0.15), 0.0, 0.60, "wood")
        add(lane, orient3(tube, (x, y, (2.0 if high else 1.0) - 0.30), (0.0, 0.0, 1.0)))
    for i, (r, c, kind) in enumerate(DIAGONALS):
        x, y = cell(c + 0.5, r + 0.5)
        yaw = math.radians(-45.0 if kind == "\\" else 45.0)
        # Two 120 cm 2x4s on edge, overlapping in the middle, spanning corner to corner.
        for j, (length, offset) in enumerate(((1.70, 0.0), (0.675, 0.05))):
            rail = box(f"diagonal_{i:02d}_{j}", (0.0, 0.0, 0.05), (length, 0.05, 0.10))
            cy, sy = math.cos(yaw), math.sin(yaw)
            v = rail.vertices.copy()
            rail.vertices[:, 0] = cy * v[:, 0] - sy * v[:, 1] + x - sy * offset
            rail.vertices[:, 1] = sy * v[:, 0] + cy * v[:, 1] + y + cy * offset
            add(lane, rail)
    for i, ((c, r), targets) in enumerate(ROOMS):
        x, y = cell(c, r)
        for j, (dx, dy) in enumerate(((1, 0), (0, 1), (-1, 0), (0, -1))):
            add(
                lane,
                box(
                    f"room{i}_arm{j}",
                    (x + dx * 0.75, y + dy * 0.75, 0.05),
                    (1.10, 0.05, 0.10) if dx else (0.05, 1.10, 0.10),
                ),
            )
        omni_inspect_task(lane, f"room{i}_omni", (x, y), 0.0, targets)
    # Blackout tarp over the walls: no collision, just darkness inside.
    (x0, y0), (x1, y1) = cell(-0.65, 5.7), cell(9.0, 0.0)
    add(
        lane,
        box("tarp", ((x0 + x1) / 2, (y0 + y1) / 2, WALL[1] + 0.005), (x1 - x0 + 0.2, y1 - y0 + 0.2, 0.01), "tarp", False),
    )
    for name, material, (c, r) in (("entry_pad", "blue", ENTRY), ("return_pad", "green", RETURN)):
        x, y = cell(c, r)
        add(lane, box(name, (x, y, 0.0075), (1.2, 1.2, 0.015), material, scan=True))
    return lane
