"""The lanes that are not the standard four-floor lane: Push/Pull Doors (printed pp. 56–59),
Avoid Holes/Posts (pp. 60–62) and Stair Debris | Pallet Climb (pp. 50–55)."""

from __future__ import annotations

import math

import numpy as np

from .geometry import (
    DECK,
    OSB,
    PALLET_TOP,
    Floor,
    Joint,
    Lane,
    Mesh,
    add,
    box,
    build_floor,
    combine,
    grid_pallet,
    pipe_run,
    purchased_pallet,
    railing,
    yaw_quaternion,
)

PAD = 0.015  # simulation start/return mats on the hall floor


def _pads(lane, entry, back, height=PAD):
    for name, material, (x, y) in (("entry_pad", "blue", entry), ("return_pad", "green", back)):
        add(lane, box(name, (x, y, height / 2), (1.2, 1.2, height), material, scan=True))


# --- Push/Pull Doors -------------------------------------------------------------------
STEP_HEIGHT = 0.10  # pallet-height floor pieces on double skids (pp. 58–59)
LEAF = (0.90, 2.00, 0.04)  # purchased 90 cm door
LEAF_MASS = 12.0
CLOSER_STIFFNESS = 2.0  # N m / rad, the hanging weight's closing pull (p. 57)
CLOSER_DAMPING = 2.0


def door_lane(options, origin):
    """The 2.4 m square door apparatus (p. 57). A framed 1.2 x 2.4 m wall with the door
    stands on the red base at the back half of the centre line, a thin 1.2 m panel continues
    it to the front, and yellow square steps and orange half steps floor the two halves at
    the same 10 cm. The robot enters the push side (x < 0) from the open front, pushes
    through to the pull side and leaves; `door_floor` removes the yellow, then also the
    orange, pieces to add the guide's stoops. The leaf is a rigid body on a sprung hinge."""
    lane = Lane(
        "doors",
        "Push/Pull Doors",
        origin,
        [],
        (origin[0] - 0.6, -1.85, PAD + 0.42),
        spawn_rotation=yaw_quaternion(math.radians(90.0)),
        subtitle="2.4 m × 2.4 m apparatus · push through, U-turn, pull back",
    )
    pieces = [("base", "red", (0.0, 0.6), (1.2, 1.2))]
    if options.door_floor in ("flat", "square"):
        pieces += [("half_step_west", "orange", (-0.9, 0.6), (0.6, 1.2)), ("half_step_east", "orange", (0.9, 0.6), (0.6, 1.2))]
    if options.door_floor == "flat":
        pieces += [("square_step_west", "yellow", (-0.6, -0.6), (1.2, 1.2)), ("square_step_east", "yellow", (0.6, -0.6), (1.2, 1.2))]
    for name, material, (x, y), (sx, sy) in pieces:
        add(lane, box(name, (x, y, STEP_HEIGHT / 2), (sx, sy, STEP_HEIGHT), material, scan=True))

    # Framed wall around the door: studs, plates, header, OSB skins, on the base.
    z0 = STEP_HEIGHT
    for i, y in enumerate((0.025, 0.125, 1.075, 1.175)):
        add(lane, box(f"stud{i}", (0.0, y, z0 + 1.15), (0.10, 0.05, 2.30)))
    add(lane, box("bottom_plate", (0.0, 0.6, z0 + 0.025), (0.10, 1.2, 0.05)))
    add(lane, box("top_plate", (0.0, 0.6, z0 + 2.325), (0.10, 1.2, 0.05)))
    add(lane, box("header", (0.0, 0.6, z0 + 2.125), (0.10, 0.9, 0.05)))
    for side, x in (("push", -0.0555), ("pull", 0.0555)):
        add(lane, box(f"skin_{side}_top", (x, 0.6, z0 + 2.25), (OSB, 1.2, 0.30), "osb"))
        add(lane, box(f"skin_{side}_front", (x, 0.075, z0 + 1.20), (OSB, 0.15, 2.40), "osb"))
        add(lane, box(f"skin_{side}_back", (x, 1.125, z0 + 1.20), (OSB, 0.15, 2.40), "osb"))
    # The rear thin panel continues the wall to the front, standing on the hall floor.
    add(lane, box("rear_panel", (0.0, -0.6, 0.6), (OSB, 1.2, 1.2), "osb"))
    # Door leaf with a lever handle each side, hinged on the back jamb, opening to the pull
    # side; closed it sits in the wall plane just above the bottom plate.
    # 1 cm of play to each jamb and a 1 cm undercut above the bottom plate, as a hung door
    # has: resting on the plate, its friction held the leaf shut against any push.
    leaf = box("door_leaf", (0.0, 0.6, z0 + 0.06 + LEAF[1] / 2), (LEAF[2], LEAF[0] - 0.02, LEAF[1]), "door")
    handles = [
        box(f"handle_{i}", (x, 0.25, z0 + 1.05), (0.02, 0.12, 0.02), "metal")
        for i, x in enumerate((-0.035, 0.035))
    ]
    door = combine([leaf] + handles, "door_leaf")
    door.material, door.dynamic, door.mass = "door", True, LEAF_MASS
    add(lane, door)
    lane.joints.append(
        Joint(
            "door_hinge",
            "stud2",
            "door_leaf",
            (origin[0], origin[1] + 1.05, z0 + 1.05),
            (0.0, 100.0),
            CLOSER_STIFFNESS,
            CLOSER_DAMPING,
        )
    )
    # Low slatted walls round the sides and back; the front is open.
    for name, (x, y), along in (("wall_west", (-1.2, 0.0), "y"), ("wall_east", (1.2, 0.0), "y"), ("wall_back", (0.0, 1.2), "x")):
        length = 2.4
        size = (length, 0.05, 0.10) if along == "x" else (0.05, length, 0.10)
        add(lane, box(f"{name}_bottom", (x, y, 0.05), size))
        add(lane, box(f"{name}_top", (x, y, 0.40), size))
        for i, t in enumerate(np.linspace(-1.1, 1.1, 8)):
            px, py = (x + t, y) if along == "x" else (x, y + t)
            add(lane, box(f"{name}_picket{i}", (px, py, 0.225), (0.10, OSB, 0.45) if along == "x" else (OSB, 0.10, 0.45), "osb"))
    _pads(lane, (-0.6, -1.85), (0.6, -1.85))
    return lane


# --- Avoid Holes/Posts -----------------------------------------------------------------
POST = (0.05, 0.10, 0.45)  # 2x4 posts, 45 cm (p. 62)
POST_MASS = 1.2
# The meandering route as straight runs of pallets laid end to end, one pallet's long side
# along the run (p. 62 shows an example; any meander is allowed). Post pairs straddle the
# second pallet of every run, 90 cm apart across it.
ROUTE = (("+x", 2), ("+y", 2), ("+x", 2), ("-y", 2), ("+x", 2))


def avoid_lane(options, origin):
    """Purchased pallets in a meander with pairs of loose 45 cm posts to steer between;
    knocking one over is the guide's 10 s penalty, so they are rigid bodies (pp. 61–62)."""
    directions = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}
    centres, posts = [], []
    at, previous = np.array([0.0, 0.0]), None
    for run, (heading, count) in enumerate(ROUTE):
        d = np.array(directions[heading], dtype=float)
        for k in range(count):
            if previous is None:
                centre = at.copy()
            elif k == 0:
                centre = previous + d * 1.1  # half the last pallet's width plus half a length
            else:
                centre = previous + d * 1.2
            centres.append((centre, heading))
            if k == 1:
                posts.append((centre, d))
            previous = centre
    shift = np.mean([c for c, _ in centres], axis=0)
    lane = Lane(
        "avoid",
        "Avoid Holes/Posts",
        origin,
        [],
        (origin[0] + centres[0][0][0] - shift[0] - 1.2, centres[0][0][1] - shift[1], PAD + 0.42),
        spawn_rotation=yaw_quaternion(0.0),
        subtitle="meandering pallet path · five post gates 90 cm apart",
    )
    for i, (centre, heading) in enumerate(centres):
        c = centre - shift
        purchased_pallet(lane, f"pallet_{i:02d}", tuple(c), 0.0, along=heading[1])
    for i, (centre, d) in enumerate(posts):
        c = centre - shift
        across = np.array([-d[1], d[0]])
        for j, side in enumerate((-1, 1)):
            p = c + across * 0.45 * side
            post = box(f"post_{2 * i + j:02d}", (p[0], p[1], 0.14 + POST[2] / 2), POST, "post")
            post.dynamic, post.mass = True, POST_MASS
            add(lane, post)
    last, heading = centres[-1]
    d = np.array(directions[heading])
    entry = tuple(centres[0][0] - shift + np.array([-1.2, 0.0]))
    back = tuple(last - shift + d * 1.2)
    _pads(lane, entry, back)
    return lane


# --- Stair Debris | Pallet Climb --------------------------------------------------------
RISE = 0.20  # 20 cm steps, five to the 100 cm landing (pp. 51–52)
TREAD = (0.90, 0.27, 0.05)  # width, depth (three 2x4 tops), thickness
LANDING = 1.00
STACKS = (7, 4, 1)  # grid pallets per stack east of the landing: 30 cm plateaus down to the floor


def _rolled_box(name, center, size, roll, material="wood"):
    """A box rotated about the x axis through its centre; positive roll raises its +y end."""
    mesh = box(name, (0.0, 0.0, 0.0), size, material)
    c, s = math.cos(roll), math.sin(roll)
    v = mesh.vertices.copy()
    mesh.vertices[:, 1] = c * v[:, 1] - s * v[:, 2]
    mesh.vertices[:, 2] = s * v[:, 1] + c * v[:, 2]
    mesh.vertices += np.asarray(center)
    return mesh


def _yawed_box(name, center, size, yaw, material="wood"):
    mesh = box(name, (0.0, 0.0, 0.0), size, material)
    c, s = math.cos(yaw), math.sin(yaw)
    v = mesh.vertices.copy()
    mesh.vertices[:, 0] = c * v[:, 0] - s * v[:, 1]
    mesh.vertices[:, 1] = s * v[:, 0] + c * v[:, 1]
    mesh.vertices += np.asarray(center)
    return mesh


def stair_lane(options, origin):
    """An elevated 100 cm landing (p. 53) reached by the adjustable stair from the south
    (pp. 51–52) and left down three pallet-and-pipe plateaus to the east (p. 54). The entry
    floor and its railings sit south of the stair; belay arches are not modelled."""
    angle = math.radians(options.stair_angle)
    run = RISE / math.tan(angle)
    entry = Floor("entry", (-0.6, -2.2), (1.2, 2.4))
    lane = Lane(
        "stairs",
        "Stair Debris | Pallet Climb",
        origin,
        [entry],
        (origin[0] - 0.6, -4.05, DECK + 0.42),
        spawn_rotation=yaw_quaternion(math.radians(90.0)),
        subtitle=f"{options.stair_angle:g}° stair to a 1.0 m landing · pallet climb down",
    )
    build_floor(lane, entry)
    for i in range(2):
        railing(lane, entry, (-0.6, -1.2 + i * 1.2), (-0.6, i * 1.2), f"entry_west_{i}")
        railing(lane, entry, (0.6, -1.2 + i * 1.2), (0.6, i * 1.2), f"entry_east_{i}")

    # Stair: treads climbing north from y = -1.0 to the landing edge at y = 0.
    y_bottom = -5 * run
    for k in range(1, 6):
        y_front = y_bottom + (k - 1) * run
        top = RISE * k
        add(lane, box(f"tread{k}", (-0.6, y_front + TREAD[1] / 2, top - TREAD[2] / 2), (TREAD[0], TREAD[1], TREAD[2]), scan=True))
        for i, dy in enumerate((0.05, TREAD[1] - 0.05)):
            add(lane, box(f"tread{k}_bottom{i}", (-0.6, y_front + dy, top - TREAD[2] - 0.045), (0.75, 0.05, 0.09)))
    length = max(1.2, 5 * run + 0.07)
    for i, x in enumerate((-0.6 - 0.45 - OSB / 2, -0.6 + 0.45 + OSB / 2)):
        add(lane, box(f"stair_wall{i}", (x, y_bottom + length / 2, 0.5), (OSB, length, 1.0), "osb"))
        # Diagonal 2x4 railings above the walls, parallel to the stair (p. 54).
        add(lane, _rolled_box(f"stair_rail{i}", (x, y_bottom + 5 * run / 2, 1.5), (0.05, 1.2, 0.09), angle))
    # Optional hinged debris rails laid across the treads, colour-coded as on p. 51.
    for k in range(options.stair_debris):
        tread_index, material = ((1, "yellow"), (3, "orange"), (4, "red"))[k]
        y_front = y_bottom + (tread_index - 1) * run
        yaw = math.radians(25.0 if k % 2 == 0 else -25.0)
        add(lane, _yawed_box(f"debris{k}", (-0.6, y_front + 0.12, RISE * tread_index + 0.045), (0.75, 0.05, 0.09), yaw, material))

    # Landing: 2.4 x 1.2 framed floor on legs, top at 1.0 m, thin OSB on the west side only
    # so the pallet side and the drive-under passage stay open.
    add(lane, box("landing_osb", (0.0, 0.6, LANDING - OSB / 2), (2.4, 1.2, OSB), "osb", scan=True))
    for i, (x, y, sx, sy) in enumerate(((0.0, 0.025, 2.4, 0.05), (0.0, 1.175, 2.4, 0.05), (-1.175, 0.6, 0.05, 1.1), (1.175, 0.6, 0.05, 1.1))):
        add(lane, box(f"landing_frame{i}", (x, y, LANDING - OSB - 0.05), (sx, sy, 0.10)))
    for i, x in enumerate((-1.15, 0.0, 1.15)):
        for j, y in enumerate((0.05, 1.15)):
            add(lane, box(f"landing_leg{i}{j}", (x, y, (LANDING - OSB - 0.10) / 2), (0.10, 0.10, LANDING - OSB - 0.10)))
    add(lane, box("landing_west_panel", (-1.2 - OSB / 2, 0.6, 0.55), (OSB, 1.2, 0.9), "osb"))
    for i in range(2):
        railing(lane, None, (-1.2 + i * 1.2, 1.2), (i * 1.2, 1.2), f"landing_north_{i}", z=LANDING)
    railing(lane, None, (-1.2, 0.0), (-1.2, 1.2), "landing_west", z=LANDING)
    railing(lane, None, (0.0, 0.0), (1.2, 0.0), "landing_south_east", z=LANDING)

    # Pallet climb: stacks of grid pallets stepping down 30 cm at a time, three pipes on each
    # lower surface against the higher face, one pipe on the floor against the last stack.
    for i, count in enumerate(STACKS):
        x = 1.8 + 1.2 * i
        z = 0.0
        for k in range(count):
            z = grid_pallet(lane, None, f"stack{i}_pallet{k}", (x, 0.6), z)
        pipe_run(lane, None, f"stack{i}_pipes", (x - 0.6 + 0.051, 0.1), (0.0, 1.0), 1.0, z, count=3)
    pipe_run(lane, None, "floor_pipe", (1.8 + 1.2 * len(STACKS) - 0.6 + 0.051, 0.1), (0.0, 1.0), 1.0, 0.0, count=1)
    for i in range(len(STACKS)):
        railing(lane, None, (1.2 + 1.2 * i, 0.0), (2.4 + 1.2 * i, 0.0), f"climb_south_{i}")
        railing(lane, None, (1.2 + 1.2 * i, 1.2), (2.4 + 1.2 * i, 1.2), f"climb_north_{i}")
    add(lane, box("entry_pad", (-0.6, -4.05, DECK / 2), (1.2, 1.2, DECK), "blue", scan=True))
    add(lane, box("return_pad", (1.8 + 1.2 * len(STACKS), 0.6, PAD / 2), (1.2, 1.2, PAD), "green", scan=True))
    return lane
