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
    cylinder,
    grid_pallet,
    pipe_run,
    purchased_pallet,
    prism,
    railing,
    ring,
    tube,
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


def _door_handle(side, height):
    """Round rose and a bent tubular lever, mirrored onto either face of the door.

    The reference has a shallow bevel on the rose, a short projecting neck and a
    smooth 90-degree elbow into the horizontal grip, with a softened end cap.
    """
    segments = 48
    theta = np.arange(segments) * 2 * math.pi / segments

    def skin(rings, name):
        vertices = np.concatenate(rings)
        faces = []
        for row in range(len(rings) - 1):
            for i in range(segments):
                j = (i + 1) % segments
                a, b = row * segments + i, row * segments + j
                faces.extend(((a, b, b + segments), (a, b + segments, a + segments)))
        end = (len(rings) - 1) * segments
        for i in range(1, segments - 1):
            faces.extend(((0, i + 1, i), (end, end + i, end + i + 1)))
        return Mesh(name, vertices, np.asarray(faces, dtype=np.int32), "handle_metal")

    # 52 mm rose, 6.5 mm proud of the door, with a bevel at each rim.
    rose = skin([
        np.column_stack((np.full(segments, x), radius * np.cos(theta), radius * np.sin(theta)))
        for x, radius in ((0.0, 0.025), (0.0015, 0.026), (0.0045, 0.026), (0.0065, 0.024))
    ], "rose")
    # Sweep an 18 mm tube along the neck, elbow and straight lever. Each circle
    # stays normal to the path, so the elbow meets both straight sections smoothly.
    path = [(0.005, 0.0, 0.0, 0.009)]
    for angle in np.linspace(0, math.pi / 2, 13):
        path.append((0.030 + 0.014 * math.sin(angle), 0.014 * (1 - math.cos(angle)), angle, 0.009))
    path += [(0.044, 0.137, math.pi / 2, 0.009),
             (0.044, 0.139, math.pi / 2, 0.0085), (0.044, 0.140, math.pi / 2, 0.007)]
    grip = skin([
        np.column_stack((x + radius * np.sin(theta) * math.sin(angle),
                         y - radius * np.sin(theta) * math.cos(angle),
                         radius * np.cos(theta)))
        for x, y, angle, radius in path
    ], "lever")
    handle = combine([rose, grip], f"handle_{'push' if side < 0 else 'pull'}")
    handle.material = "handle_metal"
    handle.vertices[:, 0] = side * (LEAF[2] / 2 + handle.vertices[:, 0])
    handle.vertices[:, 1] += 0.25
    handle.vertices[:, 2] += height
    if side < 0:
        handle.faces = handle.faces[:, ::-1]  # reflection must preserve outward winding
    return handle


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
    handles = [_door_handle(side, z0 + 1.05) for side in (-1, 1)]
    door = combine([leaf] + handles, "door_leaf")
    door.material, door.dynamic, door.mass = "door", True, LEAF_MASS
    # Keep both handles on the moving leaf, while giving their faces a metal finish.
    door.material_faces["handle_metal"] = np.arange(len(leaf.faces), len(door.faces), dtype=np.int32)
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


def _stair_rail(name, x, end_y, end_z, angle):
    """120 cm 2x4 with parallel vertical mitres (p. 54), mounted flat to the wall."""
    span = 1.2 * math.cos(angle)
    # Shear a prism to cut the ends vertically while retaining a 10 cm face width
    # perpendicular to the incline. Unlike a rolled box, neither end protrudes.
    mesh = box(name, (x, end_y - span / 2, end_z), (0.05, span, 0.10 / math.cos(angle)))
    mesh.vertices[:, 2] += (mesh.vertices[:, 1] - end_y) * math.tan(angle)
    return mesh


def _floor_wall_beam(name, floor, wall, length, material):
    """A rectangular 2x4 leaning from a horizontal floor onto a vertical wall.

    `floor` supplies the low end's plane and `wall` the high end's plane; their XY
    separation supplies the direction. Seat the outside corners against both planes
    after rotating the lumber, without shearing it or burying its ends in a support.
    """
    direction = np.asarray(wall, dtype=float)[:2] - np.asarray(floor, dtype=float)[:2]
    direction /= np.linalg.norm(direction)
    rise = wall[2] - floor[2]
    across = math.sqrt(length**2 - rise**2)
    along = np.r_[direction * across, rise] / length
    depth = np.array([-direction[1], direction[0], 0.0])
    face = np.cross(along, depth)
    mesh = box(name, (0, 0, 0), (length, 0.05, 0.09), material, scan=True)
    mesh.vertices = mesh.vertices @ np.stack((along, depth, face))
    mesh.vertices[:, 2] += floor[2] - mesh.vertices[:, 2].min()
    # Use the wall as the XY origin, then pull the whole beam back until its
    # high corner just touches the wall plane along the leaning direction.
    mesh.vertices[:, :2] += np.asarray(wall)[:2] - direction * np.max(mesh.vertices[:, :2] @ direction)
    return mesh


def _belay(lane, name, center, width, along, posts):
    """Grounded 240 cm uprights, a drilled crossbar and an unloaded visual rope (p. 54)."""
    x, y = center
    for i, (px, py) in enumerate(posts):
        size = (0.05, 0.10, 2.4) if along == "x" else (0.10, 0.05, 2.4)
        add(lane, box(f"{name}_post{i}", (px, py, 1.2), size))

    # Four solid pieces leave an oversized central rope opening through the beam.
    hole = 0.03
    parts = []
    for side in (-1, 1):
        offset = side * (width + hole) / 4
        parts.append(box("end", (offset, 0, 0), ((width - hole) / 2, 0.05, 0.10)))
        parts.append(box("rim", (0, side * (0.05 + hole) / 4, 0), (hole, (0.05 - hole) / 2, 0.10)))
    top = combine(parts, f"{name}_top")
    if along == "y":
        top.vertices[:, :2] = top.vertices[:, :2] @ np.array([[0, 1], [-1, 0]])
    top.vertices += (x, y, 2.35)
    add(lane, top)
    # These are the hanging, unused belays. They exert no force on the robot.
    rope = prism(f"{name}_rope", ring(0.006) + (x, y), 1.55, 2.4, "orange")
    rope.collision = False
    add(lane, rope)
    eye = tube(f"{name}_rope_eye", (x, y - 0.006, 1.53), (0, 1), 0.012, 0.025, 0.018, "metal")
    eye.collision = False
    add(lane, eye)


def _stair_debris(lane, index, run, y_bottom):
    """75 cm beams rising from a single tread to alternating side walls (pp. 51–52)."""
    tread, material = ((1, "yellow"), (3, "orange"), (5, "red"))[index]
    side = 1 if index % 2 == 0 else -1
    front = y_bottom + (tread - 1) * run
    # The top beam meets the belay upright above the 1 m OSB wall. All beams lie
    # across one tread (constant Y), with a 35 cm rise from foot to wall contact.
    y = front + min(run, TREAD[1]) / 2 if tread < 5 else -0.05
    wall_x = -0.6 + side * (0.45 if tread < 5 else 0.45 + OSB)
    contact_z = RISE * tread + 0.35
    add(lane, _floor_wall_beam(
        f"debris{index}", (-0.6, y, RISE * tread), (wall_x, y, contact_z),
        0.75, material,
    ))
    add(lane, box(
        f"stair_hinge{index}_leaf", (wall_x - side * 0.005, y, contact_z),
        (0.01, 0.07, 0.08), "metal",
    ))
    add(lane, cylinder(
        f"stair_hinge{index}_pin", (wall_x - side * 0.012, y - 0.035, contact_z),
        (0, 1), 0.07, 0.012, "metal",
    ))


def _landing_braces(lane):
    """Red floor-to-post diagonals crossing both routes at the landing centre (p. 54).

    In plan these run front-to-back along X=0, perpendicular to the route from the
    stair half to the pallet half, as in the guide's top view. One is above the
    landing; the other is below it, across the drive-under passage.
    """
    add(lane, box("landing_brace_post", (0.0, 0.025, 0.95), (0.10, 0.10, 1.9), "red"))
    for name, floor in (("under", 0.0), ("over", LANDING)):
        add(lane, _floor_wall_beam(
            f"landing_brace_{name}", (0.0, 1.15, floor), (0.0, 0.075, floor + 0.75),
            1.25, "red",
        ))


def stair_lane(options, origin, *, debris=False):
    """An elevated 100 cm landing (p. 53) reached by the adjustable stair from the south
    (pp. 51–52) and left down three pallet-and-pipe plateaus to the east (p. 54). The entry
    floor follows the foot of the stair as its incline changes. Four mitred side rails
    and two grounded safety belay arches follow p. 54 (PDF page 53)."""
    angle = math.radians(options.stair_angle)
    run = RISE / math.tan(angle)
    y_bottom = -5 * run
    entry = Floor("entry", (-0.6, y_bottom - 1.2), (1.2, 2.4))
    lane = Lane(
        "stair_debris" if debris else "stairs",
        "Stair Debris | Pallet Climb" if debris else "Clear Stairs | Pallet Climb",
        origin,
        [entry],
        (origin[0] - 0.6, origin[1] + y_bottom - 3.05, origin[2] + DECK + 0.42),
        spawn_rotation=yaw_quaternion(math.radians(90.0)),
        subtitle=(f"{options.stair_angle:g}° stair to a 1.0 m landing · pallet climb down · "
                  + (f"{options.stair_debris} debris rails and red landing braces" if debris else "clear stairs")),
        arena="Stairs | Pallet Climb",
        setting="Debris" if debris else "Clear",
    )
    build_floor(lane, entry)
    for i in range(2):
        railing(lane, entry, (-0.6, -1.2 + i * 1.2), (-0.6, i * 1.2), f"entry_west_{i}")
        railing(lane, entry, (0.6, -1.2 + i * 1.2), (0.6, i * 1.2), f"entry_east_{i}")

    # Stair: treads climbing north from y = -1.0 to the landing edge at y = 0.
    for k in range(1, 6):
        y_front = y_bottom + (k - 1) * run
        top = RISE * k
        add(lane, box(f"tread{k}", (-0.6, y_front + TREAD[1] / 2, top - TREAD[2] / 2), (TREAD[0], TREAD[1], TREAD[2]), scan=True))
        for i, dy in enumerate((0.05, TREAD[1] - 0.05)):
            add(lane, box(f"tread{k}_bottom{i}", (-0.6, y_front + dy, top - TREAD[2] - 0.045), (0.75, 0.05, 0.09)))
    length = max(1.2, 5 * run + 0.07)
    for i, x in enumerate((-0.6 - 0.45 - OSB / 2, -0.6 + 0.45 + OSB / 2)):
        add(lane, box(f"stair_wall{i}", (x, y_bottom + length / 2, 0.5), (OSB, length, 1.0), "osb"))
        # Two 120 cm rails per side, flush to the OUTER wall face. Their low ends
        # fasten to OSB and their high ends fasten to the grounded belay uprights.
        rail_x = x + (-1 if i == 0 else 1) * (OSB / 2 + 0.025)
        for j, top in enumerate((1.35, 1.70)):
            add(lane, _stair_rail(f"stair_rail{i}_{j}", rail_x, -0.05, top, angle))
    _belay(lane, "stair_belay", (-0.6, -0.05), 1.0, "x",
           [(-0.6 + side * (0.45 + OSB + 0.025), -0.05) for side in (-1, 1)])
    _belay(lane, "pallet_belay", (1.225, 0.6), 1.4, "y", [(1.225, -0.075), (1.225, 1.275)])
    for k in range(options.stair_debris if debris else 0):
        _stair_debris(lane, k, run, y_bottom)

    # Landing: 2.4 x 1.2 framed floor on legs, top at 1.0 m, thin OSB on the west side only
    # so the pallet side and the drive-under passage stay open.
    add(lane, box("landing_osb", (0.0, 0.6, LANDING - OSB / 2), (2.4, 1.2, OSB), "osb", scan=True))
    for i, (x, y, sx, sy) in enumerate(((0.0, 0.025, 2.4, 0.05), (0.0, 1.175, 2.4, 0.05), (-1.175, 0.6, 0.05, 1.1), (1.175, 0.6, 0.05, 1.1))):
        add(lane, box(f"landing_frame{i}", (x, y, LANDING - OSB - 0.05), (sx, sy, 0.10)))
    for i, x in enumerate(np.arange(-0.8, 0.81, 0.4)):
        add(lane, box(f"landing_joist{i}", (x, 0.6, LANDING - OSB - 0.05), (0.05, 1.1, 0.10)))
    for i, x in enumerate((-1.15, 0.0, 1.15)):
        for j, y in enumerate((0.05, 1.15)):
            add(lane, box(f"landing_leg{i}{j}", (x, y, (LANDING - OSB - 0.10) / 2), (0.10, 0.10, LANDING - OSB - 0.10)))
    add(lane, box("landing_west_panel", (-1.2 - OSB / 2, 0.6, 0.55), (OSB, 1.2, 0.9), "osb"))
    for i in range(2):
        railing(lane, None, (-1.2 + i * 1.2, 1.2), (i * 1.2, 1.2), f"landing_north_{i}", z=LANDING)
    railing(lane, None, (-1.2, 0.0), (-1.2, 1.2), "landing_west", z=LANDING)
    railing(lane, None, (0.0, 0.0), (1.2, 0.0), "landing_south_east", z=LANDING)
    if debris:
        _landing_braces(lane)

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
        railing(lane, None, (1.2 + 1.2 * i, 0.0), (2.4 + 1.2 * i, 0.0), f"climb_south_{i}", z=STACKS[i] * PALLET_TOP)
        railing(lane, None, (1.2 + 1.2 * i, 1.2), (2.4 + 1.2 * i, 1.2), f"climb_north_{i}", z=STACKS[i] * PALLET_TOP)
    add(lane, box("entry_pad", (-0.6, y_bottom - 3.05, DECK / 2), (1.2, 1.2, DECK), "blue", scan=True))
    add(lane, box("return_pad", (1.8 + 1.2 * len(STACKS), 0.6, PAD / 2), (1.2, 1.2, PAD), "green", scan=True))
    return lane
