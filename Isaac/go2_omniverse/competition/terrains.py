"""Terrains on the guide's standard 4.8 x 2.4 m lane, beyond gravel and K-Rails.

Half-Cubic Stepfields (printed pp. 33–35), Pitch/Roll Ramps (pp. 36–41) and Pallets &
Pipes (pp. 45–49) receive a lane that already carries the four floors and 14 railings from
`geometry.standard_lane` and add the terrain on top, in floor coordinates so it tilts with
the floors in the sloped settings. Center in Alleys (pp. 42–44) arranges its floors differently and
builds its own lane.
"""

from __future__ import annotations

import math

import numpy as np

from .geometry import (
    DECK,
    FLAT,
    OBSTACLES,
    OSB,
    PALLET_TOP,
    Floor,
    Joint,
    Lane,
    add,
    box,
    build_floor,
    grid_pallet,
    orient3,
    pipe_run,
    prism,
    railing,
    ring,
    wedge,
)

# --- Half-Cubic Stepfields ---------------------------------------------------------
STEP_BASE = 0.594  # thin OSB quad base, four across the floor width (p. 35)
PLATEAU = 0.297  # thick OSB step top
THICK_OSB = 0.018


def step_height(gx, gy):
    """Plateau height at cell (gx, gy) of the lane's 30 cm grid, counted from its south-west
    corner: 30 cm on a ridge, 15 cm beside one, zero for the bare base.

    "Two QUAD-STEPS on 60 cm OSB panels form diagonal hills. SINGLE-STEPS on separate 60 cm
    OSB bases fill in corners" (p. 33). So each 1.2 m block of the lane is two quad-steps on
    one diagonal, each with 30 cm plateaus on the ridge and 15 cm ones either side of it, and
    two single-steps whose one 15 cm plateau sits in the corner against the ridge: the cut
    list's 80 tops, 32 on long legs and 48 on short (p. 35). Neighbouring blocks' ridges run
    opposite ways, so they zigzag between the railings and the centre line and cross in a
    2 x 2 of 30 cm plateaus at the lane centre, where p. 34's renders put the omni task. Every
    side of a 30 cm plateau meets a 15 cm one, or another 30 cm one where two ridges meet,
    never the bare base.
    """
    x, y = gx % 4, gy % 4
    rising = (gx // 4 + gy // 4) % 2  # ridge runs south-west to north-east
    off = abs(x - y) if rising else abs(x + y - 3)
    return (0.30, 0.15, 0.0, 0.0)[off]


def plateau(lane, floor, name, center, z_base, height):
    """A 30 cm thick-OSB top on four 2x4 legs (p. 35), its top at z_base + height. Tops are
    coloured as the guide's renders colour them: 15 cm yellow, 30 cm orange."""
    x, y = center
    top = z_base + height
    add(
        lane,
        box(
            f"{name}_top",
            (x, y, top - THICK_OSB / 2),
            (PLATEAU, PLATEAU, THICK_OSB),
            "orange" if height > 0.2 else "yellow",
            scan=True,
        ),
        floor,
    )
    leg = height - THICK_OSB
    for i, (dx, dy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        add(
            lane,
            box(f"{name}_leg{i}", (x + dx * 0.11, y + dy * 0.09, z_base + leg / 2), (0.05, 0.10, leg)),
            floor,
        )


def stepfield_lane(lane, options):
    """16 quad-steps and 16 single-steps on 60 cm bases, in the diagonal zigzag of
    `step_height` over the four floors."""
    for floor in lane.floors:
        sx, sy = floor.size
        cols, rows = int(round(sx / 0.6)), int(round(sy / 0.6))
        for i in range(cols):
            for j in range(rows):
                cx, cy = -sx / 2 + 0.3 + 0.6 * i, -sy / 2 + 0.3 + 0.6 * j
                name = f"{floor.name}_step{i}{j}"
                add(
                    lane,
                    box(f"{name}_base", (cx, cy, OSB / 2), (STEP_BASE, STEP_BASE, OSB), "osb", scan=True),
                    floor,
                )
                for a in range(2):
                    for b in range(2):
                        x, y = cx - 0.15 + 0.3 * b, cy - 0.15 + 0.3 * a
                        # Cell on the 4.8 x 2.4 m lane's grid, whose first cell centre is at
                        # (-2.25, -1.05); the end floors' 5 cm offsets round away.
                        gx = round((floor.center[0] + x + 2.25) / 0.3)
                        gy = round((floor.center[1] + y + 1.05) / 0.3)
                        height = step_height(gx, gy)
                        if height:
                            plateau(lane, floor, f"{name}_p{a}{b}", (x, y), OSB, height)


# --- Pitch/Roll Ramps ----------------------------------------------------------------
RAMP = 0.594  # thick OSB top, less than half the floor width (p. 38)
RAMP_HEIGHT = 0.15  # 15 cm fronts give the 15 degree slope
_REVERSE = {"+x": "-x", "-x": "+x", "+y": "-y", "-y": "+y"}
# Rotating slip disks (pp. 40–41): 3 mm x 50 cm thin round disks, one centred on each ramp on a
# loose bolt. Modelled 5 mm thick: PhysX will not cook a 3 mm convex hull for the GPU and falls
# back to CPU collision for it. The mass is a 3 mm disk's, of wood at an assumed 650 kg/m^3.
DISK_RADIUS, DISK_THICKNESS = 0.25, 0.005
DISK_MASS = 650.0 * math.pi * DISK_RADIUS**2 * 0.003
SEAT = 0.002  # the OSB under the disk, carrying the disk-on-ramp friction (geometry.FRICTION)
DISK_SIDES = 24  # within 2 mm of round, and well inside PhysX's 64-vertex limit for a GPU hull
BOLT_PLAY = 0.005  # how far the loose bolt lets the disk lift off its seat, either way
BOLT_DAMPING = 0.01  # N m s / rad: the bolt turns freely; the seat's friction resists


def _slip_disk(lane, floor, k, bounds, rise):
    """A slip disk on the ramp over `bounds` rising towards `rise`: a thin orange disk with a
    black line from its centre to its rim that shows how far it has turned (p. 40), on a
    seat of its ramp's OSB and under a fender washer. The disk is a rigid body on a loose
    bolt: it turns about the ramp's normal and may lift a few millimetres, but cannot slide
    or tilt, so a foot pushing across its radius turns it as soon as the push beats the seat's
    friction. Pushing along the radius does not turn it."""
    x0, y0, x1, y1 = bounds
    slope = (RAMP_HEIGHT - OSB) / RAMP
    ux, uy = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}[rise]
    normal = np.array([-slope * ux, -slope * uy, 1.0])
    normal /= np.linalg.norm(normal)
    # The ramp's top at its centre: an OSB-thin low edge climbing to RAMP_HEIGHT (`wedge`).
    centre = np.array([(x0 + x1) / 2, (y0 + y1) / 2, OSB + OSB + (RAMP_HEIGHT - OSB) / 2])
    seat = orient3(prism(f"slip_seat_{k:02d}", ring(DISK_RADIUS - 0.01, DISK_SIDES), 0.0, SEAT, "slip_seat"), centre, normal)
    add(lane, seat, floor)
    disk = prism(f"slip_disk_{k:02d}", ring(DISK_RADIUS, DISK_SIDES), 0.0, DISK_THICKNESS, "slip_disk")
    disk.uv = disk.vertices[:, :2] / (2 * DISK_RADIUS) + 0.5
    disk = orient3(disk, centre + SEAT * normal, normal)
    disk.dynamic, disk.mass, disk.contact_offset = True, DISK_MASS, 0.002
    add(lane, disk, floor)
    washer = orient3(prism(f"disk_washer_{k:02d}", ring(0.02), 0.0, 0.002, "metal"),
                     centre + (SEAT + DISK_THICKNESS) * normal, normal)
    washer.collision = False
    add(lane, washer, floor)
    pivot = centre + (SEAT + DISK_THICKNESS / 2) * normal
    ends = floor.transform([pivot, pivot + normal]) + np.asarray(lane.origin)
    lane.joints.append(Joint(
        f"disk_bolt_{k:02d}", "", disk.name, tuple(ends[0]), None, 0.0, BOLT_DAMPING,
        axis=tuple(ends[1] - ends[0]), lift=BOLT_PLAY,
    ))


def ramp_lane(lane, options):
    """Eight 1.2 m elements of four ramps on a half OSB panel, peaks and valleys alternating
    (pp. 38–39). A peak has clockwise up-slopes meeting at the centre: the NW ramp rises
    east, NE rises south, SE rises west, SW rises north; a valley is the reverse. Under
    Additional Obstacles every ramp carries a rotating slip disk (pp. 37, 40)."""
    disks = lane.setting == OBSTACLES
    n = disk = 0
    quadrants = (((-1, 1), "+x"), ((1, 1), "-y"), ((1, -1), "-x"), ((-1, -1), "+y"))
    for floor in lane.floors:
        sx, sy = floor.size
        for i in range(int(round(sx / 1.2))):
            for j in range(int(round(sy / 1.2))):
                cx, cy = -sx / 2 + 0.6 + 1.2 * i, -sy / 2 + 0.6 + 1.2 * j
                name = f"{floor.name}_ramps{i}{j}"
                add(lane, box(f"{name}_backing", (cx, cy, OSB / 2), (1.2, 1.2, OSB), "osb", scan=True), floor)
                peak = (i + j + n) % 2 == 0
                for k, ((qx, qy), rise) in enumerate(quadrants):
                    x0 = cx + 0.003 if qx > 0 else cx - 0.003 - RAMP
                    y0 = cy + 0.003 if qy > 0 else cy - 0.003 - RAMP
                    bounds = (x0, y0, x0 + RAMP, y0 + RAMP)
                    rise_k = rise if peak else _REVERSE[rise]
                    add(lane, wedge(f"{name}_r{k}", bounds, rise_k, OSB, RAMP_HEIGHT), floor)
                    if disks:
                        _slip_disk(lane, floor, disk, bounds, rise_k)
                        disk += 1
        n += 1
    if disks:
        lane.subtitle += f" · {disk} rotating slip disks"


# --- Center in Alleys ----------------------------------------------------------------
PANEL = (1.2, 0.8)  # sliding thin OSB, 120 cm along the divider, 80 cm tall (p. 44)
# Divider x, the side its doorway is on, then the floors carrying its railing and its panel.
# p. 43's arrows: down the first hallway, up the second, down the third, out along the fourth.
DIVIDERS = ((-1.2, "south", "second", "blue_end"), (0.0, "north", "second", "second"), (1.2, "south", "far_north", "far_south"))


def alleys_lane(options, origin, setting=FLAT):
    """Center in Alleys as the guide lays it out (pp. 43–44).

    Unlike the other lanes, the two flat floors run across the lane at the door end and the
    two tilt-up floors lie side by side at the far end, so under the SLOPES setting both rise
    15 degrees toward the far end and every hallway crossing them has a side slope. Twelve railings
    close the perimeter, the gate swung open at the north-west corner. Three dividers split
    the lane into four 1.2 m hallways; each is a railing on one half of the width plus a
    sliding OSB panel clamped to it, leaving a doorway of `alley_width` at the other side.
    The doorways alternate south, north, south, so the route snakes through all four."""
    angle = math.radians(15) if setting != FLAT else 0.0
    w = options.alley_width
    # A tilted floor keeps its 2.4 m length, so its plan shrinks to 2.4 cos(angle). Centre it
    # so the low edge stays hinged to the flat floor at x = 0 rather than opening a slot.
    far_x = 1.2 * math.cos(angle)
    floors = [
        Floor("blue_end", (-1.8, 0.0), (1.2, 2.4)),
        Floor("second", (-0.6, 0.0), (1.2, 2.4)),
        Floor("far_north", (far_x, 0.6), (2.4, 1.2), angle),
        Floor("far_south", (far_x, -0.6), (2.4, 1.2), angle),
    ]
    blue, second, far_north, far_south = floors
    lane = Lane(
        "alleys" if setting == FLAT else "alleys_slopes",
        # Its far floors rise together (p. 44), not in opposite directions: "SLOPED 15°" (p. 43).
        "Center in Alleys" + (" · Sloped 15°" if setting != FLAT else ""),
        origin,
        floors,
        (origin[0] - 1.8, 1.85, DECK + 0.42),
        subtitle=(f"4.8 m × 2.4 m lane · three dividers · {w * 100:g} cm doorways"
                  + (" · far floors rise 15°" if angle else "")),
        arena="Center in Alleys",
        setting=setting,
    )
    for floor in floors:
        build_floor(lane, floor)

    # Perimeter.
    for i in range(2):
        railing(lane, blue, (-0.6, -1.2 + i * 1.2), (-0.6, i * 1.2), f"blue_end_outer_{i}")
    railing(lane, blue, (-0.6, -1.2), (0.6, -1.2), "blue_end_south")
    # Gate swings outward around the outer north corner, clear of the entry.
    railing(lane, blue, (-0.6, 1.2), (-0.6, 2.4), "entry_gate_open")
    railing(lane, second, (-0.6, -1.2), (0.6, -1.2), "second_south")
    railing(lane, second, (-0.6, 1.2), (0.6, 1.2), "second_north")
    for floor, y in ((far_north, 0.6), (far_south, -0.6)):
        for i in range(2):
            railing(lane, floor, (-1.2 + i * 1.2, y), (i * 1.2, y), f"{floor.name}_outer_{i}")
        railing(lane, floor, (1.2, -0.6), (1.2, 0.6), f"{floor.name}_end")
        if floor.pitch:
            # Tilt-up legs under the high end (p. 25), as in the standard lane.
            top = floor.transform([[1.0, 0, -OSB]])[0]
            for j, y_leg in enumerate((floor.center[1] - 0.45, floor.center[1] + 0.45)):
                add(lane, box(f"{floor.name}_leg_{j}", (top[0], y_leg, top[2] / 2), (0.1, 0.05, top[2])))
            add(lane, box(f"{floor.name}_leg_brace", (top[0], floor.center[1], top[2] - 0.15), (0.011, 1.0, 0.3), "osb"))

    # Dividers. The panel is clamped on the side the robot arrives from.
    named = {floor.name: floor for floor in floors}

    def local(floor, x, y):
        return (x - floor.center[0], y - floor.center[1])

    for k, (x, doorway, rail_name, panel_name) in enumerate(DIVIDERS):
        rail_floor, panel_floor = named[rail_name], named[panel_name]
        if doorway == "south":
            rail_span, panel_span = (0.0, 1.2), (w - PANEL[0], w)
        else:
            rail_span, panel_span = (-1.2, 0.0), (-w, PANEL[0] - w)
        railing(
            lane,
            rail_floor,
            local(rail_floor, x, rail_span[0]),
            local(rail_floor, x, rail_span[1]),
            f"divider{k}",
        )
        px, py = local(panel_floor, x - 0.035, (panel_span[0] + panel_span[1]) / 2)
        add(
            lane,
            box(f"divider{k}_panel", (px, py, PANEL[1] / 2), (OSB, PANEL[0], PANEL[1]), "osb"),
            panel_floor,
        )

    add(lane, box("entry_pad", (-1.8, 1.85, DECK / 2), (1.2, 1.2, DECK), "blue", scan=True))
    # Colour-coded start/finish strips along the south railing tops.
    add(lane, box("blue_end_marker", (0, -1.17, 0.91), (1.0, 0.015, 0.025), "blue", False), blue)
    add(lane, box("far_south_marker", (0.6, -0.57, 0.91), (1.0, 0.015, 0.025), "green", False), far_south)
    return lane


# --- Pallets & Pipes -----------------------------------------------------------------
RAISED = {("lower", 0), ("upper", 1)}  # cells stacked two pallets high, 20 cm (p. 46)


def pallet_lane(lane, options):
    """A fabricated grid pallet on every 1.2 m cell, two cells stacked to 20 cm, and a pipe
    on the lower surface against each raised face along the lane so climbing them means
    stepping on a pipe that spins (pp. 46–49). Grid side up, the prelims setting."""
    blue, lower, upper, green = lane.floors

    def local(floor, x, y):
        return (x - floor.center[0], y - floor.center[1])

    for floor in lane.floors:
        sx, sy = floor.size
        for i in range(int(round(sx / 1.2))):
            for j in range(int(round(sy / 1.2))):
                cx, cy = -sx / 2 + 0.6 + 1.2 * i, -sy / 2 + 0.6 + 1.2 * j
                name = f"{floor.name}_pallet{i}{j}"
                top = grid_pallet(lane, floor, name, (cx, cy), 0.0)
                if (floor.name, i) in RAISED:
                    grid_pallet(lane, floor, f"{name}_upper", (cx, cy), top)
    # Raised lower-0 spans x in [-1.2, 0], y < 0; raised upper-1 spans x in [0, 1.2], y > 0.
    for name, floor, x, y0 in (
        ("pipe_lower_west", blue, -1.2 - 0.051, -1.1),
        ("pipe_lower_east", lower, 0.051, -1.1),
        ("pipe_upper_west", upper, -0.051, 0.1),
        ("pipe_upper_east", green, 1.2 + 0.051, 0.1),
    ):
        pipe_run(lane, floor, name, local(floor, x, y0), (0.0, 1.0), 1.0, PALLET_TOP, count=1)
