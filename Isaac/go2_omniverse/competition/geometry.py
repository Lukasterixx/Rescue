"""Measured lane geometry. Metres, Z up; no Isaac/Omniverse imports.

Source: RoboCupRescue-Arena-Fabrication-Guide-2026C-Korea-1.pdf.
Printed pp. 6, 21–25, 27–28, 30–32 and 69–72 (PDF index diverges after p. 20).
Nominal metric lumber dimensions are intentional, as permitted by the guide.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

PANEL = 1.2
OSB = 0.011  # Thin OSB: purchase list, printed p. 14.
DECK = 0.1 + OSB
RAILING_HEIGHT = 0.9
GRAVEL_DEPTH = 0.1
LANE_SPACING = 7.0
COLORS = {
    "wood": (0.57, 0.36, 0.17),
    "osb": (0.65, 0.48, 0.27),
    "gravel": (0.26, 0.28, 0.29),
    "ground": (0.19, 0.21, 0.23),
    "blue": (0.025, 0.32, 0.72),
    "green": (0.03, 0.55, 0.25),
    "metal": (0.30, 0.32, 0.34),
    "pvc": (0.92, 0.92, 0.90),
}
# Visual/colour acuity targets, printed p. 72: ring colour and the gap directions of the
# three nested Landolt Cs (degrees, counter-clockwise from the viewer's right). Set 1 is the
# first linear task's five pipes in viewer order (left 90°, left 45°, centre, right 45°,
# right 90°), set 2 the second's. The gap directions are ours; the page shows a mix.
ACUITY_TARGETS = {
    "1A": ((0.96, 0.55, 0.10), (90, 270, 0)),
    "1B": ((0.85, 0.10, 0.10), (0, 180, 90)),
    "1": ((0.98, 0.80, 0.05), (270, 90, 180)),
    "1C": ((0.10, 0.55, 0.25), (180, 0, 270)),
    "1D": ((0.10, 0.30, 0.75), (90, 0, 180)),
    "2A": ((0.85, 0.10, 0.10), (180, 90, 0)),
    "2B": ((0.10, 0.55, 0.25), (0, 270, 90)),
    "2": ((0.96, 0.55, 0.10), (90, 180, 270)),
    "2C": ((0.10, 0.30, 0.75), (270, 0, 180)),
    "2D": ((0.98, 0.80, 0.05), (0, 90, 270)),
}
COLORS.update({f"target_{name}": color for name, (color, _) in ACUITY_TARGETS.items()})


@dataclass(frozen=True)
class Options:
    difficulty: str = "flat"
    k_rail_height: float = 0.10
    gravel: str = "dynamic"
    seed: int = 2026

    def __post_init__(self):
        if self.difficulty not in ("flat", "slopes"):
            raise ValueError("difficulty must be flat or slopes")
        if self.gravel not in ("dynamic", "static"):
            raise ValueError("gravel must be dynamic or static")
        if not 0.1 <= self.k_rail_height <= 0.4 or not math.isclose(
            self.k_rail_height / 0.05, round(self.k_rail_height / 0.05), abs_tol=1e-7
        ):
            raise ValueError("K-Rails must be 0.10–0.40 m tall, in 0.05 m increments")


@dataclass
class Mesh:
    name: str
    vertices: np.ndarray
    faces: np.ndarray
    material: str = "wood"
    collision: bool = True
    scan: bool = False
    uv: np.ndarray | None = None  # per-vertex texture coordinates; None = planar projection


@dataclass(frozen=True)
class Floor:
    name: str
    center: tuple[float, float]
    size: tuple[float, float]
    pitch: float = 0.0

    def transform(self, points):
        """Rotate about the low hinge, keeping the low deck edge at DECK."""
        p = np.array(points, dtype=float, copy=True)
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        x, z = p[..., 0].copy(), p[..., 2].copy()
        p[..., 0] = c * x - s * z + self.center[0]
        p[..., 1] += self.center[1]
        p[..., 2] = s * x + c * z + DECK + abs(s) * self.size[0] / 2
        return p


@dataclass(frozen=True)
class Stone:
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]  # USD wxyz
    variant: int


@dataclass
class Lane:
    key: str
    title: str
    origin: tuple[float, float, float]
    floors: list[Floor]
    spawn: tuple[float, float, float]
    meshes: list[Mesh] = field(default_factory=list)
    stones: list[Stone] = field(default_factory=list)
    railing_count: int = 0
    # USD wxyz. The 4.8 m lanes start facing -y, into their entry gate.
    spawn_rotation: tuple[float, float, float, float] = (math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5))
    subtitle: str = "4.8 m × 2.4 m nominal lane · 5 cm end-zone offsets"


def prism(name, polygon, bottom, top, material="wood", collision=True, scan=False):
    """Closed, triangulated extrusion of a counterclockwise convex polygon."""
    xy = np.asarray(polygon, dtype=float)
    n = len(xy)
    vertices = np.concatenate(
        (
            np.column_stack((xy, np.full(n, bottom))),
            np.column_stack((xy, np.full(n, top))),
        )
    )
    faces = []
    for i in range(1, n - 1):
        faces.extend(((0, i + 1, i), (n, n + i, n + i + 1)))
    for i in range(n):
        j = (i + 1) % n
        faces.extend(((i, j, n + j), (i, n + j, n + i)))
    return Mesh(
        name, vertices, np.array(faces, dtype=np.int32), material, collision, scan
    )


def box(name, center, size, material="wood", collision=True, scan=False):
    x, y, z = center
    sx, sy, sz = np.asarray(size) / 2
    return prism(
        name,
        [(x - sx, y - sy), (x + sx, y - sy), (x + sx, y + sy), (x - sx, y + sy)],
        z - sz,
        z + sz,
        material,
        collision,
        scan,
    )


def diagonal(name, bounds, rising, width, bottom, height):
    """Trapezoidal rail with 45° mitres, entirely inside its square panel.

    The outer edge connects the exact panel corners; the inner edge is inset
    by width * sqrt(2), reproducing the guide's opposing 45° cuts.
    """
    x0, y0, x1, y1 = bounds
    d = width * math.sqrt(2)
    if rising:
        polygon = [(x0, y0), (x0 + d, y0), (x1, y1 - d), (x1, y1)]
    else:
        polygon = [(x0, y1 - d), (x1 - d, y0), (x1, y0), (x0, y1)]
    return prism(name, polygon, bottom, bottom + height, scan=True)


def combine(meshes, name="combined"):
    vertices, faces, offset = [], [], 0
    for mesh in meshes:
        vertices.append(mesh.vertices)
        faces.append(mesh.faces + offset)
        offset += len(mesh.vertices)
    return Mesh(name, np.concatenate(vertices), np.concatenate(faces))


def add(lane, mesh, floor=None):
    if floor is not None:
        mesh.vertices = floor.transform(mesh.vertices)
    mesh.vertices += np.asarray(lane.origin)
    lane.meshes.append(mesh)


def railing(lane, floor, start, end, name):
    """1.2 m frame: two 10 cm posts and three 10 cm horizontal members.

    The horizontals' tops are at 30, 60 and 90 cm ("spacing between tops 30 cm", p. 22),
    which is also what puts the inspect tasks' rails at their 60 cm elevation (p. 69).
    """
    a, b = np.asarray(start), np.asarray(end)
    direction = b - a
    length = np.linalg.norm(direction)
    direction /= length
    side = np.array([-direction[1], direction[0]])
    middle = (a + b) / 2
    for suffix, x, z, sx, sz in (
        ("post_a", -length / 2 + 0.05, 0.45, 0.10, 0.90),
        ("post_b", length / 2 - 0.05, 0.45, 0.10, 0.90),
        ("lower", 0, 0.25, length - 0.20, 0.10),
        ("middle", 0, 0.55, length - 0.20, 0.10),
        ("upper", 0, 0.85, length - 0.20, 0.10),
    ):
        mesh = box(f"{name}_{suffix}", (x, 0, z), (sx, 0.05, sz))
        xy = mesh.vertices[:, :2].copy()
        mesh.vertices[:, :2] = middle + xy[:, :1] * direction + xy[:, 1:2] * side
        add(lane, mesh, floor)
    lane.railing_count += 1


def build_floor(lane, floor):
    """Framed floor: thin OSB on 10 cm long borders and three cross members (p. 24)."""
    sx, sy = floor.size
    add(
        lane,
        box(f"{floor.name}_osb", (0, 0, -OSB / 2), (sx, sy, OSB), "osb", scan=True),
        floor,
    )
    for i, y in enumerate((-sy / 2 + 0.025, sy / 2 - 0.025)):
        add(
            lane,
            box(f"{floor.name}_frame_long_{i}", (0, y, -OSB - 0.05), (sx, 0.05, 0.1)),
            floor,
        )
    for i, x in enumerate(np.linspace(-sx / 2 + 0.025, sx / 2 - 0.025, 3)):
        add(
            lane,
            box(
                f"{floor.name}_joist_{i}",
                (x, 0, -OSB - 0.05),
                (0.05, sy - 0.1, 0.1),
            ),
            floor,
        )


def build_structure(lane):
    for floor in lane.floors:
        build_floor(lane, floor)
        if floor.name in ("blue_end", "green_end"):
            left = floor.name == "blue_end"
            x = -0.6 if left else 0.6
            for i in range(2):
                railing(
                    lane,
                    floor,
                    (x, -1.2 + i * 1.2),
                    (x, i * 1.2),
                    f"{floor.name}_outer_{i}",
                )
            railing(lane, floor, (-0.6, -1.2), (0.6, -1.2), f"{floor.name}_south")
            if left:
                # Gate swings outward around the outer north corner, clear of the entry.
                railing(lane, floor, (-0.6, 1.2), (-0.6, 2.4), "entry_gate_open")
            else:
                railing(lane, floor, (-0.6, 1.2), (0.6, 1.2), f"{floor.name}_north")
        else:
            lower = floor.name == "lower"
            y = -0.6 if lower else 0.6
            for i in range(2):
                railing(
                    lane,
                    floor,
                    (-1.2 + i * 1.2, y),
                    (i * 1.2, y),
                    f"{floor.name}_outer_{i}",
                )
            x = 1.2 if lower else -1.2
            railing(lane, floor, (x, -0.6), (x, 0.6), f"{floor.name}_end_barrier")
            # 10 x 10 x 120 cm hinge beam at the LOW end, guide p. 6.
            hinge_x = (
                -math.copysign(1.2, floor.pitch)
                if floor.pitch
                else (-1.2 if lower else 1.2)
            )
            add(
                lane,
                box(
                    f"{floor.name}_hinge",
                    (hinge_x, floor.center[1], DECK - 0.05),
                    (0.1, 1.2, 0.1),
                ),
            )
            if floor.pitch:
                high_x = math.copysign(1.0, floor.pitch)
                top = floor.transform([[high_x, 0, -OSB]])[0]
                height = top[2]
                for j, y in enumerate((floor.center[1] - 0.45, floor.center[1] + 0.45)):
                    add(
                        lane,
                        box(
                            f"{floor.name}_leg_{j}",
                            (top[0], y, height / 2),
                            (0.1, 0.05, height),
                        ),
                    )
                add(
                    lane,
                    box(
                        f"{floor.name}_leg_brace",
                        (top[0], floor.center[1], height - 0.15),
                        (0.011, 1.0, 0.3),
                        "osb",
                    ),
                )


def gravel_lane(lane, options):
    rng = np.random.default_rng(options.seed)
    for floor in lane.floors:
        sx, sy = floor.size
        for i, y in enumerate((-sy / 2 + 0.025, sy / 2 - 0.025)):
            add(
                lane,
                box(
                    f"{floor.name}_gravel_border_y{i}",
                    (0, y, 0.05),
                    (sx, 0.05, 0.1),
                    scan=True,
                ),
                floor,
            )
        for i, x in enumerate((-sx / 2 + 0.025, sx / 2 - 0.025)):
            add(
                lane,
                box(
                    f"{floor.name}_gravel_border_x{i}",
                    (x, 0, 0.05),
                    (0.05, sy - 0.1, 0.1),
                    scan=True,
                ),
                floor,
            )
        crossed = floor.name in ("lower", "upper")
        rail_polygons = []
        if crossed:
            for cell in range(2):
                # Two Xs in each 2.4 x 1.2 centre floor; not one large X.
                bounds = (-1.15 + 1.15 * cell, -0.55, 1.15 * cell, 0.55)
                for rising in (False, True):
                    rail = diagonal(
                        f"{floor.name}_x_{cell}_{int(rising)}",
                        bounds,
                        rising,
                        0.05,
                        0,
                        0.1,
                    )
                    rail_polygons.append(rail.vertices[:4, :2].copy())
                    add(lane, rail, floor)
        # Static raycaster sees the perceived gravel plane. In dynamic mode this
        # has NO collision; foot contacts are exclusively the individual stones.
        add(
            lane,
            box(
                f"{floor.name}_gravel_scan",
                (0, 0, 0.05),
                (sx - 0.1, sy - 0.1, 0.1),
                "gravel",
                collision=False,
                scan=True,
            ),
            floor,
        )
        # 25+ mm aggregate in the purchase list. Here: 40–46 mm angular stones,
        # two layers, deterministic staggered packing, clear of borders/X-rails.
        pitch = 0.048
        for layer in range(2):
            for row, y in enumerate(np.arange(-sy / 2 + 0.075, sy / 2 - 0.065, pitch)):
                for x in np.arange(-sx / 2 + 0.075, sx / 2 - 0.065, pitch):
                    xx = x + ((row + layer) % 2) * 0.005
                    yy = y + layer * 0.003
                    if any(
                        point_polygon_distance((xx, yy), p) < 0.024
                        for p in rail_polygons
                    ):
                        continue
                    point = (
                        floor.transform([[xx, yy, 0.024 + layer * 0.046]])[0]
                        + lane.origin
                    )
                    yaw = rng.uniform(-math.pi, math.pi)
                    # Only yaw changes the prototype's fixed vertical extent.
                    qy = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
                    qp = (
                        math.cos(floor.pitch / 2),
                        0.0,
                        -math.sin(floor.pitch / 2),
                        0.0,
                    )
                    w, xq, yq, zq = qp
                    a, b, c, d = qy
                    quat = (
                        w * a - xq * b - yq * c - zq * d,
                        w * b + xq * a + yq * d - zq * c,
                        w * c - xq * d + yq * a + zq * b,
                        w * d + xq * c - yq * b + zq * a,
                    )
                    lane.stones.append(
                        Stone(tuple(point), quat, int(rng.integers(0, 6)))
                    )


def k_rail_cell(lane, floor, cell, center, rising, options):
    """One 1.2 m square OSB backing with its stacked diagonal (pp. 31–32)."""
    x, y = center
    add(
        lane,
        box(
            f"{floor.name}_backing_{cell}",
            (x, y, OSB / 2),
            (1.2, 1.2, OSB),
            "osb",
            scan=True,
        ),
        floor,
    )
    # A 10 cm base, then 5 cm lifts; preserve the visible layer seams.
    bottom, remaining, layer = OSB, options.k_rail_height, 0
    while remaining > 1e-7:
        height = min(0.1 if layer == 0 else 0.05, remaining)
        add(
            lane,
            diagonal(
                f"{floor.name}_k_{cell}_layer{layer}",
                (x - 0.6, y - 0.6, x + 0.6, y + 0.6),
                rising,
                0.1,
                bottom,
                height,
            ),
            floor,
        )
        bottom += height
        remaining -= height
        layer += 1


def k_rail_lane(lane, options):
    for floor in lane.floors:
        for cell in range(2):
            if floor.name in ("blue_end", "green_end"):
                center = (0.0, -0.6 + cell * 1.2)
                rising = (cell == 1) == (floor.name == "blue_end")
            else:
                center = (-0.6 + cell * 1.2, 0.0)
                rising = (cell == 0) == (floor.name == "lower")
            k_rail_cell(lane, floor, cell, center, rising, options)


# --- Linear Align/Inspect tasks (pp. 69–72) and the practice square ---------------------
PIPE_OUTER, PIPE_INNER, PIPE_LENGTH = 0.030, 0.025, 0.05  # 5 cm ID drain pipe, 5 cm long
CAP_RADIUS, CAP_THICKNESS = 0.034, 0.006
TARGET_RADIUS = 0.024
TASK_ELEVATION = 0.60  # "mounted on the walls at 60 cm elevation", p. 69


def viewer_frame(axis):
    """(right, up, forward) for a horizontal unit `axis`: right-handed, and `right` is the
    right-hand side of someone looking INTO the axis, so a target laid out in this frame
    reads correctly from the pipe's opening."""
    ax, ay = axis
    return np.array([-ay, ax, 0.0]), np.array([0.0, 0.0, 1.0]), np.array([ax, ay, 0.0])


def ring(radius, sides=16):
    angles = np.arange(sides) * 2 * math.pi / sides
    return np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))


def orient(mesh, start, axis):
    """Map a mesh built along +z onto the horizontal `axis`, its base at `start`."""
    right, up, forward = viewer_frame(axis)
    v = mesh.vertices
    mesh.vertices = (
        np.asarray(start, dtype=float)
        + v[:, :1] * right
        + v[:, 1:2] * up
        + v[:, 2:3] * forward
    )
    return mesh


def cylinder(name, start, axis, length, radius, material, textured=False):
    mesh = prism(name, ring(radius), 0.0, length, material)
    if textured:
        # The whole texture across the diameter, upright, as seen from the open end.
        mesh.uv = mesh.vertices[:, :2] / (2 * radius) + 0.5
    return orient(mesh, start, axis)


def tube(name, start, axis, length, outer, inner, material, sides=16):
    """Closed hollow cylinder: a pipe a camera can look into."""
    o, i = ring(outer, sides), ring(inner, sides)
    n = sides
    vertices = np.concatenate(
        (
            np.column_stack((o, np.zeros(n))),
            np.column_stack((o, np.full(n, length))),
            np.column_stack((i, np.zeros(n))),
            np.column_stack((i, np.full(n, length))),
        )
    )
    faces = []
    for k in range(n):
        j = (k + 1) % n
        ob, ot, ib, it = k, n + k, 2 * n + k, 3 * n + k
        jb, jt, jib, jit = j, n + j, 2 * n + j, 3 * n + j
        faces += [(ob, jb, jt), (ob, jt, ot)]  # outer wall, outward
        faces += [(ib, jit, jib), (ib, it, jit)]  # inner wall, facing the bore
        faces += [(ob, ib, jib), (ob, jib, jb)]  # base annulus, facing -z
        faces += [(ot, jt, jit), (ot, jit, it)]  # open-end annulus, facing +z
    mesh = Mesh(name, vertices, np.array(faces, dtype=np.int32), material)
    return orient(mesh, start, axis)


def linear_inspect_task(lane, floor, center, facing, name, targets):
    """Linear Align/Inspect task, p. 70, on a railing's middle horizontal.

    A 90 cm 2x2 rail lies flat on the wall with a 30 cm 2x4 centre piece cut to a trapezoid
    on its face; five capped 5 cm pipes point into the arena: two straight out at ±30 cm, one
    on each 45° face and one on the top face. Each pipe holds a 5 cm acuity target (p. 72)
    at its cap. The rail sits on the middle horizontal, whose top is at 60 cm (p. 69).
    `center` is the mount point on the railing line in floor coordinates, `facing` the unit
    direction into the arena, `targets` the five ACUITY_TARGETS keys in the viewer's order.
    """
    fx, fy = facing
    forward = np.array([fx, fy, 0.0])
    along = np.array([fy, -fx, 0.0])  # right-handed with forward and up
    up = np.array([0.0, 0.0, 1.0])
    base = np.array([center[0], center[1], 0.0])

    def place(mesh):
        v = mesh.vertices
        mesh.vertices = base + v[:, :1] * along + v[:, 1:2] * forward + v[:, 2:3] * up
        add(lane, mesh, floor)

    z0, z1 = TASK_ELEVATION, TASK_ELEVATION + 0.05
    place(box(f"{name}_rail", (0.0, 0.0, (z0 + z1) / 2), (0.9, 0.05, 0.05), "green"))
    place(
        prism(
            f"{name}_centre",
            [(-0.15, 0.025), (0.15, 0.025), (0.05, 0.125), (-0.05, 0.125)],
            z0,
            z1,
            "green",
        )
    )
    s = math.sqrt(0.5)
    # Viewer's left to right when facing the wall, which is -along: (x along the rail,
    # y out from the railing line, pipe axis).
    pipes = (
        (0.30, 0.025, (0.0, 1.0)),
        (0.10, 0.075, (s, s)),
        (0.0, 0.125, (0.0, 1.0)),
        (-0.10, 0.075, (-s, s)),
        (-0.30, 0.025, (0.0, 1.0)),
    )
    for k, ((x, y, axis), target) in enumerate(zip(pipes, targets)):
        foot = (x, y, (z0 + z1) / 2)
        cap_end = np.array(foot) + CAP_THICKNESS * np.array([*axis, 0.0])
        place(cylinder(f"{name}_pipe{k}_cap", foot, axis, CAP_THICKNESS, CAP_RADIUS, "pvc"))
        place(
            tube(f"{name}_pipe{k}_tube", cap_end, axis, PIPE_LENGTH, PIPE_OUTER, PIPE_INNER, "pvc")
        )
        place(
            cylinder(
                f"{name}_pipe{k}_target",
                cap_end,
                axis,
                0.002,
                TARGET_RADIUS,
                f"target_{target}",
                textured=True,
            )
        )


def square_lane(options, origin):
    """The 2.4 m practice square: the K-Rails lane's central four panels, fenced along the
    north and south edges and along the north half of the east edge and the south half of
    the west edge, each half-railing carrying a Linear Align/Inspect task facing inward. A
    milk crate sits on the crossing. The two open half-edges are the ways in, and the
    drawing's START|END boxes are floor pads outside the opposite corners."""
    south = Floor("south", (0.0, -0.6), (2.4, 1.2))
    north = Floor("north", (0.0, 0.6), (2.4, 1.2))
    pad = 0.015
    spawn = (origin[0] - 1.85, -1.2, pad + 0.42)
    lane = Lane(
        "square",
        "K-Rail Square",
        origin,
        [south, north],
        spawn,
        spawn_rotation=(math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)),  # facing +y, up the west side
        subtitle="2.4 m × 2.4 m square · linear inspect tasks at 60 cm",
    )
    for floor in (south, north):
        build_floor(lane, floor)
        for cell in range(2):
            # Same pattern as the lane's centre floors: the four diagonals make one X.
            k_rail_cell(lane, floor, cell, (-0.6 + cell * 1.2, 0.0), (cell == 0) == (floor is south), options)
    for floor, y in ((south, -0.6), (north, 0.6)):
        for i in range(2):
            railing(lane, floor, (-1.2 + i * 1.2, y), (i * 1.2, y), f"{floor.name}_outer_{i}")
    railing(lane, north, (1.2, -0.6), (1.2, 0.6), "east_task_railing")
    railing(lane, south, (-1.2, -0.6), (-1.2, 0.6), "west_task_railing")
    linear_inspect_task(lane, north, (1.2, 0.0), (-1.0, 0.0), "east_inspect", ("1A", "1B", "1", "1C", "1D"))
    linear_inspect_task(lane, south, (-1.2, 0.0), (1.0, 0.0), "west_inspect", ("2A", "2B", "2", "2C", "2D"))
    add(lane, box("centre_crate", (0.0, 0.0, DECK + OSB + 0.14), (0.33, 0.33, 0.28), "blue"))
    # START|END boxes of the drawing: thin pads on the hall floor, since both marked paths
    # run round the outside of the structure before entering through an open half-edge.
    for name, material, (x, y) in (
        ("entry_pad", "blue", (-1.85, -1.2)),
        ("return_pad", "green", (1.85, 0.9)),
    ):
        add(lane, box(name, (x, y, pad / 2), (1.2, 1.2, pad), material, scan=True))
    return lane


def build_lanes(options=Options()):
    lanes = []
    for i, (key, title) in enumerate(
        (("gravel", "Shifty Gravel"), ("krails", "Diagonal K-Rails"))
    ):
        angle = math.radians(15) if options.difficulty == "slopes" else 0.0
        floors = [
            Floor("blue_end", (-1.8, 0.05), (1.2, 2.4)),
            Floor("lower", (0.0, -0.6), (2.4, 1.2), angle),
            Floor("upper", (0.0, 0.6), (2.4, 1.2), -angle),
            Floor("green_end", (1.8, -0.05), (1.2, 2.4)),
        ]
        origin = (i * LANE_SPACING, 0.0, 0.0)
        surface = DECK + (GRAVEL_DEPTH if key == "gravel" else OSB)
        # Simulation entry pad, outside the guide's footprint and open gate.
        spawn = (origin[0] - 1.8, 1.85, surface + 0.42)
        lane = Lane(key, title, origin, floors, spawn)
        build_structure(lane)
        if key == "gravel":
            gravel_lane(lane, options)
        else:
            k_rail_lane(lane, options)
        add(
            lane,
            box(
                "entry_pad",
                (-1.8, 1.85, surface / 2),
                (1.2, 1.2, surface),
                "blue",
                scan=True,
            ),
        )
        # Colour-coded start/finish strips, kept off the traversable obstacles.
        for floor, material in ((floors[0], "blue"), (floors[-1], "green")):
            add(
                lane,
                box(
                    f"{floor.name}_marker",
                    (0, -1.17, 0.91),
                    (1.0, 0.015, 0.025),
                    material,
                    False,
                ),
                floor,
            )
        lanes.append(lane)
    lanes.append(square_lane(options, (2 * LANE_SPACING, 0.0, 0.0)))
    return lanes


def point_polygon_distance(point, polygon):
    """Distance to a convex polygon (zero inside), used to avoid rock overlaps."""
    a = np.asarray(polygon)
    edges = np.roll(a, -1, axis=0) - a
    delta = np.asarray(point) - a
    cross = edges[:, 0] * delta[:, 1] - edges[:, 1] * delta[:, 0]
    if np.all(cross >= -1e-10):
        return 0.0
    t = np.clip(np.sum(delta * edges, axis=1) / np.sum(edges * edges, axis=1), 0.0, 1.0)
    return float(np.linalg.norm(delta - t[:, None] * edges, axis=1).min())


def ground_mesh(count=3):
    """Hall floor under `count` lanes, 5 m beyond the first and last lane origins."""
    span = (count - 1) * LANE_SPACING
    return box(
        "hall_floor",
        (span / 2, 0.0, -0.05),
        (span + 10.0, 10.0, 0.1),
        "ground",
        scan=True,
    )


def stone_shape(variant):
    """Small faceted convex rock with deterministic variation and a stable base."""
    from scipy.spatial import ConvexHull

    rng = np.random.default_rng(100 + variant)
    points = []
    for z, radius in ((-0.022, 0.011), (0.0, 0.022), (0.022, 0.009)):
        for i in range(6):
            angle = i * math.pi / 3
            r = radius * rng.uniform(0.92, 1.04)
            points.append((r * math.cos(angle), r * math.sin(angle), z))
    points = np.array(points)
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    for i, face in enumerate(faces):
        a, b, c = points[face]
        if np.dot(np.cross(b - a, c - a), hull.equations[i, :3]) < 0:
            faces[i] = face[::-1]
    return Mesh(f"stone_{variant}", points, faces, "gravel"), hull.volume * 2600
