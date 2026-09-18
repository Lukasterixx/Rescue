# Competition lanes

> **This is a copy** of `~/Rescue/competition`, taken on 2026-09-18 so the rescue sim could be built while the
> original gains more arenas. The rescue sim (`../rescue_sim/`, launched with `../run_sim.sh`) uses its geometry,
> build, textures and `GravelReset`; its lanes are the sim's levels. The launcher and `CompetitionRuntime` described
> below are the original's adapter onto the old sim and are not used here. `../rescue_sim/README.md` says how the
> two are to be merged.

The first two terrain lanes from `RoboCupRescue-Arena-Fabrication-Guide-2026C-Korea-1.pdf`,
**Shifty Gravel** and **Diagonal K-Rails**, plus the **K-Rail Square**: the 2.4 m practice
square from the course drawing, built from the same components, with the guide's Linear
Align/Inspect tasks on its walls. All three exist together, 7 m apart, so switching
teleports the Go2 and D1 to the next lane without rebuilding the scene.

All new source, generated scenes and previews live here. The launcher imports the
current `Isaac/go2_omniverse` simulator and installs an adapter in that process;
it does not edit or copy those source files. Robot, arm, policy, sensors, ROS and
normal driving controls continue to come from that simulator. Its normal
`run_sim.sh` still launches its usual world.

## Run

From the repository root, using the same Isaac Lab installation as the existing sim:

```bash
./competition/run_sim.sh
```

Click the viewport to give it keyboard focus.

| Key | Action |
| --- | --- |
| F1 | Teleport to Shifty Gravel |
| F2 | Teleport to Diagonal K-Rails |
| F3 | Teleport to K-Rail Square |
| Page Down / Page Up | Next / previous lane, wrapping at the ends |
| Home or R | Restore the current lane's start pose and reset the gravel |
| Other keys | Existing simulator's driving, D1 and camera controls |

Jumps restore the standing joints, zero root/joint velocities and motion commands,
reset the arm through the existing D1 reset path, and request the existing ROS
odometry reset. Policy action/history buffers are cleared and height observations
are recomputed before the next inference. Both welded and teleported arm modes
use the existing simulator's implementation. A jump does not clear an external
SLAM system's accumulated map, and an external controller can send new commands
after the jump.

```bash
# Opposing 15-degree central floors, same two lanes
./competition/run_sim.sh --difficulty slopes

# Start at K-Rails; raise the rails in 5 cm increments (the square's rails follow too)
./competition/run_sim.sh --start-arena krails --k-rail-height 0.15

# Start outside the practice square, on its south-west START|END pad
./competition/run_sim.sh --start-arena square

# Fixed aggregate geometry for lower physics cost
./competition/run_sim.sh --gravel static

# Exercise both jumps and a reset, then exit (requires the Isaac GPU runtime)
./competition/run_sim.sh --headless --smoke-steps 60
```

`--gravel-seed` controls the deterministic stone variants/orientations. Defaults
are flat floors, 10 cm K-Rails and dynamic gravel. Isaac options such as
`--arm_mount teleport`, `--arm_mass`, and `--device` are forwarded. This first pass
supports one Go2; conflicting robot, world or terrain arguments fail early.
`ISAAC_SIM_CONDA_ENV` selects a different conda environment. `--help` works without
starting Isaac.

## Fabrication details

References below use the guide's **printed** page numbers; after printed page 20
these differ from the PDF viewer's page index by one.

| Feature | Implemented dimensions / arrangement | Guide |
| --- | --- | --- |
| Lane floors | Four 1.2 × 2.4 m framed OSB floors; nominal 4.8 × 2.4 m footprint | pp. 6, 24 |
| End zones | Opposing 5 cm lateral offsets; open entry gate on the paddock side | p. 6 |
| Railings | 14 frames per lane, 1.2 m wide × 0.9 m high; 5 × 10 cm lumber, three horizontals with tops at 30, 60 and 90 cm | pp. 21–22 |
| OSB | 11 mm thin OSB over 10 cm framing | pp. 14, 24 |
| Slopes | Opposing 15° centre floors; railings and obstacles tilt with their floors; supports and hinge beams | pp. 6, 25 |
| Gravel | 10 cm containment borders, two crossed-rail squares in each central floor, gravel in all four floors | pp. 27–28 |
| K-Rails | Eight 1.2 m square OSB backings, eight diagonals in the drawing's pattern, opposing 45° mitred ends | pp. 31–32 |
| Rail height | 10 cm base rail, optional 5 cm lifts; 10 cm width | pp. 30–32 |
| K-Rail Square | Two 2.4 × 1.2 m framed floors carrying the lane's four central K-Rail panels (one X); railings on the north and south edges, the north half of the east edge and the south half of the west edge; 33 cm milk crate on the crossing; START\|END floor pads outside the south-west and north-east corners | course drawing; pp. 31–32 |
| Linear Align/Inspect | Green 90 cm 2x2 rail on the railing's middle horizontal (60 cm) with a 30 cm 2x4 trapezoid centre piece; five capped, hollow 5 cm × 5 cm pipes, two straight out at ±30 cm, one on each 45° face, one on top; a 5 cm acuity target at each cap (sets 1 and 2, viewer order left 90°, left 45°, centre, right 45°, right 90°) | pp. 69–72 |

The guide mixes imperial stock sizes and nominal metric sizes and explicitly
permits small variations. These models use the metric dimensions. K-Rail diagonal
length follows the 1.2 m square's corners (about 1.697 m), instead of treating the
rounded 169 cm cut-list length as a gap. Lumber and OSB have procedural textures.
Ground, framing, railings, borders, diagonal rails and individual stones collide.

Two blue entry pads **outside** the guide's footprint provide clear standing
positions facing the open gates. These are simulation conveniences. The guide's
blue/green end zones remain inside each lane.

The square's START|END pads are 1.5 cm mats on the hall floor rather than raised platforms,
because the drawing's routes run round the outside of the structure before entering through
an open half-edge, so the robot steps up the 12 cm framed deck there. Its floors do not tilt
under `--difficulty slopes`. The centre object is modelled as a milk crate; it is not one of
the guide's fixtures. The acuity targets are procedural Landolt-C rings with the page's
colours and labels rather than the printable page, and the gauge, QR-code and hazmat
variants are not included.

Gravel is an approximation: about 7,800 separate convex rigid stones, roughly
40–46 mm across (the guide specifies 25+ mm aggregate), with two initial layers
inside the 10 cm borders. Their masses use an assumed stone density of
2,600 kg/m³. They settle, slide and can be displaced by feet; this is not a
calibrated granular-material model. Packing, friction and achievable sinkage need
checking on the actual GPU simulation. Static mode freezes the same rocks and
does not reproduce yielding gravel. Resets restore the initial packing in both
end zones and central floors.

The locomotion height scanner uses a separate invisible mesh containing the
walkable floors, K-Rails and the **nominal initial gravel surface**, excluding
tall railings. Isaac Lab's static raycaster cannot follow the individual moving
stones. Camera/lidar rendering and foot contacts use the actual scene geometry.

This pass includes the flat/preliminary and opposing-slope layouts. The optional
finals pinch-point panels are not yet included, and of the dexterity fixtures only the
square's two Linear Align/Inspect tasks are.

## Files and checks

- `geometry.py`: dimensions, lane catalogue, mesh construction and start poses;
  adding another lane here extends the next/previous selection list.
- `build.py`, `textures.py`: standalone USD export, materials, scanner geometry,
  scene manifest and a plan/3D preview.
- `runtime.py`: configuration adapter, queued hotkeys, state reset, gravel reset,
  status window and bounded smoke run.
- `main.py`, `run_sim.sh`: separate entry point and Isaac environment setup.
- `tests/`: geometry, real USD-reference composition, and CPU-tensor control tests.

Export without launching the simulator, using a Python environment with `numpy`,
`scipy`, `usd-core` and `Pillow` (plus `matplotlib` for `--preview`):

```bash
python3 -m competition.build --output competition/generated/competition.usda --preview
python3 -m unittest discover -s competition/tests -v
```

The USD test skips when USD bindings are unavailable; the runtime control tests
skip when PyTorch is unavailable. Run the latter with your Isaac environment's
Python to use real CPU tensors. Geometry and export tests do not need a GPU.
Generated USD files, JSON manifests, textures and previews are ignored by Git.

The implementation was checked with geometry/topology tests, both static and
dynamic USD exports composed under the real terrain namespace, CPU-tensor reset
tests, and visual inspection of the generated preview. The dynamic-gravel launcher
has also passed 60-step GPU smoke runs in headless and windowed modes on the
RTX 4080, exercising both arena jumps and resets.

The launch-crash fix disables `RigidPrim`'s default contact-sensor preparation
when binding the gravel reset view. Adding physics schemas and sleep attributes
to all 7,816 already-running stones invalidated their GPU body indices, producing
`Unresolved rigid dynamic index` followed by a CUDA illegal-memory-access crash.
The reset view now binds without reauthoring the stones' physics settings.
The smoke runs verify startup and resets; gravel calibration and course traversal
still need evaluation.
