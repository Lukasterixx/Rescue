# Competition lanes

The terrain lanes and obstacles from `RoboCupRescue-Arena-Fabrication-Guide-2026C-Korea-1.pdf`:
**Shifty Gravel**, **Diagonal K-Rails**, **Half-Cubic Stepfields**, **Pitch/Roll Ramps**,
**Center in Alleys**, **Pallets & Pipes**, **Push/Pull Doors**, **Avoid Holes/Posts**,
**Stair Debris | Pallet Climb** and the **Search & Map Maze**, plus the **K-Rail Square**: the
2.4 m practice square from the course drawing, built from the same components, with the
guide's Linear Align/Inspect tasks on its walls. The stair arena has separate clear and debris
variants. The K-Rails, stepfields and ramps also come in the guide's two harder settings
(pp. 5, 30, 34, 37): **Opposing Slopes (15°)**, and **Additional Obstacles**, which adds pinch
points to the slopes plus layered K-Rails or rotating slip disks. Center in Alleys also has its
**Sloped 15°** layout (p. 43). All nineteen lanes exist together along one hall,
so switching teleports the Go2 and D1 to the next lane without rebuilding the scene.

This package is the geometry and its USD export. The rescue sim (`../rescue_sim/`) builds the hall from it at
startup, and each lane becomes one of its levels. The robot, arm, camera, policy, ROS and controls all belong to
the sim. The package also holds the resets for the loose bodies (`runtime.py`).

## Run

From `Isaac/go2_omniverse`:

```bash
./run_sim.sh                                   # starts on Shifty Gravel
./run_sim.sh --level stairs                    # any lane key, or cup for the cup demo
./run_sim.sh --level stair_debris              # stairs with debris and red landing braces
./run_sim.sh --level ramps_obstacles           # sloped ramps with slip disks and pinch points
```

The "Rescue sim" window, docked beside Stage, has a row for each arena and a button for each of its
settings. Levels are chosen there (or on `/sim/level`), not by key; Home or R resets the current one.
Loading a lane teleports the robot to its entry pad and resets the gravel, the avoid posts and the slip
disks. `../rescue_sim/README.md` has the rest: the other controls, `/sim/level`, and what a load resets.

The geometry options are the sim's arguments:

```bash
# Opposing 15-degree central floors in the gravel and pallets lanes, which have no sloped level
./run_sim.sh --difficulty slopes

# Raise the rails in 5 cm increments (the sloped lane's and the square's follow too)
./run_sim.sh --level krails --k-rail-height 0.15
# Stack the Additional Obstacles lane's rails higher (default 20 cm)
./run_sim.sh --level krails_obstacles --layered-k-rail-height 0.30

# Doorways 10 cm wider than the robot; door apparatus with the yellow steps removed;
# a 35-degree stair with all three debris rails
./run_sim.sh --level alleys --alley-width 0.41
./run_sim.sh --level doors --door-floor square
./run_sim.sh --level stair_debris --stair-angle 35 --stair-debris 3

# Fixed aggregate geometry for lower physics cost
./run_sim.sh --gravel static
```

`--gravel-seed` controls the deterministic stone variants and orientations. The defaults are:
- flat gravel and pallets floors;
- 10 cm K-Rails, and 20 cm in the Additional Obstacles lane;
- dynamic gravel;
- 45 cm doorways and the full door floor;
- two 45-degree stair arenas: clear stairs, and stairs with all three debris rails and red landing braces.

The lane keys are gravel, krails, square, stepfields, ramps, alleys, pallets, doors, avoid, stairs,
stair_debris, maze, krails_slopes, krails_obstacles, stepfields_slopes, stepfields_obstacles,
ramps_slopes, ramps_obstacles and alleys_slopes. `--stair-debris 0–3` changes only `stair_debris`;
`stairs` always stays clear. Likewise `--difficulty` tilts only gravel and pallets: `krails`,
`stepfields`, `ramps` and `alleys` stay flat, and their `_slopes` and `_obstacles` copies are always
sloped.

The maze's tarp leaves its interior dark. Turn on the wrist light from the sim window to see in there with the
wrist camera.

## Fabrication details

References below use the guide's **printed** page numbers; after printed page 20
these differ from the PDF viewer's page index by one.

| Feature | Implemented dimensions / arrangement | Guide |
| --- | --- | --- |
| Lane floors | Four 1.2 × 2.4 m framed OSB floors; nominal 4.8 × 2.4 m footprint | pp. 6, 24 |
| End zones | Opposing 5 cm lateral offsets; open entry gate on the paddock side | p. 6 |
| Railings | 14 frames per lane, 1.2 m wide × 0.9 m high; 5 × 10 cm lumber, three horizontals with tops at 30, 60 and 90 cm | pp. 21–22 |
| OSB | 11 mm thin OSB over 10 cm framing | pp. 14, 24 |
| Slopes | Opposing 15° centre floors; railings and obstacles tilt with their floors; supports and hinge beams. Always on in the `_slopes` and `_obstacles` lanes; `--difficulty slopes` adds them to gravel and pallets | pp. 6, 25 |
| Difficulty settings | K-Rails, stepfields and ramps each as three lanes: flat; opposing 15° slopes; and Additional Obstacles, the same slopes with four pinch points plus the lane's own extra (layered K-Rails, slip disks; the stepfields have none). Center in Alleys flat and sloped | pp. 5, 30, 34, 37, 43 |
| Pinch points | Two 60 × 60 cm thin OSB panels at right angles on a 60 cm 2x2 spine, making a triangle with the railing that juts 42 cm into the lane; 15 cm 2x4 hanging blocks on the top rail with 15 cm 2x2 posts behind it; panels 35–95 cm above the floor, red as in the renders. Four per Additional Obstacles lane, on alternate walls where p. 5's render hangs them: the blue end's west wall inside the gate, the upper floor's north wall (west half), the lower floor's south wall (east half), the green end's north wall | pp. 5, 86–87 |
| Layered K-Rails | The Additional Obstacles lane's diagonals stacked to `--layered-k-rail-height` (default 20 cm: the 10 cm base and two 5 cm layers, seams visible) | pp. 30, 32 |
| Rotating slip disks | One 50 cm disk centred on each of the 32 ramps, orange with the black marker line from its centre to its rim, under a fender washer. A rigid body on a loose bolt (a D6 joint): it turns freely about the ramp's normal and lifts up to 5 mm, but cannot slide or tilt. It rests on a seat of its ramp's OSB whose friction with it is 0.30 static, 0.25 dynamic | pp. 37, 40–41 |
| Gravel | 10 cm containment borders, two crossed-rail squares in each central floor, gravel in all four floors | pp. 27–28 |
| K-Rails | Eight 1.2 m square OSB backings, eight diagonals in the drawing's pattern, opposing 45° mitred ends | pp. 31–32 |
| Rail height | 10 cm base rail, optional 5 cm lifts; 10 cm width | pp. 30–32 |
| K-Rail Square | Two 2.4 × 1.2 m framed floors carrying the lane's four central K-Rail panels (one X); railings on the north and south edges, the north half of the east edge and the south half of the west edge; 33 cm milk crate on the crossing; START\|END floor pads outside the south-west and north-east corners | course drawing; pp. 31–32 |
| Linear Align/Inspect | Green 90 cm 2x2 rail on the railing's middle horizontal (60 cm) with a 30 cm 2x4 trapezoid centre piece; five capped, hollow 5 cm × 5 cm pipes, two straight out at ±30 cm, one on each 45° face, one on top; a 5 cm acuity target at each cap (sets 1 and 2, viewer order left 90°, left 45°, centre, right 45°, right 90°) | pp. 69–72 |
| Half-Cubic Stepfields | Standard lane; 32 cells of 59.4 cm thin-OSB bases in eight 1.2 m blocks, each two quad-steps on one diagonal and two single-steps in the other corners; 80 29.7 cm thick-OSB plateaus on 2x4 legs, 32 at 30 cm (orange) in diagonal ridges that zigzag between the railings and cross at the lane centre, 48 at 15 cm (yellow) against every side of them | pp. 33–35 |
| Pitch/Roll Ramps | Standard lane; eight 1.2 m half-panel elements of four 59.4 cm ramps, 15 cm at the high edge, peaks and valleys alternating with clockwise up-slopes about the element centre | pp. 36–39 |
| Center in Alleys | Two flat 1.2 × 2.4 m floors across the lane at the door end and two tilt-up 2.4 × 1.2 m floors side by side at the far end, which in `alleys_slopes` both rise 15° toward the far end; 12 perimeter railings; three dividers at x = −1.2, 0, +1.2 m, each a railing on one half of the width plus a 120 × 80 cm sliding OSB panel clamped to it, doorways of `--alley-width` alternating south, north, south | pp. 42–44 |
| Pallets & Pipes | Standard lane; a fabricated 1.2 m grid pallet (OSB bottom, five 2x4 rails each way) on every cell, grid side up; two cells stacked to 20 cm; a 10 cm × 100 cm pipe in 60 cm sleeves on the lower surface against each raised face along the lane | pp. 45–49 |
| Push/Pull Doors | 2.4 m square; framed 1.2 × 2.4 m wall with a 90 cm door on the red 1.2 m base at the back half of the centre line, thin 1.2 m panel to the front; orange 60 × 120 cm half steps and yellow 120 cm square steps, all 10 cm; slatted 45 cm side and back walls, open front; door leaf a 12 kg rigid body on a revolute hinge, 0–100°, sprung closed | pp. 56–59 |
| Avoid Holes/Posts | Ten purchased 120 × 100 × 14 cm pallets (stringers, bottom boards, seven deck boards with gaps) in a meander of five runs; five pairs of loose 45 cm 2x4 posts 90 cm apart across the second pallet of each run | pp. 60–62 |
| Clear Stairs / Stair Debris \| Pallet Climb | Two copies: `stairs` clear, `stair_debris` with three 75 cm yellow/orange/red beams leaning from individual treads to alternating side supports, and red diagonal beams crossing the upper and lower routes at the landing centre. Both have a 90 cm stair with five 20 cm rises, `--stair-angle` 35–45°, four 120 cm mitred side rails fastened to the OSB and belay posts; two belay arches with 240 cm uprights and 100/140 cm tops; 2.4 × 1.2 m landing at 1.0 m on six legs with joists every 40 cm and a west OSB panel; pallet stacks of 7, 4 and 1 (70, 40, 10 cm), pipes at the transitions, railings on each stack | pp. 50–55 (belays/braces: PDF page 53) |
| Search & Map Maze | The guide's example layout on a 9 × 5 grid of 1.22 m cells: 36 wall panels 10 mm × 2.2 m tall (L-walls flattened to segments), three entrances with their vestibule walls, ten 30 cm mapping fiducial tubes at 1 m and 2 m, 19 hallway diagonals of paired 2x4s on edge, two room crosses of 1.3 m 2x4s with an omni align/inspect task, a blackout tarp at 2.2 m | pp. 63–67 |
| OMNI Align/Inspect | 30 cm OSB base, two 30 cm 2x4 trapezoids crossed on edge, five capped pipes: one up, one on each 45° end face; acuity sets 1 and 2 | p. 71 |

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

The eight later lanes are approximations of the guide's drawings where it leaves the
arrangement free. The stepfields follow p. 33 (quad-steps form diagonal hills, single-steps
fill corners), p. 35's cut list and the zigzags of p. 34's renders; those renders' lane has
turns, so which way each block's ridge runs is ours. Every side of a 30 cm top meets a 15 cm
top, a railing, or another 30 cm top where two ridges meet. The ramp peak/valley chequer is
ours; the guide only fixes the ramp sizes. The omni tasks the renders show at the centre of
the stepfield and ramp lanes are not modelled. Pallets & Pipes uses the
prelims layout with the grid side up; its pipes and the stair lane's pipes are static
cylinders, not free-spinning ones. The door's weighted closer is an angular drive (2 N·m/rad,
2 N·m·s/rad) rather than a hanging weight. Both faces carry satin nickel lever handles with
52 mm round rose plates, projecting necks, curved elbows and 18 mm cylindrical grips.
The handles move with the leaf and remain part of its collision hull; they do not operate a latch. The
avoid lane is single level; the guide's optional stacked pallets are not included, and its
posts are 1.2 kg rigid bodies that fall when hit and come back on reset. The stair landing
keeps only its west OSB panel so the pallet side and the drive-under passage stay open. Both
stair copies include the two belay frames, with visual-only unloaded ropes; the ropes do not
restrain the robot. The debris rails have visible hinges but fixed collision poses, and enter
the policy's height scan. Each stair beam rises 35 cm from its tread to the side wall
(the top beam meets a belay upright). The red braces run front-to-back through the landing
centre, crossing the route to the pallets and the passage underneath, as in the guide's
top view. These are collision obstacles included in the height scan. The maze is the guide's example map read off the
drawing at 1.22 m per cell, with 19 of the 20 diagonals it cuts; its tarp is a black
non-colliding sheet over the walls, so the interior is dark and the robot's light matters.
Doors, avoid and maze lanes have no framed floor: their pads are 1.5 cm mats on the hall.

The Additional Obstacles lanes are our reading of the guide's renders and short notes. p. 87
shows how a pinch point is built but not how high it hangs; the renders hang them from the
top rail, so ours span 35–95 cm and clear everything walkable under them, even rails layered
to 40 cm (the diagonals meet the walls at cell corners, the pinch points hang mid-cell).
Where each one hangs follows p. 5's render, and they are fixed, not loose. "Stack high enough to
challenge the maximum capability of the robot" (p. 30) leaves the layered K-Rail height to
the organisers; 20 cm is ours. The slip disks are modelled 5 mm thick rather than the guide's
3 mm (with a 3 mm disk's mass), because PhysX will not cook a 3 mm convex hull for the GPU and
falls back to CPU collision for it. The disk-on-OSB friction of 0.30 / 0.25 is an assumption for
a thin wood or plastic disk, not a measurement; the seat takes it whatever touches it
(`min` combine), while the disk's own face has the same friction as every other wooden surface
in the scene, so a foot grips the disk as it grips the ramp beside it. A rigid disk
bears on its seat at its rim, so the push that turns it is 0.30 × load × 25 cm over the
foot's distance from the bolt: more than a real, flexing disk needs when the foot is nearer
the bolt. A physics probe on the GPU pipeline at the sim's 5 ms step (2026-09-18) measured, on
one disk: seated to 0.001 mm over 2 s; with no load it held against 1 N across its radius
at 20 cm and turned at 2 N; under 100 N at 20 cm it held at 30 N and turned at 40 N
(0.30 × 104 N × 25/20 = 39 N); 50 N along the radius moved it 0.0 mm; 20 N pulling it off
lifted it 5.0 mm, the bolt's play. Walking on the disks, or anywhere in the new lanes, is not
evaluated.

The standard lanes' flat and opposing-slope settings are all included, as is the Additional
Obstacles setting of the K-Rails, stepfields and ramps. Not included: Additional Obstacles for
gravel (pinch points, p. 27) and pallets (p. 46); sloped gravel and pallets as levels of their
own (`--difficulty slopes` tilts them instead); the slip disks p. 43 suggests for Center in
Alleys; and the magnetised angled obstacles (p. 88). Of the dexterity fixtures, the square's
two Linear Align/Inspect tasks and the maze rooms' two omni tasks are included.

## Files and checks

- `geometry.py`: dimensions, lane catalogue (`LANE_ORIGINS`), the difficulty settings, shared
  builders (floors, railings, K-rails, pallets, pipes, inspect tasks, pinch points) and start
  poses. Adding a lane to the catalogue adds a level to the sim, in its arena's row.
- `terrains.py`: stepfields, ramps and their slip disks, alleys and pallets on the standard lane.
- `structures.py`: doors, avoid and stairs.
- `maze.py`: the maze layout and fixtures.
- `build.py`, `textures.py`: USD export, materials, scanner geometry, the scene
  manifest, a plan/3D preview, and the geometry arguments the sim takes.
- `runtime.py`: resets for the loose bodies (gravel, avoid posts and slip disks).
- `tests/`: geometry, the difficulty settings, real USD-reference composition, and the
  loose-body binding.

Export without launching the simulator, using a Python environment with `numpy`,
`scipy`, `usd-core` and `Pillow` (plus `matplotlib` for `--preview`):

```bash
python3 -m competition.build --output competition/generated/competition.usda --preview   # from Isaac/go2_omniverse
python3 -m unittest discover -s competition/tests -v
```

The USD test skips when USD bindings are unavailable. Geometry and export tests do not need a GPU.
Generated USD files, JSON manifests, textures and previews are ignored by Git.

The implementation was checked with geometry/topology tests, both static and
dynamic USD exports composed under the real terrain namespace, and visual inspection
of the generated preview. In the rescue sim, `./run_sim.sh --headless --smoke-steps N`
loads every lane in turn.

Binding a reset view disables `RigidPrim`'s default contact-sensor preparation. Adding physics schemas and sleep attributes
to all 7,816 already-running stones invalidated their GPU body indices, producing
`Unresolved rigid dynamic index` followed by a CUDA illegal-memory-access crash.
The reset view now binds without reauthoring the stones' physics settings.
Smoke runs verify startup and resets. Gravel calibration and course traversal
still need evaluation.
