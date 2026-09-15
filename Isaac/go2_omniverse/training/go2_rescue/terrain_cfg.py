# The ground the Rescue policy trains on: Rescue's stairs, not the stock generator's.
#
# WHY NOT THE STOCK ROUGH TERRAIN. The 2026 policy trained on Isaac Lab's `ROUGH_TERRAINS_CFG`
# and its run finished at terrain level 1.5 of 10 (the promotion rule, since fixed, could
# not promote a robot that was allowed to turn). But even at full difficulty that terrain
# is not Rescue's ground:
#
#                        stock rough        Rescue's maze (maze_terrain.py)
#   riser height         5-23 cm            10 cm, in flights of 2-4
#   tread depth          30 cm              20 cm
#   walls                none               1.0 m, either side of a 1.2 m corridor
#   slopes               0-22 deg           none
#
# WALLS ARE BACK, AND THE REASON IS A MEASUREMENT, not a change of mind. They were dropped on
# the argument that avoiding walls is the nav stack's job and a gait policy should have no
# opinion about them. That argument is right and this change SERVES it. What the first run
# proved is that a policy which has never seen a wall does not have "no opinion" -- it has an
# untrained one, and it is bad. `training/measure_scan.py`, flat ground, +1.0 m/s commanded:
#
#     clean scan                         +0.53 m/s
#     a wall 0.5 m ahead in the scan     -0.47 m/s     the robot drives BACKWARDS
#     side walls only, a 1.2 m corridor  -0.03 m/s     the robot stops
#
# A 1 m wall clips the scan at -1.0; the tallest riser in the first training terrain was
# 0.16 m, reading -0.33. The only meaning the policy has for a large negative ahead is "a step
# up", so a wall is a step it cannot climb and it retreats. Clamping the scan does not help
# (swept: every value from -1.00 to -0.40 still reverses, worst at -0.70) because a clamp only
# makes the wall a SHORTER step.
#
# So the point of the tall boxes below is not to teach avoidance. It is to teach that a
# clipped column is simply NOT GROUND, so the policy stops reacting to one at all -- which is
# precisely the "no opinion" the nav stack needs. They are sparse, so most commanded paths
# never meet one, and the reward pays only for velocity tracking, so the cheapest way to
# satisfy it near an obstacle is to keep tracking rather than to swerve.
#
# Set `walls`' proportion to 0.0 to drop them again; nothing else depends on them.
#
# The 20 cm tread is the geometric difference that matters. A Go2's feet are 0.387 m apart
# fore-aft, so on a 30 cm tread the front and rear feet can share a step and on a 20 cm one
# they never do; a policy that only ever placed feet on 30 cm treads has not practised the
# footfall pattern the maze demands. And the walls matter to the OBSERVATION: the height
# scan clips at -1.0, a wall top reads well past that, and the stock terrain has nothing
# tall enough to ever produce a clipped column -- so every wall the robot sees in the maze is
# an input the policy never saw in training. See the next paragraph for why that is left so.
#
# WHAT THIS GENERATOR DOES ABOUT IT, by proportion of the 200 tiles:
#
# THE MIX WAS REBALANCED AFTER TWO FAILED RUNS, and the evidence for it is the most useful
# thing learned all night. Both runs used a stairs-dominated mix (60 % of tiles) and neither
# learned to climb: at iteration 3300 not one robot in 64 gained 10 cm of height. The CONTROL
# is what settles it -- P2Dingo's policy, trained on slopes, rough ground and boxes with NO
# STAIRS AT ALL, climbs these same stairs on this same task with the same welded robot:
#
#                                   ours, stairs-heavy      P2Dingo, no stairs
#     rose >10 cm at 1 m/s                    0.0 %                  14.1 %
#     best height gained                      0.05 m                  1.14 m
#
# WHY STAIRS-HEAVY TAUGHT LESS THAN NO STAIRS. A stair is a DISCRETE barrier: a policy that
# cannot clear a riser gets no gradient toward clearing it, and `measure_climbcost.py` shows
# it is actively paid not to try -- a climbing robot earns 61 % of what one on level ground
# earns, and the shortfall is almost entirely the POSITIVE terms (gait -27, velocity tracking
# -10 over a 12 s window) rather than any penalty. Slopes and rough ground have no barrier:
# difficulty is continuous, so vertical competence is learned by degrees and then transfers to
# steps. So the ladder now starts where P2Dingo's did and stairs are the minority:
#
#   0.15  stairs on 20 cm treads, the maze's own geometry, ascending and descending Rescue's 10 cm riser is level ~4.5
#         of 10, so a policy at level 6+ has margin on both sides of it. 16 cm on a 20 cm
#         tread is a 39 degree stair, which is the hardest thing here on purpose.
#   0.15  the same stairs on 30 cm treads -- easier, and the stock geometry
#   0.30  slopes, up and down, to 22 degrees: the continuous path to vertical competence
#   0.20  random rough ground: foot placement on uneven footing, P2Dingo's largest share
#   0.15  boxes, Go2-scaled (the stock config applies this scaling by name; carried here)
#   0.05  tall obstacles, for the height scan -- see below
#
#
# PLATFORMS ARE 2 m, NOT THE STOCK 3 m. The robot spawns within +-0.5 m of the platform
# centre and is about 0.7 m long, so 2 m keeps its feet on the flat at spawn and puts the
# first step under a metre away -- the terrain is met within the first second of every
# episode rather than after a 1.5 m walk, which is what makes the curriculum's path-length
# rule a test of the stairs rather than of the platform.

from isaaclab.terrains import TerrainGeneratorCfg
import isaaclab.terrains as terrain_gen

# Rescue's maze geometry, from ../../terrain_cfg.py and ../../maze_terrain.py.
RESCUE_RISER_M = 0.10
RESCUE_TREAD_M = 0.20

# Riser range across the curriculum. Rescue's 10 cm riser now sits at difficulty
# (0.10 - 0.05) / (0.13 - 0.05) = 0.63, i.e. level ~6.3 of 10. The ceiling is 13 cm, not 16:
# the maze needs 10, and the first two runs stalled around level 1.7 (a 7 cm riser), so range
# spent above 13 cm is range the policy never reaches and never learns from.
STAIR_RISER_RANGE = (0.05, 0.13)

# Maze wall height, so a training obstacle clips the height scan the way a real wall does.
RESCUE_WALL_HEIGHT_M = 1.0

_STAIR_PLATFORM_M = 2.0

RESCUE_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    # Enabled again in `UnitreeGo2RescueRoughEnvCfg.__post_init__`, which is where the
    # stock chain enables it on ITS generator; set here too so the intent is visible.
    curriculum=True,
    sub_terrains={
        # -- Rescue's stairs: 20 cm treads ------------------------------------------------
        # "Inverted" is the ascending one: the platform is the pit and every direction out of
        # it is up. "Pyramid" puts the platform at the apex, so every direction out is down.
        "stairs_up_20": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.075,
            step_height_range=STAIR_RISER_RANGE,
            step_width=RESCUE_TREAD_M,
            platform_width=_STAIR_PLATFORM_M,
            border_width=1.0,
            holes=False,
        ),
        "stairs_down_20": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.075,
            step_height_range=STAIR_RISER_RANGE,
            step_width=RESCUE_TREAD_M,
            platform_width=_STAIR_PLATFORM_M,
            border_width=1.0,
            holes=False,
        ),
        # -- the stock 30 cm tread, kept for generality ------------------------------------
        "stairs_up_30": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.075,
            step_height_range=STAIR_RISER_RANGE,
            step_width=0.30,
            platform_width=_STAIR_PLATFORM_M,
            border_width=1.0,
            holes=False,
        ),
        "stairs_down_30": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.075,
            step_height_range=STAIR_RISER_RANGE,
            step_width=0.30,
            platform_width=_STAIR_PLATFORM_M,
            border_width=1.0,
            holes=False,
        ),
        # -- general footing, at the Go2 scaling the stock config applies by name -----------
        "boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.15, grid_width=0.45, grid_height_range=(0.025, 0.1), platform_width=_STAIR_PLATFORM_M
        ),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.20, noise_range=(0.01, 0.06), noise_step=0.01, border_width=0.25
        ),
        # -- tall obstacles: what a maze wall looks like to the height scan ------------------
        # 1 m tall, the maze's wall height, so they clip the scan exactly as a wall does.
        # `platform_height` 0.02 keeps the spawn platform a flush slab rather than raising the
        # robot onto a block (this terrain is written for stepping stones, where the default
        # is to start on top of one).
        "walls": terrain_gen.MeshRepeatedBoxesTerrainCfg(
            proportion=0.05,
            platform_width=_STAIR_PLATFORM_M,
            platform_height=0.02,
            object_params_start=terrain_gen.MeshRepeatedBoxesTerrainCfg.ObjectCfg(
                num_objects=3, height=RESCUE_WALL_HEIGHT_M, size=(0.4, 0.4), max_yx_angle=0.0
            ),
            object_params_end=terrain_gen.MeshRepeatedBoxesTerrainCfg.ObjectCfg(
                num_objects=7, height=RESCUE_WALL_HEIGHT_M, size=(0.4, 0.4), max_yx_angle=0.0
            ),
        ),
        # -- slopes, up and down, to 22 degrees ---------------------------------------------
        "slope_up": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.15, slope_range=(0.0, 0.4), platform_width=_STAIR_PLATFORM_M, border_width=0.25
        ),
        "slope_down": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.15, slope_range=(0.0, 0.4), platform_width=_STAIR_PLATFORM_M, border_width=0.25
        ),
    },
)
