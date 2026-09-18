# RealSense D435 housing mesh (realsense2_description)

Intel's own CAD mesh of the D435 aluminium case, from the ROS 2 package `realsense2_description`
(<https://github.com/IntelRealSense/realsense-ros>), maintained by the LibRealSense ROS Team.
Licensed under the Apache License 2.0 (full text in `LICENCE` in this folder). The D435i shares this case.

Copied unmodified from Lukas's D1Training repository (commit 5e19028,
`demos/cup/pick_demo/assets/realsense/d435_housing.ply`), where `meshes/d435.dae` from
`ros-kilted-realsense2-description` 4.58.3 was converted to binary PLY: the DAE's 18 sub-meshes merged into one
and its (uniform default) material dropped, vertices and triangles otherwise untouched. D1Training's
`third_party/realsense2_description/NOTICE.md` has the download and conversion commands.

**Changes here.** None to the file. At run time `rescue_sim/camera_asset.py` bakes it into the colour optical frame
and removes the colour lens element (so the simulated pinhole camera, which sits at the sensor plane, can see out),
writing a visual-only USD to `rescue_sim/generated/`.

The material is provided as-is, with no warranties, as section 7 of the licence states.
