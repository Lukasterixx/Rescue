# Cup model (Sketchfab)

"High-Resolution 3D Cup Model (FBX)" by **fayazg1aa** (<https://sketchfab.com/fayazg1aa>), from
<https://sketchfab.com/3d-models/high-resolution-3d-cup-model-fbx-9030ed8db34a4110b6d3c508a9d57807>.
Licensed under the Creative Commons Attribution 4.0 International licence (CC BY 4.0,
<https://creativecommons.org/licenses/by/4.0/>; full text in `LICENCE` in this folder).

Copied unmodified from Lukas's D1Training repository (commit 5e19028,
`demos/cup/pick_demo/assets/High-Resolution_3D_Cup_Model_FBX.usdz`, sha256 `da2194b0…5dc677`), where it was
downloaded on 2026-09-17 as Sketchfab's USDZ export and its licence confirmed against Sketchfab's model API.

**Changes.** The committed file is unmodified. At run time `rescue_sim/cup_asset.py` derives the simulation asset
from it, written to `rescue_sim/generated/` and not committed: metres with Z up, scaled independently in diameter
and height to a 55 x 100 mm mug, a plain ceramic material, and a cylinder-and-box collider with mass and friction.

The material is provided as-is, with no warranties, as section 5 of the licence states.
