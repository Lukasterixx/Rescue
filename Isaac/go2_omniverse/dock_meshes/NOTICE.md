# Unitree Go2 with expansion dock (CAD)

Copied unmodified from Lukas's D1Training repository (`description/meshes/go2/`, 054983a), whose
`third_party/unitree_go2_cad/NOTICE.md` this repeats.

"GO2 simplified, with expansion dock" (`GO2简化-带拓展坞-0911.stp`, a STEP file exported 2023-09-11), by Unitree
Robotics, downloaded 2026-09-26 from
<https://oss-global-cdn.unitree.com/static/GO2%E7%AE%80%E5%8C%96-%E5%B8%A6%E6%8B%93%E5%B1%95%E5%9D%9E-0911.7z>. The link
came from a MYBOTSHOP forum post of 2026-07-06. The unpacked STEP's sha256 is `348cf13b…a407855`. It is not in this
repository.

**No licence is stated** in the archive, the STEP file's header or on the download link. Unitree publishes the file
for anyone to download, but that does not license redistribution. The derived meshes below are committed for
visualisation in this research repository. Check Unitree's terms before this repository or the meshes are made public
or passed on.

| This repository | Source |
| --- | --- |
| `expansion_dock.stl` (sha256 `2f7a8cc4…68e85c`) | the dock's body, tessellated from the STEP |
| `expansion_dock_rails.stl` (sha256 `b37a6eed…bcbf3a634`) | the dock's two rails and their screws, tessellated from the STEP |
| `arm_weld.D1_MOUNT_POS` (numbers only) | measured from the STEP's rails and screw holes (D1Training F-117) |

**Changes** (in D1Training). `description/dock_from_cad.py` tessellates the STEP with Open CASCADE (the `cascadio`
wheel) to 0.5 mm and 0.5 rad, keeps the two parts on top of the trunk, moves them from the CAD's frame into the Go2
URDF's `base_link` (registered by the hip motors' axes, `description/mount_from_cad.py`), converts millimetres to
metres and welds shared vertices. Nothing else in the CAD is used, and none of it is committed.
