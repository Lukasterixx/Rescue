[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange.svg)](https://docs.ros.org/en/humble/index.html)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.1.0-red.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-0.3.0-purple.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Linux platform](https://img.shields.io/badge/platform-Ubuntu--22.04-green.svg)](https://releases.ubuntu.com/22.04/)

Welcome to the Rescue Repo, here you will find the source code for the autonomous SLAM quadruped first simulated on Isaac Sim with ros2 and then deployed on a Unitree Go2 EDU

## Requirements
1. Nvidia RTX 20 series or newer (4070/4080 tested)
2. Minimum 100gb of storage on Ubunutu 22.04
3. Ros2 Humble
4. IsaacSim 4.1
5. IsaacLab 0.3.1 (orbit)


## SIM Installation Guide

**Step I:** Install Ubuntu 22.04 Native Install (docker untested)

**Step I.I:** Install Omniverse: https://developer.nvidia.com/omniverse?sortBy=developer_learning_library%2Fsort%2Ffeatured_in.omniverse%3Adesc%2Ctitle%3Aasc&hitsPerPage=6#section-getting-started

**Step II:** Install Ros2 Humble following this link: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Add source `/opt/ros/humble/setup.bash` to your `~/.bashrc` to source ros2 in each terminal

**Step III:** Install MiniConda
```
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
source ~/miniconda3/bin/activate
conda init --all
conda config --set auto_activate_base false
```
**Step IV:** Install Isaac-sim 4.1 and IsaacLab 0.3.1
Download the archive version 4.1 from https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html
Extract it to your home directory `~/isaacsim`
```
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout tags/v0.3.1 -b my-v0.3.1
```
Put these in your ~/.bashrc so that it sources on each terminal
```
export ISAACSIM_PATH=$HOME/isaacsim
export ISAACSIM_PYTHON_EXE=$ISAACSIM_PATH/python.sh
```
Add a sym link to connect to isaacsim:

`ln -s ${ISAACSIM_PATH} _isaac_sim` 

Run the conda setup:
```
./orbit.sh --conda
conda activate orbit
sudo apt install cmake build-essential
pip install -U pip setuptools wheel build
pip install toml
```
Correct the rsl-rl dependancy
```
cd ~/IsaacLab/source/extensions/omni.isaac.orbit_tasks
code setup.py
```
Add this line to the extras require (replace the existing broken link)
```
"rsl-rl": ["rsl-rl@git+https://github.com/leggedrobotics/rsl_rl.git@v2.0.1#egg=rsl-rl"],`
```
Now reinstall:
```
./orbit.sh --install
```
Run these commands to correct any toml PEP517 isolation errors (make sure conda orbit is active): 
```
conda activate orbit
# omni.isaac.orbit
cd ~/IsaacLab/source/extensions/omni.isaac.orbit
cat > pyproject.toml <<'EOF'
[build-system]
requires = ["setuptools>=64", "wheel", "toml"]
build-backend = "setuptools.build_meta"
EOF
pip install -e .

# omni.isaac.orbit_tasks
cd ../omni.isaac.orbit_tasks
[bashtxt.txt](https://github.com/user-attachments/files/24633163/bashtxt.txt)
cat > pyproject.toml <<'EOF'
[build-system]
requires = ["setuptools>=64", "wheel", "toml"]
build-backend = "setuptools.build_meta"
EOF
pip install -e .

# omni.isaac.orbit_assets
cd ../omni.isaac.orbit_assets
cat > pyproject.toml <<'EOF'
[build-system]
requires = ["setuptools>=64", "wheel", "toml"]
build-backend = "setuptools.build_meta"
EOF
pip install -e .
```
Then reinstall the specific packages with problems: 
```
# reset the env flags in the same shell
unset PIP_USE_PEP517
unset PIP_NO_BUILD_ISOLATION
hash -r

# reinstall each extension (PEP 517 + isolation ON by default)
cd ~/IsaacLab/source/extensions/omni.isaac.orbit
pip install -e .

cd ../omni.isaac.orbit_tasks
pip install -e .

cd ../omni.isaac.orbit_assets
pip install -e .
```

**Step V:** Unitree SDK for ethernet connection 
```
git clone https://github.com/unitreerobotics/unitree_ros2
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl

cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
# If build failed, try run: `export LD_LIBRARY_PATH=/opt/ros/humble/lib` first.
colcon build --packages-select cyclonedds #Compile cyclone-dds package
```
make sure to change the foxy source to a humble: 
`sudo gedit ~/unitree_ros2/setup.sh`

Now follow the network setup from https://github.com/unitreerobotics/unitree_ros2
Update your bash file to source the ros2 unitree sdk:

```
source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/install/setup.bash
```
and some helper functions: 
```
# --- DDS toggle helpers (optional) ---
use_cyclonedds() { export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; echo "DDS: CycloneDDS"; }
use_fastrtps()   { unset CYCLONEDDS_URI CYCLONEDDS_HOME CYCLONEDDS_CONFIG; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; echo "DDS: Fast DDS"; }

# Run Isaac sim with Fast DDS regardless of current shell env
sim() {
  (
    # Force Fast DDS for this subshell only
    unset CYCLONEDDS_URI CYCLONEDDS_HOME CYCLONEDDS_CONFIG
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    # (optional) ensure Humble is sourced if your shell hasn't already
    # source /opt/ros/humble/setup.bash

    cd ~/Rescue/Isaac/go2_omniverse && ./run_sim.sh
  )
}

slam() {
  use_fastrtps
  cd ~/Rescue/Isaac/go2_ws || return

  source install/setup.bash

  # Start RViz2 in the background and capture PID
  rviz2 -d ~/Rescue/Isaac/go2_ws/src/go2_control_cpp/config/mapping.rviz &
  RVIZ_PID=$!

  # Run ros2 launch in the foreground (interactive)
  ros2 launch go2_control_cpp minimal_bt_launch_sim.py
}
```
**Step VII:** Install all ros2 dependancies to build code in `~/Rescue/Isaac/go2_ws` 

```
cd ~/Rescue/Isaac/go2_ws
conda deactivate
sudo rosdep init || true
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
sudo apt install nlohmann-json3-dev
```
**Step VII:** Setup Isaac Sim world

You may have to adjust the dependancy of streamsdk if errors appear:
```
code ~/IsaacLab/source/apps/orbit.python.kit
"omni.kit.streamsdk.plugins" = {version = "2.5.1", exact = true}
```
Open `FarmReal.usd` from ~/Rescue/Isaac/go2_omniverse/envs/FarmReal in Isaac Sim:
```
cd ~/isaacsim
./isaac-sim.selector.sh
# make sure you enable ros2 humble bridge (not deprecated version)
```
Export FLATTENED USD to ~/Rescue/Isaac/go2_omniverse/envs `rotate.usd`. This should Allow the launcher script to attach materials and props to the world.

```
If dependancies for isaac lab are missing its because the setup failed to install them onto the sim's Kit python, fix with this: 
```
# --- Use KIT'S embedded Python everywhere ---
PY=~/IsaacLab/_isaac_sim/kit/python/bin/python3

# 0) Make sure pip exists in Kit Python
$PY -m ensurepip --upgrade

# 1) Upgrade packaging tooling
$PY -m pip install --upgrade pip setuptools wheel

# 2) (optional) Quiet some ROS/launch warnings
$PY -m pip install pyyaml pydot

# 3) Install PyTorch built for CUDA 11.8 (works on RTX 4080)
#    (Pick ONE of the two blocks below. If you already used 2.0.1+cu118 successfully, keep it.)

# --- Option A: what you used earlier (stable with Isaac Sim 2023.1) ---
$PY -m pip install --index-url https://download.pytorch.org/whl/cu118 \
  torch==2.0.1+cu118 torchvision==0.15.2+cu118

# --- Option B: newer torch (only if you want to update) ---
# $PY -m pip install --index-url https://download.pytorch.org/whl/cu118 \
#   torch==2.1.2 torchvision torchaudio

# 4) Misc deps some Orbit/Sim code expects
$PY -m pip install gitpython hydra-core omegaconf tensorboard

# 5) Install the exact Isaac Lab pin for RSL-RL without touching Torch
$PY -m pip install --no-deps \
  "rsl-rl@git+https://github.com/leggedrobotics/rsl_rl.git@v2.0.1#egg=rsl-rl"

# 6) Quick verification
$PY - <<'PYCODE'
import torch, sys
print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
print("device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")
import rsl_rl
print("rsl_rl OK")
PYCODE
```

## User Guide
If you installed the unitree_ros2 robot sdk cyclonedds will complain whenever the robot isn't attached to your computer. To fix this, Make sure to enter `use_fastrtps` anytime you want to run a ros2 command alongside the sim, like `ros2 topic echo /livox/imu`

## Ros2 Topics from SIM

**Command and Control**  
- `/robot0/cmd_vel`:  Topic to send velocity commands to the robot for motion control.

**Camera control**:
- `/camera_pose`: Direct the camera in a quaternion direction
- `/record_video_str`: Send once to start video, again to stop video. Saved in share/go2_control_cpp/photos/<str>/
- `/photo_request_str`: Send to take a photo which will be saved in share/go2_control_cpp/photos/<str>/

**Front Camera**  
- `/unitree_go2/front_cam/color_image`: Publishes RGB color images captured by the front camera.
- `/unitree_go2/front_cam/depth_image`: Publishes depth images from the front camera.
- `unitree_go2/front_cam/semantic_segmentation_image`: Publishes semantic segmentation images from the front camera.
- `/unitree_go2/front_cam/info`: Publishes camera information, including intrinsic parameters.

**LIDAR**  
- `/glim_rosnode/points`:  Publishes a point cloud generated by the robot's LIDAR sensor. P2 code uses /odom topic to transform it into the world
- `/livox/imu`: publishes imu data, mostly useless because its at 10Hz

**Odometry and Localization**  
- `/odom`:  Publishes odometry data, including the robot's position, orientation, and velocity.
- `/tf`: publishes the transform odom->livox frame. This is usually provided by GLIM.

**Hotkeys**
- `arrow keys`: Teleport robot in sim (hold to go faster). Updates Odom transform
- `c`: checkpoint. Press to save a loaction
- `r`: reset: teleport robot to checkpoint and reset odom so SLAM begins correctly

## Tutorial

1. Open new terminal and enter `sim`.
2. Once loaded, move the robot around if desired, create obstacles with `create->shape->` in the top left GUI
3. in the `Orbit` panel, set `Follow Mode` to `Robot` for the camera to follow the robot around
4. Open another terminal, enter `slam` to run the go2_control_cpp code for Rescue. If no `map2d.pgm` exists in share/go2_control_cpp/map, it will begin in exploration mode and create the map. If `map2d.pgm` does exist, it will pathplan on it and begin routine scans

## Adding a new node
1. Write your cpp in `/src/`, and hpp in `include/go2_control_cpp/`.
2. Make sure to register as a plugin: 

```
#include <behaviortree_cpp_v3/bt_factory.h>

// Register node as plugin
BT_REGISTER_NODES(factory)
{ 
  factory.registerBuilder<go2_control_cpp::DataServer>(
    "DataServer",
    [](auto & name, auto & config) {
      return std::make_unique<go2_control_cpp::DataServer>(name, config);
    });
}

```
3. Find `walk_bt_node` in `/src` and add the factory constructor to the existing list, Make sure to include the header file, and write `factory_`:
```
#include "go2_control_cpp/data_server.hpp"

factory_.registerBuilder<DataServer>(
    "DataServer",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<DataServer>(name, config);
    });

```
4. In `CMakeLists.txt` add the following:
```
# add any used packages at the top
find_package(sensor_msgs REQUIRED)
```
```
# Link libraries for DataServer BT node
add_library(go2_data_server_node SHARED src/data_server.cpp)
ament_target_dependencies(go2_data_server_node
  ${common_deps}
  sensor_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
)
target_compile_definitions(go2_data_server_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(go2_data_server_node
  ${PCL_LIBRARIES}
)
```
```
# Add to the install block: 
install(
  TARGETS
    ...
    go2_data_server_node
    ...
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```
```
# Add to the walk_bt_node install:

# link in all of your custom‐BT‐node libraries so their constructors get pulled in
target_link_libraries(walk_bt_node
  ...
  go2_data_server_node
  ...
)
```
5. Add the model xml description to `/go2_control_cpp/behaviour_trees/go2_models`:
(The imports in Dataserver use the blackboard, so they are `{enclosed}`. Normal Groot GUI inputs aren't)
```
<Action ID="DataServer">
        <input_port name="panels" 
                    type="std::vector<go2_control_cpp::Panel>" 
                    default="{panels}"
                    description="Vector of Panel structs from PathPlanner"/>
        <input_port name="average_angle" 
                    type="double" 
                    default="{average_angle}"/>
        
        <output_port name="current_panel" 
                      type="go2_control_cpp::CurrentPanel" 
                      default="{current_panel}"
                      description="Struct containing ID, geometry (len/width), and scan settings"/>
    </Action>
```
6. Download Goot 2 from https://www.behaviortree.dev/groot/ and create a launch function in your `~/.bashrc`.
`alias groot='~/Groot2/bin/groot2'`
7. Launch a new terminal and enter `groot` (i am groot)
<img width="1103" height="708" alt="image" src="https://github.com/user-attachments/assets/9ec540fa-e753-4900-b685-85d7a7610bc5" />
Open `/go2_control_cpp/behaviour_trees/go2_tree` (top left next to project)
Import `/go2_control_cpp/behaviour_trees/go2_models` (right side of the models tab)

8. Drag and drop your node, connect it with a wire, then make sure to right click on the project and press save (this updates go2_tree.xml)
9. Now open a new terminal in `~/Rescue/Isaac/go2_ws` and colcon build
