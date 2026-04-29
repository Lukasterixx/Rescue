[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange.svg)](https://docs.ros.org/en/humble/index.html)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.1.0-red.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-0.3.0-purple.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Linux platform](https://img.shields.io/badge/platform-Ubuntu--22.04-green.svg)](https://releases.ubuntu.com/22.04/)

Welcome to the Rescue Repo, here you will find the source code for the autonomous SLAM quadruped first simulated on Isaac Sim with ros2 and then deployed on a Unitree Go2 EDU

# SIM Installation Guide

This guide installs:

- Ubuntu 22.04 native Linux
- ROS 2 Humble
- Miniconda
- Isaac Sim 5.1 from pip
- Latest Isaac Lab from source
- Unitree ROS 2 SDK
- Rescue ROS 2 workspace dependencies

This guide replaces the old Isaac Sim 4.1 / Orbit / Isaac Lab 0.3.1 installation flow. Do **not** install Omniverse Launcher or download the old archive build unless you specifically want the binary install method.

---

## 0. Assumptions

This guide assumes:

- You are on Ubuntu 22.04 native install.
- You have a supported NVIDIA GPU and recent NVIDIA driver installed.
- You want ROS 2 Humble for your external ROS stack.
- Isaac Sim will be installed into a Miniconda environment using pip.
- Isaac Sim 5.1 uses Python 3.11.
- ROS 2 Humble from apt uses Python 3.10, which is normal.

Check GLIBC before continuing:

```bash
ldd --version
```

Ubuntu 22.04 should report GLIBC 2.35, which is suitable for Isaac Sim 5.1 pip packages.

---

## 1. Install Ubuntu 22.04 Native

Use a native Ubuntu 22.04 install for this workflow.

Docker is not covered by this guide.

---

## 2. Install ROS 2 Humble

Follow the official ROS 2 Humble debian package install:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

After installing ROS 2 Humble, add this to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
```

Then reload your shell:

```bash
source ~/.bashrc
```

Install common ROS 2 build tools:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  build-essential \
  cmake \
  git
```

Initialize rosdep:

```bash
sudo rosdep init || true
rosdep update
```

---

## 3. Install Miniconda

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh

source ~/miniconda3/bin/activate
conda init --all
conda config --set auto_activate_base false
```

Close and reopen the terminal, or run:

```bash
source ~/.bashrc
```

---

## 4. Create Isaac Lab Conda Environment

Isaac Sim 5.1 requires Python 3.11, so create the environment with Python 3.11.

```bash
conda create -n env_isaaclab python=3.11 -y
conda activate env_isaaclab

python -m pip install --upgrade pip setuptools wheel
```

Optional but useful:

```bash
pip install toml pyyaml pydot gitpython hydra-core omegaconf tensorboard
```

---

## 5. Install Isaac Sim 5.1 with pip

Make sure the `env_isaaclab` environment is active:

```bash
conda activate env_isaaclab
```

Install Isaac Sim 5.1:

```bash
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

Install the PyTorch version recommended by the current Isaac Lab pip-install guide for x86_64 Linux:

```bash
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
```

Verify Isaac Sim starts:

```bash
isaacsim
```

On first launch, accept the NVIDIA Omniverse EULA when prompted.

You can also launch the full experience directly:

```bash
isaacsim isaacsim.exp.full.kit
```

---

## 6. Install Latest Isaac Lab from Source

Clone Isaac Lab:

```bash
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd ~/IsaacLab
```

Use the latest branch:

```bash
git checkout main
git pull
```

Install Linux build dependencies:

```bash
sudo apt update
sudo apt install -y cmake build-essential
```

Install Isaac Lab and all learning framework dependencies:

```bash
conda activate env_isaaclab
cd ~/IsaacLab
./isaaclab.sh --install
```

If you only need RSL-RL, you can install a smaller set instead:

```bash
./isaaclab.sh --install rsl_rl
```

Verify Isaac Lab:

```bash
cd ~/IsaacLab
conda activate env_isaaclab

./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
```

You should see Isaac Sim open with an empty black viewport.

Optional RSL-RL training smoke test:

```bash
cd ~/IsaacLab
conda activate env_isaaclab

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task=Isaac-Velocity-Rough-Anymal-C-v0 \
  --headless
```

Stop it with `Ctrl+C` once you know it starts.

---

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

Here are some helper functions to start the sim and nav code (place in your ~/.bashrc): 

```
rvizz() {
    use_fastrtps
    ros2 launch go2_control_cpp visualize_go2.launch.py
}

nav() {
  use_fastrtps
  cd ~/Rescue/Isaac/go2_ws || return

  source install/setup.bash

  rm ~/Rescue/Isaac/go2_ws/install/go2_control_cpp/share/go2_control_cpp/maps/map2d.pgm


  # Start RViz2 in the background and capture PID
  rvizz &
  RVIZ_PID=$!

  # Run ros2 launch in the foreground (interactive)
  ros2 launch go2_control_cpp minimal_bt_launch_sim.py
}

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
```
