#!/usr/bin/env bash
set -e

cd ~/Rescue/Isaac/go2_omniverse

eval "$(conda shell.bash hook)"
conda activate env_isaaclab

# Prevent system ROS Python 3.10 paths leaking into Isaac Sim's Python 3.11
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# ROS 2 bridge config
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI CYCLONEDDS_HOME CYCLONEDDS_CONFIG ROS_DISCOVERY_SERVER

# Use Isaac Sim's bundled ROS 2 bridge libraries, not /opt/ros/humble Python packages
ISAAC_BRIDGE_EXT="$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge"
export LD_LIBRARY_PATH="$ISAAC_BRIDGE_EXT/humble/lib:${LD_LIBRARY_PATH}"

# Optional, but often still needed on Ubuntu
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# RTX lidar config location for Isaac Sim 5.x
LIDAR_CONFIG_DIR="$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs"
mkdir -p "$LIDAR_CONFIG_DIR"
cp -f ./Isaac_sim/Unitree/Unitree_L1.json "$LIDAR_CONFIG_DIR/"

python main.py --robot_amount 1 --robot go2 --terrain rough --custom_env maze