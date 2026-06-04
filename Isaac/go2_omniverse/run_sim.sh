#!/usr/bin/env bash
set -e

cd ~/Rescue/Isaac/go2_omniverse

CONDA_ENV_NAME="${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"

# Locate a conda installation even when `conda` isn't on PATH (e.g. when this
# script is launched from a non-interactive shell such as the map web Dev tab).
find_conda_base() {
  if [ -n "${CONDA_EXE:-}" ] && [ -x "$CONDA_EXE" ]; then
    dirname "$(dirname "$CONDA_EXE")"
    return 0
  fi

  for candidate in \
    "$HOME/miniconda3" \
    "$HOME/anaconda3" \
    "/opt/conda" \
    "/usr/local/miniconda3" \
    "/usr/local/anaconda3"; do
    if [ -x "$candidate/bin/conda" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  if command -v conda >/dev/null 2>&1; then
    dirname "$(dirname "$(command -v conda)")"
    return 0
  fi

  return 1
}

activate_conda_env() {
  local conda_base
  if ! conda_base="$(find_conda_base)"; then
    echo "[run_sim] Conda is not available. Install Miniconda or set CONDA_EXE/ISAAC_SIM_CONDA_ENV." >&2
    return 127
  fi

  if [ -f "$conda_base/etc/profile.d/conda.sh" ]; then
    # shellcheck disable=SC1090
    . "$conda_base/etc/profile.d/conda.sh"
  else
    export PATH="$conda_base/bin:$PATH"
    eval "$("$conda_base/bin/conda" shell.bash hook)"
  fi

  conda activate "$CONDA_ENV_NAME"
}

activate_conda_env

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