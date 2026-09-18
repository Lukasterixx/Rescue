#!/usr/bin/env bash
# The rescue sim: the Go2 with its D1 arm and wrist RealSense, the competition's lanes and D1Training's cup demo.
#
#   ./run_sim.sh                               # windowed, starting on the Shifty Gravel lane
#   ./run_sim.sh --level cup                   # start in the cup demo, the Go2 lying beside a mug
#   ./run_sim.sh --headless --smoke-steps 800  # load every level, lie down and stand up, then exit
#   ./run_sim.sh --help                        # every option
#
# Also launched by the VIP-Rescue website's Dev tab, from a non-interactive shell, so conda is found without PATH.
set -e

SIM_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SIM_DIR"

CONDA_ENV_NAME="${ISAAC_SIM_CONDA_ENV:-env_isaaclab}"

find_conda_base() {
  if [ -n "${CONDA_EXE:-}" ] && [ -x "$CONDA_EXE" ]; then
    dirname "$(dirname "$CONDA_EXE")"
    return 0
  fi
  for candidate in "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda /usr/local/miniconda3 /usr/local/anaconda3; do
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

if [ "${CONDA_DEFAULT_ENV:-}" != "$CONDA_ENV_NAME" ]; then
  if ! conda_base="$(find_conda_base)"; then
    echo "[run_sim] Conda is not available. Install Miniconda or set CONDA_EXE/ISAAC_SIM_CONDA_ENV." >&2
    exit 127
  fi
  # Conda's activation scripts can trip over unset variables and failing commands.
  set +e
  # shellcheck disable=SC1091
  . "$conda_base/etc/profile.d/conda.sh"
  conda activate "$CONDA_ENV_NAME"
  set -e
fi

# Keep system ROS (Python 3.10) out of Isaac Sim's Python 3.11.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH

# The ROS 2 bridge: Isaac's bundled Humble libraries, Fast DDS, domain 0 unless told otherwise. The D1 arm's own DDS
# traffic is separate (CycloneDDS, see rescue_sim/d1_arm.py) and does not care which RMW ROS uses.
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI CYCLONEDDS_HOME CYCLONEDDS_CONFIG ROS_DISCOVERY_SERVER
ISAAC_BRIDGE_EXT="$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge"
export LD_LIBRARY_PATH="$ISAAC_BRIDGE_EXT/humble/lib:${LD_LIBRARY_PATH:-}"
if [ -f /usr/lib/x86_64-linux-gnu/libstdc++.so.6 ]; then
  export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6${LD_PRELOAD:+:$LD_PRELOAD}"
fi

# RTX lidar config location for Isaac Sim 5.x.
LIDAR_CONFIG_DIR="$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs"
mkdir -p "$LIDAR_CONFIG_DIR"
cp -f ./Isaac_sim/Unitree/Unitree_L1.json "$LIDAR_CONFIG_DIR/"

export PYTHONDONTWRITEBYTECODE=1
exec python main.py "$@"
