#!/usr/bin/env bash
set -eo pipefail

# --- NETWORK SETUP: MULTI-SUBNET SUPPORT ---
# The Unitree Go2 internal network is 192.168.123.x
# The SIYI Camera default network is 192.168.144.x
# We add an alias IP to eth0 so we can talk to both.

INTERFACE="enP8p1s0"
TARGET_IP="192.168.144.100/24"

# Check if we have privileges to modify networking
if ip link show "$INTERFACE" > /dev/null 2>&1; then
    # Check if the interface already has an IP in the 144 subnet
    if ! ip addr show "$INTERFACE" | grep -q "192.168.144."; then
        echo "[entrypoint] Setting up multi-subnet alias on $INTERFACE..."
        if ip addr add "$TARGET_IP" dev "$INTERFACE"; then
            echo "[entrypoint] Successfully added $TARGET_IP to $INTERFACE"
        else
            echo "[entrypoint] WARNING: Failed to add IP alias. Ensure container runs with --privileged"
        fi
    else
        echo "[entrypoint] Subnet 192.168.144.x already configured on $INTERFACE."
    fi
else
    echo "[entrypoint] WARNING: Interface $INTERFACE not found. Skipping network configuration."
fi

# --- SOURCE ENVIRONMENTS ---
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source Livox Workspace
if [[ -f "/root/ws_livox/install/setup.bash" ]]; then
    source "/root/ws_livox/install/setup.bash"
fi

# Source Main Workspace
if [[ -f "/root/ws/install/setup.bash" ]]; then
    source "/root/ws/install/setup.bash"
fi

# Prioritize NVIDIA/Local libraries
export LD_LIBRARY_PATH=/lib:/usr/lib/aarch64-linux-gnu/nvidia:${LD_LIBRARY_PATH}

# Fix Double Free: Force Preload of NVIDIA OpenCV (4.8) and GTSAM
export LD_PRELOAD=/usr/lib/libopencv_core.so.4.10.0:/usr/lib/libopencv_highgui.so.4.10.0:/usr/lib/libopencv_imgproc.so.4.10.0:/opt/gtsam_stack/lib/libgtsam.so

# Fallback: if no command is provided, start an interactive shell
if [[ $# -eq 0 ]]; then
  set -- bash
fi

echo "[entrypoint] Ready. Executing: $*"
exec "$@"
