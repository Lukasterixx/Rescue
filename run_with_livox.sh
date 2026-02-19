#!/bin/bash
set -e

# Note: ROS and Workspace sourcing is now handled by entrypoint.sh automatically.

# --- START LIVOX ---
echo "[container] Starting Livox driver in background…"

# NOTE: Using 'mid360_driver_only.launch.py' is better for the robot than 'rviz_...'
# because Rviz consumes huge CPU/GPU resources. Change this line only if you
# strictly need Rviz inside the container.
ros2 launch livox_ros_driver2 mid360_driver_only.launch.py &
LAUNCH_PID=$!

# --- APPLY REAL-TIME PRIORITY FIX ---
echo "[container] Waiting for Livox Node to spawn..."
sleep 5  # Wait for ROS 2 to bring up the C++ node

# Find the PID of the actual driver binary (not the python launch script)
# We search for "livox_ros_driver2_node" which is the compiled C++ executable
DRIVER_PID=$(pgrep -f "livox_ros_driver2_node" | head -n 1)

if [ -n "$DRIVER_PID" ]; then
    echo "[container] Found Driver PID: $DRIVER_PID. Applying Real-Time Priority..."
    
    # Try to set FIFO scheduling with priority 90
    if chrt -f -p 90 "$DRIVER_PID"; then
        echo "[container] ✅ SUCCESS: Real-Time Priority set to 90."
    else
        echo "[container] ❌ FAILED: Could not set priority."
        echo "    -> Ensure you run docker with '--privileged' or '--cap-add=SYS_NICE'"
    fi
else
    echo "[container] ⚠️ WARNING: Could not find 'livox_ros_driver2_node'. Priority not applied."
fi

# --- START REMOTE CONNECTION ---
echo "[container] Starting P2RemoteConnection..."

# A) Setup Token
if [ -f "/root/.go2_token" ]; then
  export GO2_API_TOKEN="$(cat /root/.go2_token)"
  echo "[container] Loaded GO2_API_TOKEN"
else
  echo "[container] ❌ WARNING: ~/.go2_token not found. Web auth will fail!"
  export GO2_API_TOKEN="" 
fi

# B) Navigate to Python App
P2_PKG_DIR="/root/ws/src/P2RemoteConnection/src/p2_remote_connection"

if [ -d "$P2_PKG_DIR" ]; then
  pushd "$P2_PKG_DIR" >/dev/null

  # C) Start FastAPI Backend
  echo "[container] Starting FastAPI..."
  python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000 &
  API_PID=$!

  # D) Start Static Web Server
  echo "[container] Starting UI Server..."
  python3 -m http.server 8081 &
  UI_PID=$!

  popd >/dev/null
else
  echo "[container] ERROR: P2RemoteConnection dir not found: $P2_PKG_DIR"
fi

# --- KEEP ALIVE ---
echo "[container] System running."
echo "  - Launch PID: $LAUNCH_PID"
echo "  - Driver PID: $DRIVER_PID"

wait
