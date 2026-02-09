#!/bin/bash
set -e

# Note: ROS and Workspace sourcing is now handled by entrypoint.sh automatically.
# We can skip straight to launching nodes.

# --- START LIVOX ---
echo "[container] Starting Livox driver in background…"
# The environment is already set up, so we just launch.
ros2 launch livox_ros_driver2 mid360_driver_only.launch.py &
LIVOX_PID=$!

# --- START REMOTE CONNECTION ---
echo "[container] Starting P2RemoteConnection..."

# A) Setup Token
# Ensure you bind mount this file at runtime (e.g., -v ~/.go2_token:/root/.go2_token)
if [ -f "/root/.go2_token" ]; then
  export GO2_API_TOKEN="$(cat /root/.go2_token)"
  echo "[container] Loaded GO2_API_TOKEN"
else
  echo "[container] ❌ WARNING: ~/.go2_token not found. Web auth will fail!"
  export GO2_API_TOKEN="" 
fi

# B) Navigate to Python App
# Path matches the Dockerfile: $WS/src/P2RemoteConnection/...
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

  # Optional: Uncomment if you want the bridge node to start automatically
  # ros2 run p2_remote_connection web_teleop_bridge &
  BRIDGE_PID=""

else
  echo "[container] ERROR: P2RemoteConnection dir not found: $P2_PKG_DIR"
  exit 1
fi

# --- KEEP ALIVE ---
echo "[container] System running."
echo "  - Livox PID: $LIVOX_PID"
echo "  - API PID:   $API_PID"
echo "  - UI PID:    $UI_PID"

wait
