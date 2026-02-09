#!/usr/bin/env bash
set -e

PKG=go2_control_cpp
CONTAINER=p2dingo
# Note: Inside the container, the path is still /root/ws 
# because of the docker volume map we fixed in Step 1.
CONTAINER_WS=/root/ws

echo "▶ [Remote] Starting Incremental Build for $PKG..."

docker exec $CONTAINER bash -c "
  source /opt/ros/humble/setup.bash
  cd $CONTAINER_WS
  
  colcon build \
    --packages-select $PKG \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
"

echo "✅ Build Complete"
