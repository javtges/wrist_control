#!/usr/bin/env bash
# Build usb_cam 0.8.1 with the image-timestamp fix as an overlay in odrive_ros_ws.
#
# Why: the apt usb_cam (0.8.1) computes its epoch shift with `tv_usec / 1000.0`
# instead of `tv_usec`, so every image/camera_info stamp lags the ROS clock by a
# random 0-1 s fixed at node start (0.54 s measured on 2026-10-20). The
# overlay fixes the one line; everything else is upstream 0.8.1.
#
#   cd ~/hsa-ws/wrist_control/odrive_ros_ws
#   src/camera_tracking/tools/build_usb_cam_patched.sh
#   source install/setup.bash          # the overlay now shadows /opt/ros/kilted's usb_cam
set -euo pipefail
ws="$(cd "$(dirname "$0")/../../.." && pwd)"
here="$(cd "$(dirname "$0")" && pwd)"
cd "$ws"
if [ ! -d src/usb_cam ]; then
  git clone --branch 0.8.1 --depth 1 https://github.com/ros-drivers/usb_cam.git src/usb_cam
fi
if git -C src/usb_cam apply --check --reverse "$here/usb_cam_timestamp.patch" 2>/dev/null; then
  echo "usb_cam already patched"
else
  git -C src/usb_cam apply "$here/usb_cam_timestamp.patch"
fi
source /opt/ros/kilted/setup.bash
# pin the ROS python: a pyenv/venv python on PATH makes CMake pick one without catkin_pkg
rm -rf build/usb_cam
PATH=/usr/bin:$PATH colcon build --packages-select usb_cam --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
echo "usb_cam overlay built; source install/setup.bash and relaunch the camera"
