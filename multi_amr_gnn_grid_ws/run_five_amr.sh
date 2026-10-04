#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +local:root >/dev/null
trap 'xhost -local:root >/dev/null' EXIT

docker run --rm -it \
  --network host \
  --security-opt label=disable \
  --env DISPLAY="${DISPLAY:-:0}" \
  --env QT_X11_NO_MITSHM=1 \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  --env FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/multi_amr/config/fastdds_udp.xml \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume "$workspace_dir":/workspaces/multi_amr:rw \
  --name five_amr_sim \
  ros2-humble-gazebo-multi-amr:latest \
  bash -lc '
    cd /workspaces/multi_amr
    colcon build --symlink-install --packages-select \
      multi_modal_worlds multi_modal_robot_description multi_amr_coordination
    source install/setup.bash
    ros2 launch multi_amr_coordination warehouse_five_robots.launch.py
  '
