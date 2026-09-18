#!/usr/bin/env bash
# Sourced by the launch helpers; do not enable nounset while sourcing ROS.
set -eo pipefail
DONGFENG_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DONGFENG_ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ ! -f "$DONGFENG_ROS_SETUP" ]]; then
  echo "未找到 $DONGFENG_ROS_SETUP。请在已安装 ROS 2 Jazzy 的 Ubuntu 24.04 中运行。" >&2
  return 1
fi
source "$DONGFENG_ROS_SETUP"
for DONGFENG_PACKAGE in ros_gz_sim ros_gz_bridge xacro robot_state_publisher rclpy; do
  if ! ros2 pkg prefix "$DONGFENG_PACKAGE" >/dev/null 2>&1; then
    echo "缺少 ROS 包：$DONGFENG_PACKAGE。依赖安装命令见 README 第 8 节。" >&2
    return 1
  fi
done
