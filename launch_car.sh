#!/usr/bin/env bash
set -eo pipefail
DONGFENG_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DONGFENG_ROOT/scripts/ros_environment.sh"
bash "$DONGFENG_ROOT/scripts/build_control.sh"
source "$DONGFENG_ROOT/install_control/local_setup.bash"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
DONGFENG_LAUNCH=simulation.launch.py
if [[ "${1:-}" == "--autonomy" ]]; then
  DONGFENG_LAUNCH=autonomy.launch.py
  shift
  echo "启动自主巡航与红绿灯；按所选任务放置小车，完整场景默认从停车场出发。"
  echo "传感器就绪后自动行驶，窗口同步显示各组灯色。"
else
  echo "启动地图与小车；小车在 AB 端直路中间。"
  echo "请在另一个终端运行：cd \"$DONGFENG_ROOT\" && bash keyboard_control.sh"
fi
exec ros2 launch dongfeng_bringup "$DONGFENG_LAUNCH" "$@"
