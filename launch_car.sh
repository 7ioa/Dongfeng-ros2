#!/usr/bin/env bash
set -eo pipefail
DONGFENG_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DONGFENG_ROOT/scripts/ros_environment.sh"
bash "$DONGFENG_ROOT/scripts/build_control.sh"
source "$DONGFENG_ROOT/install_control/local_setup.bash"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
echo "启动地图与小车；小车在 AB 端直路中间。"
echo "请在另一个终端运行：cd \"$DONGFENG_ROOT\" && bash keyboard_control.sh"
DONGFENG_LAUNCH=simulation.launch.py
if [[ "${1:-}" == "--autonomy" ]]; then
  DONGFENG_LAUNCH=autonomy.launch.py
  shift
fi
exec ros2 launch dongfeng_bringup "$DONGFENG_LAUNCH" "$@"
