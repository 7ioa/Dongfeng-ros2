#!/usr/bin/env bash
set -eo pipefail
DONGFENG_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DONGFENG_ROOT/scripts/ros_environment.sh"
if [[ ! -f "$DONGFENG_ROOT/install_control/local_setup.bash" ]]; then
  echo "请先在另一个终端运行 bash launch_car.sh，完成编译和地图启动。" >&2
  exit 1
fi
source "$DONGFENG_ROOT/install_control/local_setup.bash"
exec ros2 run dongfeng_bringup teleop_keyboard "$@"
