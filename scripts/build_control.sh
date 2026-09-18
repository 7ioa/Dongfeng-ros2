#!/usr/bin/env bash
set -eo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/ros_environment.sh"
if ! command -v colcon >/dev/null 2>&1; then
  echo "缺少 colcon，请执行：sudo apt install python3-colcon-common-extensions" >&2
  exit 1
fi
cd "$DONGFENG_ROOT"
# Separate portable build; do not reuse a companion's absolute CMake paths.
# Clean the CMake cache on each build in case the whole folder has been moved.
colcon --log-base "$DONGFENG_ROOT/log_control" build \
  --base-paths "$DONGFENG_ROOT/src" \
  --build-base "$DONGFENG_ROOT/build_control" \
  --install-base "$DONGFENG_ROOT/install_control" \
  --packages-select dongfeng_description dongfeng_bringup \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=OFF
