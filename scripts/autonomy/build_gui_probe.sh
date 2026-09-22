#!/usr/bin/env bash
set -eo pipefail
DONGFENG_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$DONGFENG_ROOT/scripts/ros_environment.sh"
for DONGFENG_PC in /opt/ros/jazzy/opt/*/lib/pkgconfig; do
    export PKG_CONFIG_PATH="$DONGFENG_PC:${PKG_CONFIG_PATH:-}"
done
DONGFENG_PROBE="$DONGFENG_ROOT/build_control/gui_probe"
mkdir -p "$DONGFENG_PROBE"
rcc "$DONGFENG_ROOT/scripts/autonomy/gui_probe/resources.qrc" -o "$DONGFENG_PROBE/resources.cc"
g++ -std=c++17 -O2 -shared -fPIC "$DONGFENG_ROOT/scripts/autonomy/gui_probe/GuiSignalProbe.cc" "$DONGFENG_PROBE/resources.cc" -o "$DONGFENG_PROBE/libGuiSignalProbe.so" $(pkg-config --cflags --libs gz-sim8-gui gz-gui8 gz-rendering8 gz-transport13 gz-msgs10 Qt5Core Qt5Quick)
