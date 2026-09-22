#!/usr/bin/env bash
set -eo pipefail
DONGFENG_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Use exactly the car launcher's build, display and scene-loading entry point.
exec bash "$DONGFENG_ROOT/launch_car.sh" --autonomy "$@"
