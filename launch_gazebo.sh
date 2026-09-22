#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec gz sim --render-engine ogre -r "${PROJECT_DIR}/worlds/dongfeng.sdf" "$@"
