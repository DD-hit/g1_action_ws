#!/usr/bin/env bash
set -euo pipefail
ACTION_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
WORKSPACE=$(cd -- "${ACTION_WS}/../.." && pwd -P)
for arg in "$@"; do
  case "$arg" in --build-base|--build-base=*|--install-base|--install-base=*|--log-base|--log-base=*)
    echo "构建产物统一位于 ${WORKSPACE}/build、install、log" >&2; exit 2;; esac
done
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "请先运行 brsd_manipulation/src/brsd_upbody_control/script/build.sh unitree_g1" >&2
  exit 2
fi
source "${WORKSPACE}/install/setup.bash"
set -u
export MAKEFLAGS="-j${BUILD_JOBS:-4}"
cd "${WORKSPACE}"
colcon --log-base "${WORKSPACE}/log" build --base-paths "${ACTION_WS}/src" --packages-select g1_action_interfaces g1_actions --build-base "${WORKSPACE}/build" --install-base "${WORKSPACE}/install" --ignore-user-meta --symlink-install --executor sequential "$@"
