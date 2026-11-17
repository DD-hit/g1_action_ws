#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
if [[ ! -x "${ROOT}/.venv/bin/python" ]]; then
  echo "请先运行 bash scripts/setup_handshake_video.sh" >&2
  exit 2
fi
unset PYTHONHOME
export PYTHONNOUSERSITE=1
set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/../../install/setup.bash"
set -u
export PYTHONPATH="${ROOT}/src/g1_actions:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=${ELE_ROS_DOMAIN_ID:-1}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec "${ROOT}/.venv/bin/python" -m g1_actions.perception.rgbd_live "$@"
