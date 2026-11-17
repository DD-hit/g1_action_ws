#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/../../install/setup.bash"
set -u
export ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONNOUSERSITE=1 PYTHONPATH="${ROOT}/src/g1_actions:${PYTHONPATH:-}"
exec "${ROOT}/.venv/bin/python" -m g1_actions.actions.handshake.read_state "$@"
