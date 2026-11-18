#!/usr/bin/env bash
# One production runtime for all actions; no action-specific startup routes.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/../../install/setup.bash"
set -u
export ELE_ROS_DOMAIN_ID=${G1_ROS_DOMAIN_ID:-${ELE_ROS_DOMAIN_ID:-1}}
export ROS_DOMAIN_ID=${ELE_ROS_DOMAIN_ID} RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source "${ROOT}/scripts/ros_transport.sh"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${ROOT}/src/g1_actions:/opt/ele_elevator/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/opt/ele_elevator/lib:/opt/unitree_sdk2/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
exec python3 "${ROOT}/src/g1_actions/scripts/serve.py" "$@"
