#!/usr/bin/env bash
# Model-only preflight; this script never starts a robot driver.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
WORKSPACE=$(cd -- "${ROOT}/../.." && pwd -P)
if [[ $# -lt 1 ]]; then
  echo '用法：handshake-preflight <RGB-D 目录> --arm right --output <新结果目录> [--samples 8]' >&2
  exit 2
fi
INPUT=$1; shift
ARM=right
OUTPUT=
SAMPLES=8
STATE=
while (( $# )); do
  case "$1" in
    --arm) ARM=$2; shift 2 ;;
    --output) OUTPUT=$2; shift 2 ;;
    --samples) SAMPLES=$2; shift 2 ;;
    --state) STATE=$2; shift 2 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done
[[ "$ARM" == left || "$ARM" == right ]] || { echo '手臂必须为 left 或 right' >&2; exit 2; }
[[ "$SAMPLES" =~ ^[1-9][0-9]*$ ]] && (( SAMPLES <= 100 )) || { echo 'samples 必须为 1～100' >&2; exit 2; }
[[ -n "$OUTPUT" ]] || { echo '必须指定 --output 新结果目录' >&2; exit 2; }
bash "${ROOT}/scripts/g1.sh" handshake-target-preview "$INPUT" --arm "$ARM" --output "$OUTPUT"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u
BUILD="${WORKSPACE}/build/handshake_preflight"
cmake -S "${ROOT}/tools/handshake_preflight" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release > "$OUTPUT/cmake.log" 2>&1
cmake --build "$BUILD" -j2 > "$OUTPUT/build.log" 2>&1
DRIVER="${WORKSPACE}/src/brsd_manipulation/src/brsd_upbody_control"
STATE_ARGS=()
if [[ -n "$STATE" ]]; then STATE_ARGS+=("$STATE"); fi
CHECK_EXIT=0
# Isolate all MoveIt diagnostic topics from the production ROS graph.
ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  "$BUILD/handshake_preflight" \
  "${ROOT}/src/g1_actions/robot/g1_29dof_mode_15.urdf" \
  "${ROOT}/src/g1_actions/robot/g1_29dof_mode_15.srdf" \
  "$DRIVER/robot/moveit/unitree_g1/joint_limits.yaml" \
  "$OUTPUT/targets.jsonl" "$OUTPUT/preflight.yaml" "$SAMPLES" "${STATE_ARGS[@]}" \
  > "$OUTPUT/planning.log" 2>&1 || CHECK_EXIT=$?
if (( CHECK_EXIT )); then
  echo "预检查未通过，未发送运动指令：" >&2
  tail -15 "$OUTPUT/planning.log" >&2
  exit "$CHECK_EXIT"
fi
"${ROOT}/.venv/bin/python" - "$OUTPUT" <<'PY'
import json,sys
from pathlib import Path
import yaml
root=Path(sys.argv[1]);report=yaml.safe_load((root/'preflight.yaml').read_text())
(root/'preflight.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))
PY
