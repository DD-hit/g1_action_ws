#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
PYTHON="${ROOT}/.venv/bin/python"
if [[ ! -x "${PYTHON}" ]]; then
  echo "请先运行 bash scripts/setup_handshake_video.sh，创建本项目的 .venv" >&2
  exit 2
fi
unset PYTHONHOME
export PYTHONNOUSERSITE=1
export PYTHONPATH="${ROOT}/src/g1_actions"
MODULE=g1_actions.perception.video_review
if [[ "${1:-}" == --axes-review ]]; then
  shift
  MODULE=g1_actions.perception.palm_frame_review
elif [[ "${1:-}" == --target-preview ]]; then
  shift
  MODULE=g1_actions.actions.handshake.target_preview
elif [[ "${1:-}" == --rgbd-replay ]]; then
  shift
  MODULE=g1_actions.perception.rgbd_replay
fi
exec "${PYTHON}" -m "${MODULE}" "$@"
