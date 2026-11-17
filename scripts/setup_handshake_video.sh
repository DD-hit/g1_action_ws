#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
if [[ ! -x "${ROOT}/.venv/bin/python" ]]; then
  selected=${HANDSHAKE_PYTHON:-}
  if [[ -z "${selected}" ]]; then
    for candidate in python3.12 python3.11 python3.10; do
      if command -v "${candidate}" >/dev/null 2>&1; then selected=${candidate}; break; fi
    done
  fi
  if [[ -z "${selected}" ]]; then
    echo "请安装 Python 3.10～3.12，或通过 HANDSHAKE_PYTHON 指定解释器" >&2
    exit 2
  fi
  "${selected}" -m venv "${ROOT}/.venv"
fi
PYTHON="${ROOT}/.venv/bin/python"
if ! "${PYTHON}" -m pip --version >/dev/null 2>&1; then "${PYTHON}" -m ensurepip; fi
"${PYTHON}" -m pip install -r "${ROOT}/requirements-handshake-video.txt"
"${PYTHON}" - "${ROOT}" <<'PY'
import hashlib
from pathlib import Path
import sys
import urllib.request

model = Path(sys.argv[1]) / 'src/g1_actions/models/hand_landmarker.task'
expected = 'fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1'
if not model.is_file() or hashlib.sha256(model.read_bytes()).hexdigest() != expected:
    url = 'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task'
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError('Downloaded model checksum mismatch')
    model.parent.mkdir(parents=True, exist_ok=True)
    model.write_bytes(data)
print('Environment: ' + sys.executable)
print('Model: ' + str(model))
PY
