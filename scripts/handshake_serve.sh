#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
echo "handshake-serve 已合并到统一后台：g1.sh serve" >&2
exec bash "${ROOT}/scripts/g1.sh" serve "$@"
