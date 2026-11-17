#!/usr/bin/env bash
set -euo pipefail
ACTION_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
action=${1:-list}
if (( $# )); then shift; fi
case "$action" in
  up|down) exec bash "$0" run elevator --direction "$action" "$@" ;;
  serve) exec bash "${ACTION_WS}/src/g1_actions/scripts/ele.sh" serve "$@" ;;
  elevator)
    command=${1:-status}
    if (( $# )); then shift; fi
    case "$command" in
      serve) exec bash "$0" serve "$@" ;;
      start) direction=${1:-up}; if (( $# )) && [[ "$1" != --* ]]; then shift; else direction=up; fi
        exec bash "$0" run elevator --direction "$direction" "$@" ;;
      stop|status) exec bash "$0" "$command" "$@" ;;
      model) exec bash "${ACTION_WS}/src/g1_actions/scripts/ele.sh" model "$@" ;;
      *) echo "电梯动作入口：elevator serve | start up/down | stop | status；请勿绕过统一调度。" >&2; exit 2 ;;
    esac ;;
  handshake) exec bash "$0" run handshake "$@" ;;
  run|list|status|stop|get)
    set +u
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    set -u
    export ROS_DOMAIN_ID=${ELE_ROS_DOMAIN_ID:-1}
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    exec python3 -m g1_actions.client "$action" "$@" ;;
  *) echo "用法：bash scripts/g1.sh serve | run <动作> | list | status | stop | get <请求ID>" >&2; exit 2 ;;
esac
