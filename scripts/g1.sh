#!/usr/bin/env bash
set -euo pipefail
ACTION_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
export ELE_ROS_DOMAIN_ID=${G1_ROS_DOMAIN_ID:-${ELE_ROS_DOMAIN_ID:-1}}
action=${1:-help}
if (( $# )); then shift; fi
case "$action" in
  -h|--help|help)
    cat <<'EOF'
启动后台： bash scripts/g1.sh serve
执行动作： bash scripts/g1.sh run elevator --direction up|down [--arm auto|left|right]
           bash scripts/g1.sh run handshake [--arm auto|left|right] [--hold-s 4]
管理任务： bash scripts/g1.sh list | status | stop | get <请求ID>
关闭后台： bash scripts/g1.sh shutdown
调试工具： bash scripts/g1.sh debug handshake <live|video|axes|replay|target-preview|preflight|read-state|approach> ...
run 默认同步等待最终结果；--feedback 显示阶段；--json 输出结构化结果。
握手 auto：人伸左手→左臂，人伸右手→右臂；目标预检失败时尝试同侧固定备用位姿。
up/down 和旧握手命令仅作为兼容别名；所有执行共用一个调度器。
EOF
    exit 0 ;;
  debug)
    domain=${1:-}; operation=${2:-}
    if [[ "$domain" != handshake || -z "$operation" ]]; then
      echo "用法：g1.sh debug handshake <工具> ..." >&2; exit 2
    fi
    shift 2
    exec bash "$0" "handshake-${operation}" "$@" ;;

  handshake-serve) exec bash "${ACTION_WS}/scripts/handshake_serve.sh" "$@" ;;
  handshake-approach) exec bash "${ACTION_WS}/scripts/handshake_approach.sh" "$@" ;;
  handshake-read-state) exec bash "${ACTION_WS}/scripts/handshake_read_state.sh" "$@" ;;
  handshake-preflight) exec bash "${ACTION_WS}/scripts/handshake_preflight.sh" "$@" ;;
  handshake-target-preview) exec bash "${ACTION_WS}/scripts/handshake_video.sh" --target-preview "$@" ;;
  handshake-replay) exec bash "${ACTION_WS}/scripts/handshake_video.sh" --rgbd-replay "$@" ;;
  handshake-live) exec bash "${ACTION_WS}/scripts/handshake_live.sh" "$@" ;;
  handshake-axes) exec bash "${ACTION_WS}/scripts/handshake_video.sh" --axes-review "$@" ;;
  handshake-video) exec bash "${ACTION_WS}/scripts/handshake_video.sh" "$@" ;;
  up|down) exec bash "$0" run elevator --direction "$action" "$@" ;;
  serve) exec bash "${ACTION_WS}/scripts/serve.sh" "$@" ;;
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
  run|list|status|stop|get|shutdown)
    set +u
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    set -u
    export ROS_DOMAIN_ID=${G1_ROS_DOMAIN_ID:-${ELE_ROS_DOMAIN_ID:-1}}
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export PYTHONPATH="${ACTION_WS}/src/g1_actions:${PYTHONPATH:-}"
    exec python3 -m g1_actions.client "$action" "$@" ;;
  *) echo "用法：bash scripts/g1.sh serve | run <动作> | list | status | stop | get <请求ID>" >&2; exit 2 ;;
esac
