#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
set +u
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ACTION_WS=$(cd -- "${ROOT}/../.." && pwd -P)
set -u
export ROS_DOMAIN_ID=${ELE_ROS_DOMAIN_ID:-1}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export UNITREE_SDK2_ROOT=/opt/unitree_sdk2
export UNITREE_DDS_ROOT=/opt/unitree_sdk2
export LD_LIBRARY_PATH="/opt/ele_elevator/lib:/opt/unitree_sdk2/lib:/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/opt/ele_elevator/python:${PYTHONPATH:-}"
export ELE_ELEVATOR_CONFIG=${ELE_ELEVATOR_CONFIG:-${ROOT}/config/elevator.yaml}
if [[ -f "${ROOT}/config/realsense_runtime.env" ]]; then
  source "${ROOT}/config/realsense_runtime.env"
fi
export ELE_CAMERA_BACKEND=${ELE_CAMERA_BACKEND:-pipeline}
export ELE_CAMERA_FPS=${ELE_CAMERA_FPS:-6}
export ELE_CAMERA_OUTPUT_FPS=${ELE_CAMERA_OUTPUT_FPS:-6}
command=${1:-serve}
if (( $# )); then shift; fi
case "$command" in
  serve) exec bash "${ACTION_WS}/scripts/serve.sh" --direction "${1:-up}" --arm "${2:-auto}" ;;
  up|down) exec python3 "${ROOT}/scripts/service_command.py" start "$command" "$@" ;;
  start) exec python3 "${ROOT}/scripts/service_command.py" start "$@" ;;
  stop) exec python3 "${ROOT}/scripts/service_command.py" stop "$@" ;;
  camera)
    if [[ "${ELE_CAMERA_BACKEND:-ros}" == pipeline ]]; then
      camera_binary=/opt/ele_realsense/pipeline_bridge/bin/ele_realsense_pipeline
      camera_args=(--ros-args -r __node:=camera -r __ns:=/camera -p width:=640 -p height:=480 -p "fps:=${ELE_CAMERA_FPS}" -p "publish_fps:=${ELE_CAMERA_OUTPUT_FPS}")
      for arg in "$@"; do
        case "${arg%%:=*}" in
          width|height|fps|publish_fps|serial_number|reliable_qos) camera_args+=(-p "${arg}") ;;
          *) echo "Pipeline camera parameters: width:=N height:=N fps:=N publish_fps:=N serial_number:='<serial>' reliable_qos:=true|false" >&2; exit 2 ;;
        esac
      done
      exec python3 "${ROOT}/scripts/managed_service.py" camera -- "${camera_binary}" "${camera_args[@]}"
    fi
    if [[ -n "${ELE_REALSENSE_SETUP:-}" ]]; then
      set +u
      source "${ELE_REALSENSE_SETUP}"
      set -u
    fi
    depth_profile=depth_module.depth_profile
    color_profile=rgb_camera.color_profile
    camera_command=(ros2 launch realsense2_camera rs_launch.py)
    camera_launch="$(ros2 pkg prefix realsense2_camera)/share/realsense2_camera/launch/rs_launch.py"
    if grep -q "'name': 'depth_module.profile'" "${camera_launch}"; then
      depth_profile=depth_module.profile
      color_profile=rgb_camera.profile
      camera_command=(ros2 launch "${ROOT}/scripts/realsense_compat.launch.py")
    fi
    exec python3 "${ROOT}/scripts/managed_service.py" camera -- "${camera_command[@]}" \
      camera_namespace:=camera camera_name:=camera \
      enable_color:=true enable_depth:=true enable_infra1:=false enable_infra2:=false \
      enable_gyro:=false enable_accel:=false enable_sync:=true align_depth.enable:=true \
      "${color_profile}:=640x480x${ELE_CAMERA_FPS}" "${depth_profile}:=640x480x${ELE_CAMERA_FPS}" "$@" ;;
  vision) exec python3 "${ROOT}/scripts/managed_service.py" vision -- ros2 run g1_actions elevator_vision "$@" ;;
  perception) exec python3 "${ROOT}/scripts/perception.py" ;;
  stop-perception) exec python3 "${ROOT}/scripts/managed_service.py" stop ;;
  preview) exec ros2 run g1_actions elevator_press --direction "${1:-up}" --arm "${2:-auto}" ;;
  press-once) exec python3 "${ROOT}/scripts/press_once.py" --direction "${1:-up}" --arm "${2:-auto}" ;;
  press) exec ros2 run g1_actions elevator_press --direction "${1:-up}" --arm "${2:-auto}" --execute ;;
  hover) exec ros2 run g1_actions elevator_press --direction "${1:-up}" --arm "${2:-auto}" --execute --hover-only ;;
  driver)
    export CONFIG_FILE="${G1_ROBOT_CONFIG:-${ROOT}/config/robot.yaml}"
    exec bash "${ACTION_WS}/../brsd_manipulation/src/brsd_upbody_control/script/run_driver.sh" unitree_g1 "${1:-fake}" ;;
  check) exec python3 "${ROOT}/scripts/check_runtime.py" "$@" ;;
  height-check) exec env PYTHONPATH="${ROOT}:${PYTHONPATH:-}" python3 -m g1_actions.actions.elevator.height_check "$@" ;;
  model) exec ros2 run g1_actions elevator_model_check "$@" ;;
  calibrate) exec ros2 run g1_actions elevator_calibrate "$@" ;;
  test) python3 -m unittest discover -s "${ROOT}/test" -p 'test_core.py' -v
    exec python3 -m unittest discover -s "${ROOT}/test" -p 'test_panel_motion.py' -v ;;
  test-motion) exec python3 "${ROOT}/test/test_two_stage_driver_fake.py" ;;
  record-right) exec python3 "${ROOT}/scripts/record_right.py" "$@" ;;
  status) exec python3 "${ROOT}/scripts/service_command.py" status "$@" ;;
  *)
    cat <<'EOF'
常驻启动（终端 1，保持运行）：
  bash scripts/ele.sh

调用（终端 2，或其他 ROS 2 程序）：
  bash scripts/ele.sh start up
  bash scripts/ele.sh start down
  bash scripts/ele.sh stop

ROS 2 接口：
  /elevator/start  std_srvs/srv/Trigger
  /elevator/stop   std_srvs/srv/Trigger
  /elevator/service_status  std_msgs/msg/String

脚本自动设置 ROS_DOMAIN_ID=1 和 rmw_cyclonedds_cpp。
start 默认方向为 up；stop 只停止任务，不关闭相机、视觉和驱动。
Ctrl+C 退出常驻服务并清理本次启动的组件。
EOF
    ;;
esac
