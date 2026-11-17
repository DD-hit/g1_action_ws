cmake_minimum_required(VERSION 3.8)
project(g1_action_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Execute.action"
  "msg/ActionInfo.msg"
  "msg/ExecutionFeedback.msg"
  "msg/ExecutionGoal.msg"
  "msg/ExecutionResult.msg"
  "msg/HandObservation.msg"
  "msg/PalmObservation.msg"
  "msg/ServerStatus.msg"
  "srv/GetExecution.srv"
  "srv/ListActions.srv"
  "srv/StopActive.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
