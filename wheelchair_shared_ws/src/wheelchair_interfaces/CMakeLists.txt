cmake_minimum_required(VERSION 3.8)
project(wheelchair_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlIntent.msg" "msg/DiscreteAction.msg" "msg/ControlStatus.msg" "msg/ExecutionStatus.msg" "msg/ObstacleMap.msg"
  "msg/CandidateStatus.msg" "msg/IntentEvent.msg" "msg/InputHealth.msg" "msg/IntentAck.msg" "msg/TaskStatus.msg" "msg/EyeSample.msg"
  "msg/DirectionEvent.msg" "msg/NavigationIntentStatus.msg"
  "srv/ComputeDiscretePath.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
