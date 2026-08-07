cmake_minimum_required(VERSION 3.8)
project(qbo_home_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HomeParameter.msg"
  "msg/HomeTarget.msg"
  "msg/HomeIntent.msg"
  "msg/IntentResult.msg"
  "msg/DialogueContext.msg"
  "srv/ParseHomeCommand.srv"
  "srv/ResolveHomeIntent.srv"
  "srv/ExecuteHomeIntent.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
