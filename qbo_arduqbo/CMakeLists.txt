cmake_minimum_required(VERSION 3.8)
project(qbo_arduqbo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(qbo_msgs REQUIRED)

include_directories(include)

# Bibliothèque du contrôleur Dynamixel
add_library(dynamixel_controller_lib
  src/controllers/dynamixel_controller.cpp
)
ament_target_dependencies(dynamixel_controller_lib
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamixel_sdk
  dynamixel_workbench_toolbox
  diagnostic_updater
  qbo_msgs
)

# Executable principal
add_executable(qbo_dynamixel
  src/qbo_dynamixel.cpp
)
target_link_libraries(qbo_dynamixel
  dynamixel_controller_lib
)
ament_target_dependencies(qbo_dynamixel
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  dynamixel_sdk
  dynamixel_workbench_toolbox
  diagnostic_updater
  qbo_msgs
)

# Installation
install(TARGETS
  qbo_dynamixel
  dynamixel_controller_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Lancement auto des tests si activé
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
