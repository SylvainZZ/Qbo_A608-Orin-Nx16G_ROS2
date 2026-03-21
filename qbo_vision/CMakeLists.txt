cmake_minimum_required(VERSION 3.8)
project(qbo_vision)

# --------------------
# Options de compilation
# --------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------
# Trouver les dépendances nécessaires
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(qbo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Include dirs
include_directories(include ${OpenCV_INCLUDE_DIRS})


# add_executable(face_follower_node src/face_follower_node.cpp)
add_executable(face_follower_node
  src/face_follower.cpp
)
ament_target_dependencies(face_follower_node
  rclcpp
  sensor_msgs
  geometry_msgs
  qbo_msgs
  cv_bridge
  OpenCV
  diagnostic_updater
)
install(TARGETS face_follower_node
  DESTINATION lib/${PROJECT_NAME}
)


# add_executable(face_tracker_node src/face_tracker_node.cpp)
add_executable(face_tracker_node
  src/face_tracker.cpp
)

ament_target_dependencies(face_tracker_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  image_transport
  qbo_msgs
  diagnostic_msgs
  diagnostic_updater
)

target_include_directories(face_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(face_tracker_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS face_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

#add_executable(face_recognition_node src/face_recognition_node.cpp)
add_executable(face_recognition_node
  src/face_recognition.cpp
  src/face_database.cpp
)

ament_target_dependencies(face_recognition_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  qbo_msgs
  message_filters
  diagnostic_msgs
  diagnostic_updater
)

target_include_directories(face_recognition_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(face_recognition_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

install(TARGETS
  face_recognition_node
  DESTINATION lib/${PROJECT_NAME}
)


# add_executable(face_learning_node src/face_learning_node.cpp)
# ament_target_dependencies(face_learning_node
#   rclcpp
#   rclcpp_components
#   cv_bridge
#   sensor_msgs
#   std_msgs
#   image_transport
#   OpenCV
#   qbo_msgs
# )
# install(TARGETS
#   face_learning_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   qbo_vision/face_recognizer_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(PROGRAMS
#   qbo_vision/face_capture_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# ========= ⬇️ Ajout pour nœuds Python ==============
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(qbo_vision_py)

install(
  PROGRAMS
    qbo_vision_py/face_capture_node.py
    # qbo_vision_py/face_recognizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# add_executable(imx219_camera_node src/imx219_camera_node.cpp)
# ament_target_dependencies(imx219_camera_node rclcpp sensor_msgs cv_bridge)
# target_link_libraries(imx219_camera_node ${OpenCV_LIBS})
# install(TARGETS imx219_camera_node DESTINATION lib/${PROJECT_NAME})

# add_executable(gstreamer_camera_node src/gstreamer_camera_node.cpp)
# ament_target_dependencies(gstreamer_camera_node rclcpp sensor_msgs cv_bridge)
# target_link_libraries(gstreamer_camera_node ${OpenCV_LIBS})
# install(TARGETS gstreamer_camera_node DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
