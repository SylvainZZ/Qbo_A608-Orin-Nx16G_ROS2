

colcon build --packages-select qbo_vision   --cmake-clean-cache   --allow-overriding qbo_vision
source install/setup.bash

ros2 run usb_cam usb_cam_node_exe --ros-args   -p video_device:=/dev/video0   -p pixel_format:=yuyv   -p image_width:=640   -p image_height:=480   -p framerate:=30.0

ros2 run camera_calibration cameracalibrator   --size 8x6   --square 0.0285   --no-service-check   --ros-args   -r image:=/image_raw

mkdir -p /tmp/calib_temp && \
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/calib_temp && \
mv /tmp/calib_temp/ost.yaml ~/qbo_ws/src/qbo_vision/config/calibration/new_calibration.yaml && \
rm -r /tmp/calib_temp

ros2 launch qbo_vision camera_with_calibration.launch.py
ros2 run qbo_vision face_follower_node --ros-args --params-file src/qbo_vision/config/face_follower.yaml
ros2 run qbo_vision face_detector_node --ros-args --params-file src/qbo_vision/config/face_detector.yaml

ros2 topic pub /qbo_face_tracking/face_pos_and_dist qbo_msgs/msg/FacePosAndDist '{face_detected: false, u: 0.0, v: 0.0, distance_to_head: 1.0, image_width: 800, image_height: 600}'

sudo apt install ros-humble-rqt-graph
sudo apt install ros-humble-image-pipeline

rosparam list /face_follower
ros2 topic echo /qbo_face_tracking/face_pos_and_dist


