
Outil de compilation :
    colcon build --packages-select qbo_vision   --cmake-clean-cache   --allow-overriding qbo_vision
    source install/setup.bash

Calibartion caméra :

    ros2 run usb_cam usb_cam_node_exe --ros-args   -p video_device:=/dev/video0   -p pixel_format:=yuyv   -p image_width:=640   -p image_height:=480   -p framerate:=30.0

    ros2 run camera_calibration cameracalibrator   --size 8x6   --square 0.0285   --no-service-check   --ros-args   -r image:=/image_raw

    mkdir -p /tmp/calib_temp && \
    tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/calib_temp && \
    mv /tmp/calib_temp/ost.yaml ~/qbo_ws/src/qbo_vision/config/calibration/new_calibration.yaml && \
    rm -r /tmp/calib_temp

Démarage caméra :

    ros2 launch qbo_vision camera_with_calibration.launch.py


Node face tracker :
    ros2 run qbo_vision face_tracker_node --ros-args --params-file ~/qbo_ws/src/qbo_vision/config/face_tracker.yaml
    ros2 run rqt_image_view rqt_image_view /qbo_face_tracking/debug_image
    ros2 topic echo /qbo_face_tracking/face_pos_and_dist

    # ✅ ACTIVER le tracking (CPU va monter)
    ros2 service call /qbo_face_tracker/enable std_srvs/srv/SetBool "{data: true}"

    # Réponse attendue :
    # success: True
    # message: 'Face tracking enabled'

    # ❌ DÉSACTIVER le tracking (CPU retourne à ~0%)
    ros2 service call /qbo_face_tracker/enable std_srvs/srv/SetBool "{data: false}"

    # Si vous voulez qu'il démarre activé
    ros2 run qbo_vision face_tracker --ros-args -p start_enabled:=true

    # Voir les logs détaillés
    ros2 run qbo_vision face_tracker --ros-args --log-level debug

    # Voir les services disponibles
    ros2 service list | grep enable

    # Voir le type du service
    ros2 service type /qbo_face_tracker/enable


Node face follower :
    ros2 run qbo_vision face_follower_node --ros-args --params-file src/qbo_vision/config/face_follower.yaml

    ros2 topic pub /qbo_face_tracking/face_pos_and_dist qbo_msgs/msg/FacePosAndDist '{face_detected: false, u: 0.0, v: 0.0, distance_to_head: 1.0, image_width: 800, image_height: 600}'


Node face recognition :

    ros2 topic echo /qbo_face_recognition/result

    # Enregistrer une nouvelle personne
    ros2 service call /face_recognition/start_enroll qbo_msgs/srv/StartEnrollPerson "{name: 'Sylvain'}"

    # Lister les personnes enregistrées
    ros2 service call /face_recognition/list_persons qbo_msgs/srv/ListPersons

Dépandance :
    sudo apt install ros-humble-rqt-graph
    sudo apt install ros-humble-image-pipeline

Commande divers utile :
    rosparam list /face_follower

    ros2 topic hz /camera_left/image_raw
    ros2 topic delay /camera_left/image_raw
    ros2 topic hz /qbo_face_tracking/face_pos_and_dist
    v4l2-ctl --list-formats-ext -d /dev/video0

    ros2 run tf2_tools view_frames
    pkill -f realsense2_camera_node

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
    pose:
        header:
          frame_id: 'odom'
        pose:
          position:
            x: 0.5
            y: 0.2
            z: 0.0
          orientation:
            z: 0.0
            w: 1.0
    "



