#ifndef QBO_VISION_FACE_FOLLOWER_HPP_
#define QBO_VISION_FACE_FOLLOWER_HPP_

#include <chrono>
#include <memory>
#include <cmath>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <qbo_msgs/msg/face_pos_and_dist.hpp>

#include <opencv2/core.hpp>

class FaceFollower : public rclcpp::Node
{
public:
    FaceFollower();
    ~FaceFollower() = default;

    void headToZeroPosition();  // appelé à la fin du main

private:
    void onInit();
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<qbo_msgs::msg::FacePosAndDist>::SharedPtr face_pos_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_control_pub_;

    // Paramètres ROS
    void declare_and_get_parameters();
    bool move_base_bool_;
    bool move_head_bool_;
    double search_min_pan_, search_max_pan_, search_pan_vel_;
    double search_min_tilt_, search_max_tilt_, search_tilt_vel_;
    double desired_distance_;
    bool send_stop_;

    // Données image/caméra
    int image_width_, image_height_;
    cv::Mat p_;  // Matrice de projection P -> utilisée pour l'angle

    // État de la tête
    float yaw_from_joint_, pitch_from_joint_, min_pitch_;

    // PID Pan (u)
    float u_prev_, u_act_, diff_u_;
    float kp_u_, ki_u_, kd_u_;
    // PID Tilt (v)
    float v_prev_, v_act_, diff_v_;
    float kp_v_, ki_v_, kd_v_;
    // PID Distance (avance)
    float distance_prev_, distance_act_, diff_distance_;
    float kp_distance_, ki_distance_, kd_distance_;
    // PID Yaw (rotation base)
    float yaw_prev_, yaw_act_, diff_yaw_;
    float kp_yaw_, ki_yaw_, kd_yaw_;

    // Callbacks
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void facePositionCallback(const qbo_msgs::msg::FacePosAndDist::SharedPtr msg);

    // Fonctions de mouvement
    void setHeadPositionToFace(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright);
    void setHeadPositionGlobal(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright);
    void sendVelocityBase(float linear_vel, float angular_vel);

    // Contrôle PID
    float controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd);
};

#endif  // QBO_VISION_FACE_FOLLOWER_HPP_
