/*
 * Software License Agreement (GPLv2 License)
 *
 * Copyright (c) 2012 Thecorpora, S.L.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 * Author: Sylvain <sylvain-zwolinski@orange.fr>
 */

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
    FaceFollower(const rclcpp::NodeOptions & options);
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

    rclcpp::Time last_scan_move_;     // horodatage du dernier mouvement aléatoire
    rclcpp::Time pause_start_time_;
    rclcpp::Duration scan_interval_ = rclcpp::Duration::from_seconds(1.0);
    rclcpp::Duration pause_duration_ = rclcpp::Duration::from_seconds(3.0);

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
    // Random
    float last_pan_ = 0.0f;
    float last_tilt_ = 0.0f;
    float min_angle_step_ = 0.1f;  // rad

    enum class ScanPhase { INIT, PAN, RETURN, PAUSE };
    ScanPhase scan_phase_ = ScanPhase::INIT;

    std::vector<float> tilt_levels_;
    int tilt_index_ = 0;

    float pan_step_;
    float current_pan_;
    float current_tilt_;
    bool sens_aller_ = true;
    float last_sent_pan_ = 999.0f;
    float last_sent_tilt_ = 999.0f;

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
