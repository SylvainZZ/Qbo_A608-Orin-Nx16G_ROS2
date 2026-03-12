#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <atomic>
#include <random>
#include <string>

#include <qbo_msgs/msg/face_pos_and_dist.hpp>

class FaceFollower : public rclcpp::Node
{
public:
    enum class FollowerState
    {
        SPIN_TEST,
        TRACK_NEAR,
        TRACK_FAR,
        LOST_RECENT,
        LOST_LONG
    };

public:
    FaceFollower();

    void headToZeroPosition();

private:
    void faceCallback(
        const qbo_msgs::msg::FacePosAndDist::SharedPtr msg);

    void controlLoop();

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void pickSearchTarget();

    float clamp(float v, float lo, float hi);

    float deadband(float v, float db);

    float ema(float x, float prev, float alpha);

    float stepAxis(
        float current_pos,
        float &current_vel,
        float target,
        float dt,
        float max_vel,
        float max_acc);

    const char *stateToString(FollowerState s) const;

    void updateState(FollowerState next, const char *reason);

    bool runBaseSpinTest(const rclcpp::Time &t);

    void runBaseLostRecent(float lost_time);

    void runBaseLostLong(const rclcpp::Time &t);

    void runBaseTracking();

    void publishHead();

    void publishBase(float yaw_rate);

private:
    rclcpp::Subscription<qbo_msgs::msg::FacePosAndDist>::SharedPtr face_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    float u_f_;
    float v_f_;
    bool filter_init_;

    float target_pan_;
    float target_tilt_;

    float cmd_pan_;
    float cmd_tilt_;
    float vel_pan_;
    float vel_tilt_;

    rclcpp::Time last_time_;
    bool have_time_;
    rclcpp::Time last_face_time_;
    rclcpp::Time last_search_time_;

    rclcpp::Time base_spin_test_start_;
    bool base_spin_test_started_ = false;
    bool base_spin_test_done_ = false;

    bool move_head_;
    bool move_base_;

    float ema_alpha_;
    float deadband_u_px_;
    float deadband_v_px_;

    float pan_min_;
    float pan_max_;
    float tilt_min_;
    float tilt_max_;

    float head_max_vel_pan_;
    float head_max_vel_tilt_;
    float head_max_acc_pan_;
    float head_max_acc_tilt_;

    float search_max_vel_pan_;
    float search_max_vel_tilt_;
    float search_max_acc_pan_;
    float search_max_acc_tilt_;

    float tracking_gain_pan_;
    float tracking_gain_tilt_;
    float camera_lateral_offset_m_;
    float camera_offset_min_dist_m_;

    float base_deadband_px_;
    float base_max_yaw_rate_;
    float base_yaw_sign_;
    bool base_debug_log_;
    float base_assist_release_factor_;
    float base_head_limit_margin_;

    std::string base_cmd_topic_;
    bool base_spin_test_mode_;
    float base_spin_test_yaw_rate_;
    float base_spin_test_duration_sec_;
    bool base_search_spin_enabled_;
    float base_search_yaw_rate_;
    float base_search_long_lost_sec_;
    float base_search_restart_delay_sec_;
    float base_search_side_deadband_px_;

    float lost_hold_sec_;
    float lost_reacquire_sec_;
    float distance_to_head_m_;
    float distance_hysteresis_m_;

    float search_interval_sec_;
    float search_pan_min_step_;
    float search_tilt_min_step_;
    float search_walk_pan_step_;
    float search_walk_tilt_step_;
    int search_tilt_update_every_;
    float search_min_pan_;
    float search_max_pan_;
    float search_min_tilt_;
    float search_max_tilt_;

    int image_width_;
    int image_height_;
    bool have_camera_info_;

    float yaw_from_joint_ = 0.0;
    float pitch_from_joint_ = 0.0;

    float fx_ = 535.0;
    float fy_ = 535.0;
    float cx_ = 320.0;
    float cy_ = 240.0;

    std::mt19937 rng_;

    bool is_zeroing_head_ = false;
    int search_update_count_ = 0;

    float last_seen_u_ = 0.0f;
    bool have_last_seen_u_ = false;
    bool was_face_lost_ = false;
    rclcpp::Time face_lost_since_;

    bool base_search_active_ = false;
    float base_search_dir_sign_ = 1.0f;
    float base_search_accum_yaw_ = 0.0f;
    rclcpp::Time base_search_last_time_;
    rclcpp::Time base_search_last_end_time_;
    bool base_assist_active_ = false;

    float last_face_distance_ = 0.0f;
    bool have_last_face_distance_ = false;
    bool last_face_detected_msg_ = false;

    FollowerState current_state_ = FollowerState::TRACK_NEAR;
    FollowerState previous_state_ = FollowerState::TRACK_NEAR;
};