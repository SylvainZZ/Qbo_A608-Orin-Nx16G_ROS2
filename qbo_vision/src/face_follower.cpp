#include "face_follower.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <random>
#include <thread>

using std::placeholders::_1;

std::shared_ptr<FaceFollower> g_node;
std::atomic_bool g_request_shutdown{false};
constexpr float kTwoPi = 6.283185307f;

FaceFollower::FaceFollower()
: Node("qbo_face_following")
{
    move_head_ = declare_parameter("move_head", true);
    move_base_ = declare_parameter("move_base", false);

    ema_alpha_ = declare_parameter("ema_alpha", 0.25);

    deadband_u_px_ = declare_parameter("deadband_u_px", 7.0);
    deadband_v_px_ = declare_parameter("deadband_v_px", 7.0);

    pan_min_ = declare_parameter("pan_min", -0.8);
    pan_max_ = declare_parameter("pan_max", 1.22);

    tilt_min_ = declare_parameter("tilt_min", -0.62);
    tilt_max_ = declare_parameter("tilt_max", 0.35);

    head_max_vel_pan_ = declare_parameter("head_max_vel_pan", 1.2);
    head_max_vel_tilt_ = declare_parameter("head_max_vel_tilt", 1.0);

    head_max_acc_pan_ = declare_parameter("head_max_acc_pan", 4.0);
    head_max_acc_tilt_ = declare_parameter("head_max_acc_tilt", 4.0);
    search_max_vel_pan_ = declare_parameter("search_max_vel_pan", 0.7);
    search_max_vel_tilt_ = declare_parameter("search_max_vel_tilt", 0.55);
    search_max_acc_pan_ = declare_parameter("search_max_acc_pan", 1.8);
    search_max_acc_tilt_ = declare_parameter("search_max_acc_tilt", 1.5);

    tracking_gain_pan_ = declare_parameter("tracking_gain_pan", 2.5);
    tracking_gain_tilt_ = declare_parameter("tracking_gain_tilt", 2.0);
    camera_lateral_offset_m_ = declare_parameter("camera_lateral_offset_m", 0.035);
    camera_offset_min_dist_m_ = declare_parameter("camera_offset_min_dist_m", 0.20);

    base_deadband_px_ = declare_parameter("base_deadband_u_px", 40.0);

    base_max_yaw_rate_ = declare_parameter("base_max_yaw_rate", 0.6);
    base_yaw_sign_ = declare_parameter("base_yaw_sign", 1.0);
    base_debug_log_ = declare_parameter("base_debug_log", true);
    base_assist_release_factor_ = declare_parameter("base_assist_release_factor", 0.70);
    base_cmd_topic_ = declare_parameter(
        "base_cmd_topic",
        std::string("/qbo_arduqbo/base_ctrl/cmd_vel"));
    base_spin_test_mode_ = declare_parameter("base_spin_test_mode", false);
    base_spin_test_yaw_rate_ = declare_parameter("base_spin_test_yaw_rate", 0.25);
    base_spin_test_duration_sec_ = declare_parameter("base_spin_test_duration_sec", 3.0);
    base_search_spin_enabled_ = declare_parameter("base_search_spin_enabled", true);
    base_search_yaw_rate_ = declare_parameter("base_search_yaw_rate", 0.25);
    base_search_long_lost_sec_ = declare_parameter("base_search_long_lost_sec", 6.0);
    base_search_restart_delay_sec_ = declare_parameter("base_search_restart_delay_sec", 1.0);
    base_search_side_deadband_px_ = declare_parameter("base_search_side_deadband_px", 35.0);

    base_head_limit_margin_ =
        declare_parameter("base_head_limit_margin", 0.15);

    lost_hold_sec_ = declare_parameter("lost_hold_sec", 0.8);
    lost_reacquire_sec_ = declare_parameter("lost_reacquire_sec", 1.2);
    distance_to_head_m_ = declare_parameter("distance_to_head_m", 1.0);
    distance_hysteresis_m_ = declare_parameter("distance_hysteresis_m", 0.10);
    search_interval_sec_ = declare_parameter("search_interval_sec", 0.8);
    search_pan_min_step_ = declare_parameter("search_pan_min_step", 0.20);
    search_tilt_min_step_ = declare_parameter("search_tilt_min_step", 0.14);
    search_walk_pan_step_ = declare_parameter("search_walk_pan_step", 0.20);
    search_walk_tilt_step_ = declare_parameter("search_walk_tilt_step", 0.10);
    search_tilt_update_every_ = declare_parameter("search_tilt_update_every", 3);
    search_min_pan_ = declare_parameter("search_min_pan", -0.8);
    search_max_pan_ = declare_parameter("search_max_pan", 0.5);
    search_min_tilt_ = declare_parameter("search_min_tilt", -0.62);
    search_max_tilt_ = declare_parameter("search_max_tilt", -0.35);

    RCLCPP_INFO(
        this->get_logger(),
        "Head limits loaded: pan[%.3f, %.3f] tilt[%.3f, %.3f]",
        pan_min_,
        pan_max_,
        tilt_min_,
        tilt_max_);
    RCLCPP_INFO(
        this->get_logger(),
        "Search window loaded: pan[%.3f, %.3f] tilt[%.3f, %.3f]",
        search_min_pan_,
        search_max_pan_,
        search_min_tilt_,
        search_max_tilt_);
    RCLCPP_INFO(
        this->get_logger(),
        "Base config: topic=%s sign=%.1f debug=%s spin_test=%s yaw=%.2f dur=%.1fs search=%s search_yaw=%.2f long_lost=%.1fs reacquire=%.1fs",
        base_cmd_topic_.c_str(),
        base_yaw_sign_,
        base_debug_log_ ? "true" : "false",
        base_spin_test_mode_ ? "true" : "false",
        base_spin_test_yaw_rate_,
        base_spin_test_duration_sec_,
        base_search_spin_enabled_ ? "true" : "false",
        base_search_yaw_rate_,
        base_search_long_lost_sec_,
        lost_reacquire_sec_);
    RCLCPP_INFO(
        this->get_logger(),
        "Tracking distance: target=%.2fm hysteresis=%.2fm",
        distance_to_head_m_,
        distance_hysteresis_m_);

    face_sub_ =
        create_subscription<qbo_msgs::msg::FacePosAndDist>(
            "/qbo_face_tracking/face_pos_and_dist",
            10,
            std::bind(&FaceFollower::faceCallback, this, _1));

    camera_info_sub_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_left/camera_info",
            10,
            std::bind(&FaceFollower::cameraInfoCallback, this, _1));

    head_pub_ =
        create_publisher<sensor_msgs::msg::JointState>(
            "/cmd_joints",
            10);

    base_pub_ =
        create_publisher<geometry_msgs::msg::Twist>(
            base_cmd_topic_,
            10);

    joint_sub_ =
        create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&FaceFollower::jointCallback, this, _1));

    control_timer_ =
        create_wall_timer(
            std::chrono::milliseconds(80),
            std::bind(&FaceFollower::controlLoop, this));

    filter_init_ = false;
    have_time_ = false;

    cmd_pan_ = 0.0;
    cmd_tilt_ = 0.0;

    vel_pan_ = 0.0;
    vel_tilt_ = 0.0;

    target_pan_ = 0.0;
    target_tilt_ = 0.0;

    image_width_ = 640;
    image_height_ = 480;
    cx_ = image_width_ * 0.5f;
    cy_ = image_height_ * 0.5f;
    have_camera_info_ = false;

    std::random_device rd;
    rng_.seed(rd());

    auto now = this->get_clock()->now();

    last_time_ = now;
    last_face_time_ = now;
    last_search_time_ = now;
    base_spin_test_start_ = now;
    face_lost_since_ = now;
    base_search_last_time_ = now;
    base_search_last_end_time_ = now;

    if (!move_base_)
        base_spin_test_done_ = true;

    have_time_ = false;
}

float FaceFollower::clamp(float v, float lo, float hi)
{
    return std::min(std::max(v, lo), hi);
}

float FaceFollower::deadband(float v, float db)
{
    if (std::abs(v) < db)
        return 0.0;

    return v;
}

float FaceFollower::ema(float x, float prev, float alpha)
{
    return alpha * x + (1.0 - alpha) * prev;
}

void FaceFollower::faceCallback(
    const qbo_msgs::msg::FacePosAndDist::SharedPtr msg)
{
    if (!msg->face_detected)
    {
        last_face_detected_msg_ = false;
        return;
    }

    last_face_detected_msg_ = true;

    last_face_time_ = this->get_clock()->now();

    if (!filter_init_)
    {
        u_f_ = msg->u;
        v_f_ = msg->v;
        filter_init_ = true;
    }
    else
    {
        u_f_ = ema(msg->u, u_f_, ema_alpha_);
        v_f_ = ema(msg->v, v_f_, ema_alpha_);
    }

    last_seen_u_ = u_f_;
    have_last_seen_u_ = true;
    last_face_distance_ = msg->distance_to_head;
    have_last_face_distance_ = true;

    image_width_ = msg->image_width;
    image_height_ = msg->image_height;

    if (!have_camera_info_)
    {
        cx_ = image_width_ * 0.5f;
        cy_ = image_height_ * 0.5f;
    }

    float u = deadband(u_f_, deadband_u_px_);
    float v = deadband(v_f_, deadband_v_px_);

    float px = u + image_width_ / 2.0f;
    float py = v + image_height_ / 2.0f;

    float gain_pan = tracking_gain_pan_;
    float gain_tilt = tracking_gain_tilt_;

    float theta_pan = atan2(px - cx_, fx_);
    float theta_tilt = atan2(py - cy_, fy_);

    float dist_m = std::max(
        static_cast<float>(msg->distance_to_head),
        camera_offset_min_dist_m_);

    float pan_offset = 0.0f;
    if (std::isfinite(dist_m) && dist_m > 0.0f)
        pan_offset = atan2(camera_lateral_offset_m_, dist_m);

    float d_pan  = gain_pan  * (theta_pan + pan_offset);
    float d_tilt = gain_tilt * theta_tilt;

    target_pan_ =
        clamp(yaw_from_joint_ - d_pan, pan_min_, pan_max_);

    target_tilt_ =
        clamp(pitch_from_joint_ + d_tilt, tilt_min_, tilt_max_);
}

void FaceFollower::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // Prefer K matrix, fallback to P if K is not valid.
    if (msg->k[0] > 1.0 && msg->k[4] > 1.0)
    {
        fx_ = static_cast<float>(msg->k[0]);
        fy_ = static_cast<float>(msg->k[4]);
        cx_ = static_cast<float>(msg->k[2]);
        cy_ = static_cast<float>(msg->k[5]);
        have_camera_info_ = true;
    }
    else if (msg->p[0] > 1.0 && msg->p[5] > 1.0)
    {
        fx_ = static_cast<float>(msg->p[0]);
        fy_ = static_cast<float>(msg->p[5]);
        cx_ = static_cast<float>(msg->p[2]);
        cy_ = static_cast<float>(msg->p[6]);
        have_camera_info_ = true;
    }
}

void FaceFollower::pickSearchTarget()
{
    float pan_lo = std::max(search_min_pan_, pan_min_);
    float pan_hi = std::min(search_max_pan_, pan_max_);
    float tilt_lo = std::max(search_min_tilt_, tilt_min_);
    float tilt_hi = std::min(search_max_tilt_, tilt_max_);

    if (pan_lo >= pan_hi)
    {
        pan_lo = pan_min_;
        pan_hi = pan_max_;
    }

    if (tilt_lo >= tilt_hi)
    {
        tilt_lo = tilt_min_;
        tilt_hi = tilt_max_;
    }

    std::uniform_real_distribution<float> pan_step_dist(
        -std::abs(search_walk_pan_step_),
        std::abs(search_walk_pan_step_));
    std::uniform_real_distribution<float> tilt_step_dist(
        -std::abs(search_walk_tilt_step_),
        std::abs(search_walk_tilt_step_));

    float pan_step = pan_step_dist(rng_);
    float next_pan = clamp(target_pan_ + pan_step, pan_lo, pan_hi);

    if (std::abs(next_pan - target_pan_) < search_pan_min_step_)
    {
        float sign = (pan_step >= 0.0f) ? 1.0f : -1.0f;
        next_pan = clamp(
            target_pan_ + sign * search_pan_min_step_,
            pan_lo,
            pan_hi);
    }

    float next_tilt = target_tilt_;
    int tilt_every = std::max(1, search_tilt_update_every_);

    if ((search_update_count_ % tilt_every) == 0)
    {
        float tilt_step = tilt_step_dist(rng_);
        next_tilt = clamp(target_tilt_ + tilt_step, tilt_lo, tilt_hi);

        if (std::abs(next_tilt - target_tilt_) < search_tilt_min_step_)
        {
            float sign = (tilt_step >= 0.0f) ? 1.0f : -1.0f;
            next_tilt = clamp(
                target_tilt_ + sign * search_tilt_min_step_,
                tilt_lo,
                tilt_hi);
        }
    }

    target_pan_ = next_pan;
    target_tilt_ = next_tilt;
    search_update_count_++;
}

float FaceFollower::stepAxis(
    float pos,
    float &vel,
    float target,
    float dt,
    float max_vel,
    float max_acc)
{
    float err = target - pos;

    const float Kp = 4.0f;

    float desired_vel = Kp * err;

    desired_vel = clamp(desired_vel, -max_vel, max_vel);

    float dv = desired_vel - vel;

    float max_dv = max_acc * dt;

    dv = clamp(dv, -max_dv, max_dv);

    vel += dv;

    pos += vel * dt;

    return pos;
}

const char *FaceFollower::stateToString(FollowerState s) const
{
    switch (s)
    {
        case FollowerState::SPIN_TEST:
            return "SPIN_TEST";
        case FollowerState::TRACK_NEAR:
            return "TRACK_NEAR";
        case FollowerState::TRACK_FAR:
            return "TRACK_FAR";
        case FollowerState::LOST_RECENT:
            return "LOST_RECENT";
        case FollowerState::LOST_LONG:
            return "LOST_LONG";
        default:
            return "UNKNOWN";
    }
}

void FaceFollower::updateState(FollowerState next, const char *reason)
{
    if (next == current_state_)
        return;

    previous_state_ = current_state_;
    current_state_ = next;

    RCLCPP_INFO(
        this->get_logger(),
        "STATE: %s -> %s (%s)",
        stateToString(previous_state_),
        stateToString(current_state_),
        reason);
}

bool FaceFollower::runBaseSpinTest(const rclcpp::Time &t)
{
    if (!base_spin_test_started_)
    {
        base_spin_test_started_ = true;
        base_spin_test_start_ = t;
        RCLCPP_INFO(
            this->get_logger(),
            "Starting base spin test: yaw=%.2f rad/s for %.1f s",
            base_spin_test_yaw_rate_,
            base_spin_test_duration_sec_);
    }

    float elapsed = (t - base_spin_test_start_).seconds();

    if (elapsed < base_spin_test_duration_sec_)
    {
        float yaw_rate = clamp(
            base_spin_test_yaw_rate_,
            -base_max_yaw_rate_,
            base_max_yaw_rate_);
        if (base_debug_log_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "BASE spin-test: elapsed=%.2f/%.2f cmd_z=%.3f",
                elapsed,
                base_spin_test_duration_sec_,
                yaw_rate);
        }
        publishBase(yaw_rate);
        return true;
    }

    publishBase(0.0f);
    base_spin_test_done_ = true;
    RCLCPP_INFO(this->get_logger(), "Base spin test done, switching to face-follow mode");
    return false;
}

void FaceFollower::runBaseLostRecent(float lost_time)
{
    base_assist_active_ = false;

    if (have_last_seen_u_ && std::abs(last_seen_u_) >= base_deadband_px_)
    {
        float u_norm = clamp(
            (last_seen_u_ / (image_width_ / 2.0f)),
            -1.0f,
            1.0f);
        float yaw_rate =
            u_norm *
            base_max_yaw_rate_ *
            (base_yaw_sign_ >= 0.0f ? 1.0f : -1.0f);

        if (base_debug_log_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "BASE reacquire: lost_t=%.2f last_u=%.1f cmd_z=%.3f",
                lost_time,
                last_seen_u_,
                yaw_rate);
        }
        publishBase(yaw_rate);
    }
    else
    {
        if (base_debug_log_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "BASE reacquire idle: last_u unavailable or below deadband");
        }
        publishBase(0.0f);
    }
}

void FaceFollower::runBaseLostLong(const rclcpp::Time &t)
{
    base_assist_active_ = false;

    if (base_search_spin_enabled_)
    {
        if (!base_search_active_)
        {
            float since_end = (t - base_search_last_end_time_).seconds();
            bool can_restart = since_end >= base_search_restart_delay_sec_;

            if (can_restart)
            {
                bool long_lost = (t - face_lost_since_).seconds() >= base_search_long_lost_sec_;

                float dir_sign = 0.0f;
                const float yaw_sign = (base_yaw_sign_ >= 0.0f) ? 1.0f : -1.0f;

                if (!long_lost && have_last_seen_u_ &&
                    std::abs(last_seen_u_) >= base_search_side_deadband_px_)
                {
                    float side_sign = (last_seen_u_ >= 0.0f) ? 1.0f : -1.0f;
                    dir_sign = side_sign * yaw_sign;
                }
                else
                {
                    std::uniform_int_distribution<int> coin(0, 1);
                    dir_sign = (coin(rng_) == 0) ? -1.0f : 1.0f;
                }

                if (dir_sign != 0.0f)
                {
                    base_search_active_ = true;
                    base_search_dir_sign_ = dir_sign;
                    base_search_accum_yaw_ = 0.0f;
                    base_search_last_time_ = t;

                    if (base_debug_log_)
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "BASE search start: dir=%.1f last_u=%.1f long_lost=%s",
                            base_search_dir_sign_,
                            last_seen_u_,
                            long_lost ? "true" : "false");
                    }
                }
            }
        }

        if (base_search_active_)
        {
            float dt_search = (t - base_search_last_time_).seconds();
            dt_search = clamp(dt_search, 0.001f, 0.2f);
            base_search_last_time_ = t;

            float yaw_mag = std::min(std::abs(base_search_yaw_rate_), base_max_yaw_rate_);
            float yaw_rate = yaw_mag * base_search_dir_sign_;

            base_search_accum_yaw_ += yaw_mag * dt_search;

            if (base_search_accum_yaw_ >= kTwoPi)
            {
                publishBase(0.0f);
                base_search_active_ = false;
                base_search_last_end_time_ = t;

                if (base_debug_log_)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "BASE search end: one full turn completed");
                }
            }
            else
            {
                if (base_debug_log_)
                {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        500,
                        "BASE search running: dir=%.1f yaw=%.3f progress=%.2f/%.2f",
                        base_search_dir_sign_,
                        yaw_rate,
                        base_search_accum_yaw_,
                        kTwoPi);
                }
                publishBase(yaw_rate);
            }
            return;
        }
    }

    if (base_debug_log_)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "BASE idle: face_lost");
    }
    publishBase(0.0f);
}

void FaceFollower::runBaseTracking()
{
    float u = u_f_;

    if (std::abs(u) < base_deadband_px_)
    {
        if (base_debug_log_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "BASE idle: u=%.1f deadband=%.1f",
                u,
                base_deadband_px_);
        }
        publishBase(0.0f);
        return;
    }

    bool head_limit =
        (yaw_from_joint_ > pan_max_ - base_head_limit_margin_) ||
        (yaw_from_joint_ < pan_min_ + base_head_limit_margin_);

    if (head_limit)
    {
        base_assist_active_ = true;
    }
    else if (std::abs(u) < (base_deadband_px_ * base_assist_release_factor_))
    {
        base_assist_active_ = false;
    }

    if (!base_assist_active_)
    {
        if (base_debug_log_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "BASE idle: assist off pan_joint=%.3f u=%.1f lim=[%.3f, %.3f] margin=%.3f rel=%.2f",
                yaw_from_joint_,
                u,
                pan_min_,
                pan_max_,
                base_head_limit_margin_,
                base_assist_release_factor_);
        }
        publishBase(0.0f);
        return;
    }

    float u_norm = clamp(
        (u / (image_width_ / 2.0f)),
        -1.0f,
        1.0f);

    float yaw_rate =
        u_norm *
        base_max_yaw_rate_ *
        (base_yaw_sign_ >= 0.0f ? 1.0f : -1.0f);

    if (base_debug_log_)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "BASE cmd: u=%.1f u_norm=%.3f pan=%.3f head_limit=%s sign=%.1f cmd_z=%.3f",
            u,
            u_norm,
            yaw_from_joint_,
            head_limit ? "true" : "false",
            base_yaw_sign_,
            yaw_rate);
    }

    publishBase(yaw_rate);
}

void FaceFollower::controlLoop()
{
    auto t = this->get_clock()->now();

    float dt = 0.03;

    if (have_time_)
    {
        dt = (t - last_time_).seconds();
        dt = std::clamp(dt, 0.001f, 0.1f);
    }

    last_time_ = t;
    have_time_ = true;

    float lost_time = (t - last_face_time_).seconds();
    bool face_lost = lost_time > lost_hold_sec_;

    FollowerState next_state = current_state_;

    // Spin test is only meaningful when base motion is enabled.
    if (move_base_ && base_spin_test_mode_ && !base_spin_test_done_)
    {
        next_state = FollowerState::SPIN_TEST;
    }
    else if (face_lost)
    {
        if (lost_time <= (lost_hold_sec_ + lost_reacquire_sec_))
            next_state = FollowerState::LOST_RECENT;
        else
            next_state = FollowerState::LOST_LONG;
    }
    else
    {
        float dist = have_last_face_distance_ ? last_face_distance_ : distance_to_head_m_;
        float near_to_far = distance_to_head_m_ + distance_hysteresis_m_;
        float far_to_near = distance_to_head_m_ - distance_hysteresis_m_;

        bool near = false;
        if (current_state_ == FollowerState::TRACK_NEAR)
            near = dist <= near_to_far;
        else if (current_state_ == FollowerState::TRACK_FAR)
            near = dist <= far_to_near;
        else
            near = dist <= distance_to_head_m_;

        next_state = near ? FollowerState::TRACK_NEAR : FollowerState::TRACK_FAR;
    }

    const bool state_changed = (next_state != current_state_);

    if (state_changed)
    {
        const char *reason = "transition";
        if (next_state == FollowerState::SPIN_TEST)
            reason = "startup spin-test active";
        else if (next_state == FollowerState::LOST_RECENT)
            reason = "face lost (recent)";
        else if (next_state == FollowerState::LOST_LONG)
            reason = "face lost (long)";
        else if (next_state == FollowerState::TRACK_NEAR)
            reason = "face tracked near";
        else if (next_state == FollowerState::TRACK_FAR)
            reason = "face tracked far";

        updateState(next_state, reason);
    }

    bool in_search_mode = (current_state_ == FollowerState::LOST_LONG);
    bool is_tracking =
        (current_state_ == FollowerState::TRACK_NEAR) ||
        (current_state_ == FollowerState::TRACK_FAR);
    face_lost = !is_tracking && (current_state_ != FollowerState::SPIN_TEST);

    float search_pan_lo = std::max(search_min_pan_, pan_min_);
    float search_pan_hi = std::min(search_max_pan_, pan_max_);
    float search_tilt_lo = std::max(search_min_tilt_, tilt_min_);
    float search_tilt_hi = std::min(search_max_tilt_, tilt_max_);

    if (is_tracking)
    {
        was_face_lost_ = false;
        base_search_active_ = false;
        base_search_accum_yaw_ = 0.0f;
        // Keep this state only for active base assist during tracked face.
    }

    if (in_search_mode)
    {
        if (state_changed)
        {
            was_face_lost_ = true;
            face_lost_since_ = t;
            base_search_active_ = false;
            base_search_accum_yaw_ = 0.0f;
            base_search_last_time_ = t;
            if (base_debug_log_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "BASE lost transition: last_u=%.1f have_last=%s",
                    last_seen_u_,
                    have_last_seen_u_ ? "true" : "false");
            }
        }

        if ((t - last_search_time_).seconds() >= search_interval_sec_)
        {
            last_search_time_ = t;
            pickSearchTarget();
        }

        // Keep both target and command inside the search window while face is lost.
        target_pan_ = clamp(target_pan_, search_pan_lo, search_pan_hi);
        target_tilt_ = clamp(target_tilt_, search_tilt_lo, search_tilt_hi);
    }

    float max_vel_pan = in_search_mode ? search_max_vel_pan_ : head_max_vel_pan_;
    float max_vel_tilt = in_search_mode ? search_max_vel_tilt_ : head_max_vel_tilt_;
    float max_acc_pan = in_search_mode ? search_max_acc_pan_ : head_max_acc_pan_;
    float max_acc_tilt = in_search_mode ? search_max_acc_tilt_ : head_max_acc_tilt_;

    cmd_pan_ = stepAxis(
        cmd_pan_,
        vel_pan_,
        target_pan_,
        dt,
        max_vel_pan,
        max_acc_pan);

    cmd_tilt_ = stepAxis(
        cmd_tilt_,
        vel_tilt_,
        target_tilt_,
        dt,
        max_vel_tilt,
        max_acc_tilt);

    if (in_search_mode)
    {
        cmd_pan_ = clamp(cmd_pan_, search_pan_lo, search_pan_hi);
        cmd_tilt_ = clamp(cmd_tilt_, search_tilt_lo, search_tilt_hi);
    }
    else
    {
        cmd_pan_ = clamp(cmd_pan_, pan_min_, pan_max_);
        cmd_tilt_ = clamp(cmd_tilt_, tilt_min_, tilt_max_);
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "detected=%s tracking=%s dist=%.2f lost_t=%.2f u=%.1f servo=%.2f target=%.2f",
        last_face_detected_msg_ ? "true" : "false",
        is_tracking ? "true" : "false",
        have_last_face_distance_ ? last_face_distance_ : -1.0f,
        lost_time,
        u_f_,
        yaw_from_joint_,
        target_pan_);

    publishHead();

    if (!move_base_)
    {
        if (base_debug_log_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "BASE idle: move_base=false");
        }
        publishBase(0.0f);
        return;
    }

    switch (current_state_)
    {
        case FollowerState::SPIN_TEST:
            if (runBaseSpinTest(t))
                return;
            runBaseTracking();
            return;
        case FollowerState::LOST_RECENT:
            runBaseLostRecent(lost_time);
            return;
        case FollowerState::LOST_LONG:
            runBaseLostLong(t);
            return;
        case FollowerState::TRACK_NEAR:
        case FollowerState::TRACK_FAR:
            runBaseTracking();
            return;
        default:
            publishBase(0.0f);
            return;
    }
}

void FaceFollower::publishHead()
{
    if (!move_head_)
        return;

    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->get_clock()->now();

    msg.name = {"head_pan_joint", "head_tilt_joint"};

    msg.position = {cmd_pan_, cmd_tilt_};

    msg.velocity = {vel_pan_, vel_tilt_};

    head_pub_->publish(msg);
}

void FaceFollower::publishBase(float yaw_rate)
{
    geometry_msgs::msg::Twist msg;

    msg.angular.z = yaw_rate;

    base_pub_->publish(msg);
}

void FaceFollower::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->name.size() != msg->position.size())
    {
        RCLCPP_WARN(this->get_logger(), "Malformed JointState: name/position size mismatch");
        return;
    }

    for (size_t i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == "head_pan_joint")
            yaw_from_joint_ = msg->position[i];

        if (msg->name[i] == "head_tilt_joint")
            pitch_from_joint_ = msg->position[i];
    }
}

void FaceFollower::headToZeroPosition()
{
    if (!move_head_)
        return;

    if (is_zeroing_head_)
        return;

    is_zeroing_head_ = true;

    if (move_base_)
        publishBase(0.0f);

    RCLCPP_INFO(this->get_logger(), "Resetting head to zero position before shutdown");

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.name = {"head_pan_joint", "head_tilt_joint"};
    joint_msg.position = {0.0, 0.0};
    joint_msg.velocity = {0.5, 0.5};

    const auto start = this->get_clock()->now();
    rclcpp::Rate rate(10.0);

    while (true)
    {
        joint_msg.header.stamp = this->get_clock()->now();
        head_pub_->publish(joint_msg);

        rclcpp::spin_some(shared_from_this());

        if (std::abs(yaw_from_joint_) < 0.03f &&
            std::abs(pitch_from_joint_) < 0.03f)
        {
            break;
        }

        if ((this->get_clock()->now() - start).seconds() >= 3.0)
            break;

        rate.sleep();
    }

    is_zeroing_head_ = false;
}

void handleSigInt(int)
{
    g_request_shutdown.store(true);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    g_node = std::make_shared<FaceFollower>();
    signal(SIGINT, handleSigInt);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(g_node);

    while (rclcpp::ok() && !g_request_shutdown.load())
    {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (g_request_shutdown.load() && g_node)
        g_node->headToZeroPosition();

    if (rclcpp::ok())
        rclcpp::shutdown();

    return 0;
}