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

#include <qbo_msgs/msg/face_observation.hpp>
#include <qbo_msgs/msg/follower_status.hpp>
#include <qbo_msgs/srv/set_follower_status.hpp>

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
    rclcpp::Subscription<qbo_msgs::msg::FaceObservation>::SharedPtr face_pos_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_control_pub_;
    rclcpp::Publisher<qbo_msgs::msg::FollowerStatus>::SharedPtr status_pub_;

    rclcpp::Service<qbo_msgs::srv::SetFollowerStatus>::SharedPtr control_service_;

    rclcpp::Time last_scan_move_;     // horodatage du dernier mouvement aléatoire
    rclcpp::Duration scan_interval_;
    rclcpp::Duration pause_duration_;

    // Paramètres ROS
    void declare_and_get_parameters();
    bool move_base_bool_;
    bool move_head_bool_;
    double search_min_pan_, search_max_pan_, search_pan_vel_;
    double search_min_tilt_, search_max_tilt_, search_tilt_vel_;
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
    // PID Yaw (rotation base)
    float yaw_prev_, yaw_act_, diff_yaw_;
    float kp_yaw_, ki_yaw_, kd_yaw_;
    // Random
    float last_pan_ = 0.0f;
    float last_tilt_ = 0.0f;
    float min_angle_step_ = 0.1f;  // rad
    // Intégrale pour PID (yaw base mobile rotation only)
    float integral_yaw_;

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
    std::string base_cmd_vel_topic_;
    bool invert_linear_vel_;
    std::string camera_info_topic_;


    rclcpp::Duration auto_disable_timeout_;
    rclcpp::Time last_active_tracking_time_;
    bool tracking_was_active_;
    bool movements_auto_disabled_;
    uint8_t previous_tracking_state_;  // Pour détecter les changements d'état

    // Throttle des logs répétitifs (log seulement lors des transitions)
    bool backward_blocked_ = false;  // True si on bloque actuellement le recul
    bool head_border_blocked_ = false;  // True si on bloque l'avance (tête en limite)
    bool joint_states_received_ = false;  // True après réception du premier joint_state

    // Contrôle des capacités (modifiables via service)
    bool head_movement_enabled_ = true;   // Tête activée par défaut
    bool base_rotation_enabled_ = true;   // Rotation base activée par défaut

    // Dernières données du visage (pour le status topic)
    float last_face_distance_ = 0.0f;
    float last_face_x_ = 0.0f;
    float last_face_y_ = 0.0f;
    float last_face_z_ = 0.0f;
    float last_face_u_ = 0.0f;
    float last_face_v_ = 0.0f;
    uint8_t last_faces_detected_ = 0;
    uint8_t current_tracking_state_ = 0;  // IDLE par défaut
    std::string current_blocking_reason_ = "";

    // Callbacks
    void setFollowerStatusCallback(
        const std::shared_ptr<qbo_msgs::srv::SetFollowerStatus::Request> request,
        std::shared_ptr<qbo_msgs::srv::SetFollowerStatus::Response> response);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void facePositionCallback(const qbo_msgs::msg::FaceObservation::SharedPtr msg);

    // Fonctions de mouvement
    void setHeadPositionToFace(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright);
    void setHeadPositionGlobal(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright);
    void sendVelocityBase(float linear_vel, float angular_vel);

    // Contrôle PID
    float controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd, float dead_zone = 2.0f);

    // Publication du statut
    void publishStatus();
    
    // Calcul des coordonnées 3D du visage
    void calculateFace3DPosition(float distance, float head_pan, float head_tilt,
                                  float& x, float& y, float& z);
};

#endif  // QBO_VISION_FACE_FOLLOWER_HPP_
