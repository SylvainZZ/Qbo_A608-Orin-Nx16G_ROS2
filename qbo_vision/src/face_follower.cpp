#include "../include/face_follower.hpp"
#include <signal.h>  // pour signal()



FaceFollower::FaceFollower(const rclcpp::NodeOptions & options)
: rclcpp::Node("qbo_face_following", options),
  last_scan_move_(this->get_clock()->now()),
  scan_interval_(rclcpp::Duration::from_seconds(1.0)),
  pause_duration_(rclcpp::Duration::from_seconds(3.0)),
  auto_disable_timeout_(rclcpp::Duration::from_seconds(10.0)),
  last_active_tracking_time_(this->get_clock()->now())
{
    RCLCPP_INFO(this->get_logger(), "Initializing Qbo face follower node");
    onInit();
    RCLCPP_INFO(this->get_logger(), "Ready for face following. Waiting for face positions");
}


void FaceFollower::declare_and_get_parameters()
{
    this->declare_parameter("send_stop", true);

    // Paramètres de contrôle du follower (modifiables via service)
    this->declare_parameter("head_movement_enabled", true);
    this->declare_parameter("base_rotation_enabled", true);

    this->declare_parameter("search_pan_vel", 0.3);
    this->declare_parameter("search_pan_step", 0.2);
    this->declare_parameter("search_min_pan", -0.3);
    this->declare_parameter("search_max_pan", 0.3);

    this->declare_parameter("search_tilt_vel", 0.3);
    this->declare_parameter("search_tilt_levels", std::vector<double>{-0.5, 0.0, 0.3});
    this->declare_parameter("search_min_tilt", -0.5);
    this->declare_parameter("search_max_tilt", 0.3);

    this->declare_parameter("scan_interval", 1.0);
    this->declare_parameter("search_pause_duration", 3.0);

    // PID gains for head control
    this->declare_parameter("head_pan_kp", 0.005);
    this->declare_parameter("head_pan_ki", 0.0003);
    this->declare_parameter("head_pan_kd", 0.001);
    this->declare_parameter("head_tilt_kp", 0.005);
    this->declare_parameter("head_tilt_ki", 0.0003);
    this->declare_parameter("head_tilt_kd", 0.001);

    // PID gains for base control (rotation only)
    this->declare_parameter("base_yaw_kp", 0.8);
    this->declare_parameter("base_yaw_ki", 0.01);
    this->declare_parameter("base_yaw_kd", 0.05);
    // Base control configuration
    this->declare_parameter("base_cmd_vel_topic", std::string("/qbo_arduqbo/base_ctrl/cmd_vel"));
    this->declare_parameter("invert_linear_vel", false);
    this->declare_parameter("camera_info_topic", std::string("camera_left/camera_info"));

    // Auto-disable timeout when tracker is disabled
    this->declare_parameter("auto_disable_timeout", 10.0);

    // Lire send_stop
    this->get_parameter("send_stop", send_stop_);

    this->get_parameter("search_pan_vel", search_pan_vel_);
    this->get_parameter("search_pan_step", pan_step_);
    this->get_parameter("search_min_pan", search_min_pan_);
    this->get_parameter("search_max_pan", search_max_pan_);

    this->get_parameter("search_tilt_vel", search_tilt_vel_);
    std::vector<double> tilt_d;
    this->get_parameter("search_tilt_levels", tilt_d);
    tilt_levels_.assign(tilt_d.begin(), tilt_d.end());
    this->get_parameter("search_min_tilt", search_min_tilt_);
    this->get_parameter("search_max_tilt", search_max_tilt_);

    double scan_int, pause_dur;
    this->get_parameter("scan_interval", scan_int);
    this->get_parameter("search_pause_duration", pause_dur);
    scan_interval_ = rclcpp::Duration::from_seconds(scan_int);
    pause_duration_ = rclcpp::Duration::from_seconds(pause_dur);

    // Get PID gains
    this->get_parameter("head_pan_kp", kp_u_);
    this->get_parameter("head_pan_ki", ki_u_);
    this->get_parameter("head_pan_kd", kd_u_);
    this->get_parameter("head_tilt_kp", kp_v_);
    this->get_parameter("head_tilt_ki", ki_v_);
    this->get_parameter("head_tilt_kd", kd_v_);
    this->get_parameter("base_yaw_kp", kp_yaw_);
    this->get_parameter("base_yaw_ki", ki_yaw_);
    this->get_parameter("base_yaw_kd", kd_yaw_);

    this->get_parameter("base_cmd_vel_topic", base_cmd_vel_topic_);
    this->get_parameter("invert_linear_vel", invert_linear_vel_);
    this->get_parameter("camera_info_topic", camera_info_topic_);

    double auto_disable_timeout;
    this->get_parameter("auto_disable_timeout", auto_disable_timeout);
    auto_disable_timeout_ = rclcpp::Duration::from_seconds(auto_disable_timeout);

    // Lire les paramètres de contrôle du follower
    this->get_parameter("head_movement_enabled", head_movement_enabled_);
    this->get_parameter("base_rotation_enabled", base_rotation_enabled_);

    RCLCPP_INFO(this->get_logger(), "Base control topic: %s, invert_linear: %s",
               base_cmd_vel_topic_.c_str(), invert_linear_vel_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Auto-disable timeout: %.1fs", auto_disable_timeout);
    RCLCPP_INFO(this->get_logger(), "Follower control at startup: head=%s, base_rotation=%s",
               head_movement_enabled_ ? "ENABLED" : "DISABLED",
               base_rotation_enabled_ ? "ENABLED" : "DISABLED");
}


void FaceFollower::onInit()
{
    declare_and_get_parameters();

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, 10,
        std::bind(&FaceFollower::cameraInfoCallback, this, std::placeholders::_1));

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/cmd_joints", 1);
    base_control_pub_ = create_publisher<geometry_msgs::msg::Twist>(base_cmd_vel_topic_, 1);
    status_pub_ = create_publisher<qbo_msgs::msg::FollowerStatus>("/qbo_face_following/status", 10);

    // Service pour contrôler les capacités du follower (appelé par le SBE)
    control_service_ = create_service<qbo_msgs::srv::SetFollowerStatus>(
        "/qbo_face_following/set_follower_status",
        std::bind(&FaceFollower::setFollowerStatusCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service '/qbo_face_following/set_follower_status' ready");

    yaw_from_joint_ = 0.0f;
    pitch_from_joint_ = 0.0f;
    min_pitch_ = -0.5f;

    // PID init
    // For head's pan movement
    u_act_ = u_prev_ = diff_u_ = 0.0f;
    // Default gains (will be overridden by parameters)
    kp_u_ = 0.005f;
    ki_u_ = 0.0003f;
    kd_u_ = 0.001f;


    // For head's tilt movement
    v_act_ = v_prev_ = diff_v_ = 0.0f;
    kp_v_ = 0.005f;
    ki_v_ = 0.0003f;
    kd_v_ = 0.001f;

    // For base's angular movement
    yaw_act_ = yaw_prev_ = diff_yaw_ = 0.0f;
    integral_yaw_ = 0.0f;  // Accumulation intégrale
    kp_yaw_ = 0.35f;
    ki_yaw_ = 0.01f;
    kd_yaw_ = 0.05f;

    image_width_ = 640;
    image_height_ = 480;
    last_scan_move_ = this->get_clock()->now();

    // Initialize auto-disable tracking
    last_active_tracking_time_ = this->get_clock()->now();
    tracking_was_active_ = false;
    movements_auto_disabled_ = false;
    previous_tracking_state_ = 255;  // Valeur invalide pour forcer le premier log
    backward_blocked_ = false;  // Pas de blocage au démarrage
    head_border_blocked_ = false;  // Pas de blocage au démarrage

    // Initialiser les variables de statut
    current_tracking_state_ = 0;  // IDLE
    current_blocking_reason_ = "";
    last_face_distance_ = 0.0f;
    last_face_x_ = 0.0f;
    last_face_y_ = 0.0f;
    last_face_z_ = 0.0f;
    last_face_u_ = 0.0f;
    last_face_v_ = 0.0f;
    last_faces_detected_ = 0;
}


void FaceFollower::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() != msg->name.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Malformed JointState: name/position size mismatch");
        return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "head_pan_joint")
        {
            yaw_from_joint_ = msg->position[i];
        }
        else if (msg->name[i] == "head_tilt_joint")
        {
            pitch_from_joint_ = msg->position[i];
        }
    }

    joint_states_received_ = true;  // On a maintenant les vraies positions de tête

    // Publier le statut avec les nouvelles positions de tête
    publishStatus();
}


void FaceFollower::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // Initialiser la matrice de projection seulement une fois
    if (p_.empty())
    {
        cv::Mat p = cv::Mat(3, 4, CV_64F);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                p.at<double>(i, j) = msg->p[4 * i + j];
            }
        }

        // Extraire la sous-matrice 3x3
        p(cv::Rect(0, 0, 3, 3)).convertTo(p_, CV_32F);

        // Maintenant que la caméra est calibrée, on peut s’abonner aux autres topics
        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&FaceFollower::jointStateCallback, this, std::placeholders::_1));

        face_pos_sub_ = create_subscription<qbo_msgs::msg::FaceObservation>(
            "/qbo_face_tracking/face_observation", 10,
            std::bind(&FaceFollower::facePositionCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Camera calibration received, subscriptions initialized");
    }
}


void FaceFollower::facePositionCallback(const qbo_msgs::msg::FaceObservation::SharedPtr msg)
{
    // Ignorer les messages tant qu'on n'a pas reçu les vraies positions de tête
    // (évite de perturber un tracking stable au démarrage)
    if (!joint_states_received_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Ignoring face message - waiting for joint states");
        return;
    }

    image_width_ = msg->image_width;
    image_height_ = msg->image_height;

    auto now = this->get_clock()->now();

    // 📊 Mapper l'état du tracker vers le statut du follower
    // 0=DISABLED -> IDLE, 1=SEARCH -> SEARCHING, 2=CANDIDATE -> SEARCHING, 3=TRACKING -> TRACKING
    uint8_t received_state = msg->tracking_state;
    if (msg->tracking_state == 0) {
        current_tracking_state_ = 0;  // IDLE
    } else if (msg->tracking_state == 1 || msg->tracking_state == 2) {
        current_tracking_state_ = 1;  // SEARCHING
    } else if (msg->tracking_state == 3) {
        current_tracking_state_ = 2;  // TRACKING
    }

    RCLCPP_DEBUG(this->get_logger(),
        "FaceObservation received: tracker_state=%d (mapped to follower_state=%d), "
        "faces=%d, distance=%.2f, center=(%.0f,%.0f)",
        received_state, current_tracking_state_,
        msg->faces_detected, msg->distance,
        msg->center_x, msg->center_y);

    // Sauvegarder les données du visage pour le topic de statut
    last_face_distance_ = msg->distance;
    last_faces_detected_ = msg->faces_detected;

    // Logger uniquement les changements d'état importants
    if (msg->tracking_state != previous_tracking_state_)
    {
        const char* state_names[] = {"🔴 DISABLED", "🔍 SEARCH", "⚠️  CANDIDATE", "✅ TRACKING"};
        const char* state_str = (msg->tracking_state < 4) ? state_names[msg->tracking_state] : "❓ UNKNOWN";
        RCLCPP_INFO(this->get_logger(), "État changé: %s", state_str);
        previous_tracking_state_ = msg->tracking_state;
    }

    // Check if tracking is disabled and handle auto-disable
    if (msg->tracking_state == 0)  // 0 = DISABLED
    {
        if (tracking_was_active_)
        {
            // Just became disabled, start the timeout
            last_active_tracking_time_ = now;
            tracking_was_active_ = false;
        }

        auto elapsed = now - last_active_tracking_time_;
        if (elapsed >= auto_disable_timeout_ && !movements_auto_disabled_)
        {
            // Timeout reached, disable movements
            RCLCPP_WARN(this->get_logger(),
                "⏱️ Tracking DISABLED for %.1fs - auto-disabling head and base movements",
                elapsed.seconds());

            // Stop base
            if (base_rotation_enabled_)
            {
                sendVelocityBase(0.0f, 0.0f);
            }

            movements_auto_disabled_ = true;

            // Don't process further
            return;
        }
        else if (!movements_auto_disabled_)
        {
            RCLCPP_DEBUG(this->get_logger(),
                "Tracking DISABLED for %.1fs / %.1fs before auto-disable",
                elapsed.seconds(), auto_disable_timeout_.seconds());
        }

        // If movements are auto-disabled, don't do anything
        if (movements_auto_disabled_)
        {
            return;
        }
    }
    else
    {
        // Tracking is active (SEARCH, CANDIDATE, or TRACKING)
        if (movements_auto_disabled_)
        {
            RCLCPP_INFO(this->get_logger(), "✅ Mouvements réactivés");
            movements_auto_disabled_ = false;
        }
        tracking_was_active_ = true;
        last_active_tracking_time_ = now;
    }

    // Rafraîchir les paramètres dynamiques
    this->get_parameter("send_stop", send_stop_);
    this->get_parameter("search_min_pan", search_min_pan_);
    this->get_parameter("search_max_pan", search_max_pan_);
    this->get_parameter("search_pan_vel", search_pan_vel_);
    this->get_parameter("search_min_tilt", search_min_tilt_);
    this->get_parameter("search_max_tilt", search_max_tilt_);
    this->get_parameter("search_tilt_vel", search_tilt_vel_);

    // Face is being tracked (TRACKING state = 3)
    if (msg->tracking_state == 3 && msg->center_x > 0 && msg->center_y > 0)
    {
        // Calculate u and v from center position (centered coordinates)
        float u = msg->center_x - (msg->image_width / 2.0f);
        float v = msg->center_y - (msg->image_height / 2.0f);

        // Sauvegarder pour le statut
        last_face_u_ = u;
        last_face_v_ = v;

        // Calculer les coordonnées 3D du visage dans le repère robot
        calculateFace3DPosition(msg->distance, yaw_from_joint_, pitch_from_joint_,
                                last_face_x_, last_face_y_, last_face_z_);

        current_blocking_reason_ = "";

        // === CONTRÔLE DE LA TÊTE (si activé) ===
        if (head_movement_enabled_)
        {
            // === PAN PID (horizontal) ===
            u_act_ = u;
            diff_u_ = u_act_ - u_prev_;
            float pan_vel = controlPID(u_act_, 0.0f, diff_u_, kp_u_, ki_u_, kd_u_, 2.0f);  // 2 pixels
            u_prev_ = u_act_;

            // === TILT PID (vertical) ===
            v_act_ = v;
            diff_v_ = v_act_ - v_prev_;
            float tilt_vel = controlPID(v_act_, 0.0f, diff_v_, kp_v_, ki_v_, kd_v_, 2.0f);  // 2 pixels
            v_prev_ = v_act_;

            RCLCPP_DEBUG(this->get_logger(), "Moving head: pos(%.2f, %.2f), vel(%.3f, %.3f)", v_act_, u_act_, tilt_vel, pan_vel);
            setHeadPositionToFace(v_act_, u_act_, tilt_vel, pan_vel);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Head movement disabled by SBE or parameter");
        }

        // === CONTRÔLE DE LA ROTATION BASE (si activé) ===
        // Note: le contrôle linéaire (avance/recul) est délégué au SBE + move_base
        if (base_rotation_enabled_)
        {
            // === YAW PID (rotation base pour centrer le visage) ===
            yaw_act_ = yaw_from_joint_;
            diff_yaw_ = yaw_act_ - yaw_prev_;
            integral_yaw_ += yaw_act_;  // Accumulation de l'erreur
            // Anti-windup : limiter l'intégrale
            integral_yaw_ = std::clamp(integral_yaw_, -10.0f, 10.0f);

            float angular_vel = controlPID(yaw_act_, integral_yaw_, diff_yaw_, kp_yaw_, ki_yaw_, kd_yaw_, 0.05f);  // ~3 degrés
            yaw_prev_ = yaw_act_;

            // Limiter les vitesses maximales
            angular_vel = std::clamp(angular_vel, -0.8f, 0.8f);  // ±0.8 rad/s max

            RCLCPP_DEBUG(this->get_logger(), "Base rotation: head_pan=%.2f rad, angular_vel=%.3f", yaw_from_joint_, angular_vel);

            // ⚠️ Contrôle rotation uniquement (linear = 0)
            sendVelocityBase(0.0f, angular_vel);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Base rotation disabled by SBE or parameter");
            // Arrêter la base si désactivé
            sendVelocityBase(0.0f, 0.0f);
        }

        // Publier le statut après traitement
        publishStatus();
    }
    else
    {
        // Visage non détecté ou en SEARCH/CANDIDATE
        // Réinitialiser les coordonnées 3D
        last_face_x_ = 0.0f;
        last_face_y_ = 0.0f;
        last_face_z_ = 0.0f;
        last_face_u_ = 0.0f;
        last_face_v_ = 0.0f;

        // Réinitialiser l'intégrale du yaw quand le visage est perdu
        integral_yaw_ = 0.0f;

        auto now = this->get_clock()->now();

        if ((now - last_scan_move_) > scan_interval_)
        {
            last_scan_move_ = now;

            static std::random_device rd;
            static std::mt19937 gen(rd());

            std::uniform_real_distribution<float> rand_pan(
                static_cast<float>(search_min_pan_), static_cast<float>(search_max_pan_));
            std::uniform_real_distribution<float> rand_tilt(
                static_cast<float>(search_min_tilt_), static_cast<float>(search_max_tilt_));

            float pan = last_pan_;
            float tilt = last_tilt_;
            int max_attempts = 5;
            do {
                pan = rand_pan(gen);
                tilt = rand_tilt(gen);
                max_attempts--;
            } while ((std::abs(pan - last_pan_) < min_angle_step_ ||
                    std::abs(tilt - last_tilt_) < min_angle_step_) && max_attempts > 0);

            last_pan_ = pan;
            last_tilt_ = tilt;

            RCLCPP_DEBUG(this->get_logger(), "Scanning: head pos(%.2f, %.2f)", tilt, pan);

            if (head_movement_enabled_)
            {
                setHeadPositionGlobal(tilt, pan, static_cast<float>(search_tilt_vel_), static_cast<float>(search_pan_vel_));
            }
        }

        if (send_stop_ && base_rotation_enabled_)
        {
            sendVelocityBase(0.0f, 0.0f);
        }

        // Publier le statut même quand pas de visage
        publishStatus();
    }
}


void FaceFollower::setHeadPositionToFace(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright)
{
    if (p_.empty())
        return;

    float pan_pos = pos_leftright + image_width_ / 2.0f;
    float tilt_pos = pos_updown + image_height_ / 2.0f;

    pan_pos = std::atan2((pan_pos - p_.at<float>(0, 2)), p_.at<float>(0, 0));
    tilt_pos = std::atan2((tilt_pos - p_.at<float>(1, 2)), p_.at<float>(1, 1));

    pan_pos = yaw_from_joint_ - pan_pos;
    tilt_pos = tilt_pos + pitch_from_joint_;

    if (std::abs(pan_pos - yaw_from_joint_) < 0.05 && std::abs(tilt_pos - pitch_from_joint_) < 0.05)
    {
        RCLCPP_DEBUG(this->get_logger(), "Minimal head movement, ignoring");
        return;
    }

    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->get_clock()->now();

    joint_msg.name = {"head_pan_joint", "head_tilt_joint"};
    joint_msg.position = {pan_pos, tilt_pos};
    joint_msg.velocity = {std::abs(vel_leftright), std::abs(vel_updown)};

    joint_pub_->publish(joint_msg);
}


void FaceFollower::setHeadPositionGlobal(float pos_updown, float pos_leftright, float vel_updown, float vel_leftright)
{
    // Contrainte : valeur entre -1.4 et +1.4 (pan)
    pos_leftright = std::clamp(pos_leftright, -1.4f, 1.4f);
    pos_updown    = std::clamp(pos_updown,   -0.5f, 0.3f);

    // 🧠 Ignorer les commandes déjà envoyées (évite les à-coups)
    if (std::abs(pos_leftright - last_sent_pan_) < 0.01f &&
        std::abs(pos_updown - last_sent_tilt_) < 0.01f)
    {
        return;
    }

    last_sent_pan_ = pos_leftright;
    last_sent_tilt_ = pos_updown;

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"head_pan_joint", "head_tilt_joint"};
    joint_state.position = {pos_leftright, pos_updown};
    joint_state.velocity = {std::abs(vel_leftright), std::abs(vel_updown)};
    joint_state.header.stamp = this->get_clock()->now();

    joint_pub_->publish(joint_state);
}


void FaceFollower::sendVelocityBase(float linear_vel, float angular_vel)
{
    // Appliquer l'inversion si nécessaire
    if (invert_linear_vel_) {
        linear_vel = -linear_vel;
    }

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_vel;
    twist.angular.z = angular_vel;

    RCLCPP_DEBUG(this->get_logger(), "🚗 Publishing Twist: linear.x=%.3f, angular.z=%.3f",
                twist.linear.x, twist.angular.z);

    base_control_pub_->publish(twist);
}


float FaceFollower::controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd, float dead_zone)
{
    if (std::abs(x) < dead_zone)
    {
        RCLCPP_DEBUG(this->get_logger(), "PID error %.3f below threshold %.3f, skipped", x, dead_zone);
        return 0.0f;
    }
    return Kp * x + Ki * ix + Kd * dx;
}


void FaceFollower::setFollowerStatusCallback(
    const std::shared_ptr<qbo_msgs::srv::SetFollowerStatus::Request> request,
    std::shared_ptr<qbo_msgs::srv::SetFollowerStatus::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "📡 Service call received: head=%s, base_rotation=%s",
                request->enable_head_movement ? "ON" : "OFF",
                request->enable_base_rotation ? "ON" : "OFF");

    head_movement_enabled_ = request->enable_head_movement;
    base_rotation_enabled_ = request->enable_base_rotation;

    response->success = true;
    response->message = "Follower status updated successfully";

    // Publier le nouveau statut immédiatement
    publishStatus();

    RCLCPP_INFO(this->get_logger(), "✅ Follower capabilities updated");
}


void FaceFollower::calculateFace3DPosition(float distance, float head_pan, float head_tilt,
                                            float& x, float& y, float& z)
{
    // Calcul des coordonnées 3D du visage dans le repère du robot
    // Repère robot : X = avant, Y = gauche, Z = haut
    // head_pan : rotation horizontale (positif = gauche)
    // head_tilt : rotation verticale (positif = BAS pour le QBO, donc on inverse)

    float cos_tilt = std::cos(head_tilt);

    x = distance * cos_tilt * std::cos(head_pan);  // Avant
    y = distance * cos_tilt * std::sin(head_pan);  // Gauche
    z = -distance * std::sin(head_tilt);           // Haut (signe inversé pour convention QBO)
}


void FaceFollower::publishStatus()
{
    auto status_msg = qbo_msgs::msg::FollowerStatus();
    status_msg.header.stamp = this->get_clock()->now();
    status_msg.header.frame_id = "base_link";

    // État de tracking
    status_msg.tracking_state = current_tracking_state_;

    // Capacités activées
    status_msg.head_movement_enabled = head_movement_enabled_;
    status_msg.base_rotation_enabled = base_rotation_enabled_;

    // Raison de blocage
    status_msg.blocking_reason = current_blocking_reason_;

    // Position et distance du visage
    status_msg.face_distance = last_face_distance_;
    status_msg.face_x = last_face_x_;
    status_msg.face_y = last_face_y_;
    status_msg.face_z = last_face_z_;

    // Position actuelle de la tête
    status_msg.head_pan = yaw_from_joint_;
    status_msg.head_tilt = pitch_from_joint_;

    // Position dans l'image
    status_msg.face_u = last_face_u_;
    status_msg.face_v = last_face_v_;

    // Nombre de visages
    status_msg.faces_detected = last_faces_detected_;

    status_pub_->publish(status_msg);
}


void FaceFollower::headToZeroPosition()
{
    RCLCPP_INFO(this->get_logger(), "Resetting head to default position...");

    // On utilise le publisher ROS 2 existant, pas besoin d’en créer un autre
    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"head_pan_joint", "head_tilt_joint"};
    joint_state.position = {0.0, 0.0};
    joint_state.velocity = {0.4, 0.4};

    yaw_from_joint_ = 1.0f;
    pitch_from_joint_ = 1.0f;

    rclcpp::Time start = this->get_clock()->now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok())
    {
        joint_state.header.stamp = this->get_clock()->now();
        joint_pub_->publish(joint_state);
        rclcpp::spin_some(shared_from_this());

        if (yaw_from_joint_ == 0.0f && pitch_from_joint_ == 0.0f)
            break;

        if ((this->get_clock()->now() - start).seconds() >= 4.0)
            break;

        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Reset complete.");
}

std::shared_ptr<FaceFollower> node;

void handleSigInt(int)
{
    if (node) {
        node->headToZeroPosition();
    }
    rclcpp::shutdown();  // Ensuite shutdown ROS proprement
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    // options.automatically_declare_parameters_from_overrides(true);

    node = std::make_shared<FaceFollower>(options);
    signal(SIGINT, handleSigInt);  // Remplace le handler ROS interne
    rclcpp::spin(node);

    return 0;
}






