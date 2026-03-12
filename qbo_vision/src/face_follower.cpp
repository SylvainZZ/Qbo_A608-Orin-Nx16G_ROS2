#include "../include/face_follower.hpp"
#include <signal.h>  // pour signal()



FaceFollower::FaceFollower(const rclcpp::NodeOptions & options)
: rclcpp::Node("qbo_face_following", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Qbo face follower node");
    onInit();
    RCLCPP_INFO(this->get_logger(), "Ready for face following. Waiting for face positions");
}


void FaceFollower::declare_and_get_parameters()
{
    this->declare_parameter("move_base", false);
    this->declare_parameter("move_head", true);
    this->declare_parameter("send_stop", true);

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

    this->declare_parameter("desired_distance", 1.0);

    // PID gains for head control
    this->declare_parameter("head_pan_kp", 0.005);
    this->declare_parameter("head_pan_ki", 0.0003);
    this->declare_parameter("head_pan_kd", 0.001);
    this->declare_parameter("head_tilt_kp", 0.005);
    this->declare_parameter("head_tilt_ki", 0.0003);
    this->declare_parameter("head_tilt_kd", 0.001);

    // PID gains for base control
    this->declare_parameter("base_distance_kp", 0.3);
    this->declare_parameter("base_distance_ki", 0.02);
    this->declare_parameter("base_distance_kd", 0.15);
    this->declare_parameter("base_yaw_kp", 0.8);
    this->declare_parameter("base_yaw_ki", 0.01);
    this->declare_parameter("base_yaw_kd", 0.05);
    // Base control configuration
    this->declare_parameter("base_cmd_vel_topic", std::string("/qbo_arduqbo/base_ctrl/cmd_vel"));
    this->declare_parameter("invert_linear_vel", false);

    this->get_parameter("move_base", move_base_bool_);
    this->get_parameter("move_head", move_head_bool_);
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

    this->get_parameter("desired_distance", desired_distance_);

    // Get PID gains
    this->get_parameter("head_pan_kp", kp_u_);
    this->get_parameter("head_pan_ki", ki_u_);
    this->get_parameter("head_pan_kd", kd_u_);
    this->get_parameter("head_tilt_kp", kp_v_);
    this->get_parameter("head_tilt_ki", ki_v_);
    this->get_parameter("head_tilt_kd", kd_v_);
    this->get_parameter("base_distance_kp", kp_distance_);
    this->get_parameter("base_distance_ki", ki_distance_);
    this->get_parameter("base_distance_kd", kd_distance_);
    this->get_parameter("base_yaw_kp", kp_yaw_);
    this->get_parameter("base_yaw_ki", ki_yaw_);
    this->get_parameter("base_yaw_kd", kd_yaw_);

    this->get_parameter("base_cmd_vel_topic", base_cmd_vel_topic_);
    this->get_parameter("invert_linear_vel", invert_linear_vel_);

    RCLCPP_INFO(this->get_logger(), "Base control topic: %s, invert_linear: %s",
               base_cmd_vel_topic_.c_str(), invert_linear_vel_ ? "true" : "false");
}


void FaceFollower::onInit()
{
    declare_and_get_parameters();

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_left/camera_info", 10,
        std::bind(&FaceFollower::cameraInfoCallback, this, std::placeholders::_1));

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/cmd_joints", 1);
    base_control_pub_ = create_publisher<geometry_msgs::msg::Twist>(base_cmd_vel_topic_, 1);

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

    // For base's linear movement
    distance_act_ = distance_prev_ = diff_distance_ = 0.0f;
    integral_distance_ = 0.0f;  // Accumulation intégrale
    kp_distance_ = 0.35f;
    ki_distance_ = 0.02f;
    kd_distance_ = 0.15f;

    // For base's angular movement
    yaw_act_ = yaw_prev_ = diff_yaw_ = 0.0f;
    integral_yaw_ = 0.0f;  // Accumulation intégrale
    kp_yaw_ = 0.35f;
    ki_yaw_ = 0.01f;
    kd_yaw_ = 0.05f;

    image_width_ = 640;
    image_height_ = 480;
    last_scan_move_ = this->get_clock()->now();
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

        face_pos_sub_ = create_subscription<qbo_msgs::msg::FacePosAndDist>(
            "/qbo_face_tracking/face_pos_and_dist", 10,
            std::bind(&FaceFollower::facePositionCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Camera calibration received, subscriptions initialized");
    }
}


void FaceFollower::facePositionCallback(const qbo_msgs::msg::FacePosAndDist::SharedPtr msg)
{
    image_width_ = msg->image_width;
    image_height_ = msg->image_height;

    // Rafraîchir les paramètres dynamiques
    this->get_parameter("move_base", move_base_bool_);
    this->get_parameter("move_head", move_head_bool_);
    this->get_parameter("send_stop", send_stop_);
    this->get_parameter("search_min_pan", search_min_pan_);
    this->get_parameter("search_max_pan", search_max_pan_);
    this->get_parameter("search_pan_vel", search_pan_vel_);
    this->get_parameter("search_min_tilt", search_min_tilt_);
    this->get_parameter("search_max_tilt", search_max_tilt_);
    this->get_parameter("search_tilt_vel", search_tilt_vel_);
    this->get_parameter("desired_distance", desired_distance_);

    if (msg->face_detected)
    {
        // === PAN PID (horizontal) ===
        u_act_ = msg->u;
        diff_u_ = u_act_ - u_prev_;
        float pan_vel = controlPID(u_act_, 0.0f, diff_u_, kp_u_, ki_u_, kd_u_, 2.0f);  // 2 pixels
        u_prev_ = u_act_;

        // === TILT PID (vertical) ===
        v_act_ = msg->v;
        diff_v_ = v_act_ - v_prev_;
        float tilt_vel = controlPID(v_act_, 0.0f, diff_v_, kp_v_, ki_v_, kd_v_, 2.0f);  // 2 pixels
        v_prev_ = v_act_;

        RCLCPP_INFO(this->get_logger(), "Moving head: pos(%.2f, %.2f), vel(%.3f, %.3f)", v_act_, u_act_, tilt_vel, pan_vel);

        if (move_head_bool_)
        {
            setHeadPositionToFace(v_act_, u_act_, tilt_vel, pan_vel);
        }

        // === transition vers base plus tard
        if (!move_base_bool_)
        {
            RCLCPP_INFO(this->get_logger(), "Base movement disabled");
            return;
        }

                // === DISTANCE PID (avance base) ===
        distance_act_ = msg->distance_to_head - desired_distance_;
        diff_distance_ = distance_act_ - distance_prev_;
        integral_distance_ += distance_act_;  // Accumulation de l'erreur
        // Anti-windup : limiter l'intégrale
        integral_distance_ = std::clamp(integral_distance_, -10.0f, 10.0f);
        
        float linear_vel = controlPID(distance_act_, integral_distance_, diff_distance_, kp_distance_, ki_distance_, kd_distance_, 0.03f);  // 3 cm
        distance_prev_ = distance_act_;

        RCLCPP_INFO(this->get_logger(), "Distance: actual=%.2fm, desired=%.2fm, error=%.2fm, linear_vel_raw=%.3f",
                    msg->distance_to_head, desired_distance_, distance_act_, linear_vel);

        // Protéger contre l'effet "rebond tête vers le haut"
        bool head_near_border = (msg->v + (msg->image_height / 2)) < 100;
        if (pitch_from_joint_ <= min_pitch_ && head_near_border && linear_vel > 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Head near border (pitch=%.2f, v=%.2f), blocking forward motion",
                       pitch_from_joint_, msg->v);
            linear_vel = 0.0;
        }

        // Bloquer le recul (sécurité : pas de capteur arrière)
        if (linear_vel < 0.0f)
        {
            RCLCPP_WARN(this->get_logger(), "⛔ Recul interdit pour sécurité (linear_vel=%.3f), bloqué à 0",
                       linear_vel);
            linear_vel = 0.0f;
        }

        // Limiter la vitesse maximale d'avance
        linear_vel = std::clamp(linear_vel, 0.0f, 0.3f);  // 0 à 30 cm/s max (avance seule)

        // === YAW PID (rotation base) ===
        yaw_act_ = yaw_from_joint_;
        diff_yaw_ = yaw_act_ - yaw_prev_;
        integral_yaw_ += yaw_act_;  // Accumulation de l'erreur
        // Anti-windup : limiter l'intégrale
        integral_yaw_ = std::clamp(integral_yaw_, -10.0f, 10.0f);

        float angular_vel = controlPID(yaw_act_, integral_yaw_, diff_yaw_, kp_yaw_, ki_yaw_, kd_yaw_, 0.05f);  // ~3 degrés
        yaw_prev_ = yaw_act_;

        // Limiter les vitesses maximales
        angular_vel = std::clamp(angular_vel, -0.8f, 0.8f);  // ±0.8 rad/s max

        RCLCPP_INFO(this->get_logger(), "Yaw: head_pan=%.2f rad, angular_vel=%.3f", yaw_from_joint_, angular_vel);
        RCLCPP_INFO(this->get_logger(), ">>> Sending to base: linear=%.3f m/s, angular=%.3f rad/s", linear_vel, angular_vel);

        sendVelocityBase(linear_vel, angular_vel);
    }
    else
    {
        // Réinitialiser les intégrales quand le visage est perdu
        integral_distance_ = 0.0f;
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

            RCLCPP_INFO(this->get_logger(), "No face. Scanning with head pos(%.2f, %.2f)", tilt, pan);

            if (move_head_bool_)
                setHeadPositionGlobal(tilt, pan, search_tilt_vel_, search_pan_vel_);

            if (move_base_bool_ && send_stop_)
                sendVelocityBase(0.0f, 0.0f);
        }
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
        RCLCPP_INFO(this->get_logger(), "Minimal head movement, ignoring");
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






