#include "../include/face_follower.hpp"


FaceFollower::FaceFollower()
: rclcpp::Node("face_follower")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Qbo face follower node");
    onInit();
    RCLCPP_INFO(this->get_logger(), "Ready for face following. Waiting for face positions");
}

// FaceFollower::~FaceFollower()
// {
// 	// deleteROSParams();
// 	printf("Qbo face tracking successfully ended\n");
// }

void FaceFollower::declare_and_get_parameters()
{
    this->declare_parameter("move_base", false);
    this->declare_parameter("move_head", true);
    this->declare_parameter("search_min_pan", -0.3);
    this->declare_parameter("search_max_pan", 0.3);
    this->declare_parameter("search_pan_vel", 0.3);
    this->declare_parameter("search_min_tilt", 0.7);
    this->declare_parameter("search_max_tilt", 0.7);
    this->declare_parameter("search_tilt_vel", 0.3);
    this->declare_parameter("desired_distance", 1.0);
    this->declare_parameter("send_stop", true);

    this->get_parameter("move_base", move_base_bool_);
    this->get_parameter("move_head", move_head_bool_);
    this->get_parameter("search_min_pan", search_min_pan_);
    this->get_parameter("search_max_pan", search_max_pan_);
    this->get_parameter("search_pan_vel", search_pan_vel_);
    this->get_parameter("search_min_tilt", search_min_tilt_);
    this->get_parameter("search_max_tilt", search_max_tilt_);
    this->get_parameter("search_tilt_vel", search_tilt_vel_);
    this->get_parameter("desired_distance", desired_distance_);
    this->get_parameter("send_stop", send_stop_);
}


void FaceFollower::onInit()
{
    declare_and_get_parameters();

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/stereo/left/camera_info", 10,
        std::bind(&FaceFollower::cameraInfoCallback, this, std::placeholders::_1));

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/cmd_joints", 1);
    base_control_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    yaw_from_joint_ = 0.0f;
    pitch_from_joint_ = 0.0f;
    min_pitch_ = -0.5f;

    // PID init
    // For head's pan movement
    u_act_ = u_prev_ = diff_u_ = 0.0f;
    kp_u_ = 0.0015f; // Réduction du gain proportionnel
    ki_u_ = 0.0001f; // Réduction du gain dérivé
    kd_u_ = 0.0003f; // Ajout d'un peu d'intégration pour lisser le suivi

    // For head's tilt movement
    v_act_ = v_prev_ = diff_v_ = 0.0f;
    kp_v_ = 0.0015f; ki_v_ = 0.0003f; kd_v_ = 0.0001f;

    // For base's linear movement
    distance_act_ = distance_prev_ = diff_distance_ = 0.0f;
    kp_distance_ = 0.1f; ki_distance_ = 0.0f; kd_distance_ = 0.1f;

    // For base's angular movement
    yaw_act_ = yaw_prev_ = diff_yaw_ = 0.0f;
    kp_yaw_ = 0.5f; ki_yaw_ = 0.0f; kd_yaw_ = 0.0f;

    image_width_ = 640;
    image_height_ = 480;
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
        float pan_vel = controlPID(u_act_, 0.0f, diff_u_, kp_u_, ki_u_, kd_u_);
        u_prev_ = u_act_;

        // === TILT PID (vertical) ===
        v_act_ = msg->v;
        diff_v_ = v_act_ - v_prev_;
        float tilt_vel = controlPID(v_act_, 0.0f, diff_v_, kp_v_, ki_v_, kd_v_);
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
        float linear_vel = controlPID(distance_act_, 0.0f, diff_distance_, kp_distance_, ki_distance_, kd_distance_);
        distance_prev_ = distance_act_;

        // Protéger contre l'effet "rebond tête vers le haut"
        bool head_near_border = (msg->v + (msg->image_height / 2)) < 100;
        if (pitch_from_joint_ <= min_pitch_ && head_near_border && linear_vel > 0.0)
        {
            RCLCPP_INFO(this->get_logger(), "Head near border, blocking forward motion");
            linear_vel = 0.0;
        }

        // === YAW PID (rotation base) ===
        yaw_act_ = yaw_from_joint_;
        diff_yaw_ = yaw_act_ - yaw_prev_;
        float angular_vel = controlPID(yaw_act_, 0.0f, diff_yaw_, kp_yaw_, ki_yaw_, kd_yaw_);
        yaw_prev_ = yaw_act_;

        RCLCPP_INFO(this->get_logger(), "Moving base: linear %.3f, angular %.3f", linear_vel, angular_vel);

        sendVelocityBase(linear_vel, angular_vel);
    }
    else
    {
        // Générer des angles aléatoires dans les bornes définies
        std::random_device rd;
        std::mt19937 gen(rd());

        std::uniform_real_distribution<float> rand_pan(search_min_pan_, search_max_pan_);
        std::uniform_real_distribution<float> rand_tilt(search_min_tilt_, search_max_tilt_);

        float pan = rand_pan(gen);
        float tilt = rand_tilt(gen);

        RCLCPP_INFO(this->get_logger(), "No face. Scanning with head pos(%.2f, %.2f)", tilt, pan);

        if (move_head_bool_)
            setHeadPositionGlobal(tilt, pan, search_tilt_vel_, search_pan_vel_);

        if (move_base_bool_ && send_stop_)
            sendVelocityBase(0.0f, 0.0f);
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
    // Contrainte : valeur entre -1.5 et +1.5 (pan)
    pos_leftright = std::clamp(pos_leftright, -1.5f, 1.5f);
    pos_updown    = std::clamp(pos_updown,   -1.5f, 1.5f);

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"head_pan_joint", "head_tilt_joint"};
    joint_state.position = {pos_leftright, pos_updown};
    joint_state.velocity = {std::abs(vel_leftright), std::abs(vel_updown)};
    joint_state.header.stamp = this->get_clock()->now();

    joint_pub_->publish(joint_state);
}


void FaceFollower::sendVelocityBase(float linear_vel, float angular_vel)
{
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_vel;
    twist.angular.z = angular_vel;

    base_control_pub_->publish(twist);
}


float FaceFollower::controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd)
{
    float dead_zone = 2.0f;  // pixels
    if (std::abs(x) < dead_zone)
    {
        RCLCPP_DEBUG(this->get_logger(), "PID error below threshold, skipped");
        return 0.0f;
    }
    return Kp * x + Ki * ix + Kd * dx;
}


void FaceFollower::headToZeroPosition()
{
    RCLCPP_INFO(this->get_logger(), "Resetting head to default position...");

    auto joint_pub_tmp = create_publisher<sensor_msgs::msg::JointState>("/cmd_joints", 1);
    auto joint_sub_tmp = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&FaceFollower::jointStateCallback, this, std::placeholders::_1));

    auto joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"head_pan_joint", "head_tilt_joint"};
    joint_state.position = {0.0, 0.0};
    joint_state.velocity = {0.2, 0.2};

    yaw_from_joint_ = 1.0f;
    pitch_from_joint_ = 1.0f;

    rclcpp::Time start = this->now();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        joint_state.header.stamp = this->get_clock()->now();
        joint_pub_tmp->publish(joint_state);
        rclcpp::spin_some(shared_from_this());

        if (yaw_from_joint_ == 0.0f && pitch_from_joint_ == 0.0f)
            break;

        if ((this->now() - start).seconds() >= 4.0)
            break;

        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Reset complete.");
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceFollower>();
    rclcpp::spin(node);
    node->headToZeroPosition();
    rclcpp::shutdown();
    return 0;
}






