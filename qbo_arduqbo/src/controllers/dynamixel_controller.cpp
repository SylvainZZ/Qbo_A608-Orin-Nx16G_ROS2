#include "qbo_arduqbo/controllers/dynamixel_controller.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

DynamixelServo::DynamixelServo(const std::shared_ptr<rclcpp::Node> & node,
                               const std::string & name,
                               dynamixel::PortHandler *portHandler,
                               dynamixel::PacketHandler *packetHandler)
    : name_(name), joint_name_(name), node_(node), portHandler_(portHandler), packetHandler_(packetHandler), port_owner_(false)
{
    id_ = 1;
    neutral_ = 512;
    ticks_ = 1024;
    max_angle_ = radians(150.0);
    min_angle_ = radians(-150.0);
    max_speed_ = radians(720.0);
    invert_ = false;
    angle_ = 0.0f;
    range_ = radians(300.0);
    rad_per_tick_ = range_ / ticks_;

    servo_torque_enable_srv_ = node_->create_service<qbo_msgs::srv::TorqueEnable>(
        name_ + "/torque_enable",
        std::bind(&DynamixelServo::servoTorqueEnable, this, _1, _2)
    );
}

DynamixelServo::~DynamixelServo()
{
    if (port_owner_) {
        uint8_t dxl_error = 0;
        packetHandler_->write1ByteTxRx(portHandler_, id_, P_TORQUE_ENABLE, 0, &dxl_error);
        portHandler_->closePort();
    }
}

bool DynamixelServo::servoTorqueEnable(
    const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
    std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res)
{
    uint8_t dxl_error = 0;
    int comm_result = packetHandler_->write1ByteTxRx(portHandler_, id_, P_TORQUE_ENABLE, req->torque_enable, &dxl_error);
    res->success = (comm_result == COMM_SUCCESS && dxl_error == 0);
    return true;
}

void DynamixelServo::setParams(const std::string & joint_name)
{
    std::string base = "dynamixel.motors." + joint_name + ".";

    node_->declare_parameter(base + "id", id_);
    node_->get_parameter(base + "id", id_);

    node_->declare_parameter(base + "neutral", neutral_);
    node_->get_parameter(base + "neutral", neutral_);

    node_->declare_parameter(base + "ticks", ticks_);
    node_->get_parameter(base + "ticks", ticks_);

    node_->declare_parameter(base + "invert", invert_);
    node_->get_parameter(base + "invert", invert_);

    double max_deg = 150.0;
    node_->declare_parameter(base + "max_angle_degrees", max_deg);
    node_->get_parameter(base + "max_angle_degrees", max_deg);
    max_angle_ = radians(max_deg);

    double min_deg = -150.0;
    node_->declare_parameter(base + "min_angle_degrees", min_deg);
    node_->get_parameter(base + "min_angle_degrees", min_deg);
    min_angle_ = radians(min_deg);

    double range_deg = 300.0;
    node_->declare_parameter(base + "range", range_deg);
    node_->get_parameter(base + "range", range_deg);
    range_ = radians(range_deg);

    double speed_val = radians(720.0);
    node_->declare_parameter(base + "max_speed", speed_val);
    node_->get_parameter(base + "max_speed", speed_val);
    max_speed_ = speed_val;

    rad_per_tick_ = range_ / ticks_;

    RCLCPP_INFO(node_->get_logger(),
        "[%s] Parametres chargés : id=%d, invert=%s, neutral=%d, min=%.2f rad, max=%.2f rad, ticks=%d, range=%.2f rad",
        joint_name.c_str(), id_, invert_ ? "true" : "false", neutral_,
        min_angle_, max_angle_, ticks_, range_);
}



void DynamixelServo::setAngle(float ang, float velocity)
{
    if (ang > max_angle_) ang = max_angle_;
    if (ang < min_angle_) ang = min_angle_;
    if (velocity > max_speed_) velocity = max_speed_;
    if (invert_) ang = -ang;

    int goal_ticks = static_cast<int>(std::round(ang / rad_per_tick_)) + neutral_;
    int speed_val = static_cast<int>(std::round(velocity / rad_per_tick_));
    if (speed_val == 0) speed_val = 1;

    changeTorque(254);
    uint8_t dxl_error = 0;

    packetHandler_->write2ByteTxRx(portHandler_, id_, P_TORQUE_LIMIT_L, 1023, &dxl_error);
    packetHandler_->write2ByteTxRx(portHandler_, id_, P_GOAL_POSITION_L, goal_ticks, &dxl_error);
    packetHandler_->write2ByteTxRx(portHandler_, id_, P_GOAL_SPEED_L, speed_val, &dxl_error);
}

void DynamixelServo::changeTorque(int torque)
{
    uint8_t dxl_error = 0;
    packetHandler_->write1ByteTxRx(portHandler_, id_, P_CW_COMPILANCE_SLOPE, torque, &dxl_error);
    packetHandler_->write1ByteTxRx(portHandler_, id_, P_CCW_COMPILANCE_SLOPE, torque, &dxl_error);
}

//
// DynamixelController
//

DynamixelController::DynamixelController(const std::shared_ptr<rclcpp::Node> & node) : node_(node)
{
    node_->declare_parameter("dynamixel.usb_port", "/dev/ttyUSB0");
    node_->declare_parameter("dynamixel.baud_rate", 57600);
    node_->declare_parameter("dynamixel.protocol_version", 1.0);

    usb_port_ = node_->get_parameter("dynamixel.usb_port").as_string();
    baud_rate_ = node_->get_parameter("dynamixel.baud_rate").as_int();
    protocol_version_ = node_->get_parameter("dynamixel.protocol_version").as_double();

    portHandler_ = dynamixel::PortHandler::getPortHandler(usb_port_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

    if (!portHandler_->openPort())
        throw std::runtime_error("Failed to open port " + usb_port_);
    if (!portHandler_->setBaudRate(baud_rate_))
        throw std::runtime_error("Failed to set baud rate");

    // Ajout Workbench
    if (!dxl_wb_.init(usb_port_.c_str(), baud_rate_)) {
        throw std::runtime_error("DynamixelWorkbench init failed");
    }
    dxl_wb_.setPacketHandler(1.0);

    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    dynamixel_state_pub_ = node_->create_publisher<qbo_msgs::msg::MotorState>("/dynamixel_state", 10);

    joint_cmd_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/cmd_joints", 10, std::bind(&DynamixelController::jointCmdCallback, this, _1)
    );

    int joint_timer_rate = 30; // par défaut
    node_->declare_parameter("dynamixel_joint_rate_hz", joint_timer_rate);
    node_->get_parameter("dynamixel_joint_rate_hz", joint_timer_rate);

    RCLCPP_INFO(node_->get_logger(), "⏱ Fréquence /dynamixel_joint = %d Hz", joint_timer_rate);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / joint_timer_rate));
    joint_state_timer_ = node_->create_wall_timer(period, std::bind(&DynamixelController::publishJointStates, this));
    cmd_timer_ = node_->create_wall_timer(period, std::bind(&DynamixelController::cmdTimerCallback, this));

    int state_rate = 5;  // par défaut
    node_->declare_parameter("dynamixel_state_rate_hz", state_rate);
    node_->get_parameter("dynamixel_state_rate_hz", state_rate);

    RCLCPP_INFO(node_->get_logger(), "⏱ Fréquence /dynamixel_state = %d Hz", state_rate);

    int period_ms = static_cast<int>(1000.0 / state_rate);
    dynamixel_state_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&DynamixelController::publishMotorStates, this));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    RCLCPP_INFO(node_->get_logger(), "DynamixelController ready.");

    // auto all = node_->list_parameters({}, 10);
    // RCLCPP_INFO(node_->get_logger(), "🔎 Paramètres déclarés :");
    // for (const auto &p : all.names) {
    //     RCLCPP_INFO(node_->get_logger(), " - %s", p.c_str());
    // }

    std::vector<std::string> keys;
    if (!node_->get_parameter("dynamixel.motor_keys", keys)) {
        RCLCPP_FATAL(node_->get_logger(), "❌ Paramètre 'dynamixel.motor_keys' introuvable");
        throw std::runtime_error("motor_keys manquant");
    }

    std::map<std::string, std::string> motor_to_joint;

    for (const auto &key : keys) {
        std::string joint_name = key;
        RCLCPP_INFO(node_->get_logger(), "🔍 Test lecture: dynamixel.motors.%s.name", key.c_str());
        node_->get_parameter("dynamixel.motors." + key + ".name", joint_name);
        motor_to_joint[key] = joint_name;
    }

    for (const auto &[motor_key, joint_name] : motor_to_joint) {
        RCLCPP_INFO(node_->get_logger(), "⏺ Chargement du servo : %s", joint_name.c_str());
        RCLCPP_INFO(node_->get_logger(), "🧩 Bloc YAML %s → joint: %s", motor_key.c_str(), joint_name.c_str());
        auto servo = std::make_unique<DynamixelServo>(node_, joint_name, portHandler_, packetHandler_);
        servo->setParams(motor_key);
        servo->setAngle(0.0f, 1.0f);
        servos_.push_back(std::move(servo));
    }

    for (const auto &servo : servos_) {
        uint16_t model_number = 0;
        if (dxl_wb_.ping(servo->id_, &model_number)) {
            RCLCPP_INFO(node_->get_logger(), "✔️ ID %d détecté (modèle %d)", servo->id_, model_number);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "❌ Ping échoué pour ID %d", servo->id_);
        }
    }
    // 🔧 DIAGNOSTICS
    diagnostics_ = std::make_shared<diagnostic_updater::Updater>(node_);
    diagnostics_->setHardwareID("qbo_dynamixel");

    double temp_warn = 65.0;
    double volt_min = 8.0;
    double volt_max = 12.5;
    int error_thresh = 20;

    node_->declare_parameter("diagnostic_temp_warn", temp_warn);
    node_->declare_parameter("diagnostic_voltage_min", volt_min);
    node_->declare_parameter("diagnostic_voltage_max", volt_max);
    node_->declare_parameter("diagnostic_position_error", error_thresh);

    node_->get_parameter("diagnostic_temp_warn", temp_warn);
    node_->get_parameter("diagnostic_voltage_min", volt_min);
    node_->get_parameter("diagnostic_voltage_max", volt_max);
    node_->get_parameter("diagnostic_position_error", error_thresh);

    for (const auto &servo : servos_) {
        diagnostics_->add(servo->joint_name_, [this, servo = servo.get(), temp_warn, volt_min, volt_max, error_thresh]
                        (diagnostic_updater::DiagnosticStatusWrapper & stat) {
            qbo_msgs::msg::MotorState state;

            int32_t goal = 0, position = 0, val = 0;

            dxl_wb_.itemRead(servo->id_, "Goal_Position", &goal);
            dxl_wb_.itemRead(servo->id_, "Present_Position", &position);
            int error = goal - position;
            dxl_wb_.itemRead(servo->id_, "Present_Temperature", &val);
            int temp = val;
            dxl_wb_.itemRead(servo->id_, "Present_Voltage", &val);
            float volt = static_cast<float>(val) / 10.0f;
            dxl_wb_.itemRead(servo->id_, "Moving", &val);
            bool moving = (val == 1);

            stat.add("ID", servo->id_);
            stat.add("Température", temp);
            stat.add("Tension", volt);
            stat.add("Erreur position", error);
            stat.add("Moving", moving);

            if (temp > temp_warn)
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "⚠️ Température élevée !");
            else if (volt < volt_min || volt > volt_max)
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "⚠️ Tension hors plage !");
            else if (std::abs(error) > error_thresh)
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "⚠️ Erreur de position importante");
            else
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
        });
    }
}

DynamixelController::~DynamixelController()
{
    for (auto &servo : servos_) {
        uint8_t dxl_error = 0;
        packetHandler_->write1ByteTxRx(portHandler_, servo->id_, P_TORQUE_ENABLE, 0, &dxl_error);
    }
    portHandler_->closePort();
}

void DynamixelController::jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_cmd_msg_ = *msg;
    has_new_cmd_ = true;
}

void DynamixelController::cmdTimerCallback()
{
    if (!has_new_cmd_) return;

    bool velocityIncluded = (joint_cmd_msg_.velocity.size() == joint_cmd_msg_.position.size());

    for (size_t i = 0; i < joint_cmd_msg_.name.size(); i++) {
        const std::string & name = joint_cmd_msg_.name[i];
        float pos = joint_cmd_msg_.position[i];
        float vel = velocityIncluded ? joint_cmd_msg_.velocity[i] : 1.0f;

        for (auto &servo : servos_) {
            if (servo->joint_name_ == name) {
                servo->setAngle(pos, vel);
                break;
            }
        }
    }
    has_new_cmd_ = false;
}

void DynamixelController::publishJointStates()
{
    auto now = node_->now();
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now;
    for (const auto &servo : servos_) {
        msg.name.push_back(servo->joint_name_);
        msg.position.push_back(servo->angle_);
        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);

        // ✅ TF dynamique selon le servo concerné
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;

        tf2::Quaternion q;

        if (servo->joint_name_ == "head_pan_joint")
        {
            tf_msg.header.frame_id = "base_link";
            tf_msg.child_frame_id = "base_pan_link";
            tf_msg.transform.translation.x = 0.045; // depuis <origin>
            tf_msg.transform.translation.y = 0.0;
            tf_msg.transform.translation.z = 0.35;
            q.setRPY(0.0, 0.0, servo->angle_); // rotation autour de Z
        }
        else if (servo->joint_name_ == "head_tilt_joint")
        {
            tf_msg.header.frame_id = "base_pan_link";
            tf_msg.child_frame_id = "head_tilt_link";
            tf_msg.transform.translation.x = 0.0;  // <origin xyz="0 0 0">
            tf_msg.transform.translation.y = 0.0;
            tf_msg.transform.translation.z = 0.0;
            q.setRPY(0.0, servo->angle_, 0.0); // rotation autour de Y
        }
        else
        {
            continue; // on ne publie pas de TF pour les autres servos
        }

        tf_msg.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf_msg);
    }
    joint_state_pub_->publish(msg);
}

void DynamixelController::publishMotorStates()
{
    for (const auto &servo : servos_) {
        qbo_msgs::msg::MotorState state;
        state.header.stamp = node_->now();
        state.header.frame_id = servo->joint_name_;
        state.id = servo->id_;
        state.goal = 0;
        state.error = 0;
        state.speed = 0;
        state.load = 0.0f;
        state.voltage = 0.0f;
        state.temperature = 0;
        state.moving = false;

        int32_t goal = 0;
        int32_t position = 0;
        int32_t val = 0;

        if (dxl_wb_.itemRead(servo->id_, "Goal_Position", &goal))
            state.goal = goal;

        if (dxl_wb_.itemRead(servo->id_, "Present_Position", &position)) {
            state.position = position;
            state.error = state.goal - state.position;
        }

        if (dxl_wb_.itemRead(servo->id_, "Present_Speed", &val))
            state.speed = val;

        if (dxl_wb_.itemRead(servo->id_, "Present_Load", &val))
            state.load = static_cast<float>(val) / 1024.0f;

        if (dxl_wb_.itemRead(servo->id_, "Present_Voltage", &val))
            state.voltage = static_cast<float>(val) / 10.0f;

        if (dxl_wb_.itemRead(servo->id_, "Present_Temperature", &val))
            state.temperature = val;

        if (dxl_wb_.itemRead(servo->id_, "Moving", &val))
            state.moving = (val == 1);

        dynamixel_state_pub_->publish(state);

    }

}

