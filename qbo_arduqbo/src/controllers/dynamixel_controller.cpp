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
                               DynamixelWorkbench* wb)
    : name_(name), joint_name_(name), node_(node), dxl_wb_(wb)
{
    id_ = 1;
    neutral_ = 512;
    ticks_ = 1024;
    max_angle_ = radians(20.0);
    min_angle_ = radians(-20.0);
    max_speed_ = radians(180.0);
    invert_ = false;
    angle_ = 0.0f;
    range_ = radians(40.0);
    rad_per_tick_ = range_ / ticks_;
    torque_limit_ = 1023;

    servo_torque_enable_srv_ = node_->create_service<qbo_msgs::srv::TorqueEnable>(
        name_ + "/torque_enable",
        std::bind(&DynamixelServo::servoTorqueEnable, this, _1, _2)
    );
    param_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&DynamixelServo::onParameterChange, this, std::placeholders::_1));

}

DynamixelServo::~DynamixelServo() = default;

bool DynamixelServo::servoTorqueEnable(
    const std::shared_ptr<qbo_msgs::srv::TorqueEnable::Request> req,
    std::shared_ptr<qbo_msgs::srv::TorqueEnable::Response> res)
{
    if (req->torque_enable)
        res->success = dxl_wb_->torqueOn(id_);
    else
        res->success = dxl_wb_->itemWrite(id_, "Torque_Enable", 0);

    return true;
}

void DynamixelServo::setParams(const std::string & joint_name)
{
    std::string base = "dynamixel.motors." + joint_name + ".";

    // ID du moteur
    node_->declare_parameter(base + "id", id_);
    node_->get_parameter(base + "id", id_);

    // Inversion logique
    node_->declare_parameter(base + "invert", invert_);
    node_->get_parameter(base + "invert", invert_);

    // Position neutre (tick central)
    node_->declare_parameter(base + "neutral", neutral_);
    node_->get_parameter(base + "neutral", neutral_);

    // Résolution
    node_->declare_parameter(base + "ticks", ticks_);
    node_->get_parameter(base + "ticks", ticks_);

    // Angle max (en degrés -> rad)
    double max_deg = 30.0;
    node_->declare_parameter(base + "max_angle_degrees", max_deg);
    node_->get_parameter(base + "max_angle_degrees", max_deg);
    max_angle_ = radians(max_deg);

    // Angle min (en degrés -> rad)
    double min_deg = -30.0;
    node_->declare_parameter(base + "min_angle_degrees", min_deg);
    node_->get_parameter(base + "min_angle_degrees", min_deg);
    min_angle_ = radians(min_deg);

    // Amplitude totale (en degrés -> rad)
    double range_deg = 60.0;
    node_->declare_parameter(base + "range", range_deg);
    node_->get_parameter(base + "range", range_deg);
    range_ = radians(range_deg);

    // Vitesse maximale autorisée (en rad/s)
    double speed_val = radians(180.0);  // par défaut : 2 tours/s
    node_->declare_parameter(base + "max_speed", speed_val);
    node_->get_parameter(base + "max_speed", speed_val);
    max_speed_ = speed_val;

    // Limite de couple
    int torque_limit = 1023;
    node_->declare_parameter(base + "torque_limit", torque_limit);
    node_->get_parameter(base + "torque_limit", torque_limit);
    torque_limit_ = torque_limit;

    // Calcul du ratio radian <-> tick
    rad_per_tick_ = range_ / static_cast<double>(ticks_);

    // Log résumé
    RCLCPP_INFO(node_->get_logger(),
        "[%s] Paramètres chargés : id=%d, invert=%s, neutral=%d, min=%.2f rad, max=%.2f rad, ticks=%d, range=%.2f rad, torque_limit=%d, max_speed=%.2f rad/s",
        joint_name.c_str(), id_, invert_ ? "true" : "false", neutral_,
        min_angle_, max_angle_, ticks_, range_, torque_limit_, max_speed_);
}


rcl_interfaces::msg::SetParametersResult DynamixelServo::onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &param : parameters)
    {
        const auto &key = param.get_name();  // nom complet du paramètre
        const std::string base = "dynamixel.motors." + name_ + ".";

        if (key == base + "max_angle_degrees") {
            max_angle_ = radians(param.as_double());
        }
        else if (key == base + "min_angle_degrees") {
            min_angle_ = radians(param.as_double());
        }
        else if (key == base + "range") {
            range_ = radians(param.as_double());
        }
        else if (key == base + "ticks") {
            ticks_ = param.as_int();
        }
        else if (key == base + "neutral") {
            neutral_ = param.as_int();
        }
        else if (key == base + "torque_limit") {
            torque_limit_ = param.as_int();
            dxl_wb_->writeRegister(id_, "Torque_Limit", torque_limit_);
        }
        else {
            continue;  // ignore les autres
        }

        rad_per_tick_ = range_ / ticks_;
        RCLCPP_INFO(node_->get_logger(), "[%s] 🔄 Paramètre mis à jour : %s", name_.c_str(), key.c_str());
    }

    return result;
}


void DynamixelServo::setAngle(float ang, float velocity)
{
    RCLCPP_INFO(node_->get_logger(), "[%s] 🧪 setAngle() appelée avec ang=%.2f, vel=%.2f, max_speed=%.2f",
            joint_name_.c_str(), ang, velocity, max_speed_);
    // 🔺 Limitation des angles
    if (ang > max_angle_) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] 🔺 Angle %.2f rad dépasse max (%.2f rad). Limité.",
            joint_name_.c_str(), ang, max_angle_);
        ang = max_angle_;
    }

    if (ang < min_angle_) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] 🔻 Angle %.2f rad en dessous min (%.2f rad). Limité.",
            joint_name_.c_str(), ang, min_angle_);
        ang = min_angle_;
    }

    // 🛑 Limitation de la vitesse
    if (velocity > max_speed_) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] ⚠️ Vitesse %.2f rad/s dépasse max (%.2f rad/s). Limité.",
            joint_name_.c_str(), velocity, max_speed_);
        velocity = max_speed_;
    }

    // ↩️ Inversion si nécessaire
    if (invert_) ang = -ang;

    // 🎯 Conversion angle → ticks
    int goal_ticks = static_cast<int>(std::round(ang / rad_per_tick_)) + neutral_;

    // ⚙️ Conversion vitesse → registre Dynamixel (AX-12/18 logic)
    const float MAX_AX_SPEED_RAD = 12.0f;  // 114 RPM ≈ 12 rad/s

    float speed_ratio = std::clamp(velocity / MAX_AX_SPEED_RAD, 0.0f, 1.0f);
    int speed_val = static_cast<int>(std::round(speed_ratio * 1023));
    if (speed_val == 0) speed_val = 1;

    // ✅ Écriture sur les registres
    dxl_wb_->writeRegister(id_, "Torque_Limit", torque_limit_);
    dxl_wb_->itemWrite(id_, "Goal_Position", goal_ticks);
    dxl_wb_->itemWrite(id_, "Moving_Speed", speed_val);

    // 📊 Log d’exécution visible (niveau INFO volontairement)
    RCLCPP_INFO(node_->get_logger(),
        "[%s] 🎯 Pos=%.2f rad (%d ticks), Vit=%.2f rad/s (cmd=%d)",
        joint_name_.c_str(), ang, goal_ticks, velocity, speed_val);
}

//
// DynamixelController
//

DynamixelController::DynamixelController(const std::shared_ptr<rclcpp::Node> & node)
    : node_(node)
{
    node_->declare_parameter("dynamixel.usb_port", "/dev/ttyUSB0");
    node_->declare_parameter("dynamixel.baud_rate", 57600);
    node_->declare_parameter("dynamixel.protocol_version", 1.0);

    usb_port_ = node_->get_parameter("dynamixel.usb_port").as_string();
    baud_rate_ = node_->get_parameter("dynamixel.baud_rate").as_int();
    protocol_version_ = node_->get_parameter("dynamixel.protocol_version").as_double();

    // Initialisation Workbench
    if (!dxl_wb_.init(usb_port_.c_str(), baud_rate_)) {
        throw std::runtime_error("DynamixelWorkbench init failed");
    }
    dxl_wb_.setPacketHandler(1.0);

    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    joint_cmd_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/cmd_joints", 10, std::bind(&DynamixelController::jointCmdCallback, this, _1));

    int joint_timer_rate = 30;
    node_->declare_parameter("dynamixel_joint_rate_hz", joint_timer_rate);
    node_->get_parameter("dynamixel_joint_rate_hz", joint_timer_rate);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / joint_timer_rate));
    joint_state_timer_ = node_->create_wall_timer(period, std::bind(&DynamixelController::publishJointStates, this));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    RCLCPP_INFO(node_->get_logger(), "DynamixelController ready.");

    std::vector<std::string> keys;
    if (!node_->get_parameter("dynamixel.motor_keys", keys)) {
        RCLCPP_FATAL(node_->get_logger(), "❌ Paramètre 'dynamixel.motor_keys' introuvable");
        throw std::runtime_error("motor_keys manquant");
    }

    std::map<std::string, std::string> motor_to_joint;

    for (const auto &key : keys) {
        std::string joint_name = key;
        node_->get_parameter("dynamixel.motors." + key + ".name", joint_name);
        motor_to_joint[key] = joint_name;
    }

    diagnostics_ = std::make_shared<diagnostic_updater::Updater>(node_);
    diagnostics_->setHardwareID("qbo_dynamixel");

    // Paramètres communs de diagnostic
    double temp_warn = 65.0, volt_min = 8.0, volt_max = 12.5;
    int error_thresh = 20;

    node_->declare_parameter("diagnostic_temp_warn", temp_warn);
    node_->declare_parameter("diagnostic_voltage_min", volt_min);
    node_->declare_parameter("diagnostic_voltage_max", volt_max);
    node_->declare_parameter("diagnostic_position_error", error_thresh);

    node_->get_parameter("diagnostic_temp_warn", temp_warn);
    node_->get_parameter("diagnostic_voltage_min", volt_min);
    node_->get_parameter("diagnostic_voltage_max", volt_max);
    node_->get_parameter("diagnostic_position_error", error_thresh);

    // Boucle unique
    for (const auto &[motor_key, joint_name] : motor_to_joint) {
        RCLCPP_INFO(node_->get_logger(), "⏺ Chargement du servo : %s", joint_name.c_str());
        auto servo = std::make_unique<DynamixelServo>(node_, joint_name, &dxl_wb_);
        servo->setParams(motor_key);

        if (dxl_wb_.ping(servo->id_, &servo->model_number_)) {
            RCLCPP_INFO(node_->get_logger(), "✔️ ID %d détecté (modèle %d)", servo->id_, servo->model_number_);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "❌ Ping échoué pour ID %d", servo->id_);
        }

        servo->setAngle(0.0f, 1.0f);

        diagnostics_->add(servo->joint_name_, [this, servo = servo.get(), temp_warn, volt_min, volt_max, error_thresh]
                        (diagnostic_updater::DiagnosticStatusWrapper & stat) {
            int32_t goal = 0, position = 0, val = 0;

            dxl_wb_.itemRead(servo->id_, "Goal_Position", &goal);
            dxl_wb_.itemRead(servo->id_, "Present_Position", &position);
            int error = goal - position;

            dxl_wb_.itemRead(servo->id_, "Present_Load", &val);
            int raw_load = val;

            dxl_wb_.itemRead(servo->id_, "Present_Temperature", &val);
            int temp = val;
            dxl_wb_.itemRead(servo->id_, "Present_Voltage", &val);
            float volt = static_cast<float>(val) / 10.0f;
            dxl_wb_.itemRead(servo->id_, "Moving", &val);
            bool moving = (val == 1);

            // Infos de base
            stat.add("ID", servo->id_);
            stat.add("Température", temp);
            stat.add("Tension", volt);
            stat.add("Erreur position", error);
            stat.add("Moving", moving);
            stat.add("Limite de couple (Torque_Limit)", servo->torque_limit_);

            // Estimation :
            // - Charge normalisée = [0.0–1.0]
            // - Hypothèse : à charge max (1023), on est au courant stall max
            // - AX-18A ≈ 1.5 A @ 12 V
            float load_ratio = static_cast<float>(raw_load & 0x3FF) / 1023.0f;
            float estimated_current = load_ratio * 1.5f;  // 1.5 A max
            float power_watts = volt * estimated_current;

            // Ajout dans le diagnostic
            stat.add("Charge brute", std::to_string(raw_load));
            stat.addf("Consommation estimée", "%.2f W", power_watts);

            // Optionnel : avertissement si > 10 W (par exemple)
            if (power_watts > 10.0f) {
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "⚠️ Consommation élevée !");
            }

            // Infos spécifiques selon modèle
            std::string model_name = "Inconnu", torque = "-", rpm = "-", gear = "-";

            switch (servo->model_number_) {
                case 18:
                    model_name = "AX-18A";
                    torque = "1.8 N·m";
                    rpm = "97 RPM";
                    gear = "254:1";
                    break;
                case 12:
                    model_name = "AX-12A";
                    torque = "1.5 N·m";
                    rpm = "59 RPM";
                    gear = "254:1";
                    break;
                default:
                    model_name = "Modèle inconnu";
                    break;
            }

            stat.add("Modèle", model_name);
            stat.add("Couple (Stall)", torque);
            stat.add("Vitesse à vide", rpm);
            stat.add("Rapport de réduction", gear);

            // Niveau de gravité
            if (temp > temp_warn)
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "⚠️ Température élevée !");
            else if (volt < volt_min || volt > volt_max)
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "⚠️ Tension hors plage !");
            else if (std::abs(error) > error_thresh)
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "⚠️ Erreur de position importante");
            else
                stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
        });

        servos_.push_back(std::move(servo));
    }
}

void DynamixelController::jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "JointState reçu à t=%.3f", node_->now().seconds());

    if (msg->name.size() != msg->position.size()) {
        RCLCPP_WARN(node_->get_logger(), "Nom et position ne sont pas de même taille !");
        return;
    }

    bool velocityIncluded = (msg->velocity.size() == msg->position.size());
    if (!velocityIncluded) {
        RCLCPP_WARN(node_->get_logger(), "Vélocité non fournie ou incomplète, valeur par défaut 1.0 utilisée.");
    }

    RCLCPP_INFO(node_->get_logger(), "Commande exécutée à t=%.3f", node_->now().seconds());

    for (size_t i = 0; i < msg->name.size(); i++) {
        const std::string & name = msg->name[i];
        float pos = msg->position[i];
        float vel = velocityIncluded ? msg->velocity[i] : 1.0f;

        for (auto &servo : servos_) {
            if (servo->joint_name_ == name) {
                servo->setAngle(pos, vel);
                break;
            }
        }
    }
}


void DynamixelController::publishJointStates()
{
    auto now = node_->now();
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now;
    msg.header.frame_id = "head";

    for (const auto &servo : servos_) {
        msg.name.push_back(servo->joint_name_);

        int32_t ticks = 0;
        float angle = 0.0f;

        if (dxl_wb_.itemRead(servo->id_, "Present_Position", &ticks)) {
            angle = (ticks - servo->neutral_) * servo->rad_per_tick_;
            if (servo->invert_) angle = -angle;
            msg.position.push_back(angle);
            servo->angle_ = angle;  // mise à jour locale
        } else {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Lecture position échouée pour ID %d", servo->id_);
            msg.position.push_back(servo->angle_);  // fallback
        }

        msg.velocity.push_back(0.0);
        msg.effort.push_back(0.0);

        // ✅ TF dynamique
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;

        tf2::Quaternion q;
        if (servo->joint_name_ == "head_pan_joint") {
            tf_msg.header.frame_id = "base_link";
            tf_msg.child_frame_id = "base_pan_link";
            tf_msg.transform.translation.x = 0.045;
            tf_msg.transform.translation.y = 0.0;
            tf_msg.transform.translation.z = 0.35;
            q.setRPY(0.0, 0.0, angle);  // rotation autour de Z
        } else if (servo->joint_name_ == "head_tilt_joint") {
            tf_msg.header.frame_id = "base_pan_link";
            tf_msg.child_frame_id = "head_tilt_link";
            tf_msg.transform.translation.x = 0.0;
            tf_msg.transform.translation.y = 0.0;
            tf_msg.transform.translation.z = 0.0;
            q.setRPY(0.0, angle, 0.0);  // rotation autour de Y
        } else {
            continue;
        }

        tf_msg.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf_msg);
    }

    joint_state_pub_->publish(msg);
}


DynamixelController::~DynamixelController() = default;

