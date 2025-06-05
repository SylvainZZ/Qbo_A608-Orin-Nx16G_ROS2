// lcd_controller_ros2.cpp
#include "qbo_arduqbo/controllers/lcd_controller.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

LcdController::LcdController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("lcd_ctrl", options),
  driver_(driver),
  updater_(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface(),
    1.0)
{
    this->declare_parameter("topic", "cmd_lcd");
    this->declare_parameter("rate", 1.0);

    std::string topic;
    this->get_parameter("topic", topic);
    this->get_parameter("rate", rate_);

    uint8_t i2c_state = 0;
    if (driver_->getI2cDevicesState(i2c_state) >= 0) {
        has_lcd_ = i2c_state & 0x08;
        i2c_status_checked_ = true;
        RCLCPP_DEBUG(this->get_logger(), "🧭 I2C Devices State: 0x%02X", i2c_state);
        RCLCPP_DEBUG(this->get_logger(), "    LCD detected: %s", has_lcd_ ? "yes" : "no");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unable to query LCD device state via I2C at startup.");
    }

    lcd_sub_ = this->create_subscription<std_msgs::msg::String>(
        topic, 1, std::bind(&LcdController::setLCD, this, std::placeholders::_1));

    diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10, std::bind(&LcdController::diagCallback, this, std::placeholders::_1));

    updater_.setHardwareID("LCD");
    updater_.add("LCD Status", this, &LcdController::diagnosticCallback);

    // Timer de mise à jour
    this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      [this]() { updater_.force_update(); });

    display_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(5.0 / rate_),
        std::bind(&LcdController::updateLCD, this));

    RCLCPP_INFO(this->get_logger(), "✅ LCDController initialized");
    RCLCPP_INFO(this->get_logger(), "       Rate: %.2f Hz", rate_);
    RCLCPP_INFO(this->get_logger(), "       Command topic: %s", topic.c_str());

    display_lines_[0] = "Hostname: ???";
    // display_lines_[1] = "IP: ???";
    // display_lines_[2] = "Batt: ???";
    // display_lines_[3] = "Temp: ???";

    // Envoie un message initial
    driver_->setLCD("Qbo Ready              ");
}

void LcdController::setLCD(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "LCD command arrived: %s", msg->data.c_str());
    std::string content = msg->data;
    content += std::string(20 - content.size(), ' ');
    driver_->setLCD(content);
}

void LcdController::diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    for (const auto &status : msg->status)
    {
        if (status.name.find("Network") != std::string::npos) {
            for (const auto &v : status.values) {
                if (v.key == "Hostname") hostname_ = "Host: " + v.value;
                if (v.key == "IP Address") ip_address_ = "IP: " + v.value;
            }
        } else if (status.name.find("Battery Status") != std::string::npos) {
            for (const auto &v : status.values) {
                if (v.key == "Voltage (V)") display_lines_[2] = "Batt: " + v.value + "V";
            }
        } else if (status.name.find("Temp") != std::string::npos) {
            for (const auto &v : status.values) {
                if (v.key == "CPU °C") display_lines_[3] = "Temp: " + v.value + "C";
            }
        }
    }
}


void LcdController::updateLCD()
{
    std::string line0 = show_hostname_ ? hostname_ : ip_address_;
    show_hostname_ = !show_hostname_;  // alterner à chaque appel (5s)

    std::string lines[4];
    lines[0] = line0 + std::string(20 - line0.size(), ' ');
    for (int i = 1; i < 4; ++i) {
        lines[i] = display_lines_[i] + std::string(20 - display_lines_[i].size(), ' ');
    }

    driver_->setLCD(lines[0]);
    // driver_->setLCD("1" + lines[1]);
    // driver_->setLCD("22" + lines[2]);
    // driver_->setLCD("333" + lines[3]);
}

void LcdController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status) {
    status.summary(
        (has_lcd_) ? diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        (i2c_status_checked_ ? "LCD status read" : "LCD I2C check failed at startup"));

    // 🟢 État dynamique
    status.add("LCD Present", has_lcd_ ? "yes" : "no");

    // 🟣 Infos techniques statiques
    status.add("LCD Model", "A trouver");

    // 🔴 Message explicite si erreur
    if (!has_lcd_) {
        std::string msg = std::string(!has_lcd_ ? "LCD missing" : "");
        status.message = msg;
    }

}

