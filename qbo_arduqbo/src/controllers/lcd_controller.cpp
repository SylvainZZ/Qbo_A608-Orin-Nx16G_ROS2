// lcd_controller_ros2.cpp
#include "qbo_arduqbo/controllers/lcd_controller.hpp"
#include <chrono>

using namespace std::chrono_literals;

LcdController::LcdController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("lcd_ctrl", "qbo_arduqbo", options),
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
    // Lecture des paramètres
    get_parameter("topic", topic_);

    uint8_t i2c_state = 0;
    if (driver_->getI2cDevicesState(i2c_state) >= 0) {
        has_lcd_ = i2c_state & 0x08;
        i2c_status_checked_ = true;
        RCLCPP_DEBUG(this->get_logger(), "🧭 I2C Devices State: 0x%02X", i2c_state);
        RCLCPP_DEBUG(this->get_logger(), "    LCD detected: %s", has_lcd_ ? "yes" : "no");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unable to query LCD device state via I2C at startup.");
    }

    lcd_sub_ = this->create_subscription<qbo_msgs::msg::LCD>(
        topic_, 1, std::bind(&LcdController::setLCD, this, std::placeholders::_1));

    updater_.setHardwareID("LCD");
    updater_.add("LCD Status", this, &LcdController::diagnosticCallback);

    RCLCPP_INFO(this->get_logger(), "✅ LCDController initialized with:\n"
                                "       - Command topic: %s",
            topic_.c_str());

    // Envoie un message initial
    driver_->setLCD("Qbo Ready              ");
    RCLCPP_INFO(this->get_logger(), "📟 Initial LCD message sent.");
}

void LcdController::setLCD(const qbo_msgs::msg::LCD::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "LCD command arrived: %s", msg->msg.c_str());
    std::string content = msg->msg.substr(0, 20);
    content += std::string(20 - content.size(), ' ');
    driver_->setLCD(content);
}

void LcdController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status) {
    status.summary(
        (has_lcd_) ? diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        (i2c_status_checked_ ? "LCD Operational" : "LCD I2C check failed at startup"));

    // 🟢 État dynamique
    status.add("LCD Present", has_lcd_ ? "yes" : "no");

    // 🟣 Infos techniques statiques
    status.add("LCD Model", "C2042A");
    status.add("I2C Address", "0x63");

    // 🔴 Message explicite si erreur
    if (!has_lcd_) {
        std::string msg = std::string(!has_lcd_ ? "LCD missing" : "");
        status.message = msg;
    }

}

