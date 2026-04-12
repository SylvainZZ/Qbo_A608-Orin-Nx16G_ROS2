#include "qbo_arduqbo/controllers/battery_controller.hpp"
#include <chrono>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

CBatteryController::CBatteryController(
    std::shared_ptr<QboDuinoDriver> driver,
    const rclcpp::NodeOptions & options)
    : rclcpp::Node("battery_ctrl", "qbo_arduqbo", options),
      updater_(
          this->get_node_base_interface(),
          this->get_node_clock_interface(),
          this->get_node_logging_interface(),
          this->get_node_parameters_interface(),
          this->get_node_timers_interface(),
          this->get_node_topics_interface(),
          1.0),
      driver_(driver),
      level_(0),
      stat_(0)
{
    // Reading parameters
    get_parameter("error_battery_level", error_battery_level_);
    get_parameter("warn_battery_level", warn_battery_level_);
    get_parameter("capacity_ah", capacity_ah_);
    get_parameter("nominal_voltage", nominal_voltage_);
    get_parameter("battery_type", battery_type_);

    // Diagnostics
    updater_.setHardwareID("Qboard_3");
    updater_.add("Battery Status", this, &CBatteryController::diagnosticCallback);

    RCLCPP_INFO(this->get_logger(), "✅ CBatteryController initialized with:\n"
                                "       - Battery type: %s\n"
                                "       - Nominal voltage: %.2f V\n"
                                "       - Capacity: %.2f Ah\n"
                                "       - Warning level: %.2f V\n"
                                "       - Error level: %.2f V",
            battery_type_.c_str(), nominal_voltage_, capacity_ah_,
            warn_battery_level_, error_battery_level_);
}

std::string formatDouble(double value, int precision = 2)
{
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;
    return out.str();
}

void CBatteryController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status)
{
    int code=driver_->getBattery(level_,stat_);
    if (code<0) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Battery Controller: No communication with the Qboard_3");
        return;
    }

    double voltage = level_ / 10.0;

    status.add("Voltage", formatDouble(voltage)); // in volts
    status.add("Type", battery_type_); // e.g. "Li-ion", "NiMH", or other
    status.add("Nominal Voltage", formatDouble(nominal_voltage_)); // in volts
    status.add("Capacity", formatDouble(capacity_ah_)); // in Ah
    status.add("Status", std::to_string(stat_));

    uint8_t charge_mode = (stat_ >> 3) & 0x07;  // bits 5-3
    bool ext_power = (stat_ >> 2) & 0x01;       // bit 2
    bool pc_on     = (stat_ >> 1) & 0x01;       // bit 1
    bool boards_on = stat_ & 0x01;              // bit 0

    status.add("Charge Mode", std::to_string(charge_mode));
    status.add("External Power", ext_power ? "Yes" : "No");
    status.add("Power PC", pc_on ? "Yes" : "No");
    status.add("Power Qboard", boards_on ? "Yes" : "No");

    std::string charge_desc;
    switch (charge_mode) {
        case 1:
            charge_desc = "Charging (constant current)";
            break;
        case 2:
            charge_desc = "Charging (constant voltage)";
            break;
        case 3:
            charge_desc = "Fully charged";
            break;
        case 4:
            charge_desc = "Battery operation";
            break;
        default:
            charge_desc = "Unknown/invalid charge mode";
            break;
    }
    status.add("Charge Mode Description", charge_desc);

    // Niveau batterie
    if (!pc_on) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Battery Controller: Power PC Off");
    } else if (voltage < error_battery_level_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Battery Controller: Empty battery");
    } else if (voltage <= warn_battery_level_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery Controller: Low battery");
    } else {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery Controller: Battery OK");
    }
}
