#pragma once

#include "rclcpp/rclcpp.hpp"
#include "qboard3_driver.hpp"
#include "qbo_msgs/msg/battery_level.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include <deque>
#include <numeric>

class CBatteryController {
public:
    CBatteryController(
        const std::string &name,
        std::shared_ptr<CQboard3Driver> driver,
        std::shared_ptr<rclcpp::Node> node);

private:
    void timerCallback();

    // ROS2
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<qbo_msgs::msg::BatteryLevel>::SharedPtr battery_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Batterie
    std::shared_ptr<CQboard3Driver> driver_;
    uint8_t level_;
    uint8_t stat_;
    std::string name_;

    // Estimation
    std::deque<double> voltage_history_;
    double last_estimated_runtime_minutes_;
    double error_battery_level_;
    double warn_battery_level_;
    double capacity_ah_;
    double nominal_voltage_;
    std::string battery_type_;
    double rate_;

    void loadParameters();
};
