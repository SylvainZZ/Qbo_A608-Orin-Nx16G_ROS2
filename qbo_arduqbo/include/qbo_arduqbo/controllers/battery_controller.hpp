#pragma once

#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/drivers/i2c_bus_driver.hpp"
#include "qbo_msgs/msg/battery_level.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <deque>
#include <numeric>

class CBatteryController : public rclcpp::Node {
public:
    CBatteryController(
        std::shared_ptr<I2CBusDriver> driver,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status);
    void loadParameters();

    // ROS2
    // std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<qbo_msgs::msg::BatteryLevel>::SharedPtr battery_pub_;
    diagnostic_updater::Updater updater_;

    // Accès bas-niveau I2C
    std::shared_ptr<I2CBusDriver> driver_;

    // État batterie brut
    std::string name_;
    uint8_t level_;
    uint8_t stat_;

    // Estimation runtime
    std::deque<double> voltage_history_;
    double last_estimated_runtime_minutes_;
    double error_battery_level_;
    double warn_battery_level_;
    double capacity_ah_;
    double nominal_voltage_;
    std::string battery_type_;
    double rate_;
};
