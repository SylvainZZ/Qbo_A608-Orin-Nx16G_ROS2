#pragma once

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "qbo_arduqbo/drivers/qboduino_driver.h"


#include <deque>
#include <numeric>

class CBatteryController : public rclcpp::Node {
public:
    CBatteryController(
        std::shared_ptr<QboDuinoDriver> driver,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status);

    // ROS2
    diagnostic_updater::Updater updater_;

    // Driver série vers la base Q.bo
    std::shared_ptr<QboDuinoDriver> driver_;

    // État batterie brut
    // std::string name_;
    float level_;
    uint8_t stat_;

    double error_battery_level_ = 12.0;
    double warn_battery_level_= 12.0;
    double capacity_ah_= 12.0;
    double nominal_voltage_= 12.0;
    std::string battery_type_ = "Unknown";

};
