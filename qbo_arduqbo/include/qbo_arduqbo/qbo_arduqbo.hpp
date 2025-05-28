#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/drivers/i2c_bus_driver.hpp"
#include "qbo_arduqbo/controllers/battery_controller.hpp"
// #include "qbo_arduqbo/controllers/imu_controller.hpp" // plus tard

class QboArduqboManager {
public:
    explicit QboArduqboManager(std::shared_ptr<rclcpp::Node> node);
    void setup();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<I2CBusDriver> i2c_driver_;
    std::vector<std::shared_ptr<void>> controllers_;  // Pour garder les instances

    // Flags
    bool enable_battery_;
    bool enable_imu_;
};
