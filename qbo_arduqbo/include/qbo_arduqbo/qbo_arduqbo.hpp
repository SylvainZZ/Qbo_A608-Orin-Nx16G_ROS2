#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/drivers/i2c_bus_driver.hpp"
#include "qbo_arduqbo/drivers/qboduino_driver.h"
#include "qbo_arduqbo/controllers/battery_controller.hpp"
// #include "qbo_arduqbo/controllers/imu_controller.hpp" // plus tard

class QboArduqboManager {
public:
    explicit QboArduqboManager(std::shared_ptr<rclcpp::Node> node);
    void setup();
    void run();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<I2CBusDriver> i2c_driver_;
    std::shared_ptr<QboDuinoDriver> arduino_driver_;
    std::vector<std::shared_ptr<rclcpp::Node>> controllers_;  // Pour garder les instances
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

    // Flags
    bool enable_battery_;
    bool enable_imu_;
};
