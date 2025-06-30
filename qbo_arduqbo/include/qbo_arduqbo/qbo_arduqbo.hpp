#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/drivers/i2c_bus_driver.hpp"
#include "qbo_arduqbo/drivers/qboduino_driver.h"
#include "qbo_arduqbo/controllers/battery_controller.hpp"
#include "qbo_arduqbo/controllers/base_controller.hpp"
#include "qbo_arduqbo/controllers/imu_controller.hpp"
#include "qbo_arduqbo/controllers/lcd_controller.hpp"
#include "qbo_arduqbo/controllers/nose_controller.hpp"
#include "qbo_arduqbo/controllers/mouth_controller.hpp"
#include "qbo_arduqbo/controllers/audio_controller.hpp"

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
    bool enable_qboard1_;
    bool enable_qboard2_;
    bool enable_battery_;
    bool enable_imu_base_;
    bool enable_imu_head_;
    bool enable_base_;
    bool enable_lcd_;
    bool enable_nose_;
    bool enable_mouth_;
    bool enable_audio_;

    int qboard1_version_ = -1;
    int qboard2_version_ = -1;

    // Diagnostics
    std::unique_ptr<diagnostic_updater::Updater> updater_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    // ➕ Fonctions utilitaires internes
    void logControllerStatus(const std::string &name, bool enabled, bool loaded);

};
