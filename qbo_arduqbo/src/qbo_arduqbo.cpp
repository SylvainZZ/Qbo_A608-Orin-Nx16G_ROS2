#include "qbo_arduqbo/qbo_arduqbo.hpp"
#include "rclcpp/rclcpp.hpp"

QboArduqboManager::QboArduqboManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {}

void QboArduqboManager::setup() {
    std::string device_path;
    int address;

    node_->declare_parameter("controllers.qboard3_driver.i2c_device", "/dev/i2c-7");
    node_->declare_parameter("controllers.qboard3_driver.i2c_address", 20);
    node_->declare_parameter("controllers.enable_battery", true);
    node_->declare_parameter("controllers.enable_imu", false);  // futur

    node_->get_parameter("controllers.qboard3_driver.i2c_device", device_path);
    node_->get_parameter("controllers.qboard3_driver.i2c_address", address);
    node_->get_parameter("controllers.enable_battery", enable_battery_);
    node_->get_parameter("controllers.enable_imu", enable_imu_);

    i2c_driver_ = std::make_shared<I2CBusDriver>(device_path, static_cast<uint8_t>(address));

    if (enable_battery_) {
        auto battery = std::make_shared<CBatteryController>("battery", i2c_driver_, node_);
        controllers_.push_back(battery);
        RCLCPP_INFO(node_->get_logger(), "✅ Battery controller enabled");
    }

    if (enable_imu_) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ IMU controller not yet implemented");
        // auto imu = std::make_shared<CImuController>("imu", i2c_driver_, node_);
        // controllers_.push_back(imu);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("qbo_arduqbo");

    QboArduqboManager manager(node);
    manager.setup();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
