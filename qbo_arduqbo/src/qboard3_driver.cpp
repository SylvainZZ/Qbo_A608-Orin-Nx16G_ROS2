#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/controllers/battery_controller.hpp"
#include "qboard3_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("qboard3_driver");

    auto driver = std::make_shared<CQboard3Driver>("/dev/i2c-1", 0x14);

    // Crée le contrôleur batterie (adaptation ROS2 requise dans battery_controller)
    auto controller = std::make_shared<CBatteryController>(
        "battery",
        driver,
        node
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
