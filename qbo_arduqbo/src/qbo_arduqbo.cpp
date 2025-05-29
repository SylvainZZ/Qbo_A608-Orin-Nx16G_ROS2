#include "qbo_arduqbo/qbo_arduqbo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "qbo_arduqbo/controllers/base_controller.hpp"

QboArduqboManager::QboArduqboManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {}

void QboArduqboManager::setup() {
    std::string device_path;
    int address;
    std::string port1, port2;
    int baud1, baud2;
    double timeout1, timeout2;
    bool enable_qboard1, enable_qboard2;
    bool enable_base;


    // Déclaration et lecture des paramètres I2c
    node_->declare_parameter("qboard3_driver.i2c_device", "/dev/i2c-7");
    node_->declare_parameter("qboard3_driver.i2c_address", 20);
    node_->declare_parameter("enable_battery", true);
    node_->declare_parameter("enable_imu", false);
    // Déclaration et lecture des paramètres série
    node_->declare_parameter("qboards.port1", "/dev/ttyUSB0");
    node_->declare_parameter("qboards.port2", "/dev/ttyUSB1");
    node_->declare_parameter("qboards.baud1", 115200);
    node_->declare_parameter("qboards.baud2", 115200);
    node_->declare_parameter("qboards.timeout1", 4.0);
    node_->declare_parameter("qboards.timeout2", 0.05);
    node_->declare_parameter("qboards.enable_qboard1", true);
    node_->declare_parameter("qboards.enable_qboard2", true);

    node_->get_parameter("qboard3_driver.i2c_device", device_path);
    node_->get_parameter("qboard3_driver.i2c_address", address);
    node_->get_parameter("enable_battery", enable_battery_);
    node_->get_parameter("enable_imu", enable_imu_);

    node_->get_parameter("qboards.port1", port1);
    node_->get_parameter("qboards.port2", port2);
    node_->get_parameter("qboards.baud1", baud1);
    node_->get_parameter("qboards.baud2", baud2);
    node_->get_parameter("qboards.timeout1", timeout1);
    node_->get_parameter("qboards.timeout2", timeout2);
    node_->get_parameter("qboards.enable_qboard1", enable_qboard1);
    node_->get_parameter("qboards.enable_qboard2", enable_qboard2);

    i2c_driver_ = std::make_shared<I2CBusDriver>(device_path, static_cast<uint8_t>(address));

    // Création du driver série (même si une seule carte est branchée)
    RCLCPP_INFO(node_->get_logger(), "PORT1: %s (%s)", port1.c_str(), enable_qboard1 ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "PORT2: %s (%s)", port2.c_str(), enable_qboard2 ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "BAUD: %d / %d | TIMEOUT: %.2f / %.2f",
        baud1, baud2, timeout1, timeout2);


    arduino_driver_ = std::make_shared<QboDuinoDriver>(
        port1, baud1,
        port2, baud2,
        timeout1, timeout2
    );

    // ➕ Tu pourras ensuite créer les contrôleurs série (odom, battery, etc.)
    if (enable_qboard1) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo base board communication active");

        // Niveau 2 : Chargement du contrôleur seulement si communication OK
        node_->declare_parameter("enable_base", true);
        node_->get_parameter("enable_base", enable_base);

        if (enable_base) {
            auto base_ctrl = std::make_shared<BaseController>(
                arduino_driver_,
                rclcpp::NodeOptions().append_parameter_override("name", "base")
            );
            controllers_.push_back(base_ctrl);

            RCLCPP_INFO(node_->get_logger(), "✅ Base controller loaded");

        } else {
            RCLCPP_INFO(node_->get_logger(), "⏹️ Base controller disabled by config");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "❌ Base board communication disabled by config");
    }

    if (enable_qboard2) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo head board communication active");
        // Ex : créer LED/mouth controllers ici
    }

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
