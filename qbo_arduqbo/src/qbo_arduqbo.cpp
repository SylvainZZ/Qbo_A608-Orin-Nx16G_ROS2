#include "qbo_arduqbo/qbo_arduqbo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "qbo_arduqbo/controllers/base_controller.hpp"

QboArduqboManager::QboArduqboManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {}

// qbo_arduqbo.cpp - refactorisation de la fonction setup()

void QboArduqboManager::setup() {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Définition helpers de déclaration / récupération
    auto declare_and_get_param = [&](const std::string &name, auto &value, const auto &default_val) {
        node_->declare_parameter(name, default_val);
        node_->get_parameter(name, value);
    };

    // =====================
    // ⚙️  Paramètres globaux
    // =====================
    std::string device_path;
    int address;
    declare_and_get_param("i2c_device", device_path, std::string("/dev/i2c-7"));
    declare_and_get_param("i2c_address", address, 20);
    declare_and_get_param("enable_battery", enable_battery_, true);
    declare_and_get_param("enable_imu", enable_imu_, false);

    std::string port1, port2;
    int baud1, baud2;
    double timeout1, timeout2;
    bool enable_qboard1, enable_qboard2;

    declare_and_get_param("port1", port1, std::string("/dev/ttyUSB0"));
    declare_and_get_param("port2", port2, std::string("/dev/ttyUSB1"));
    declare_and_get_param("baud1", baud1, 115200);
    declare_and_get_param("baud2", baud2, 115200);
    declare_and_get_param("timeout1", timeout1, 0.05);
    declare_and_get_param("timeout2", timeout2, 0.05);
    declare_and_get_param("enable_qboard1", enable_qboard1, true);
    declare_and_get_param("enable_qboard2", enable_qboard2, true);

    RCLCPP_INFO(node_->get_logger(), "PORT1: %s (%s)", port1.c_str(), enable_qboard1 ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "PORT2: %s (%s)", port2.c_str(), enable_qboard2 ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "BAUD: %d / %d | TIMEOUT: %.2f / %.2f", baud1, baud2, timeout1, timeout2);

    // =====================
    // 🔌 Initialisation drivers
    // =====================
    i2c_driver_ = std::make_shared<I2CBusDriver>(device_path, static_cast<uint8_t>(address));
    arduino_driver_ = std::make_shared<QboDuinoDriver>(port1, baud1, port2, baud2, timeout1, timeout2);

    // =====================
    // 🧩 Chargement des contrôleurs
    // =====================

    // ➕ Qboard 1 (Serial - base)
    if (enable_qboard1) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo base board communication active");
        bool enable_base = true;
        declare_and_get_param("enable_base", enable_base, true);

        if (enable_base) {
            auto base_ctrl = std::make_shared<BaseController>(
                arduino_driver_,
                rclcpp::NodeOptions().append_parameter_override("name", "base_ctrl")
            );
            controllers_.push_back(base_ctrl);
            RCLCPP_INFO(node_->get_logger(), "✅ Base controller loaded");
        } else {
            RCLCPP_INFO(node_->get_logger(), "⏹️ Base controller disabled by config");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "❌ Base board communication disabled by config");
    }

    // ➕ Qboard 2 (Serial - head)
    if (enable_qboard2) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo head board communication active");
        // TODO: Implémentation future
    } else {
        RCLCPP_WARN(node_->get_logger(), "❌ Head board communication disabled by config");
    }

    // ➕ Qboard 3 (I2C - battery)
    if (enable_battery_) {
        auto battery = std::make_shared<CBatteryController>(
            i2c_driver_,
            rclcpp::NodeOptions().append_parameter_override("name", "battery_ctrl")
        );
        controllers_.push_back(std::static_pointer_cast<rclcpp::Node>(battery));
        RCLCPP_INFO(node_->get_logger(), "✅ Battery controller enabled");
    }

    // ➕ Qboard 3 (IMU - TODO)
    if (enable_imu_) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ IMU controller not yet implemented");
    }

    // =====================
    // 🚀 Ajout au scheduler
    // =====================
    executor_->add_node(node_);
    for (auto& ctrl : controllers_) {
        executor_->add_node(ctrl);
    }
}


void QboArduqboManager::run() {
    executor_->spin();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("qbo_arduqbo");
    QboArduqboManager manager(node);

    manager.setup();  // configure les contrôleurs
    manager.run();    // lance l'exécuteur avec tous les nœuds enregistrés

    rclcpp::shutdown();
    return 0;
}

