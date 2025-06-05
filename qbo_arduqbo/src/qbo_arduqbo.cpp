#include "qbo_arduqbo/qbo_arduqbo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"


QboArduqboManager::QboArduqboManager(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {}

// qbo_arduqbo.cpp - refactorisation de la fonction setup()

void QboArduqboManager::setup() {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
    updater_->setHardwareID("qbo_arduqbo System");

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
    declare_and_get_param("enable_battery", enable_battery_, false);
    declare_and_get_param("enable_base", enable_base_, false);
    declare_and_get_param("enable_imu_base", enable_imu_base_, false);
    declare_and_get_param("enable_imu_head", enable_imu_head_, false);
    declare_and_get_param("enable_lcd", enable_lcd_, false);
    declare_and_get_param("enable_nose", enable_nose_, false);
    declare_and_get_param("enable_mouth", enable_mouth_, false);

    std::string port1, port2;
    int baud1, baud2;
    double timeout1, timeout2;
    uint8_t id = 0;
    int board_id = -1, version = -1;

    declare_and_get_param("port1", port1, std::string("/dev/ttyUSB0"));
    declare_and_get_param("port2", port2, std::string("/dev/ttyUSB1"));
    declare_and_get_param("baud1", baud1, 115200);
    declare_and_get_param("baud2", baud2, 115200);
    declare_and_get_param("timeout1", timeout1, 0.05);
    declare_and_get_param("timeout2", timeout2, 0.05);
    declare_and_get_param("enable_qboard1", enable_qboard1_, true);
    declare_and_get_param("enable_qboard2", enable_qboard2_, true);

    RCLCPP_INFO(node_->get_logger(), "PORT1: %s (%s)", port1.c_str(), enable_qboard1_ ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "PORT2: %s (%s)", port2.c_str(), enable_qboard2_ ? "enabled" : "disabled");
    RCLCPP_INFO(node_->get_logger(), "BAUD: %d / %d | TIMEOUT: %.2f / %.2f", baud1, baud2, timeout1, timeout2);

    updater_->add("Controller Status", [this](diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "QBO ArduQBO controllers initialized");
        status.add("QBoard1", enable_qboard1_ ? "Enabled" : "Disabled");
        status.add("QBoard2", enable_qboard2_ ? "Enabled" : "Disabled");
        status.add("QBoard3", enable_battery_ ? "Battery Enabled" : "Disabled");

        status.add("Base", enable_base_ ? "Enabled" : "Disabled");
        status.add("IMU Base", enable_imu_base_ ? "Enabled" : "Disabled");
        status.add("LCD", enable_lcd_ ? "Enabled" : "Disabled");
        status.add("IMU Head", enable_imu_head_ ? "Enabled" : "Disabled");
        status.add("Nose", enable_nose_ ? "Enabled" : "Disabled");
        status.add("Mouth", enable_mouth_ ? "Enabled" : "Disabled");
    });

    diagnostic_timer_ = node_->create_wall_timer(
        std::chrono::seconds(2),
        [this]() { updater_->force_update(); }
    );

    // =====================
    // 🔌 Initialisation drivers
    // =====================
    i2c_driver_ = std::make_shared<I2CBusDriver>(device_path, static_cast<uint8_t>(address));
    arduino_driver_ = std::make_shared<QboDuinoDriver>(port1, baud1, port2, baud2, timeout1, timeout2);

    // =====================
    // 🧩 Chargement des contrôleurs
    // =====================

    // ➕ Qboard 1 (Serial - base)
    if (enable_qboard1_) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo base board communication active");

        // QBoard1 (Base)
        int code1 = arduino_driver_->getVersion("base", board_id, version);
        if (code1 >= 0 && id == 0) {
            qboard1_version_ = version;
            RCLCPP_INFO(node_->get_logger(), "      QBoard1 detected — ID: %d, Version: %d", board_id, version);
        } else {
            RCLCPP_WARN(node_->get_logger(), "❌ Failed to get QBoard1 (base) version");
        }

        bool loaded = false;
        if (enable_base_) {
            auto base_ctrl = std::make_shared<BaseController>(
                arduino_driver_,
                rclcpp::NodeOptions().append_parameter_override("name", "base_ctrl")
            );
            controllers_.push_back(base_ctrl);
            loaded = true;
        }
        logControllerStatus("Base", enable_base_, loaded);

        loaded = false;
        if (enable_imu_base_) {
            auto imu_ctrl = std::make_shared<ImuController>(
                arduino_driver_,
                rclcpp::NodeOptions().append_parameter_override("name", "imu_ctrl")
            );
            controllers_.push_back(imu_ctrl);
            loaded = true;
        }
        logControllerStatus("IMU base", enable_nose_, loaded);

        loaded = false;
        if (enable_lcd_) {
            auto lcd_ctrl = std::make_shared<LcdController>(
                arduino_driver_,
                rclcpp::NodeOptions().append_parameter_override("name", "lcd_ctrl")
            );
            controllers_.push_back(lcd_ctrl);
            loaded = true;
        }
        logControllerStatus("LCD", enable_lcd_, loaded);


    } else {
        RCLCPP_WARN(node_->get_logger(), "❌ Base board communication disabled by config");
    }

    // ➕ Qboard 2 (Serial - head)
    if (enable_qboard2_) {
        RCLCPP_INFO(node_->get_logger(), "✅ Qbo head board communication active");

        // QBoard2 (Head)
        int code2 = arduino_driver_->getVersion("head", board_id, version);
        if (code2 >= 0 && id == 0) {
            qboard2_version_ = version;
            RCLCPP_INFO(node_->get_logger(), "      QBoard2 detected — ID: %d, Version: %d", board_id, version);
        } else {
            RCLCPP_WARN(node_->get_logger(), "❌ Failed to get QBoard2 (head) version");
        }

        bool loaded = false;
        if (enable_nose_) {
            auto nose_ctrl = std::make_shared<NoseController>(
                arduino_driver_,
                rclcpp::NodeOptions().append_parameter_override("name", "nose_ctrl")
            );
            controllers_.push_back(nose_ctrl);
            loaded = true;
        }
        logControllerStatus("Nose", enable_nose_, loaded);

        loaded = false;
        if (enable_mouth_) {
            auto mouth_ctrl = std::make_shared<MouthController>(
                arduino_driver_,
                rclcpp::NodeOptions().append_parameter_override("name", "mouth_ctrl")
            );
            controllers_.push_back(mouth_ctrl);
            loaded = true;
        }
        logControllerStatus("Mouth", enable_mouth_, loaded);

    } else {
        RCLCPP_WARN(node_->get_logger(), "❌ Head board communication disabled by config");
    }

    // ➕ Qboard 3 (I2C - battery)
    bool loaded = false;
    if (enable_battery_) {
        auto battery = std::make_shared<CBatteryController>(
            i2c_driver_,
            rclcpp::NodeOptions().append_parameter_override("name", "battery_ctrl")
        );
        controllers_.push_back(std::static_pointer_cast<rclcpp::Node>(battery));
        loaded = true;
    }
    logControllerStatus("Battery", enable_battery_, loaded);

    // ➕ Qboard 3 (IMU - TODO)
    loaded = false;
    if (enable_imu_head_) {
        loaded = true;
    }
    logControllerStatus("IMU head", enable_imu_head_, loaded);

    updater_->add("Controller Status", [this](diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "QBO ArduQBO controllers initialized");
        status.add("QBoard1", enable_qboard1_ ? "Enabled" : "Disabled");
        status.add("QBoard1 Version", qboard1_version_);
        status.add("QBoard2", enable_qboard2_ ? "Enabled" : "Disabled");
        status.add("QBoard2 Version", qboard2_version_);
        status.add("QBoard3", enable_battery_ ? "Battery Enabled" : "Disabled");

        status.add("Base", enable_base_ ? "Enabled" : "Disabled");
        status.add("IMU Base", enable_imu_base_ ? "Enabled" : "Disabled");
        status.add("LCD", enable_lcd_ ? "Enabled" : "Disabled");
        status.add("IMU Head", enable_imu_head_ ? "Enabled" : "Disabled");
        status.add("Nose", enable_nose_ ? "Enabled" : "Disabled");
        status.add("Mouth", enable_mouth_ ? "Enabled" : "Disabled");
    });

    diagnostic_timer_ = node_->create_wall_timer(
        std::chrono::seconds(2),
        [this]() { updater_->force_update(); }
    );

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

void QboArduqboManager::logControllerStatus(const std::string &name, bool enabled, bool loaded) {
    if (!enabled) {
        RCLCPP_INFO(node_->get_logger(), "⏹️ %s controller disabled by config", name.c_str());
    } else if (loaded) {
        return;
    } else {
        RCLCPP_WARN(node_->get_logger(), "⚠️ %s controller not loaded", name.c_str());
    }
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

