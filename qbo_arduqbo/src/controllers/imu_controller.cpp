// imu_controller_ros2.cpp
#include "qbo_arduqbo/controllers/imu_controller.hpp"

// sensitivity can be 0.07 or 0.0175 or 0.00875 and conversion degrees to radians(250*pi/(180*32768))
static constexpr double GYRO_MEASUREMENT_SCALE = 250.0 * M_PI / (180.0 * 32768.0);

// accelerometer is in +- 2g mode, so sensibility is 18 mg/digit.
static constexpr double ACC_MEASUREMENT_SCALE = 9.81 / 16384.0;  // ±2g mode

ImuController::ImuController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("imu_ctrl", options),
  updater_(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_parameters_interface(),
    this->get_node_timers_interface(),
    this->get_node_topics_interface(),
    1.0),
    driver_(driver),
    is_calibrated_(false),
    is_calibrating_(false)
{
    uint8_t i2c_state = 0;

    // Paramètres
    this->declare_parameter("topic", "imu_state");
    this->declare_parameter("rate", 1.0);
    std::string topic;
    this->get_parameter("topic", topic);
    this->get_parameter("rate", rate_);
    this->declare_parameter(this->get_name() + std::string(".is_calibrated"), false);
    this->declare_parameter(this->get_name() + std::string( ".last_calibrated"), 0.0);

    if (driver_->getI2cDevicesState(i2c_state) >= 0) {
        has_gyro_ = i2c_state & 0x02;
        has_accel_ = i2c_state & 0x04;
        i2c_status_checked_ = true;
        RCLCPP_DEBUG(this->get_logger(), "🧭 I2C Devices State: 0x%02X", i2c_state);
        RCLCPP_DEBUG(this->get_logger(), "    Gyro detected: %s", has_gyro_ ? "yes" : "no");
        RCLCPP_DEBUG(this->get_logger(), "    Accel detected: %s", has_accel_ ? "yes" : "no");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Unable to query IMU device state via I2C at startup.");
    }

    // Publishers
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic + "/data", 10);
    imu_calibrated_pub_ = this->create_publisher<std_msgs::msg::Bool>(topic + "/is_calibrated", 10);

    // Service
    calibrate_service_ = this->create_service<qbo_msgs::srv::CalibrateIMU>(
        topic + "/calibrate",
        std::bind(&ImuController::calibrateService, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate_)),
        std::bind(&ImuController::timerCallback, this)
    );

    updater_.setHardwareID("Q.Board4");
    updater_.add("IMU Status", this, &ImuController::diagnosticCallback);

    // Timer de mise à jour
    this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      [this]() { updater_.force_update(); });

    // Init IMU message
    imu_msg_.header.frame_id = "base_link";
    imu_msg_.orientation_covariance = { -1.0, 0, 0, 0, -1.0, 0, 0, 0, -1.0 };
    imu_msg_.angular_velocity_covariance = { 0.008, 0, 0, 0, 0.008, 0, 0, 0, 0.008 };
    imu_msg_.linear_acceleration_covariance = { 0.002, 0, 0, 0, 0.002, 0, 0, 0, 0.002 };

    imu_calibrated_.data = false;

    RCLCPP_INFO(this->get_logger(), "✅ ImuController initialized");
    RCLCPP_INFO(this->get_logger(), "       Rate: %.2f Hz", rate_);
    RCLCPP_INFO(this->get_logger(), "       Command topic: %s", topic.c_str());
    RCLCPP_INFO(this->get_logger(), "       IMU calibrated: %s", is_calibrated_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "       Last calibration timestamp: %.0f", last_calibrated_);
}

void ImuController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status) {
    status.summary(
        (has_gyro_ && has_accel_) ? diagnostic_msgs::msg::DiagnosticStatus::OK :
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        (i2c_status_checked_ ? "IMU status read" : "IMU I2C check failed at startup"));

    // 🟢 État dynamique
    status.add("Gyroscope Present", has_gyro_ ? "yes" : "no");
    status.add("Accelerometer Present", has_accel_ ? "yes" : "no");

    // 🟣 Infos techniques statiques
    status.add("_Accelerometer Model", "LIS35DE");
    status.add("_I2C AdressLIS35DE", "0x1C");
    status.add("_Gyroscope Model", "L3G400D");
    status.add("_I2C AdressL3G400D", "0x69");

    // 🔴 Message explicite si erreur
    if (!has_gyro_ || !has_accel_) {
        std::string msg = std::string(!has_gyro_ ? "Gyroscope missing" : "") +
                          std::string((!has_gyro_ && !has_accel_) ? " & " : "") +
                          std::string(!has_accel_ ? "Accelerometer missing" : "");
        status.message = msg;
    }
}

void ImuController::timerCallback()
{
    if (is_calibrating_) {
        RCLCPP_DEBUG(get_logger(), "IMU read skipped: calibration in progress");
        return;  // on ne lit pas pendant la calibration
    }

    int16_t gx, gy, gz, ax, ay, az;
    int code = driver_->getIMU(gx, gy, gz, ax, ay, az);

    if (code < 0) {
        RCLCPP_ERROR(get_logger(), "Unable to get IMU data from the base controller board");
        has_gyro_ = false;
        has_accel_= false;
        return;
    }

    RCLCPP_DEBUG(get_logger(), "IMU raw: gx=%d, gy=%d, gz=%d, ax=%d, ay=%d, az=%d", gx, gy, gz, ax, ay, az);

    // Conversion rad/s
    imu_msg_.angular_velocity.x = static_cast<float>(gx) * GYRO_MEASUREMENT_SCALE;
    imu_msg_.angular_velocity.y = static_cast<float>(gy) * GYRO_MEASUREMENT_SCALE;
    imu_msg_.angular_velocity.z = static_cast<float>(gz) * GYRO_MEASUREMENT_SCALE;

    // m/s²
    imu_msg_.linear_acceleration.x = static_cast<float>(ax) * ACC_MEASUREMENT_SCALE;
    imu_msg_.linear_acceleration.y = static_cast<float>(ay) * ACC_MEASUREMENT_SCALE;
    imu_msg_.linear_acceleration.z = static_cast<float>(az) * ACC_MEASUREMENT_SCALE;

    imu_msg_.header.stamp = this->now();

    imu_pub_->publish(imu_msg_);
    imu_calibrated_pub_->publish(imu_calibrated_);

    has_gyro_ = true;
    has_accel_= true;
}

bool ImuController::calibrateService(
    const std::shared_ptr<qbo_msgs::srv::CalibrateIMU::Request>,
    std::shared_ptr<qbo_msgs::srv::CalibrateIMU::Response> res)
{
    is_calibrating_ = true;
    // timer_->cancel();  // stop the timer
    RCLCPP_INFO(this->get_logger(), "📡 Starting IMU calibration...");

    uint8_t result = 0;
    int code = driver_->calibrateIMU(result);

    // Attendre la fin de la calibration (attente active 3s max)
    int wait_ms = 0;
    while (result == 0 && wait_ms < 3000) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        code = driver_->calibrateIMU(result);
        wait_ms += 100;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (code < 0 || result == 0) {
        RCLCPP_ERROR(this->get_logger(), "❌ IMU calibration failed (code: %d)", code);
        res->success = false;
        res->message = "IMU calibration failed.";
        is_calibrated_ = false;
        is_calibrating_ = false;
        timer_->reset();   // restart it
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "✅ IMU calibration successful");
    res->success = true;
    res->message = "IMU calibration successful.";
    is_calibrated_ = true;
    is_calibrating_ = false;
    // timer_->reset();   // restart it
    this->set_parameter(rclcpp::Parameter(this->get_name() + std::string(".is_calibrated"), true));
    this->set_parameter(rclcpp::Parameter(this->get_name() + std::string(".last_calibrated"), this->now().seconds()));

    return true;
}


