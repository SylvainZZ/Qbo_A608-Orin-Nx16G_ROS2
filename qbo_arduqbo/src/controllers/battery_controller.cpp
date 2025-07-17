#include "qbo_arduqbo/controllers/battery_controller.hpp"
#include <chrono>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

CBatteryController::CBatteryController(
    std::shared_ptr<I2CBusDriver> driver,
    const rclcpp::NodeOptions & options)
    : rclcpp::Node("battery_ctrl", "qbo_arduqbo", options),
      updater_(
          this->get_node_base_interface(),
          this->get_node_clock_interface(),
          this->get_node_logging_interface(),
          this->get_node_parameters_interface(),
          this->get_node_timers_interface(),
          this->get_node_topics_interface(),
          1.0),
      driver_(driver),
      level_(0),
      stat_(0),
      last_estimated_runtime_minutes_(0.0)
{
    name_ = this->get_name();

    // Déclaration des paramètres
    // this->declare_parameter("topic", "battery_state");
    // this->declare_parameter("rate", 15.0);
    // this->declare_parameter("error_battery_level", 12.0);
    // this->declare_parameter("warn_battery_level", 12.5);
    // this->declare_parameter("capacity_ah", 10.0);
    // this->declare_parameter("nominal_voltage", 13.0);
    // this->declare_parameter("battery_type", "LiFePo4");

    // Lecture des paramètres
    std::string topic;
    this->get_parameter("topic", topic);
    this->get_parameter("rate", rate_);

    diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10, std::bind(&CBatteryController::diagCallback, this, std::placeholders::_1));

    loadParameters();

    // Diagnostics
    updater_.setHardwareID("Q.board3");
    updater_.add("Battery Status", this, &CBatteryController::diagnosticCallback);

    // Timer pour les diagnostics
    this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_),
        [this]() { updater_.force_update(); });

    // std::cout << "Nom du contrôleur: " << this->get_name() << std::endl;
    RCLCPP_INFO(this->get_logger(), "✅ CBatteryController initialized");
    RCLCPP_INFO(this->get_logger(), "       Rate: %.2f Hz", rate_);
    RCLCPP_INFO(this->get_logger(), "       Command topic: %s", topic.c_str());
    RCLCPP_INFO(this->get_logger(), "       Battery type: %s", battery_type_.c_str());
    RCLCPP_INFO(this->get_logger(), "       Nominal voltage: %.2f V", nominal_voltage_);
    RCLCPP_INFO(this->get_logger(), "       Capacity: %.2f Ah", capacity_ah_);
    RCLCPP_INFO(this->get_logger(), "       Warning level: %.2f V", warn_battery_level_);
    RCLCPP_INFO(this->get_logger(), "       Error level: %.2f V", error_battery_level_);
}

void CBatteryController::loadParameters()
{
    this->get_parameter("error_battery_level", error_battery_level_);
    this->get_parameter("warn_battery_level", warn_battery_level_);
    this->get_parameter("capacity_ah", capacity_ah_);
    this->get_parameter("nominal_voltage", nominal_voltage_);
    this->get_parameter("battery_type", battery_type_);
}

std::string formatDouble(double value, int precision = 2)
{
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;
    return out.str();
}

void CBatteryController::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status)
{
    uint8_t buffer[2];
    if (!driver_->readBytes(buffer, 2)) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No communication");
        return;
    }

    stat_ = buffer[0];
    level_ = buffer[1];
    double voltage = level_ / 10.0;

    voltage_history_.push_back(voltage);
    if (voltage_history_.size() > 10)
        voltage_history_.pop_front();

    // Estimation runtime
    static int runtime_publish_counter = 0;
    runtime_publish_counter++;
    if (runtime_publish_counter >= 20) {
        runtime_publish_counter = 0;

        // Hypothèse : on a bien reçu une valeur valide
        if (A608_power_w_ > 0.0) {
            fixed_extra_power_w = 2.5; // ← à ajuster selon tes mesures

            total_power_w = A608_power_w_ + fixed_extra_power_w;

            // On suppose que le voltage moyen = tension mesurée via voltage_history_
            double smoothed_voltage = std::accumulate(voltage_history_.begin(), voltage_history_.end(), 0.0) / voltage_history_.size();

            double estimated_current_draw = total_power_w / smoothed_voltage;

            double estimated_runtime_minutes = (capacity_ah_ / estimated_current_draw) * 60.0;

            if (std::isfinite(estimated_runtime_minutes)) {
                if (last_estimated_runtime_minutes_ < 0.0 ||
                    std::abs(estimated_runtime_minutes - last_estimated_runtime_minutes_) > 10.0) {
                    last_estimated_runtime_minutes_ = estimated_runtime_minutes;
                    RCLCPP_INFO(this->get_logger(), "Updated runtime to %.1f minutes", estimated_runtime_minutes);
                }
            }
        }
    }

    status.add("Voltage (V)", formatDouble(voltage));
    status.add("_Type", battery_type_);
    status.add("_Nominal Voltage (V)", formatDouble(nominal_voltage_));
    status.add("_Capacity (Ah)", formatDouble(capacity_ah_));
    status.add("Status", std::to_string(stat_));
    status.add("_I2C_Adress", "0x14");

    uint8_t charge_mode = (stat_ >> 3) & 0x07;  // bits 5-3
    bool ext_power = (stat_ >> 2) & 0x01;       // bit 2
    bool pc_on     = (stat_ >> 1) & 0x01;       // bit 1
    bool boards_on = stat_ & 0x01;              // bit 0

    status.add("Charge Mode", std::to_string(charge_mode));
    status.add("External Power", ext_power ? "Yes" : "No");
    status.add("PC On", pc_on ? "Yes" : "No");
    status.add("Boards On", boards_on ? "Yes" : "No");

    std::string charge_desc;
    switch (charge_mode) {
        case 1:
            charge_desc = "Charging (constant current)";
            break;
        case 2:
            charge_desc = "Charging (constant voltage)";
            break;
        case 3:
            charge_desc = "Fully charged";
            break;
        case 4:
            charge_desc = "Battery operation";
            break;
        default:
            charge_desc = "Unknown/invalid charge mode";
            break;
    }
    status.add("Charge Mode Description", charge_desc);

    if (last_estimated_runtime_minutes_ > 0.0) {
        status.add("Estimated Runtime (min)", formatDouble(last_estimated_runtime_minutes_));
        status.add("Estimated Power (W)", formatDouble(total_power_w));
    }
    status.add("Estimated Extras W (W)", formatDouble(fixed_extra_power_w));

    // Niveau batterie
    if (voltage < error_battery_level_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Empty battery");
    } else if (voltage <= warn_battery_level_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low battery");
    } else {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery OK");
    }
}

void CBatteryController::diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    for (const auto& status : msg->status) {
        if (status.name.find("A608 Power") != std::string::npos) {
            for (const auto& value : status.values) {
                if (value.key == "VDD_IN W") {
                    A608_power_w_ = std::stod(value.value);
                    return;  // dès qu’on l’a trouvé, inutile de continuer
                }
            }
        }
    }
}
