#include "qbo_arduqbo/controllers/battery_controller.hpp"

#include <chrono>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

CBatteryController::CBatteryController(
    const std::string &name,
    std::shared_ptr<CQboard3Driver> driver,
    std::shared_ptr<rclcpp::Node> node)
    : name_(name), driver_(driver), node_(node), level_(0), stat_(0)
{
    node_->declare_parameter("controllers." + name + ".topic", "battery_state");
    node_->declare_parameter("controllers." + name + ".rate", 15.0);
    node_->declare_parameter("controllers." + name + ".error_battery_level", 12.0);
    node_->declare_parameter("controllers." + name + ".warn_battery_level", 12.5);
    node_->declare_parameter("controllers." + name + ".capacity_ah", 10.0);
    node_->declare_parameter("controllers." + name + ".nominal_voltage", 13.0);
    node_->declare_parameter("controllers." + name + ".battery_type", "LiFePo4");

    std::string topic;
    node_->get_parameter("controllers." + name + ".topic", topic);
    node_->get_parameter("controllers." + name + ".rate", rate_);

    battery_pub_ = node_->create_publisher<qbo_msgs::msg::BatteryLevel>(topic, 1);
    diagnostic_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);

    loadParameters();

    timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_),
        std::bind(&CBatteryController::timerCallback, this));
}

void CBatteryController::loadParameters()
{
    node_->get_parameter("controllers." + name_ + ".error_battery_level", error_battery_level_);
    node_->get_parameter("controllers." + name_ + ".warn_battery_level", warn_battery_level_);
    node_->get_parameter("controllers." + name_ + ".capacity_ah", capacity_ah_);
    node_->get_parameter("controllers." + name_ + ".nominal_voltage", nominal_voltage_);
    node_->get_parameter("controllers." + name_ + ".battery_type", battery_type_);
}

std::string formatDouble(double value, int precision = 2)
{
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << value;
    return out.str();
}

void CBatteryController::timerCallback()
{
    diagnostic_msgs::msg::DiagnosticArray dia_array;
    dia_array.header.stamp = node_->now();
    dia_array.header.frame_id = "base_link";

    diagnostic_msgs::msg::DiagnosticStatus battery_status;
    battery_status.name = "Battery Level";
    battery_status.hardware_id = "Battery";

    diagnostic_msgs::msg::DiagnosticStatus qboard3_status;
    qboard3_status.name = "Battery Management";
    qboard3_status.hardware_id = "Qboard_3";

    diagnostic_msgs::msg::DiagnosticStatus runtime_status;
    runtime_status.name = "Estimated Runtime";
    runtime_status.hardware_id = "Battery";

    static int runtime_publish_counter = 0;
    runtime_publish_counter++;

    if (!driver_->getBattery(level_, stat_)) {
        battery_status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        battery_status.message = "No communication";
        qboard3_status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        qboard3_status.message = "No communication";
    }

    double voltage = level_ / 10.0;

    voltage_history_.push_back(voltage);
    if (voltage_history_.size() > 10)
        voltage_history_.pop_front();

    // Estimation runtime toutes les 20 itérations
    if (runtime_publish_counter >= 20) {
        runtime_publish_counter = 0;

        if (!voltage_history_.empty()) {
            double smoothed_voltage = std::accumulate(voltage_history_.begin(), voltage_history_.end(), 0.0) / voltage_history_.size();
            double estimated_current_draw = (14.3 - smoothed_voltage) * 2.0;

            if (estimated_current_draw > 0.0) {
                double estimated_runtime_minutes = (capacity_ah_ / estimated_current_draw) * 60.0;

                if (std::isfinite(estimated_runtime_minutes)) {
                    if (last_estimated_runtime_minutes_ < 0.0 ||
                        std::abs(estimated_runtime_minutes - last_estimated_runtime_minutes_) > 10.0) {
                        last_estimated_runtime_minutes_ = estimated_runtime_minutes;
                        RCLCPP_INFO(node_->get_logger(), "Updated runtime to %.1f minutes", estimated_runtime_minutes);
                    }
                }
            }
        }
    }

    // Diagnostic: runtime
    if (last_estimated_runtime_minutes_ > 0.0) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "Estimated Runtime (min)";
        kv.value = formatDouble(last_estimated_runtime_minutes_);
        runtime_status.values.push_back(kv);

        runtime_status.level = (last_estimated_runtime_minutes_ < 30.0)
            ? diagnostic_msgs::msg::DiagnosticStatus::WARN
            : diagnostic_msgs::msg::DiagnosticStatus::OK;
        runtime_status.message = (last_estimated_runtime_minutes_ < 30.0)
            ? "Low Estimated Runtime" : "Runtime Sufficient";
    } else {
        runtime_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        runtime_status.message = (stat_ == 31) ? "Runtime In Pause" : "Calcul Runtime In Progress";
    }

    // Diagnostic: Battery
    battery_status.values.push_back({"level(V)", formatDouble(voltage)});
    battery_status.values.push_back({"type", battery_type_});
    battery_status.values.push_back({"voltage(V)", formatDouble(nominal_voltage_)});
    battery_status.values.push_back({"capacity(Ah)", formatDouble(capacity_ah_)});
    qboard3_status.values.push_back({"status", std::to_string(stat_)});

    if (stat_ == 31) {
        battery_status.message = "Battery fully charged";
        qboard3_status.message = "External Power present";
    } else if (stat_ == 15) {
        battery_status.message = "Battery charging at constant current";
        qboard3_status.message = "External Power present";
    } else if (stat_ == 23) {
        battery_status.message = "Battery charging at constant voltage";
        qboard3_status.message = "External Power present";
    } else if (stat_ == 35) {
        battery_status.message = "Battery operation";
        qboard3_status.message = "No External Power";
    }

    if (voltage < error_battery_level_) {
        battery_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        battery_status.message = "Empty battery";
    } else if (voltage <= warn_battery_level_) {
        battery_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        battery_status.message = "Low Battery";
    } else {
        battery_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        battery_status.message = "Battery operation";
    }

    dia_array.status.push_back(battery_status);
    dia_array.status.push_back(qboard3_status);
    dia_array.status.push_back(runtime_status);

    diagnostic_pub_->publish(dia_array);

    // Publication du BatteryLevel
    auto msg = qbo_msgs::msg::BatteryLevel();
    msg.header.stamp = node_->now();
    msg.level = voltage;
    msg.stat = stat_;
    battery_pub_->publish(msg);
}

