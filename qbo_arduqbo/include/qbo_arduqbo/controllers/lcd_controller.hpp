#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <qbo_msgs/msg/lcd.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "qbo_arduqbo/drivers/qboduino_driver.h"


class LcdController : public rclcpp::Node {
public:
    LcdController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Souscriptions
  rclcpp::Subscription<qbo_msgs::msg::LCD>::SharedPtr lcd_sub_;

  std::shared_ptr<QboDuinoDriver> driver_;

  // Callback ROS
  void setLCD(const qbo_msgs::msg::LCD::SharedPtr msg);
  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status);

  // Diagnostic updater
  diagnostic_updater::Updater updater_;

  // Etat interne
  std::string topic_ = "cmd_lcd";
  bool i2c_status_checked_ = false;
  bool has_lcd_ = false;
};
