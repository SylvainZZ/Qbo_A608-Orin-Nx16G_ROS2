#include "qbo_arduqbo/controllers/nose_controller.hpp"
#include <rclcpp/rclcpp.hpp>

NoseController::NoseController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("nose_ctrl", options), driver_(driver)
{
    this->declare_parameter("topic", "cmd_nose");
    this->declare_parameter("rate", 1.0);
    this->get_parameter("topic", topic_);
    this->get_parameter("rate", rate_);

    nose_sub_ = this->create_subscription<qbo_msgs::msg::Nose>(
        topic_, 10,
        std::bind(&NoseController::setNose, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "✅ NoseController initialized");
    RCLCPP_INFO(this->get_logger(), "       Rate: %.2f Hz", rate_);
    RCLCPP_INFO(this->get_logger(), "       Command topic: %s", topic_.c_str());

}

void NoseController::setNose(const qbo_msgs::msg::Nose::SharedPtr msg)
{
    if (msg->color > 7) {
        RCLCPP_WARN(this->get_logger(), "Nose color %d is invalid. Must be between 0 and 7.", msg->color);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Nose command received: %d", msg->color);
    int code = driver_->setNose(msg->color);
    if (code < 0)
        RCLCPP_ERROR(this->get_logger(), "Unable to send nose color to the Arduino.");
    else
        RCLCPP_DEBUG(this->get_logger(), "Nose color %d sent to Arduino.", msg->color);
}
