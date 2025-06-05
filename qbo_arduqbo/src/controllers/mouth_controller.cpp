#include "qbo_arduqbo/controllers/mouth_controller.hpp"
#include <rclcpp/rclcpp.hpp>

MouthController::MouthController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("mouth_ctrl", options), driver_(driver)
{
    this->declare_parameter("topic", "cmd_mouth");
    this->declare_parameter("rate", 1.0);
    this->get_parameter("topic", topic_);
    this->get_parameter("rate", rate_);

    mouth_sub_ = this->create_subscription<qbo_msgs::msg::Mouth>(
        topic_, 10,
        std::bind(&MouthController::setMouth, this, std::placeholders::_1));

    test_leds_srv_ = this->create_service<qbo_msgs::srv::TestMouthLeds>(
        this->get_name() + std::string("/test_leds"),
        std::bind(&MouthController::testMouthLedsCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "✅ MouthController initialized");
    RCLCPP_INFO(this->get_logger(), "       Rate: %.2f Hz", rate_);
    RCLCPP_INFO(this->get_logger(), "       Command topic: %s", topic_.c_str());

}

void MouthController::setMouth(const qbo_msgs::msg::Mouth::SharedPtr msg)
{
    if (msg->mouth_image.size() != 20) {
        RCLCPP_ERROR(this->get_logger(), "Mouth message must contain 20 elements");
        return;
    }

    // Convert 20 bools into a 20-bit number
    uint32_t data = 0;
    for (int i = 0; i < 20; ++i) {
        if (msg->mouth_image[i]) {
            data |= (1 << i);
        }
    }

    // Reproduire le découpage Arduino :
    uint8_t b3 = data >> 14;           // top 6 bits
    uint8_t b2 = (data >> 7) & 0x7F;   // middle 7 bits
    uint8_t b1 = data & 0x7F;          // bottom 7 bits

    int code = driver_->setMouth(b1, b2, b3);
    if (code < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to send mouth command");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Sent mouth pattern (b1=%u, b2=%u, b3=%u)", b1, b2, b3);
    }
}

void MouthController::testMouthLedsCallback(
    const std::shared_ptr<qbo_msgs::srv::TestMouthLeds::Request>,
    std::shared_ptr<qbo_msgs::srv::TestMouthLeds::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "🚦 Starting LED test sequence");

    for (int i = 0; i < 20; ++i) {
        std::array<bool, 20> leds = {false};
        leds[i] = true;

        qbo_msgs::msg::Mouth msg;
        msg.header.stamp = this->now();
        msg.mouth_image.insert(msg.mouth_image.begin(), leds.begin(), leds.end());

        // Appelle directement la fonction existante si possible
        this->setMouth(std::make_shared<qbo_msgs::msg::Mouth>(msg));
        rclcpp::sleep_for(std::chrono::milliseconds(250));

    // Extinction de toutes les LED à la fin du test
    msg.mouth_image.resize(20, false);  // 20 LED éteintes
    this->setMouth(std::make_shared<qbo_msgs::msg::Mouth>(msg));
    }

    res->success = true;
    res->message = "All LEDs tested successfully";
}


