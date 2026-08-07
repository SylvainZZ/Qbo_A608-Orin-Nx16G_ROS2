#include "qbo_arduqbo/controllers/mouth_controller.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {
bool last_mouth_test_ok = true;
}

MouthController::MouthController(std::shared_ptr<QboDuinoDriver> driver, const rclcpp::NodeOptions & options)
: Node("mouth_ctrl", "qbo_arduqbo", options), driver_(driver), updater_(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_parameters_interface(),
        this->get_node_timers_interface(),
        this->get_node_topics_interface(),
        1.0)
{
    // Lecture des paramètres
    this->get_parameter("topic", topic_);
    this->get_parameter("rate", rate_);

    mouth_sub_ = this->create_subscription<qbo_msgs::msg::Mouth>(
        topic_, 10,
        std::bind(&MouthController::setMouth, this, std::placeholders::_1));

    test_leds_srv_ = this->create_service<qbo_msgs::srv::TestLeds>(
        this->get_name() + std::string("/test_leds"),
        std::bind(&MouthController::testMouthLedsCallback, this, std::placeholders::_1, std::placeholders::_2)
    );
    // 🔍 Diagnostic setup
    updater_.setHardwareID("Qboard_5");
    updater_.add("Mouth Status", this, &MouthController::produceDiagnostics);

    RCLCPP_INFO(this->get_logger(), "✅ MouthController initialized with:\n"
                                "       - Rate: %.2f Hz\n"
                                "       - Command topic: %s",
            rate_, topic_.c_str());
}

void MouthController::setMouth(const qbo_msgs::msg::Mouth::SharedPtr msg)
{
    if (msg->mouth_image.size() != 20) {
        RCLCPP_ERROR(this->get_logger(), "Mouth message must contain 20 elements");
        return;
    }

    // ✅ CORRECTION : L'Arduino setImage() lit les bits 23 à 4
    // Il reconstruit : data = (b1_param << 16) | (b2_param << 8) | b3_param
    // Où b1_param reçoit notre b3, b2_param notre b2, b3_param notre b1
    // Donc il faut placer nos données dans les bits 23-4 du mot final

    uint32_t data = 0;
    for (int i = 0; i < 20; ++i) {
        if (msg->mouth_image[i]) {
            data |= (1UL << (23 - i));  // bits 23 à 4 (pas 19 à 0 !)
        }
    }

    // Découpage pour correspondre à la reconstruction Arduino
    uint8_t b1 = data & 0xFF;           // bits 7-0
    uint8_t b2 = (data >> 8) & 0xFF;    // bits 15-8
    uint8_t b3 = (data >> 16) & 0xFF;   // bits 23-16

    int code = driver_->setMouth(b3, b2, b1);
    if (code < 0) {
        RCLCPP_ERROR(this->get_logger(), "Unable to send mouth command");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Sent mouth pattern (data=0x%06X, b3=%u, b2=%u, b1=%u)", data, b3, b2, b1);
    }
}

void MouthController::testMouthLedsCallback(
    const std::shared_ptr<qbo_msgs::srv::TestLeds::Request>,
    std::shared_ptr<qbo_msgs::srv::TestLeds::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "🚦 Starting LED test sequence");

    int code = driver_->testMouth();
    if (code < 0) {
        last_mouth_test_ok = false;
        RCLCPP_WARN(this->get_logger(), "⚠️ testMouth sent, but no response received (as expected)");
        res->success = true;  // ✅ car l'action a été envoyée avec succès
        res->message = "Mouth test sent, no response expected";
    } else {
        last_mouth_test_ok = true;
        res->success = true;
        res->message = "Mouth test executed successfully";
    }
}

void MouthController::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    // Ici, on pourrait ajouter des vérifications spécifiques à la bouche, par exemple :
    // - Dernier pattern envoyé
    // - Réponses du driver
    // - État de la communication

    // Pour l'instant, on se contente d'indiquer que le contrôleur est actif

    if (!last_mouth_test_ok) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Mouth controller test failed");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mouth controller operational");
    }
}

