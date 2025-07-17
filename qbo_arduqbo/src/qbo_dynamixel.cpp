#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/controllers/dynamixel_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("qbo_dynamixel", options);
    RCLCPP_INFO(node->get_logger(), "🎬 Démarrage du noeud qbo_dynamixel");

    try
    {

        std::string usb_port = "";
        int baud_rate = -1;
        double protocol_version = -1.0;
        
        // Récupération des valeurs
        node->get_parameter("dynamixel.usb_port", usb_port);
        node->get_parameter("dynamixel.baud_rate", baud_rate);
        node->get_parameter("dynamixel.protocol_version", protocol_version);

        // Vérifications simples
        if (usb_port.empty()) {
            RCLCPP_FATAL(node->get_logger(), "❌ Port USB non défini (clé : dynamixel.port)");
            return 1;
        }
        if (baud_rate <= 0) {
            RCLCPP_FATAL(node->get_logger(), "❌ Baudrate invalide : %d", baud_rate);
            return 1;
        }
        if (protocol_version != 1.0 && protocol_version != 2.0) {
            RCLCPP_FATAL(node->get_logger(), "❌ Version du protocole invalide : %.1f (attendu : 1.0 ou 2.0)", protocol_version);
            return 1;
        }

        std::vector<std::string> motor_keys;
        node->get_parameter("dynamixel.motor_keys", motor_keys);

        // ✅ Vérification présence de config
        if (motor_keys.empty()) {
            RCLCPP_FATAL(node->get_logger(),
                "❌ Aucun moteur n'est défini. Vérifie le fichier YAML (clé : dynamixel.motor_keys).");
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "✅ Configuration initiale validée, lancement du contrôleur...");
        auto controller = std::make_shared<DynamixelController>(node);
        RCLCPP_INFO(node->get_logger(), "✅ DynamixelController ready.");
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(node->get_logger(), "🛑 Exception fatale : %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
