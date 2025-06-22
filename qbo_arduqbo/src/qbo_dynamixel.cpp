#include "rclcpp/rclcpp.hpp"
#include "qbo_arduqbo/controllers/dynamixel_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    // options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("qbo_dynamixel", options);
    RCLCPP_INFO(node->get_logger(), "🎬 Démarrage du noeud qbo_dynamixel");

    try
    {
        // 🔁 Déclare ici les paramètres dynamiques que tu veux accessibles
        std::vector<std::string> motor_keys;
        node->declare_parameter("dynamixel.motor_keys", motor_keys);
        node->get_parameter("dynamixel.motor_keys", motor_keys);

        // ✅ Vérification présence de config
        if (motor_keys.empty()) {
            RCLCPP_FATAL(node->get_logger(),
                "❌ Aucun moteur n'est défini. Vérifie le fichier YAML (clé : dynamixel.motor_keys).");
            return 1;
        }

        for (const auto &key : motor_keys) {
            std::string full = "dynamixel.motors." + key + ".name";
            node->declare_parameter(full, key);
        }

        auto controller = std::make_shared<DynamixelController>(node);
        // RCLCPP_INFO(node->get_logger(), "✅ Contrôleur initialisé");
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
