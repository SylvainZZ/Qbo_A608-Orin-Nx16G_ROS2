#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <qbo_msgs/srv/learn_faces.hpp>
#include <qbo_msgs/srv/train.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <filesystem>
#include <chrono>

using std::placeholders::_1;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

class FaceLearningNode : public rclcpp::Node
{
public:
  FaceLearningNode() : Node("face_learning_node"), image_received_(false)
  {
    // Paramètres
    faces_dir_ = this->declare_parameter<std::string>("faces_dir", "faces");
    face_name_ = this->declare_parameter<std::string>("face_name", "unknown");

    // Souscription à l'image de visage détecté
    face_sub_ = image_transport::create_subscription(
      this, "/qbo_face_tracking/face_name",
      std::bind(&FaceLearningNode::imageCallback, this, _1),
      "raw");

    learn_client_ = this->create_client<qbo_msgs::srv::LearnFaces>("learn_faces");
    train_client_ = this->create_client<qbo_msgs::srv::Train>("train");

    RCLCPP_INFO(this->get_logger(), "🧠 Node d'apprentissage facial prêt");

    // Timer pour capturer les images à intervalles fixes
    timer_ = this->create_wall_timer(1500ms, std::bind(&FaceLearningNode::tryCaptureFace, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Subscriber face_sub_;
  rclcpp::Client<qbo_msgs::srv::LearnFaces>::SharedPtr learn_client_;
  rclcpp::Client<qbo_msgs::srv::Train>::SharedPtr train_client_;

  cv::Mat latest_face_;
  bool image_received_;

  std::string face_name_;
  std::string faces_dir_;
  int capture_count_ = 0;
  const int max_captures_ = 5;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {
    try
    {
      latest_face_ = cv_bridge::toCvCopy(msg, "rgb8")->image;
      image_received_ = true;
      RCLCPP_INFO(this->get_logger(), "📸 Nouvelle image de visage reçue.");
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void tryCaptureFace()
  {
    if (!image_received_ || capture_count_ >= max_captures_)
      return;

    std::string user_dir = faces_dir_ + "/" + face_name_;
    fs::create_directories(user_dir);

    std::string base_filename = user_dir + "/" + std::to_string(this->now().nanoseconds());

    // Sauvegarde image originale
    cv::imwrite(base_filename + ".jpg", latest_face_);

    // Génération de variations
    cv::Mat flip_img, bright_img, dark_img;
    cv::flip(latest_face_, flip_img, 1);
    latest_face_.convertTo(bright_img, -1, 1.2, 30);
    latest_face_.convertTo(dark_img, -1, 0.8, -30);

    cv::imwrite(base_filename + "_flip.jpg", flip_img);
    cv::imwrite(base_filename + "_bright.jpg", bright_img);
    cv::imwrite(base_filename + "_dark.jpg", dark_img);

    capture_count_++;
    RCLCPP_INFO(this->get_logger(), "✅ Capture %d/%d enregistrée pour %s", capture_count_, max_captures_, face_name_.c_str());

    if (capture_count_ == max_captures_)
    {
      sendLearnRequest();
    }
  }

  void sendLearnRequest()
  {
    auto req = std::make_shared<qbo_msgs::srv::LearnFaces::Request>();
    req->person_name = face_name_;

    learn_client_->async_send_request(
        req,
        [this](rclcpp::Client<qbo_msgs::srv::LearnFaces>::SharedFuture result) {
            if (result.get()->learned) {
                RCLCPP_INFO(this->get_logger(), "📚 Apprentissage terminé avec succès. Entraînement en cours...");
                sendTrainRequest();
            } else {
                RCLCPP_WARN(this->get_logger(), "⚠️ Apprentissage échoué.");
            }
        }
    );
  }

  void sendTrainRequest()
    {
        auto req = std::make_shared<qbo_msgs::srv::Train::Request>();
        req->update_path = "";  // Optionnel, ou à remplir si utile

        train_client_->async_send_request(
            req,
            [this](rclcpp::Client<qbo_msgs::srv::Train>::SharedFuture result) {
                if (result.get()->taught) {
                    RCLCPP_INFO(this->get_logger(), "✅ Entraînement du modèle réussi !");
                } else {
                    RCLCPP_WARN(this->get_logger(), "❌ Entraînement échoué.");
                }
            }
        );
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaceLearningNode>());
  rclcpp::shutdown();
  return 0;
}
