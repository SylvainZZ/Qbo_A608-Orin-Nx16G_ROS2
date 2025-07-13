#ifndef FACE_RECOGNIZER_HPP_
#define FACE_RECOGNIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <qbo_msgs/srv/recognize_face.hpp>
#include <qbo_msgs/srv/get_name.hpp>
#include <qbo_msgs/srv/learn_faces.hpp>
#include <qbo_msgs/srv/train.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>

#include <map>
#include <vector>
#include <string>

class FaceRecognizerLBPH : public rclcpp::Node
{
public:
    explicit FaceRecognizerLBPH(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void loadFaceDatabase(const std::string &directory);
    cv::Mat convertMsgToGrayscaleMat(const sensor_msgs::msg::Image &img_msg);
    void handleRecognize(
        const std::shared_ptr<qbo_msgs::srv::RecognizeFace::Request> request,
        std::shared_ptr<qbo_msgs::srv::RecognizeFace::Response> response);

    void handleGetName(
        const std::shared_ptr<qbo_msgs::srv::GetName::Request> request,
        std::shared_ptr<qbo_msgs::srv::GetName::Response> response);

    void handleLearnFaces(
        const std::shared_ptr<qbo_msgs::srv::LearnFaces::Request> request,
        std::shared_ptr<qbo_msgs::srv::LearnFaces::Response> response);

    void handleTrain(
        const std::shared_ptr<qbo_msgs::srv::Train::Request> request,
        std::shared_ptr<qbo_msgs::srv::Train::Response> response);

    rclcpp::Service<qbo_msgs::srv::RecognizeFace>::SharedPtr recognize_service_;
    rclcpp::Service<qbo_msgs::srv::GetName>::SharedPtr get_name_service_;
    rclcpp::Service<qbo_msgs::srv::LearnFaces>::SharedPtr learn_faces_service_;
    rclcpp::Service<qbo_msgs::srv::Train>::SharedPtr train_service_;

    cv::Ptr<cv::face::LBPHFaceRecognizer> model_;
    std::map<int, std::string> label_to_name_;

    // Paramètres configurables
    std::string faces_dir_;
    std::string model_file_;
    double confidence_threshold_;
    int min_images_per_person_;

};

#endif  // FACE_RECOGNIZER_HPP_
