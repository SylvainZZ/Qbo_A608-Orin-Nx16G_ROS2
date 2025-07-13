#ifndef FACE_RECOGNIZER_LBPH_HPP_
#define FACE_RECOGNIZER_LBPH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <qbo_msgs/srv/recognize_face.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>

class FaceRecognizerLBPH : public rclcpp::Node
{
public:
    explicit FaceRecognizerLBPH(const rclcpp::NodeOptions & options);

private:
    // Méthodes
    void handleRecognize(
        const std::shared_ptr<qbo_msgs::srv::RecognizeFace::Request> request,
        std::shared_ptr<qbo_msgs::srv::RecognizeFace::Response> response);

    void loadTrainedModelIfAvailable();
    void prepareTrainingIfNeeded();

    // Membres
    rclcpp::Service<qbo_msgs::srv::RecognizeFace>::SharedPtr recognize_service_;
    cv::Ptr<cv::face::LBPHFaceRecognizer> model_;
    std::map<int, std::string> label_to_name_;

    bool model_trained_ = false;

    // Paramètres
    std::string path_faces_learn_;
    std::string path_model_file_;
    std::string model_file_;
    double confidence_threshold_;
};

#endif  // FACE_RECOGNIZER_LBPH_HPP_
