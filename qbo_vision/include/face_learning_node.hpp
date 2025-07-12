// face_learning_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <qbo_msgs/srv/learn_faces.hpp>
#include <qbo_msgs/srv/train.hpp>

#include <opencv2/opencv.hpp>
#include <filesystem>

class FaceLearningNode : public rclcpp::Node
{
public:
    explicit FaceLearningNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void captureAndSendFace();

    image_transport::Subscriber image_sub_;
    std::shared_ptr<image_transport::ImageTransport> it_;

    rclcpp::Client<qbo_msgs::srv::LearnFaces>::SharedPtr learn_client_;
    rclcpp::Client<qbo_msgs::srv::Train>::SharedPtr train_client_;

    cv::Mat latest_image_;
    bool image_received_ = false;
    std::string faces_dir_ = "faces";  // relative to package root

    void saveFaceVariants(const cv::Mat & face, const std::string & path_prefix);
};
