#ifndef QBO_FACE_TRACKER_HPP
#define QBO_FACE_TRACKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <qbo_msgs/msg/face_pos_and_dist.hpp>
#include <qbo_msgs/msg/nose.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/video/tracking.hpp>

class FaceTracker : public rclcpp::Node
{
public:
    FaceTracker();

private:

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

    bool detectFace(const cv::Mat &gray, cv::Rect &face);

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<qbo_msgs::msg::FacePosAndDist>::SharedPtr face_pub_;

    // OpenCV
    cv::CascadeClassifier face_cascade_;
    cv::Ptr<cv::TrackerKCF> tracker_;

    bool tracker_initialized_;

    cv::Rect tracked_face_;

    // Kalman
    cv::KalmanFilter kalman_;

    // Camera
    float fx_;
    int image_width_;
    int image_height_;

    int frame_count_;

    int recheck_period_;
    int missed_rechecks_;
    int max_missed_rechecks_;

    float computeDistance(const cv::Rect &face);
};

#endif