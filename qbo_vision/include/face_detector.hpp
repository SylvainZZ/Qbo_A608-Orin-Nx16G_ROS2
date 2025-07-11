/*
 * Software License Agreement (GPLv2 License)
 *
 * Copyright (c) 2012 Thecorpora, S.L.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 * Author: Sylvain <sylvain-zwolinski@orange.fr>
 */

#ifndef FACEDETECTOR_HPP_
#define FACEDETECTOR_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/tracking.hpp>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <qbo_msgs/msg/face_pos_and_dist.hpp>
#include <qbo_msgs/msg/nose.hpp>
#include <qbo_msgs/srv/recognize_face.hpp>
#include <qbo_msgs/srv/get_name.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#define HEAD_SIZE 0.20

#ifdef WITH_GPU
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudaarithm.hpp>
#endif

class FaceDetector : public rclcpp::Node
{
public:
    explicit FaceDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~FaceDetector();

private:
    void onInit();

    // ROS Parameters
    std::string face_classifier_path_;
    std::string alternative_face_classifier_path_;
    std::string lbp_classifier_path_;
    cv::Point2d default_pos_;
    int check_Haar_;
    int check_track_obj_;
    int undetected_threshold_;
    double distance_threshold_;
    bool send_to_face_recognizer_;
    bool print_recognized_face_;

    // ROS Interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    rclcpp::Publisher<qbo_msgs::msg::FacePosAndDist>::SharedPtr face_position_and_size_pub_;
    rclcpp::Publisher<qbo_msgs::msg::Nose>::SharedPtr nose_color_pub_;
    image_transport::Publisher viewer_image_pub_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher face_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void delayedInitImageTransport();
    rclcpp::Time last_image_time_{0, 0, RCL_ROS_TIME};
    const double throttle_interval_sec_ = 0.1;  // 10 Hz (tu peux changer)
    unsigned int debug_print_every_n_ = 10;     // debug toutes les 10 frames
    int haar_detection_skip_ = 5;  // essaie Haar 1 fois toutes les 5 frames


    rclcpp::Client<qbo_msgs::srv::RecognizeFace>::SharedPtr client_recognize_;
    rclcpp::Client<qbo_msgs::srv::GetName>::SharedPtr client_get_name_;

    // ROS messages/services
    qbo_msgs::srv::RecognizeFace::Request::SharedPtr srv;
    qbo_msgs::srv::GetName::Request::SharedPtr srv_get_name;

    // Internals
    cv::Size image_size_;
    int image_width_;
    int image_height_;
    bool track_object_;
    bool face_detected_bool_;
    std::string name_detected_;
    cv::Rect detected_face_roi_;
    cv::Mat detected_face_;

    cv::KalmanFilter kalman_filter_;
    cv::Mat kalman_predicted_state_;
    std::vector<float> head_distances_;
    cv::Mat p_;

    int loop_counter_;
    float face_ratio_;
    int undetected_count_;
    float fx_ = 0.0f;

    bool dynamic_check_haar_;
    int cam_shift_detected_count_;
    int cam_shift_undetected_count_;
    bool exist_alternative_;

    cv::CascadeClassifier face_classifier_;
    cv::CascadeClassifier alternative_face_classifier_;
    cv::CascadeClassifier lbp_classifier_;
    cv::Point2f last_position_ = cv::Point2f(0, 0);

    // ROS Callbacks
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);

    // Init
    void setFaceClassifierPath(std::string face_classifier_path);
    void initializeKalmanFilter();
    void declare_and_get_parameters();

    // Recognizer
    void sendToRecognizer();
    void sendToRecognizerGetName();

    // Detection
    unsigned int detectFacesHaar(cv::Mat image, std::vector<cv::Rect> &faces);
    unsigned int detectFacesAltHaar(cv::Mat image, std::vector<cv::Rect> &faces);
    unsigned int detectFaceCamShift(cv::Mat img);
    // void classifierDetect(cv::Mat image, std::vector<cv::Rect> &detections, cv::CascadeClassifier &classifier, cv::Size size = cv::Size(30, 30));
    // void classifierDetect(cv::Mat image, std::vector<cv::Rect> &detections, cv::CascadeClassifier &classifier, int flag, cv::Size size);


    // Utils
    double euclidDist(cv::Point2f pt1, cv::Point2f pt2);
    float calcDistanceToHead(cv::Mat &head, cv::KalmanFilter &kalman_filter);
    // void deleteROSParams();

#ifdef WITH_GPU
    cv::Ptr<cv::cuda::CascadeClassifier> face_classifier_gpu_;
#endif
};

// À placer en bas du fichier .hpp (en dehors de la classe FaceDetector)

// Déclaration de la version avec flag
void classifierDetect(cv::Mat image,
                      std::vector<cv::Rect> &detections,
                      cv::CascadeClassifier &classifier,
                      int flag,
                      cv::Size size = cv::Size(30, 30));

// Déclaration de la version sans flag (aucun argument par défaut ici)
void classifierDetect(cv::Mat image,
                      std::vector<cv::Rect> &detections,
                      cv::CascadeClassifier &classifier,
                      cv::Size size);

void classifierDetect(cv::Mat image,
                      std::vector<cv::Rect> &detections,
                      cv::CascadeClassifier &classifier);


#endif  // FACEDETECTOR_HPP_


