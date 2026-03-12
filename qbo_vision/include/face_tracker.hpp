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
#include <opencv2/objdetect/face.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/dnn.hpp>

class FaceTracker : public rclcpp::Node
{
public:
    FaceTracker();

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

    // Detection
    bool detectFaceHaar(const cv::Mat &gray, cv::Rect &face);
    bool detectFaceYuNet(const cv::Mat &frame_bgr, cv::Rect &face);
    bool detectFaceYuNetInRoi(const cv::Mat &frame_bgr, const cv::Rect &roi, cv::Rect &face);

    // Validation / scoring
    bool validateFaceRect(const cv::Rect &face, const cv::Size &image_size, float *score = nullptr) const;
    cv::Rect buildLocalRoiFromPrediction(const cv::Size &frame_size, int scale = 3) const;

    // Kalman helpers
    void resetKalmanToFace(const cv::Rect &face);
    cv::Point2f predictKalmanCenter();
    cv::Point2f correctKalmanCenter(const cv::Point2f &meas);

    // Tracking helpers
    void resetTrackingState();
    void smoothTrackedFaceSize(cv::Rect &face, const cv::Size &frame_size);
    float computeDistance(const cv::Rect &face);

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Publisher<qbo_msgs::msg::FacePosAndDist>::SharedPtr face_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viewer_image_pub_;

    bool publish_debug_image_;
    std::string debug_image_topic_;

    // OpenCV detectors / tracker
    cv::CascadeClassifier face_cascade_;
    cv::Ptr<cv::FaceDetectorYN> yunet_;
    cv::Ptr<cv::TrackerKCF> tracker_;

    bool use_yunet_;
    bool use_haar_fallback_;

    std::string yunet_model_path_;
    float yunet_score_threshold_;
    float yunet_nms_threshold_;
    int yunet_top_k_;

    bool tracker_initialized_;

    cv::Rect tracked_face_;
    cv::Rect previous_face_;
    cv::Rect local_roi_;

    // Kalman
    cv::KalmanFilter kalman_;

    // Camera
    float fx_;
    int image_width_;
    int image_height_;
    int frame_count_;

    // Periodic re-check
    int recheck_period_;
    int missed_rechecks_;
    int max_missed_rechecks_;

    // Tracking state machine
    enum class TrackingState
    {
        SEARCH,
        CANDIDATE,
        TRACKING
    };

    TrackingState tracking_state_;

    // Candidate validation
    cv::Rect candidate_face_;
    bool candidate_valid_;
    int candidate_hits_;
    int candidate_required_hits_;

    // Detection tuning
    float max_face_distance_m_;
    float min_face_distance_m_;
    float max_scale_jump_;

    float face_ratio_min_;
    float face_ratio_max_;
    int edge_margin_px_;
    int max_tracking_jump_px_;

    double haar_scale_factor_;
    int haar_min_neighbors_;
    int haar_min_face_size_px_;

    // Temporal smoothing for tracked face size
    float size_smoothing_alpha_;
    bool smoothed_face_valid_;
    float smoothed_face_width_;
    float smoothed_face_height_;

    // Tracker confidence
    float tracker_confidence_;
    float confidence_rise_;
    float confidence_decay_;
    float candidate_confidence_max_;
};

#endif