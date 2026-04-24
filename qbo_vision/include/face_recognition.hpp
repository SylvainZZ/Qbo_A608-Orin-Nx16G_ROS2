#ifndef QBO_FACE_RECOGNITION_HPP
#define QBO_FACE_RECOGNITION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <qbo_msgs/msg/face_observation.hpp>
#include <qbo_msgs/msg/face_recognition_result.hpp>
#include <qbo_msgs/msg/enrollment_progress.hpp>
#include <qbo_msgs/srv/start_enroll_person.hpp>
#include <qbo_msgs/srv/list_persons.hpp>
#include <qbo_msgs/srv/remove_person.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/face.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unordered_map>
#include <deque>

#include "face_database.hpp"

class FaceRecognitionNode : public rclcpp::Node
{
public:
    FaceRecognitionNode();

private:
    void synchronizedCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr & image,
        const qbo_msgs::msg::FaceObservation::ConstSharedPtr & face);

    void startEnrollCallback(
        const std::shared_ptr<qbo_msgs::srv::StartEnrollPerson::Request> req,
        std::shared_ptr<qbo_msgs::srv::StartEnrollPerson::Response> res);

    void listPersonsCallback(
        const std::shared_ptr<qbo_msgs::srv::ListPersons::Request> req,
        std::shared_ptr<qbo_msgs::srv::ListPersons::Response> res);

    void removePersonCallback(
        const std::shared_ptr<qbo_msgs::srv::RemovePerson::Request> req,
        std::shared_ptr<qbo_msgs::srv::RemovePerson::Response> res);

    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat);

    float calculateAverageSimilarity(const std::vector<cv::Mat> & embeddings);
    float calculateMinSimilarity(const cv::Mat & new_embedding, const std::vector<cv::Mat> & existing);

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<qbo_msgs::msg::FaceObservation> face_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        qbo_msgs::msg::FaceObservation> SyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::unordered_map<uint32_t, rclcpp::Time> recognition_cache_;

    rclcpp::Publisher<qbo_msgs::msg::FaceRecognitionResult>::SharedPtr result_pub_;
    rclcpp::Publisher<qbo_msgs::msg::EnrollmentProgress>::SharedPtr enrollment_progress_pub_;
    rclcpp::Time last_recognition_time_;

    cv::Ptr<cv::FaceRecognizerSF> recognizer_;

    double recognition_cache_time_;
    double min_quality_threshold_;
    double min_similarity_threshold_;
    int min_samples_for_publication_;

    FaceDatabase database_;

    bool enroll_mode_;
    std::string enroll_name_;
    int enroll_samples_;
    std::vector<float> enroll_qualities_;  // Track quality of each sample
    rclcpp::Time last_enrollment_sample_time_;  // Track time of last sample capture

    std::vector<cv::Mat> enroll_embeddings_;

    int enroll_target_samples_;
    double enrollment_sample_interval_;  // Minimum time between samples (seconds)
    int last_countdown_value_;  // Track last published countdown value

    rclcpp::Service<qbo_msgs::srv::StartEnrollPerson>::SharedPtr enroll_service_;
    rclcpp::Service<qbo_msgs::srv::ListPersons>::SharedPtr list_persons_service_;
    rclcpp::Service<qbo_msgs::srv::RemovePerson>::SharedPtr remove_person_service_;

    std::string db_path_;
    std::string db_path;

    std::unordered_map<uint32_t, std::deque<std::string>> identity_history_;

    int identity_window_;

    // Diagnostics
    diagnostic_updater::Updater diagnostic_updater_;
    std::unordered_map<std::string, rclcpp::Time> last_positive_recognition_;
    rclcpp::Time last_face_seen_;
    std::string current_face_name_;
    float current_face_similarity_;
    bool current_face_present_;
    bool model_loaded_;

    // Statistics
    int total_recognitions_;
    int successful_recognitions_;
};

#endif