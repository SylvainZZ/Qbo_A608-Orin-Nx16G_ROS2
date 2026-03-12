#include "face_tracker.hpp"

#include <iomanip>
#include <sstream>

#define REAL_FACE_HEIGHT 0.21

FaceTracker::FaceTracker()
: Node("qbo_face_tracker")
{
    tracker_initialized_ = false;
    frame_count_ = 0;
    fx_ = 0;
    recheck_period_ = 10;
    missed_rechecks_ = 0;
    max_missed_rechecks_ = 2;

    tracking_state_ = TrackingState::SEARCH;

    candidate_valid_ = false;
    candidate_hits_ = 0;
    tracker_confidence_ = 0.0f;
    smoothed_face_valid_ = false;
    smoothed_face_width_ = 0.0f;
    smoothed_face_height_ = 0.0f;

    this->declare_parameter("publish_debug_image", true);
    this->declare_parameter("debug_image_topic", std::string("/qbo_face_tracking/debug_image"));
    this->declare_parameter("face_ratio_min", 0.65);
    this->declare_parameter("face_ratio_max", 2.0);
    this->declare_parameter("edge_margin_px", 4);
    this->declare_parameter("max_tracking_jump_px", 220);
    this->declare_parameter("haar_scale_factor", 1.1);
    this->declare_parameter("haar_min_neighbors", 5);
    this->declare_parameter("haar_min_face_size_px", 60);
    this->declare_parameter("candidate_required_hits",3);
    this->declare_parameter("max_face_distance_m",2.0);
    this->declare_parameter("min_face_distance_m",0.35);
    this->declare_parameter("max_scale_jump",1.6);
    this->declare_parameter("size_smoothing_alpha",0.35);
    this->declare_parameter("confidence_rise",0.08);
    this->declare_parameter("confidence_decay",0.20);
    this->declare_parameter("candidate_confidence_max",0.55);

    this->get_parameter("publish_debug_image", publish_debug_image_);
    this->get_parameter("debug_image_topic", debug_image_topic_);
    this->get_parameter("face_ratio_min", face_ratio_min_);
    this->get_parameter("face_ratio_max", face_ratio_max_);
    this->get_parameter("edge_margin_px", edge_margin_px_);
    this->get_parameter("max_tracking_jump_px", max_tracking_jump_px_);
    this->get_parameter("haar_scale_factor", haar_scale_factor_);
    this->get_parameter("haar_min_neighbors", haar_min_neighbors_);
    this->get_parameter("haar_min_face_size_px", haar_min_face_size_px_);
    this->get_parameter("candidate_required_hits",candidate_required_hits_);
    this->get_parameter("max_face_distance_m",max_face_distance_m_);
    this->get_parameter("min_face_distance_m",min_face_distance_m_);
    this->get_parameter("max_scale_jump",max_scale_jump_);
    this->get_parameter("size_smoothing_alpha",size_smoothing_alpha_);
    this->get_parameter("confidence_rise",confidence_rise_);
    this->get_parameter("confidence_decay",confidence_decay_);
    this->get_parameter("candidate_confidence_max",candidate_confidence_max_);

    size_smoothing_alpha_ = std::clamp(size_smoothing_alpha_, 0.0f, 1.0f);
    confidence_rise_ = std::clamp(confidence_rise_, 0.0f, 1.0f);
    confidence_decay_ = std::clamp(confidence_decay_, 0.0f, 1.0f);
    candidate_confidence_max_ = std::clamp(candidate_confidence_max_, 0.0f, 1.0f);

    auto qos = rclcpp::SensorDataQoS().keep_last(1);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera_left/image_raw",
        qos,
        std::bind(&FaceTracker::imageCallback,this,std::placeholders::_1));

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_left/camera_info",
        1,
        std::bind(&FaceTracker::cameraInfoCallback,this,std::placeholders::_1));

    face_pub_ = this->create_publisher<qbo_msgs::msg::FacePosAndDist>(
        "/qbo_face_tracking/face_pos_and_dist",10);

    if(publish_debug_image_)
    {
        viewer_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "Debug image publisher enabled on: %s", debug_image_topic_.c_str());
    }

    std::string cascade_path =
        "/home/qbo-v2/qbo_ws/src/qbo_vision/config/classifier/haarcascade_frontalface_default.xml";

    if(!face_cascade_.load(cascade_path))
    {
        RCLCPP_ERROR(this->get_logger(),
            "Impossible de charger le cascade : %s",
            cascade_path.c_str());
    }

    kalman_.init(4,2,0);

    kalman_.transitionMatrix = (cv::Mat_<float>(4,4) <<
        1,0,1,0,
        0,1,0,1,
        0,0,1,0,
        0,0,0,1);

    kalman_.measurementMatrix = cv::Mat::eye(2,4,CV_32F);

    kalman_.processNoiseCov = cv::Mat::eye(4,4,CV_32F)*0.01;
    kalman_.measurementNoiseCov = cv::Mat::eye(2,2,CV_32F)*0.1;

    kalman_.errorCovPost = cv::Mat::eye(4,4,CV_32F);

    RCLCPP_INFO(this->get_logger(),"Face tracker ready");
}

void FaceTracker::cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    fx_ = msg->k[0];
}

bool FaceTracker::detectFace(const cv::Mat &gray, cv::Rect &face)
{
    std::vector<cv::Rect> faces;

    face_cascade_.detectMultiScale(
        gray,
        faces,
        haar_scale_factor_,
        haar_min_neighbors_,
        cv::CASCADE_SCALE_IMAGE,
        cv::Size(haar_min_face_size_px_, haar_min_face_size_px_));

    if(faces.empty())
        return false;

    float best_score = -1.0f;
    cv::Rect best_face;

    for(const auto &f : faces)
    {
        // 1) Face shape ratio filter (more permissive to handle head pose).
        float ratio = (float)f.height / (float)f.width;

        if(ratio < face_ratio_min_ || ratio > face_ratio_max_)
            continue;

        // 2) Coherence with previous tracking: reject only very large jumps.
        float motion_penalty = 0.0f;
        if(tracker_initialized_)
        {
            float dx = std::abs(f.x - tracked_face_.x);
            float dy = std::abs(f.y - tracked_face_.y);

            if(dx > max_tracking_jump_px_ || dy > max_tracking_jump_px_)
                continue;

            motion_penalty = 0.35f * (dx + dy);
        }

        float score = static_cast<float>(f.area()) - motion_penalty;
        if(score > best_score)
        {
            best_score = score;
            best_face = f;
        }
    }

    if(best_score < 0.0f)
        return false;

    int margin = edge_margin_px_;

    if(best_face.x < margin ||
       best_face.y < margin ||
       best_face.x + best_face.width > gray.cols - margin ||
       best_face.y + best_face.height > gray.rows - margin)
        return false;

    float dist = computeDistance(best_face);

    if(dist > max_face_distance_m_ || dist < min_face_distance_m_)
        return false;

    face = best_face;
    return true;
}

float FaceTracker::computeDistance(const cv::Rect &face)
{
    if(fx_<=0)
        return -1;

    return (fx_ * REAL_FACE_HEIGHT) / face.height;
}

void FaceTracker::smoothTrackedFaceSize(cv::Rect &face, const cv::Size &frame_size)
{
    if(face.width <= 0 || face.height <= 0)
        return;

    if(!smoothed_face_valid_)
    {
        smoothed_face_width_ = static_cast<float>(face.width);
        smoothed_face_height_ = static_cast<float>(face.height);
        smoothed_face_valid_ = true;
        return;
    }

    float alpha = size_smoothing_alpha_;
    smoothed_face_width_ = alpha * static_cast<float>(face.width) + (1.0f - alpha) * smoothed_face_width_;
    smoothed_face_height_ = alpha * static_cast<float>(face.height) + (1.0f - alpha) * smoothed_face_height_;

    int center_x = face.x + face.width / 2;
    int center_y = face.y + face.height / 2;
    int smooth_w = std::max(1, static_cast<int>(std::round(smoothed_face_width_)));
    int smooth_h = std::max(1, static_cast<int>(std::round(smoothed_face_height_)));

    int x = center_x - smooth_w / 2;
    int y = center_y - smooth_h / 2;

    x = std::max(0, std::min(x, frame_size.width - smooth_w));
    y = std::max(0, std::min(y, frame_size.height - smooth_h));

    face = cv::Rect(x, y, smooth_w, smooth_h);
}

void FaceTracker::imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    frame_count_++;

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
    }
    catch(...)
    {
        return;
    }

    cv::Mat frame = cv_ptr->image;

    image_width_ = frame.cols;
    image_height_ = frame.rows;

    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray,gray);

    bool do_recheck = (frame_count_ % recheck_period_) == 0;

    if(!tracker_initialized_)
    {
        cv::Rect face;
        bool found_face = false;

        // Try local ROI first for faster, more stable reacquisition near last known face.
        cv::Rect roi = local_roi_ & cv::Rect(0, 0, gray.cols, gray.rows);
        if(roi.width > 0 && roi.height > 0)
        {
            cv::Rect local_face;
            cv::Mat gray_local = gray(roi);
            if(detectFace(gray_local, local_face))
            {
                face.x = local_face.x + roi.x;
                face.y = local_face.y + roi.y;
                face.width = local_face.width;
                face.height = local_face.height;
                found_face = true;
            }
        }

        if(!found_face)
            found_face = detectFace(gray, face);

        if(found_face)
        {
            if(!candidate_valid_)
            {
                candidate_face_ = face;
                candidate_hits_ = 1;
                candidate_valid_ = true;
                tracking_state_ = TrackingState::CANDIDATE;
            }
            else
            {
                float dx = std::abs(face.x - candidate_face_.x);
                float dy = std::abs(face.y - candidate_face_.y);

                if(dx < 40 && dy < 40)
                {
                    candidate_hits_++;
                    candidate_face_ = face;
                }
                else
                {
                    candidate_face_ = face;
                    candidate_hits_ = 1;
                }
            }

            float candidate_progress =
                static_cast<float>(candidate_hits_) /
                static_cast<float>(std::max(1, candidate_required_hits_));
            candidate_progress = std::clamp(candidate_progress, 0.0f, 1.0f);
            tracker_confidence_ = std::clamp(
                candidate_progress * candidate_confidence_max_, 0.0f, 1.0f);

            if(candidate_hits_ >= candidate_required_hits_)
            {
                tracker_ = cv::TrackerKCF::create();
                tracker_->init(frame,candidate_face_);

                tracked_face_ = candidate_face_;
                previous_face_ = tracked_face_;
                smoothTrackedFaceSize(tracked_face_, frame.size());
                previous_face_ = tracked_face_;

                tracker_initialized_ = true;
                tracking_state_ = TrackingState::TRACKING;
                missed_rechecks_ = 0;
                tracker_confidence_ = std::max(tracker_confidence_, 0.70f);

                candidate_valid_ = false;
                candidate_hits_ = 0;

                float cx = tracked_face_.x + tracked_face_.width/2.0f;
                float cy = tracked_face_.y + tracked_face_.height/2.0f;

                kalman_.statePost.at<float>(0) = cx;
                kalman_.statePost.at<float>(1) = cy;
                kalman_.statePost.at<float>(2) = 0;
                kalman_.statePost.at<float>(3) = 0;
            }
        }
        else
        {
            candidate_valid_ = false;
            candidate_hits_ = 0;
            tracking_state_ = TrackingState::SEARCH;
            tracker_confidence_ = std::max(0.0f, tracker_confidence_ - confidence_decay_);
        }
    }
    else if(do_recheck)
    {
        cv::Rect face;

        if(detectFace(gray,face))
        {
            tracker_ = cv::TrackerKCF::create();
            tracker_->init(frame,face);

            tracked_face_ = face;
            previous_face_ = tracked_face_;
            smoothTrackedFaceSize(tracked_face_, frame.size());
            previous_face_ = tracked_face_;
            missed_rechecks_ = 0;
            tracking_state_ = TrackingState::TRACKING;
            tracker_confidence_ = std::min(1.0f, tracker_confidence_ + confidence_rise_);
        }
        else
        {
            // KCF can keep drifting on background patterns: require periodic Haar confirmation.
            missed_rechecks_++;
            tracker_confidence_ = std::max(0.0f, tracker_confidence_ - 0.5f * confidence_decay_);
            if(missed_rechecks_ >= max_missed_rechecks_)
            {
                tracker_initialized_ = false;
                missed_rechecks_ = 0;
                tracking_state_ = TrackingState::SEARCH;
                tracker_confidence_ = 0.0f;
            }
        }
    }
    else
    {
        cv::Rect prev_face = tracked_face_;
        bool ok = tracker_->update(frame,tracked_face_);

        if(!ok)
        {
            tracker_initialized_ = false;
            missed_rechecks_ = 0;
            tracking_state_ = TrackingState::SEARCH;
            smoothed_face_valid_ = false;
            tracker_confidence_ = 0.0f;
        }

        if(tracker_initialized_)
        {
            if(prev_face.height > 0 && tracked_face_.height > 0)
            {
                float scale_ratio =
                    static_cast<float>(tracked_face_.height) /
                    static_cast<float>(prev_face.height);

                if(scale_ratio > max_scale_jump_ ||
                   scale_ratio < (1.0f / max_scale_jump_))
                {
                    tracker_initialized_ = false;
                    tracking_state_ = TrackingState::SEARCH;
                    smoothed_face_valid_ = false;
                    tracker_confidence_ = 0.0f;
                }
            }

            // Reject impossible tracking windows.
            if(tracker_initialized_ &&
               (tracked_face_.width < 40 || tracked_face_.height < 40 ||
               tracked_face_.x < 0 || tracked_face_.y < 0 ||
               tracked_face_.x + tracked_face_.width > frame.cols ||
               tracked_face_.y + tracked_face_.height > frame.rows))
            {
                tracker_initialized_ = false;
                missed_rechecks_ = 0;
                tracking_state_ = TrackingState::SEARCH;
                smoothed_face_valid_ = false;
                tracker_confidence_ = 0.0f;
            }

            if(tracker_initialized_)
            {
                smoothTrackedFaceSize(tracked_face_, frame.size());
                previous_face_ = tracked_face_;
                tracker_confidence_ = std::min(1.0f, tracker_confidence_ + confidence_rise_);

                int roi_w = tracked_face_.width * 3;
                int roi_h = tracked_face_.height * 3;

                int roi_x = tracked_face_.x + tracked_face_.width / 2 - roi_w / 2;
                int roi_y = tracked_face_.y + tracked_face_.height / 2 - roi_h / 2;

                roi_x = std::max(0, roi_x);
                roi_y = std::max(0, roi_y);

                roi_w = std::min(frame.cols - roi_x, roi_w);
                roi_h = std::min(frame.rows - roi_y, roi_h);

                local_roi_ = cv::Rect(roi_x, roi_y, roi_w, roi_h);
            }
        }
    }

    bool face_detected=false;

    float u=0;
    float v=0;
    float distance=0;

    if(tracker_initialized_)
    {
        face_detected=true;

        float cx = tracked_face_.x + tracked_face_.width/2.0;
        float cy = tracked_face_.y + tracked_face_.height/2.0;

        cv::Mat meas(2,1,CV_32F);
        meas.at<float>(0)=cx;
        meas.at<float>(1)=cy;

        // kalman_.predict();
        // kalman_.correct(meas);

        // float px = kalman_.statePost.at<float>(0);
        // float py = kalman_.statePost.at<float>(1);

        cv::Mat prediction = kalman_.predict();

        cv::Mat estimated = kalman_.correct(meas);

        float px = estimated.at<float>(0);
        float py = estimated.at<float>(1);

        u = px - image_width_/2;
        v = py - image_height_/2;

        distance = computeDistance(tracked_face_);

        if(distance <= 0.0f || !std::isfinite(distance))
        {
            face_detected = false;
            tracker_initialized_ = false;
            missed_rechecks_ = 0;
            smoothed_face_valid_ = false;
            tracker_confidence_ = 0.0f;
        }

        if(face_detected)
        {
            cv::rectangle(frame,tracked_face_,cv::Scalar(0,255,0),2);
            cv::circle(frame,{(int)px,(int)py},4,{0,0,255},2);
        }
    }

    qbo_msgs::msg::FacePosAndDist msg_out;

    msg_out.header.stamp = this->now();

    msg_out.u = u;
    msg_out.v = v;
    msg_out.distance_to_head = distance;

    msg_out.image_width = image_width_;
    msg_out.image_height = image_height_;

    msg_out.face_detected = face_detected;

    msg_out.name_signature = "";

    std::string state_str;
    switch(tracking_state_)
    {
    case TrackingState::SEARCH:
        state_str = "SEARCH";
        break;
    case TrackingState::CANDIDATE:
        state_str = "CANDIDATE";
        break;
    case TrackingState::TRACKING:
        state_str = "TRACKING";
        break;
    }
    msg_out.type_of_tracking = state_str;

    face_pub_->publish(msg_out);

    if(publish_debug_image_ && viewer_image_pub_)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        if(face_detected && std::isfinite(distance) && distance > 0.0f)
            oss << "dist: " << distance << " m";
        else
            oss << "dist: n/a";

        cv::putText(frame, oss.str(), cv::Point(20, 35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        cv::putText(frame, "tracking: " + msg_out.type_of_tracking, cv::Point(20, 70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

        std::ostringstream conf_stream;
        conf_stream << std::fixed << std::setprecision(2)
                << "confidence: " << tracker_confidence_;
        cv::putText(frame, conf_stream.str(), cv::Point(20, 105),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 200, 255), 2);

        auto debug_msg = cv_bridge::CvImage(msg_out.header, "bgr8", frame);
        viewer_image_pub_->publish(*debug_msg.toImageMsg());
    }
}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<FaceTracker>();

    rclcpp::spin(node);

    rclcpp::shutdown();
}