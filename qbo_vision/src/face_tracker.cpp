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
    this->declare_parameter("use_yunet", true);
    this->declare_parameter("use_haar_fallback", true);
    this->declare_parameter(
        "haar_cascade_path",
        std::string("/home/qbo-v2/qbo_ws/src/qbo_vision/config/classifier/haarcascade_frontalface_default.xml"));
    this->declare_parameter(
        "yunet_model_path",
        std::string("/home/qbo-v2/qbo_ws/src/qbo_vision/config/models/face_detection_yunet_2022mar.onnx"));
    this->declare_parameter("yunet_score_threshold", 0.85);
    this->declare_parameter("yunet_nms_threshold", 0.30);
    this->declare_parameter("yunet_top_k", 10);

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
    this->get_parameter("use_yunet", use_yunet_);
    this->get_parameter("use_haar_fallback", use_haar_fallback_);
    this->get_parameter("haar_cascade_path", haar_cascade_path_);
    this->get_parameter("yunet_model_path", yunet_model_path_);
    this->get_parameter("yunet_score_threshold", yunet_score_threshold_);
    this->get_parameter("yunet_nms_threshold", yunet_nms_threshold_);
    this->get_parameter("yunet_top_k", yunet_top_k_);

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

    if(!face_cascade_.load(haar_cascade_path_))
    {
        RCLCPP_ERROR(this->get_logger(),
            "Impossible de charger le cascade : %s",
            haar_cascade_path_.c_str());
    }

    if(use_yunet_)
    {
        try
        {
            yunet_ = cv::FaceDetectorYN::create(
                yunet_model_path_,
                "",
                cv::Size(640, 480),
                yunet_score_threshold_,
                yunet_nms_threshold_,
                yunet_top_k_);

            RCLCPP_INFO(this->get_logger(), "YuNet loaded: %s", yunet_model_path_.c_str());
        }
        catch(const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "YuNet load failed: %s", e.what());
            use_yunet_ = false;
        }
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


float FaceTracker::computeDistance(const cv::Rect &face) const
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

bool FaceTracker::validateFaceRect(const cv::Rect &face, const cv::Size &image_size, float *score) const
{
    if(face.width <= 0 || face.height <= 0)
        return false;

    float ratio = static_cast<float>(face.height) / static_cast<float>(face.width);
    if(ratio < face_ratio_min_ || ratio > face_ratio_max_)
        return false;

    int margin = edge_margin_px_;
    if(face.x < margin ||
    face.y < margin ||
    face.x + face.width > image_size.width - margin ||
    face.y + face.height > image_size.height - margin)
        return false;

    if(fx_ > 0.0f)
    {
        float dist = computeDistance(face);
        if(!std::isfinite(dist) || dist < min_face_distance_m_ || dist > max_face_distance_m_)
            return false;
    }

    if(tracker_initialized_)
    {
        float dx = std::abs(face.x - tracked_face_.x);
        float dy = std::abs(face.y - tracked_face_.y);

        if(dx > max_tracking_jump_px_ || dy > max_tracking_jump_px_)
            return false;

        if(tracked_face_.height > 0)
        {
            float scale_ratio =
                static_cast<float>(face.height) /
                static_cast<float>(tracked_face_.height);

            if(scale_ratio > max_scale_jump_ ||
            scale_ratio < (1.0f / max_scale_jump_))
                return false;
        }
    }

    if(score)
    {
        float cx = face.x + face.width * 0.5f;
        float cy = face.y + face.height * 0.5f;
        float icx = image_size.width * 0.5f;
        float icy = image_size.height * 0.5f;
        float dist_center = std::hypot(cx - icx, cy - icy);

        // Grande bbox + proche du centre = mieux
        *score = static_cast<float>(face.area()) - 0.3f * dist_center;
    }

    return true;
}

bool FaceTracker::detectFaceHaar(const cv::Mat &gray, cv::Rect &face)
{
    std::vector<cv::Rect> faces;

    face_cascade_.detectMultiScale(
        gray,
        faces,
        haar_scale_factor_,
        haar_min_neighbors_,
        cv::CASCADE_FIND_BIGGEST_OBJECT,
        cv::Size(haar_min_face_size_px_, haar_min_face_size_px_));

    if(faces.empty())
        return false;

    float best_score = -1.0f;
    cv::Rect best_face;

    for(const auto &f : faces)
    {
        float score = 0.0f;
        if(!validateFaceRect(f, gray.size(), &score))
            continue;

        if(score > best_score)
        {
            best_score = score;
            best_face = f;
        }
    }

    if(best_score < 0.0f)
        return false;

    face = best_face;
    return true;
}

bool FaceTracker::detectFaceYuNet(const cv::Mat &frame_bgr, cv::Rect &face)
{
    if(!yunet_)
        return false;

    try
    {
        yunet_->setInputSize(frame_bgr.size());

        cv::Mat detections;
        yunet_->detect(frame_bgr, detections);

        if(detections.empty() || detections.rows == 0)
            return false;

        float best_score = -1.0f;
        cv::Rect best_face;

        for(int i = 0; i < detections.rows; ++i)
        {
            float x = detections.at<float>(i, 0);
            float y = detections.at<float>(i, 1);
            float w = detections.at<float>(i, 2);
            float h = detections.at<float>(i, 3);
            float det_score = detections.at<float>(i, 14);

            cv::Rect f(
                static_cast<int>(std::round(x)),
                static_cast<int>(std::round(y)),
                static_cast<int>(std::round(w)),
                static_cast<int>(std::round(h)));

            float geom_score = 0.0f;
            if(!validateFaceRect(f, frame_bgr.size(), &geom_score))
                continue;

            float final_score = 1000.0f * det_score + geom_score;
            if(final_score > best_score)
            {
                best_score = final_score;
                best_face = f;
            }
        }

        if(best_score < 0.0f)
            return false;

        face = best_face;
        return true;
    }
    catch(const cv::Exception &e)
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "YuNet detect failed: %s",
            e.what());
        RCLCPP_WARN(this->get_logger(), "Disabling YuNet and falling back to Haar.");
        use_yunet_ = false;
        return false;
    }
}

bool FaceTracker::detectFaceYuNetInRoi(const cv::Mat &frame_bgr, const cv::Rect &roi, cv::Rect &face)
{
    cv::Rect bounded_roi = roi & cv::Rect(0, 0, frame_bgr.cols, frame_bgr.rows);
    if(bounded_roi.width <= 0 || bounded_roi.height <= 0)
        return false;

    cv::Mat roi_img = frame_bgr(bounded_roi).clone();

    if(!yunet_)
        return false;

    yunet_->setInputSize(roi_img.size());

    cv::Mat detections;
    yunet_->detect(roi_img, detections);

    if(detections.empty() || detections.rows == 0)
        return false;

    float best_score = -1.0f;
    cv::Rect best_face;

    for(int i = 0; i < detections.rows; ++i)
    {
        float x = detections.at<float>(i, 0);
        float y = detections.at<float>(i, 1);
        float w = detections.at<float>(i, 2);
        float h = detections.at<float>(i, 3);
        float det_score = detections.at<float>(i, 14);

        cv::Rect f(
            bounded_roi.x + static_cast<int>(std::round(x)),
            bounded_roi.y + static_cast<int>(std::round(y)),
            static_cast<int>(std::round(w)),
            static_cast<int>(std::round(h)));

        float geom_score = 0.0f;
        if(!validateFaceRect(f, frame_bgr.size(), &geom_score))
            continue;

        float final_score = 1000.0f * det_score + geom_score;
        if(final_score > best_score)
        {
            best_score = final_score;
            best_face = f;
        }
    }

    if(best_score < 0.0f)
        return false;

    face = best_face;
    return true;
}

void FaceTracker::resetKalmanToFace(const cv::Rect &face)
{
    float cx = face.x + face.width * 0.5f;
    float cy = face.y + face.height * 0.5f;

    kalman_.statePost.at<float>(0) = cx;
    kalman_.statePost.at<float>(1) = cy;
    kalman_.statePost.at<float>(2) = 0.0f;
    kalman_.statePost.at<float>(3) = 0.0f;

    kalman_.statePre = kalman_.statePost.clone();
}

cv::Point2f FaceTracker::predictKalmanCenter()
{
    cv::Mat prediction = kalman_.predict();
    return cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
}

cv::Point2f FaceTracker::correctKalmanCenter(const cv::Point2f &meas)
{
    cv::Mat measurement(2, 1, CV_32F);
    measurement.at<float>(0) = meas.x;
    measurement.at<float>(1) = meas.y;

    cv::Mat estimated = kalman_.correct(measurement);
    return cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
}

cv::Rect FaceTracker::buildLocalRoiFromPrediction(const cv::Size &frame_size, int scale) const
{
    if(tracked_face_.width <= 0 || tracked_face_.height <= 0)
        return cv::Rect();

    float cx = kalman_.statePost.at<float>(0);
    float cy = kalman_.statePost.at<float>(1);

    int roi_w = tracked_face_.width * scale;
    int roi_h = tracked_face_.height * scale;

    int roi_x = static_cast<int>(std::round(cx - roi_w / 2.0f));
    int roi_y = static_cast<int>(std::round(cy - roi_h / 2.0f));

    roi_x = std::max(0, roi_x);
    roi_y = std::max(0, roi_y);
    roi_w = std::min(frame_size.width - roi_x, roi_w);
    roi_h = std::min(frame_size.height - roi_y, roi_h);

    if(roi_w <= 0 || roi_h <= 0)
        return cv::Rect();

    return cv::Rect(roi_x, roi_y, roi_w, roi_h);
}

void FaceTracker::resetTrackingState()
{
    tracker_initialized_ = false;
    missed_rechecks_ = 0;
    candidate_valid_ = false;
    candidate_hits_ = 0;
    smoothed_face_valid_ = false;
    tracker_confidence_ = 0.0f;
    tracking_state_ = TrackingState::SEARCH;
    local_roi_ = cv::Rect();
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

        // 1) ROI locale prédite si on a une ancienne cible
        cv::Rect predicted_roi = buildLocalRoiFromPrediction(frame.size(), 3);
        if(predicted_roi.width > 0 && predicted_roi.height > 0)
        {
            if(use_yunet_)
                found_face = detectFaceYuNetInRoi(frame, predicted_roi, face);

            if(!found_face && use_haar_fallback_)
            {
                cv::Mat gray_local = gray(predicted_roi);
                cv::Rect local_face;
                if(detectFaceHaar(gray_local, local_face))
                {
                    face = cv::Rect(
                        predicted_roi.x + local_face.x,
                        predicted_roi.y + local_face.y,
                        local_face.width,
                        local_face.height);
                    found_face = true;
                }
            }
        }

        // 2) Si rien trouvé : recherche globale
        if(!found_face)
        {
            if(use_yunet_)
                found_face = detectFaceYuNet(frame, face);

            if(!found_face && use_haar_fallback_)
                found_face = detectFaceHaar(gray, face);
        }

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

                resetKalmanToFace(tracked_face_);
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
        bool found_face = false;

        cv::Rect predicted_roi = buildLocalRoiFromPrediction(frame.size(), 3);
        if(predicted_roi.width > 0 && predicted_roi.height > 0)
        {
            if(use_yunet_)
                found_face = detectFaceYuNetInRoi(frame, predicted_roi, face);

            if(!found_face && use_haar_fallback_)
            {
                cv::Mat gray_local = gray(predicted_roi);
                cv::Rect local_face;
                if(detectFaceHaar(gray_local, local_face))
                {
                    face = cv::Rect(
                        predicted_roi.x + local_face.x,
                        predicted_roi.y + local_face.y,
                        local_face.width,
                        local_face.height);
                    found_face = true;
                }
            }
        }

        if(!found_face)
        {
            if(use_yunet_)
                found_face = detectFaceYuNet(frame, face);

            if(!found_face && use_haar_fallback_)
                found_face = detectFaceHaar(gray, face);
        }

        if(found_face)
        {
            tracker_ = cv::TrackerKCF::create();
            tracker_->init(frame, face);

            tracked_face_ = face;
            previous_face_ = tracked_face_;

            smoothTrackedFaceSize(tracked_face_, frame.size());

            missed_rechecks_ = 0;
            tracker_confidence_ = std::min(1.0f, tracker_confidence_ + confidence_rise_);

            resetKalmanToFace(tracked_face_);
        }
        else
        {
            // // KCF can keep drifting on background patterns: require periodic Haar confirmation.
            missed_rechecks_++;
            if(missed_rechecks_ >= max_missed_rechecks_)
            {
                resetTrackingState();
            }
        }
    }
    else
    {
        cv::Rect prev_face = tracked_face_;
        bool ok = tracker_->update(frame,tracked_face_);

        if(!ok)
        {
            resetTrackingState();
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
                    resetTrackingState();
                }
            }

            // Reject impossible tracking windows.
            if(tracker_initialized_ &&
               (tracked_face_.width < 40 || tracked_face_.height < 40 ||
               tracked_face_.x < 0 || tracked_face_.y < 0 ||
               tracked_face_.x + tracked_face_.width > frame.cols ||
               tracked_face_.y + tracked_face_.height > frame.rows))
            {
                resetTrackingState();
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

        predictKalmanCenter();
        cv::Point2f estimated = correctKalmanCenter({cx, cy});

        float px = estimated.x;
        float py = estimated.y;

        u = px - image_width_/2;
        v = py - image_height_/2;

        distance = computeDistance(tracked_face_);

        if(distance <= 0.0f || !std::isfinite(distance))
        {
            face_detected = false;
            resetTrackingState();
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