#include "face_tracker.hpp"

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
        1.1,
        8,
        cv::CASCADE_FIND_BIGGEST_OBJECT,
        cv::Size(80,80));

    if(faces.empty())
        return false;

    for(const auto &f : faces)
    {
        // 1️⃣ filtre ratio visage
        float ratio = (float)f.height / (float)f.width;

        if(ratio < 0.8 || ratio > 1.6)
            continue;

        // 2️⃣ rejet si trop proche des bords
        if(f.x < 10 ||
           f.y < 10 ||
           f.x + f.width > gray.cols - 10 ||
           f.y + f.height > gray.rows - 10)
            continue;

        // 3️⃣ cohérence avec le tracking précédent
        if(tracker_initialized_)
        {
            float dx = std::abs(f.x - tracked_face_.x);
            float dy = std::abs(f.y - tracked_face_.y);

            if(dx > 150 || dy > 150)
                continue;
        }

        face = f;
        return true;
    }

    return false;
}

float FaceTracker::computeDistance(const cv::Rect &face)
{
    if(fx_<=0)
        return -1;

    return (fx_ * REAL_FACE_HEIGHT) / face.height;
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

    if(!tracker_initialized_ || do_recheck)
    {
        cv::Rect face;

        if(detectFace(gray,face))
        {
            tracker_ = cv::TrackerKCF::create();
            tracker_->init(frame,face);

            tracked_face_ = face;

            tracker_initialized_ = true;
            missed_rechecks_ = 0;
        }
        else if(tracker_initialized_)
        {
            // KCF can keep drifting on background patterns: require periodic Haar confirmation.
            missed_rechecks_++;
            if(missed_rechecks_ >= max_missed_rechecks_)
            {
                tracker_initialized_ = false;
                missed_rechecks_ = 0;
            }
        }
    }
    else
    {
        bool ok = tracker_->update(frame,tracked_face_);

        if(!ok)
        {
            tracker_initialized_ = false;
            missed_rechecks_ = 0;
        }

        if(tracker_initialized_)
        {
            // Reject impossible tracking windows.
            if(tracked_face_.width < 40 || tracked_face_.height < 40 ||
               tracked_face_.x < 0 || tracked_face_.y < 0 ||
               tracked_face_.x + tracked_face_.width > frame.cols ||
               tracked_face_.y + tracked_face_.height > frame.rows)
            {
                tracker_initialized_ = false;
                missed_rechecks_ = 0;
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
    msg_out.type_of_tracking = tracker_initialized_ ? "KCF":"HAAR";

    face_pub_->publish(msg_out);
}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<FaceTracker>();

    rclcpp::spin(node);

    rclcpp::shutdown();
}