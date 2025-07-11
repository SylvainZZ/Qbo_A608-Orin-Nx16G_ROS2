#include "face_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

FaceDetector::FaceDetector(const rclcpp::NodeOptions & options)
: Node("qbo_face_detector", options)
{
    RCLCPP_INFO(this->get_logger(), "🧠 Initialisation du node FaceDetector");

    this->declare_and_get_parameters();  // on crée une méthode à part pour la lisibilité
    initializeKalmanFilter();            // idem, tu peux la conserver telle quelle

    // it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    // Abonnements
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&FaceDetector::imageCallback, this, std::placeholders::_1)
    );

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10,
        std::bind(&FaceDetector::infoCallback, this, std::placeholders::_1)
    );

    // Publishers
    face_position_and_size_pub_ = this->create_publisher<qbo_msgs::msg::FacePosAndDist>("/qbo_face_tracking/face_pos_and_dist", 10);
    // face_pub_ = this->create_publisher<std_msgs::msg::String>("/face_name", 10);
    // face_pub_ = it_->advertise("/face_name", 1);
    nose_color_pub_ = this->create_publisher<qbo_msgs::msg::Nose>("/cmd_nose", 10);

    // viewer_image_pub_ = it_->advertise("/face_detector/debug_image", 1);

    // Services
    client_recognize_ = this->create_client<qbo_msgs::srv::RecognizeFace>("recognize_face");
    client_get_name_ = this->create_client<qbo_msgs::srv::GetName>("get_name");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FaceDetector::delayedInitImageTransport, this)
    );
    last_image_time_ = this->now();  // Compatible avec get_clock()

    RCLCPP_INFO(this->get_logger(), "✅ Initialisation complète du face_detector");

    onInit();
}

FaceDetector::~FaceDetector()
{
    RCLCPP_INFO(this->get_logger(), "💤 Arrêt du node FaceDetector");
}

void FaceDetector::declare_and_get_parameters()
{
    // Déclaration des paramètres avec valeurs par défaut
    this->declare_parameter("face_classifier_path", std::string("haarcascade_frontalface_alt.xml"));
    this->declare_parameter("alternative_face_classifier_path", std::string("haarcascade_frontalface_default.xml"));
    this->declare_parameter("lbp_classifier_path", std::string("lbpcascade_frontalface.xml"));

    this->declare_parameter("check_Haar", 1);
    this->declare_parameter("check_track_obj", 1);
    this->declare_parameter("undetected_threshold", 5);
    this->declare_parameter("distance_threshold", 1.2);
    this->declare_parameter("send_to_face_recognizer", false);
    this->declare_parameter("print_recognized_face", true);

    // Lecture des paramètres
    this->get_parameter("face_classifier_path", face_classifier_path_);
    this->get_parameter("alternative_face_classifier_path", alternative_face_classifier_path_);
    this->get_parameter("lbp_classifier_path", lbp_classifier_path_);

    this->get_parameter("check_Haar", check_Haar_);
    this->get_parameter("check_track_obj", check_track_obj_);
    this->get_parameter("undetected_threshold", undetected_threshold_);
    this->get_parameter("distance_threshold", distance_threshold_);
    this->get_parameter("send_to_face_recognizer", send_to_face_recognizer_);
    this->get_parameter("print_recognized_face", print_recognized_face_);
}

void FaceDetector::delayedInitImageTransport()
{
    // Ne faire cela qu’une fois
    if (it_) return;

    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    face_pub_ = it_->advertise("/face_name", 1);
    viewer_image_pub_ = it_->advertise("/face_detector/debug_image", 1);

    RCLCPP_INFO(this->get_logger(), "📷 image_transport initialisé");
}

void FaceDetector::onInit()
{
    RCLCPP_INFO(this->get_logger(), "📦 Initialisation des classifieurs de visage");

    exist_alternative_ = false;

    // Chargement du classifieur principal
    if (!face_classifier_.load(face_classifier_path_)) {
        RCLCPP_ERROR(this->get_logger(), "❌ Échec du chargement du classifieur principal : %s", face_classifier_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "✔️ Classifieur principal chargé : %s", face_classifier_path_.c_str());
    }

    // Classifieur alternatif
    if (!alternative_face_classifier_path_.empty()) {
        if (alternative_face_classifier_.load(alternative_face_classifier_path_)) {
            RCLCPP_INFO(this->get_logger(), "✔️ Classifieur alternatif chargé : %s", alternative_face_classifier_path_.c_str());
            exist_alternative_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Impossible de charger le classifieur alternatif : %s", alternative_face_classifier_path_.c_str());
        }
    }

    // Classifieur LBP
    if (!lbp_classifier_path_.empty()) {
        if (lbp_classifier_.load(lbp_classifier_path_)) {
            RCLCPP_INFO(this->get_logger(), "✔️ Classifieur LBP chargé : %s", lbp_classifier_path_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Impossible de charger le classifieur LBP : %s", lbp_classifier_path_.c_str());
        }
    }

    // Initialisation des variables de suivi
    track_object_ = false;
    face_detected_bool_ = false;
    loop_counter_ = 0;
    face_ratio_ = 0.0;
    undetected_count_ = 0;

    cam_shift_detected_count_ = 0;
    cam_shift_undetected_count_ = 0;
    default_pos_ = cv::Point2d(0.5, 0.5);
    name_detected_ = "";

    head_distances_.clear();
}


void FaceDetector::initializeKalmanFilter()
{
    kalman_filter_.init(4, 2, 0); // 4 états, 2 mesures

    // Transition matrix A
    kalman_filter_.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Mesure Matrix H
    kalman_filter_.measurementMatrix = cv::Mat::eye(2, 4, CV_32F);

    // Covariance des mesures R
    kalman_filter_.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * 0.01;

    // Bruit du processus Q
    kalman_filter_.processNoiseCov = cv::Mat::eye(4, 4, CV_32F) * 0.03;

    // Estimation de l’erreur P
    kalman_filter_.errorCovPost = cv::Mat::eye(4, 4, CV_32F);

    // Initialisation de l’état
    kalman_filter_.statePost = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);

    kalman_predicted_state_ = kalman_filter_.statePost.clone();

    RCLCPP_DEBUG(this->get_logger(), "📈 Filtre de Kalman initialisé");
}


void FaceDetector::infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    if (p_.empty())
    {
        RCLCPP_INFO(this->get_logger(), "📷 Calibration camera reçue, extraction de la matrice P");

        cv::Mat p(3, 4, CV_64F);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                p.at<double>(i, j) = info->p[4 * i + j];  // ⚠️ c'est .p (minuscule) en ROS2
            }
        }

        // Stocker seulement les 3x3 pour la focale
        cv::Mat k_matrix = p(cv::Rect(0, 0, 3, 3));
        k_matrix.convertTo(p_, CV_32F);

        RCLCPP_INFO(this->get_logger(), "✅ Matrice K extraite :\n[ %f %f %f ]\n[ %f %f %f ]\n[ %f %f %f ]",
                    p_.at<float>(0, 0), p_.at<float>(0, 1), p_.at<float>(0, 2),
                    p_.at<float>(1, 0), p_.at<float>(1, 1), p_.at<float>(1, 2),
                    p_.at<float>(2, 0), p_.at<float>(2, 1), p_.at<float>(2, 2));

        fx_ = static_cast<float>(info->k[0]);


        // Tu peux aussi désabonner ici si tu veux
        info_sub_.reset();
    }
}


// Définition avec flag
// void classifierDetect(cv::Mat image,
//                       std::vector<cv::Rect> &detections,
//                       cv::CascadeClassifier &classifier,
//                       int flag,
//                       cv::Size size)
// {
//     // classifier.detectMultiScale(image, detections, 1.1, 3, flag, size);
//     classifier.detectMultiScale(image, detections, 1.1, 5, flag, size);
// }
void classifierDetect(cv::Mat image,
                      std::vector<cv::Rect> &detections,
                      cv::CascadeClassifier &classifier,
                      int flag,
                      cv::Size size)
{
    std::vector<cv::Rect> raw_detections;

    classifier.detectMultiScale(
        image,
        raw_detections,
        1.1,         // facteur d’échelle
        5,           // minNeighbors (plus haut = plus strict)
        flag,
        size,
        cv::Size(300, 300)  // maxSize : ignore les trop grandes détections
    );

    // filtre post-détection (facultatif)
    detections.clear();
    for (const auto &rect : raw_detections)
    {
        if (rect.width >= 40 && rect.width <= 250 &&
            rect.height >= 40 && rect.height <= 250)
        {
            detections.push_back(rect);
        }
    }
}


void FaceDetector::sendToRecognizer()
{
    // Fonction à implémenter plus tard
}



void FaceDetector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image_ptr)
{
    // 🧠 Throttle : ignorer si appel trop rapproché
    if ((this->now() - last_image_time_).seconds() < throttle_interval_sec_) {
        return;
    }
    last_image_time_ = this->get_clock()->now();

    loop_counter_++;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image_received = (cv_ptr->image).clone();
    image_size_ = image_received.size();
    image_width_ = image_size_.width;
    image_height_ = image_size_.height;

    // Structure pour le message Nez
    auto nose = qbo_msgs::msg::Nose();
    nose.header.stamp = this->now();
    nose.color = 0;

    std::string detection_type;

    // ============ 🧠 Détection avec CamShift ou Haar ============

    // if (face_detected_bool_)  // Si un visage a été détecté précédemment → suivi CamShift
    // {
    //     if (detectFaceCamShift(image_received) != 0)
    //     {
    //         face_detected_bool_ = false;
    //         cam_shift_detected_count_ = 0;
    //         cam_shift_undetected_count_++;
    //     }
    //     else
    //     {
    //         detection_type = "CAMSHIFT";
    //         cam_shift_detected_count_++;
    //         cam_shift_undetected_count_ = 0;
    //     }
    // }

    if (!face_detected_bool_)  // Sinon → détection Haar classique
    {
        if (loop_counter_ % haar_detection_skip_ == 0)
        {
            std::vector<cv::Rect> faces_roi;
            detectFacesHaar(image_received, faces_roi);

            if (!faces_roi.empty())
            {
                face_detected_bool_ = true;
                track_object_ = false;
                detected_face_roi_ = faces_roi[0];
                detected_face_ = cv_ptr->image(detected_face_roi_);
                face_ratio_ = 1.3 * detected_face_roi_.height / detected_face_roi_.width;

                detection_type = "HAAR";
            }
        }
    }

    if (!face_detected_bool_ && exist_alternative_)  // Fallback : cascade alternative
    {
        std::vector<cv::Rect> faces_roi;
        detectFacesAltHaar(image_received, faces_roi);

        if (!faces_roi.empty())
        {
            face_detected_bool_ = true;
            track_object_ = false;
            detected_face_roi_ = faces_roi[0];
            detected_face_ = cv_ptr->image(detected_face_roi_);
            face_ratio_ = 1.3 * detected_face_roi_.height / detected_face_roi_.width;

            detection_type = "HAAR_ALT";
        }
    }

    // ============ 📈 Kalman filter ============

    // kalman_filter_.predict();
    if (kalman_filter_.statePre.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Kalman filter not initialized! Skipping prediction.");
        return;
    }
    kalman_filter_.predict();

    float speed_x = std::abs(kalman_filter_.statePost.at<float>(2, 0));
    float speed_y = std::abs(kalman_filter_.statePost.at<float>(3, 0));
    float dynamic_factor = std::clamp(speed_x + speed_y, 5.0f, 50.0f);

    kalman_filter_.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32FC1) * dynamic_factor;

    // RCLCPP_INFO(this->get_logger(), "📉 Kalman dynamic noise: %.2f", dynamic_factor);

    if (face_detected_bool_)
    {
        if (undetected_count_ >= undetected_threshold_)
        {
            cv::Mat new_state(4, 1, CV_32FC1);
            new_state.at<float>(0, 0) = float(detected_face_roi_.x + detected_face_roi_.width / 2.0);
            new_state.at<float>(1, 0) = float(detected_face_roi_.y + detected_face_roi_.height / 2.0);
            new_state.at<float>(2, 0) = 0.0;
            new_state.at<float>(3, 0) = 0.0;

            new_state.copyTo(kalman_filter_.statePre);
            new_state.copyTo(kalman_filter_.statePost);
        }
        else
        {
            float u_vel = float(detected_face_roi_.x + detected_face_roi_.width / 2.0f) - kalman_filter_.statePost.at<float>(0, 0);
            float v_vel = float(detected_face_roi_.y + detected_face_roi_.height / 2.0f) - kalman_filter_.statePost.at<float>(1, 0);

            cv::Mat measures(2, 1, CV_32FC1);
            measures.at<float>(0, 0) = float(detected_face_roi_.x + detected_face_roi_.width / 2.0f);
            measures.at<float>(1, 0) = float(detected_face_roi_.y + detected_face_roi_.height / 2.0f);
            // measures.at<float>(2, 0) = u_vel;
            // measures.at<float>(3, 0) = v_vel;

            // RCLCPP_INFO(this->get_logger(),
            //     "🔎 Kalman matrices:\n"
            //     "  statePre: %dx%d\n"
            //     "  statePost: %dx%d\n"
            //     "  transitionMatrix: %dx%d\n"
            //     "  measurementMatrix: %dx%d\n"
            //     "  measurementNoiseCov: %dx%d\n"
            //     "  processNoiseCov: %dx%d\n"
            //     "  measures: %dx%d",
            //     kalman_filter_.statePre.rows, kalman_filter_.statePre.cols,
            //     kalman_filter_.statePost.rows, kalman_filter_.statePost.cols,
            //     kalman_filter_.transitionMatrix.rows, kalman_filter_.transitionMatrix.cols,
            //     kalman_filter_.measurementMatrix.rows, kalman_filter_.measurementMatrix.cols,
            //     kalman_filter_.measurementNoiseCov.rows, kalman_filter_.measurementNoiseCov.cols,
            //     kalman_filter_.processNoiseCov.rows, kalman_filter_.processNoiseCov.cols,
            //     measures.rows, measures.cols
            // );

            if (kalman_filter_.statePre.empty() || kalman_filter_.statePost.empty())
            {
                RCLCPP_WARN(this->get_logger(), "⚠️ Kalman filter not initialized properly. Skipping correction.");
            }
            else
            {
                kalman_filter_.correct(measures);
            }
        }
    }
    else
    {
        kalman_filter_.statePre.copyTo(kalman_filter_.statePost);
        kalman_filter_.errorCovPre.copyTo(kalman_filter_.errorCovPost);

        if (undetected_count_ > undetected_threshold_ * 3)
        {
            RCLCPP_WARN(this->get_logger(), "🔁 Face lost too long. Resetting Kalman filter.");
            kalman_filter_.statePre = cv::Mat::zeros(4, 1, CV_32FC1);
            kalman_filter_.statePost = cv::Mat::zeros(4, 1, CV_32FC1);
        }
    }

    // ============ 📏 Calcul distance tête (moyenne sur 10 mesures) ============

    float head_distance;
    if (!face_detected_bool_)
    {
        head_distance = head_distances_.empty() ? 5.0f : head_distances_.front();
    }
    else
    {
        head_distance = calcDistanceToHead(detected_face_, kalman_filter_);
    }

    if (head_distances_.empty())
    {
        head_distances_.assign(10, head_distance);
    }
    else
    {
        head_distances_.pop_back();
        head_distances_.insert(head_distances_.begin(), head_distance);
    }

    head_distance = std::accumulate(head_distances_.begin(), head_distances_.end(), 0.0f) / head_distances_.size();

    if (!face_detected_bool_)
        undetected_count_++;
    else
        undetected_count_ = 0;

    // ============ 🎯 Création du message FacePosAndDist ============

    qbo_msgs::msg::FacePosAndDist message;
    message.image_width = image_width_;
    message.image_height = image_height_;
    message.type_of_tracking = detection_type;

    if (undetected_count_ < undetected_threshold_)
    {
        message.face_detected = true;
        message.u = kalman_filter_.statePost.at<float>(0, 0) - image_width_ / 2.0f;
        message.v = kalman_filter_.statePost.at<float>(1, 0) - image_height_ / 2.0f;
        message.distance_to_head = head_distance;
    }
    else
    {
        message.face_detected = false;
        message.u = default_pos_.x;
        message.v = default_pos_.y;
    }

    // ============ 📤 Publication image de visage et reconnaissance ============

    if (face_detected_bool_)
    {
        auto face_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, detected_face_);
        face_msg.header.stamp = this->get_clock()->now();
        // face_pub_->publish(*face_msg.toImageMsg());
        face_pub_.publish(face_msg.toImageMsg());

        if (send_to_face_recognizer_)
        {
            sendToRecognizer();  // ⚠️ Cette fonction devra être migrée aussi
        }

        nose.color = 4; // Visage détecté = Bleu
        if (head_distance < distance_threshold_)
        {
            nose.color = 2; // Tête proche = Vert
        }
    }

    // ============ 🟠 Publication couleur du nez ============

    nose_color_pub_->publish(nose);

    // ============ 🖼️ Image viewer avec overlays (détection + vitesse) ============

    int pred_x = static_cast<int>(message.u + message.image_width / 2);
    int pred_y = static_cast<int>(message.v + message.image_height / 2);
    cv::Point pred_kal(pred_x, pred_y);

    cv::Point tip_u(pred_kal.x + kalman_filter_.statePost.at<float>(2, 0), pred_kal.y);
    cv::Point tip_v(pred_kal.x, pred_kal.y + kalman_filter_.statePost.at<float>(3, 0));

    cv::circle(image_received, pred_kal, 3, cv::Scalar(0, 0, 255), 3);
    cv::rectangle(image_received, detected_face_roi_, cv::Scalar(255, 0, 0), 2);

    if (tip_u.x >= 0 && tip_u.y >= 0 && tip_u.x < image_received.cols && tip_u.y < image_received.rows)
        cv::line(image_received, pred_kal, tip_u, cv::Scalar(0, 255, 0), 2);

    if (tip_v.x >= 0 && tip_v.y >= 0 && tip_v.x < image_received.cols && tip_v.y < image_received.rows)
        cv::line(image_received, pred_kal, tip_v, cv::Scalar(255, 0, 0), 2);

    if (print_recognized_face_)
    {
        cv::putText(image_received, name_detected_, {40, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 3);
    }

    // Publication de l’image avec les éléments d’overlay
    auto viewer_msg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, image_received);
    viewer_msg.header.stamp = this->get_clock()->now();
    viewer_image_pub_.publish(*viewer_msg.toImageMsg());

    // Publication finale du message de position
    face_position_and_size_pub_->publish(message);

    // ============ ♻️ Mise à jour dynamique de Haar / CamShift ============

    if (dynamic_check_haar_)
    {
        if (undetected_count_ > 10)
        {
            check_Haar_ = 2;
            cam_shift_detected_count_ = 0;
            cam_shift_undetected_count_ = 0;
        }
        else if (cam_shift_undetected_count_ > 3)
        {
            check_Haar_ = std::max(2, static_cast<int>(check_Haar_ / 2));
        }
        else if (cam_shift_detected_count_ > 10)
        {
            check_Haar_ = std::min(100, static_cast<int>(check_Haar_ + 5));
        }

        track_object_ = (loop_counter_ % check_Haar_ == 0) ? false : track_object_;
    }

    // ============ 🖨️ Console debug ============

    // 🖨️ Console debug réduit
    if (loop_counter_ % debug_print_every_n_ == 0)
    {
        if (face_detected_bool_)
        {
            if (dynamic_check_haar_)
            {
                RCLCPP_INFO(this->get_logger(),
                    "FACE DETECTED → u=%.0f v=%.0f, dist=%.2f, checkHaar=%u, type=%s, alt=%s",
                    message.u, message.v, head_distance,
                    check_Haar_, detection_type.c_str(),
                    exist_alternative_ ? "true" : "false"
                );
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),
                    "FACE DETECTED → u=%.0f v=%.0f, dist=%.2f, type=%s",
                    message.u, message.v, head_distance,
                    detection_type.c_str()
                );
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                "NO FACE. Using Haar. checkHaar=%u, alt=%s",
                check_Haar_, exist_alternative_ ? "true" : "false"
            );
        }
    }

    // ============ 🔁 Réinitialisation périodique ============

    if (loop_counter_ % check_Haar_ == 0)
        face_detected_bool_ = false;

    if (loop_counter_ % check_track_obj_ == 0)
        track_object_ = false;

    loop_counter_++;
}

unsigned int FaceDetector::detectFaceCamShift(cv::Mat img)
{
    // === Préparation des matrices ===
    cv::Mat hsv, mask, grayscale, backproject, histimg = cv::Mat::zeros(img.size(), CV_8UC3);
    cv::Mat hue_planes[3], hue;
    cv::MatND hist;

    int vmin = 10, vmax = 256, smin = 30;
    float max_face_deslocation = detected_face_roi_.height / 3.0f;
    int mean_score_threshold = 70;

    unsigned int max_distance_width = detected_face_roi_.width / 3;
    unsigned int max_distance_height = detected_face_roi_.height / 3;

    // === Conversion et masquage ===
    cv::cvtColor(img, hsv, cv::COLOR_RGB2HSV);
    cv::cvtColor(img, grayscale, cv::COLOR_RGB2GRAY);
    cv::equalizeHist(grayscale, grayscale);
    cv::inRange(hsv, cv::Scalar(0, smin, std::min(vmin, vmax)), cv::Scalar(180, 256, std::max(vmin, vmax)), mask);
    cv::split(hsv, hue_planes);
    hue = hue_planes[0];

    cv::Rect face_roi = detected_face_roi_;
    if (!track_object_)
    {
        if (detected_face_roi_.width <= 0 || detected_face_roi_.height <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid detected_face_roi dimensions.");
            return 1;
        }

        cv::Rect little_face = face_roi;
        little_face.width -= face_roi.width / 3;
        little_face.height -= face_roi.height / 3;
        little_face.x += face_roi.width / 6;
        little_face.y += face_roi.height / 6;

        if (little_face.x < 0 || little_face.y < 0 ||
            little_face.x + little_face.width > grayscale.cols ||
            little_face.y + little_face.height > grayscale.rows)
        {
            RCLCPP_WARN(this->get_logger(), "Little face ROI is out of bounds.");
            return 1;
        }

        cv::Mat hueRoi = grayscale(little_face);
        cv::Mat maskRoi = mask(little_face);

        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        cv::calcHist(&hueRoi, 1, 0, maskRoi, hist, 1, &histSize, &histRange, true, false);

        double max_val = 0.0;
        cv::minMaxLoc(hist, 0, &max_val);
        hist.convertTo(hist, hist.type(), max_val ? 255.0 / max_val : 0.0, 0);

        track_object_ = true;
    }

    if (hue.empty() || hist.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Hue or Histogram empty, aborting CAMShift.");
        return 1;
    }

    // === Backprojection et filtrage ===
    int channels[] = {0};
    float hranges[] = {0, 180};
    const float* phranges = hranges;

    cv::calcBackProject(&hue, 1, channels, hist, backproject, &phranges);
    backproject &= mask;

    // === Tracking avec CamShift ===
    cv::Rect track_window = detected_face_roi_;
    cv::RotatedRect track_box = cv::CamShift(backproject, track_window, cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1));

    if (track_box.size.height <= 0 || track_box.size.width <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "CAMShift box size invalid.");
        return 1;
    }

    face_roi = track_box.boundingRect();
    face_roi.height = face_ratio_ * face_roi.width;

    // === Clamping aux bords de l’image ===
    face_roi &= cv::Rect(0, 0, img.cols - 3, img.rows - 3);

    if (face_roi.width <= 0 || face_roi.height <= 0)
        return 1;

    // === Score moyen sur backproject ===
    double mean_score = 0.0;
    if (face_roi.width > 20 && face_roi.height > 20)
    {
        cv::Mat temp = backproject(face_roi);
        mean_score = static_cast<double>(cv::sum(temp)[0]) / (temp.rows * temp.cols);
    }

    if (mean_score < mean_score_threshold)
    {
        face_detected_bool_ = false;
        return 1;
    }

    // === Distance euclidienne centre visage ===
    float dx = (face_roi.x + face_roi.width / 2.0f) - (detected_face_roi_.x + detected_face_roi_.width / 2.0f);
    float dy = (face_roi.y + face_roi.height / 2.0f) - (detected_face_roi_.y + detected_face_roi_.height / 2.0f);
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist > max_face_deslocation)
    {
        face_detected_bool_ = false;
        return 1;
    }

    // === Contrôle de variation de taille brutale ===
    if (std::abs(face_roi.width - detected_face_roi_.width) > max_distance_width ||
        std::abs(face_roi.height - detected_face_roi_.height) > max_distance_height)
    {
        face_roi.x += face_roi.width / 2 - detected_face_roi_.width / 2;
        face_roi.y += face_roi.height / 2 - detected_face_roi_.height / 2;
        face_roi.width = detected_face_roi_.width;
        face_roi.height = detected_face_roi_.height;
    }

    if (face_roi.x < 0 || face_roi.y < 0 ||
        face_roi.x + face_roi.width >= img.cols ||
        face_roi.y + face_roi.height >= img.rows)
        return 1;

    // === Succès du CAMShift ===
    detected_face_roi_ = face_roi;
    detected_face_ = img(detected_face_roi_);
    face_detected_bool_ = true;

    return 0;
}


double FaceDetector::euclidDist(cv::Point2f pt1, cv::Point2f pt2)
{
    return std::hypot(pt1.x - pt2.x, pt1.y - pt2.y);
}

void FaceDetector::setFaceClassifierPath(std::string face_classifier_path)
{
    face_classifier_.load(face_classifier_path);
}

// Définition sans flag (valeur par défaut déjà définie dans le .hpp)
void classifierDetect(cv::Mat image,
                      std::vector<cv::Rect> &detections,
                      cv::CascadeClassifier &classifier,
                      cv::Size size)
{
    classifier.detectMultiScale(image, detections, 1.1, 3, cv::CASCADE_FIND_BIGGEST_OBJECT, size);
}

void classifierDetect(cv::Mat image,
                      std::vector<cv::Rect> &detections,
                      cv::CascadeClassifier &classifier)
{
    // Appel la version à 4 arguments avec une taille par défaut
    classifierDetect(image, detections, classifier, cv::Size(30, 30));
}

unsigned int FaceDetector::detectFacesHaar(cv::Mat image, std::vector<cv::Rect> &faces)
{
    classifierDetect(image, faces, face_classifier_);
    return faces.size();
}

unsigned int FaceDetector::detectFacesAltHaar(cv::Mat image, std::vector<cv::Rect> &faces)
{
    classifierDetect(image, faces, alternative_face_classifier_);
    return faces.size();
}

float FaceDetector::calcDistanceToHead(cv::Mat &head, cv::KalmanFilter &kalman_filter)
{
    const float REAL_FACE_HEIGHT = 0.21f;
    if (fx_ <= 0.0f) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Focale fx inconnue. Abandon du calcul de distance.");
        return -1.0f;
    }
    float distance = (fx_ * REAL_FACE_HEIGHT) / head.rows;
    RCLCPP_INFO(this->get_logger(), "Computed distance: %.2f m (face height = %d px, fx = %.2f)", distance, head.rows, fx_);

    // if (head.rows <= 0)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Head image has invalid height. Cannot compute distance.");
    //     return -1.0f;
    // }

    // float distance = (fx * REAL_FACE_HEIGHT) / head.rows;

    // RCLCPP_INFO(this->get_logger(), "Computed distance: %.2f m (face height = %d px, fx = %.2f)", distance, head.rows, fx);

    return distance;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);

    auto node = std::make_shared<FaceDetector>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}





