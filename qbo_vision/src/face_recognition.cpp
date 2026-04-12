#include "face_recognition.hpp"

std::string majorityVote(const std::deque<std::string> & history);

FaceRecognitionNode::FaceRecognitionNode()
: Node("qbo_face_recognition"),
  image_sub_(this, "camera_left/image_raw"),
  face_sub_(this, "/qbo_face_tracking/face_observation"),
  last_recognition_time_(this->now()),
  diagnostic_updater_(this),
  last_face_seen_(this->now()),
  current_face_name_("unknown"),
  current_face_similarity_(0.0f),
  current_face_present_(false),
  model_loaded_(false),
  total_recognitions_(0),
  successful_recognitions_(0)
{
    // Augmentation de la queue pour éviter les pertes de synchronisation au démarrage
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(50),  // Augmenté de 10 à 50
        image_sub_,
        face_sub_);

    sync_->registerCallback(
        std::bind(&FaceRecognitionNode::synchronizedCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "Message filters policy: ApproximateTime with queue size 50");

    result_pub_ = this->create_publisher<qbo_msgs::msg::FaceRecognitionResult>(
        "/qbo_face_recognition/result",
        10);

    std::string model_path =
    this->declare_parameter<std::string>(
        "sface_model",
        "/home/qbo-v2/qbo_ws/src/qbo_vision/config/models/face_recognition_sface_2021dec.onnx");

    try
    {
        recognizer_ = cv::FaceRecognizerSF::create(
            model_path,
            "",
            cv::dnn::DNN_BACKEND_DEFAULT,
            cv::dnn::DNN_TARGET_CPU);
        model_loaded_ = true;
    }
    catch(const cv::Exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load SFace model: %s", e.what());
        model_loaded_ = false;
    }

    recognition_cache_time_ =
        this->declare_parameter<double>("recognition_cache_time", 3.0);

    std::string db_path =
        this->declare_parameter<std::string>(
            "faces_path",
            "/home/qbo-v2/qbo_ws/src/qbo_vision/faces");

    db_path_ = db_path;

    if(!database_.load(db_path_, this->get_logger()))
    {
        RCLCPP_WARN(this->get_logger(),
            "Face database is empty or failed to load. Recognition will mark all faces as 'unknown'.");
    }
    else
    {
        auto persons = database_.getPersonNames();
        RCLCPP_INFO(this->get_logger(),
            "Face database loaded with %zu person(s): %s",
            persons.size(),
            persons.empty() ? "none" : persons[0].c_str());
    }

    RCLCPP_INFO(this->get_logger(),
        "SFace model loaded: %s",
        model_path.c_str());

    enroll_mode_ = false;
    enroll_samples_ = 0;

    enroll_target_samples_ =
        this->declare_parameter<int>("enroll_samples", 5);

    enroll_service_ =
        this->create_service<qbo_msgs::srv::StartEnrollPerson>(
            "/face_recognition/start_enroll",
            std::bind(
                &FaceRecognitionNode::startEnrollCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    list_persons_service_ =
        this->create_service<qbo_msgs::srv::ListPersons>(
            "/face_recognition/list_persons",
            std::bind(
                &FaceRecognitionNode::listPersonsCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    identity_window_ =
    this->declare_parameter<int>("identity_window", 5);

    // Setup diagnostics
    diagnostic_updater_.setHardwareID("face_recognition");
    diagnostic_updater_.add("Face Recognition Status", this, &FaceRecognitionNode::diagnosticCallback);
    last_face_seen_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Face recognition node started (sync mode)");
}

void FaceRecognitionNode::synchronizedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const qbo_msgs::msg::FaceObservation::ConstSharedPtr & face)
{
    auto now = this->now();

    // Log de diagnostic au démarrage pour vérifier que la callback est bien appelée
    static int callback_count = 0;
    if(callback_count < 5)
    {
        RCLCPP_INFO(this->get_logger(),
            "✅ Synchronized callback #%d: face_id=%u, track_id=%u, tracking_state=%u",
            ++callback_count, face->face_id, face->track_id, face->tracking_state);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(),
            "Synced callback received: face_id=%u, track_id=%u",
            face->face_id, face->track_id);
    }

    // Update diagnostics
    last_face_seen_ = now;
    current_face_present_ = true;

    // Cleanup old cache entries first (> 10 seconds)
    for(auto it = recognition_cache_.begin(); it != recognition_cache_.end(); )
    {
        if((now - it->second).seconds() > 10.0)
            it = recognition_cache_.erase(it);
        else
            ++it;
    }

    // Check basic conditions first (before updating timers)
    if(face->tracking_state != 3)
    {
        // Log plus visible pour comprendre pourquoi la reconnaissance ne se fait pas
        static uint8_t last_logged_state = 255;
        if(face->tracking_state != last_logged_state)
        {
            const char* state_names[] = {"DISABLED", "SEARCH", "CANDIDATE", "TRACKING"};
            RCLCPP_INFO(this->get_logger(),
                "⏸️  Recognition paused: tracker is in %s mode (need TRACKING for recognition)",
                face->tracking_state < 4 ? state_names[face->tracking_state] : "UNKNOWN");
            last_logged_state = face->tracking_state;
        }
        RCLCPP_DEBUG(this->get_logger(),
            "Skipping face_id=%u (tracking_state=%u)",
            face->face_id, face->tracking_state);
        return;
    }

    if(face->face_size < 80)
    {
        static int small_face_count = 0;
        if(small_face_count < 3)
        {
            RCLCPP_WARN(this->get_logger(),
                "⚠️  Face too small for recognition: %.0f pixels (minimum: 80)",
                face->face_size);
            small_face_count++;
        }
        RCLCPP_DEBUG(this->get_logger(),
            "Skipping face_id=%u (face_size=%.0f < 80)",
            face->face_id, face->face_size);
        return;
    }

    // Check if this face was recently processed
    auto it = recognition_cache_.find(face->face_id);
    if(it != recognition_cache_.end())
    {
        double elapsed = (now - it->second).seconds();
        if(elapsed < recognition_cache_time_)
        {
            RCLCPP_DEBUG(this->get_logger(),
                "Skipping face_id=%u (cached %.1fs ago, threshold=%.1fs)",
                face->face_id, elapsed, recognition_cache_time_);
            return;
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                "Reprocessing face_id=%u (cached %.1fs ago > %.1fs threshold)",
                face->face_id, elapsed, recognition_cache_time_);
        }
    }

    // Global rate limiting (only applied after all other checks pass)
    if((now - last_recognition_time_).seconds() < 0.2)
    {
        RCLCPP_DEBUG(this->get_logger(),
            "Global rate limit: %.3fs since last recognition",
            (now - last_recognition_time_).seconds());
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;

    cv::Mat face_box(1, 15, CV_32F);

    face_box.at<float>(0) = face->x;
    face_box.at<float>(1) = face->y;
    face_box.at<float>(2) = face->width;
    face_box.at<float>(3) = face->height;

    for(int i=0;i<10;i++)
        face_box.at<float>(4+i) = face->landmarks[i];

    cv::Mat aligned_face;

    recognizer_->alignCrop(frame, face_box, aligned_face);

    cv::Mat embedding;

    recognizer_->feature(aligned_face, embedding);

    RCLCPP_DEBUG(this->get_logger(),
        "Embedding extracted for face_id=%u (%d x %d)",
        face->face_id,
        embedding.rows,
        embedding.cols);

    if(enroll_mode_)
    {
        enroll_embeddings_.push_back(embedding.clone());

        enroll_samples_++;

        RCLCPP_INFO(
            this->get_logger(),
            "Enroll sample %d/%d",
            enroll_samples_,
            enroll_target_samples_);

        if(enroll_samples_ >= enroll_target_samples_)
        {
            // Save embeddings to disk
            if(database_.savePerson(enroll_name_, enroll_embeddings_, db_path_, this->get_logger()))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Enrollment completed for %s (%d samples saved)",
                    enroll_name_.c_str(),
                    enroll_samples_);
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to save enrollment for %s",
                    enroll_name_.c_str());
            }

            enroll_mode_ = false;
            enroll_embeddings_.clear();
        }

        return;
    }

    // Update timers now that we're actually doing recognition
    last_recognition_time_ = now;
    recognition_cache_[face->face_id] = now;

    RCLCPP_DEBUG(this->get_logger(),
        "Processing recognition for face_id=%u",
        face->face_id);

    // Match against database
    auto match = database_.match(embedding);
    std::string name = match.first;
    float similarity = match.second;

    // Update statistics
    total_recognitions_++;
    current_face_name_ = name;
    current_face_similarity_ = similarity;

    auto & history = identity_history_[face->face_id];

    history.push_back(name);

    if(history.size() > static_cast<size_t>(identity_window_))
        history.pop_front();

    RCLCPP_DEBUG(this->get_logger(),
        "Matching result: name=%s, similarity=%.3f",
        name.c_str(), similarity);

    // Publish result
    qbo_msgs::msg::FaceRecognitionResult result;

    result.header = face->header;
    result.face_id = face->face_id;

    if(similarity > 0.5)
    {
        result.known = true;
        result.name = majorityVote(history);
        result.similarity = similarity;

        // Track last positive recognition
        successful_recognitions_++;
        last_positive_recognition_[result.name] = now;
    }
    else
    {
        result.known = false;
        result.name = "unknown";
        result.similarity = similarity;
    }

    result.distance = face->distance;
    result.quality = face->quality;

    result_pub_->publish(result);

    // Update diagnostics
    diagnostic_updater_.force_update();

    RCLCPP_INFO(this->get_logger(),
        "Face recognized: id=%u, name=%s, distance=%.2f m",
        face->face_id,
        result.name.c_str(),
        face->distance);
}

void FaceRecognitionNode::startEnrollCallback(
    const std::shared_ptr<qbo_msgs::srv::StartEnrollPerson::Request> req,
    std::shared_ptr<qbo_msgs::srv::StartEnrollPerson::Response> res)
{
    if(req->name.empty())
    {
        res->success = false;
        RCLCPP_ERROR(this->get_logger(), "Person name cannot be empty");
        return;
    }

    enroll_mode_ = true;
    enroll_name_ = req->name;
    enroll_samples_ = 0;
    enroll_embeddings_.clear();

    res->success = true;

    RCLCPP_INFO(
        this->get_logger(),
        "Starting enrollment for: %s (target: %d samples)",
        enroll_name_.c_str(),
        enroll_target_samples_);
}

void FaceRecognitionNode::listPersonsCallback(
    const std::shared_ptr<qbo_msgs::srv::ListPersons::Request> /*req*/,
    std::shared_ptr<qbo_msgs::srv::ListPersons::Response> res)
{
    res->names = database_.getPersonNames();

    RCLCPP_INFO(this->get_logger(), "Listed %zu persons in database", res->names.size());
}

void FaceRecognitionNode::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    // Check model status
    if(!model_loaded_)
    {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SFace model not loaded");
        return;
    }

    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Face recognition operational");

    // Database information
    auto person_names = database_.getPersonNames();
    int total_embeddings = static_cast<int>(person_names.size());
    // Each person has at least one embedding (typically 5 for enrolled persons)

    stat.add("Database present", !person_names.empty() ? "Yes" : "No");
    stat.add("Registered persons", static_cast<int>(person_names.size()));
    stat.add("Total embeddings", total_embeddings);

    // List registered persons with last positive recognition
    std::string persons_list = "";
    for(size_t i = 0; i < person_names.size(); ++i)
    {
        const auto & name = person_names[i];
        persons_list += name;

        auto it = last_positive_recognition_.find(name);
        if(it != last_positive_recognition_.end())
        {
            auto elapsed = (this->now() - it->second).seconds();
            if(elapsed < 60)
                persons_list += " (" + std::to_string(static_cast<int>(elapsed)) + "s ago)";
            else if(elapsed < 3600)
                persons_list += " (" + std::to_string(static_cast<int>(elapsed / 60)) + "m ago)";
            else
                persons_list += " (" + std::to_string(static_cast<int>(elapsed / 3600)) + "h ago)";
        }
        else
        {
            persons_list += " (never recognized)";
        }

        if(i < person_names.size() - 1)
            persons_list += ", ";
    }
    stat.add("Persons list", persons_list.empty() ? "none" : persons_list);

    // Current face detection status
    auto time_since_last_face = (this->now() - last_face_seen_).seconds();
    if(time_since_last_face < 2.0)
    {
        stat.add("Face present", "Yes");
        stat.add("Current face", current_face_name_);
        stat.add("Recognition score", current_face_similarity_);
        stat.add("Face recognized", current_face_similarity_ > 0.5 ? "Yes" : "No");
    }
    else
    {
        stat.add("Face present", "No");
        stat.add("Time since last face", std::to_string(static_cast<int>(time_since_last_face)) + "s");
    }

    // Enrollment status
    if(enroll_mode_)
    {
        stat.add("Enrollment active", "Yes");
        stat.add("Enrolling person", enroll_name_);
        stat.add("Samples collected", std::to_string(enroll_samples_) + "/" + std::to_string(enroll_target_samples_));
    }
    else
    {
        stat.add("Enrollment active", "No");
    }

    // Recognition statistics
    stat.add("Total recognitions", total_recognitions_);
    stat.add("Successful recognitions", successful_recognitions_);
    if(total_recognitions_ > 0)
    {
        float success_rate = 100.0f * static_cast<float>(successful_recognitions_) / static_cast<float>(total_recognitions_);
        stat.add("Success rate", std::to_string(static_cast<int>(success_rate)) + "%");
    }
    else
    {
        stat.add("Success rate", "N/A");
    }

    // Configuration
    stat.add("Recognition cache time", std::to_string(recognition_cache_time_) + "s");
    stat.add("Identity smoothing window", identity_window_);
}

std::string majorityVote(const std::deque<std::string> & history)
{
    std::unordered_map<std::string,int> count;

    for(auto & name : history)
        count[name]++;

    int best = 0;
    std::string best_name = "unknown";

    for(auto & p : count)
    {
        if(p.second > best)
        {
            best = p.second;
            best_name = p.first;
        }
    }

    return best_name;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FaceRecognitionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}