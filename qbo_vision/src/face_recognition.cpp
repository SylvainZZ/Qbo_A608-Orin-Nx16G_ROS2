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

    enrollment_progress_pub_ = this->create_publisher<qbo_msgs::msg::EnrollmentProgress>(
        "/qbo_face_recognition/enrollment_progress",
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

    min_quality_threshold_ =
        this->declare_parameter<double>("min_quality_threshold", 0.5);

    min_similarity_threshold_ =
        this->declare_parameter<double>("min_similarity_threshold", 0.6);

    min_samples_for_publication_ =
        this->declare_parameter<int>("min_samples_for_publication", 3);

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
    last_countdown_value_ = -1;

    enroll_target_samples_ =
        this->declare_parameter<int>("enroll_samples", 5);

    enrollment_sample_interval_ =
        this->declare_parameter<double>("enrollment_sample_interval", 5.0);

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

    remove_person_service_ =
        this->create_service<qbo_msgs::srv::RemovePerson>(
            "/face_recognition/remove_person",
            std::bind(
                &FaceRecognitionNode::removePersonCallback,
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

    // Filter by quality to reject false face detections (objects, noise, etc.)
    if(face->quality < min_quality_threshold_)
    {
        static int low_quality_count = 0;
        if(low_quality_count < 3)
        {
            RCLCPP_WARN(this->get_logger(),
                "⚠️  Face quality too low: %.3f (minimum: %.3f) - likely a false detection",
                face->quality, min_quality_threshold_);
            low_quality_count++;
        }
        RCLCPP_DEBUG(this->get_logger(),
            "Skipping face_id=%u (quality=%.3f < %.3f)",
            face->face_id, face->quality, min_quality_threshold_);
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
        // Check if enough time has passed since last sample
        if(enroll_samples_ > 0)
        {
            double elapsed_since_last_sample = (now - last_enrollment_sample_time_).seconds();
            if(elapsed_since_last_sample < enrollment_sample_interval_)
            {
                // Too soon, wait before capturing next sample
                double remaining = enrollment_sample_interval_ - elapsed_since_last_sample;
                int countdown = static_cast<int>(std::ceil(remaining));
                
                // Only publish when countdown value changes (once per second)
                if(countdown != last_countdown_value_)
                {
                    last_countdown_value_ = countdown;
                    
                    qbo_msgs::msg::EnrollmentProgress wait_msg;
                    wait_msg.person_name = enroll_name_;
                    wait_msg.current_sample = enroll_samples_;
                    wait_msg.target_samples = enroll_target_samples_;
                    wait_msg.current_quality = 0.0f;
                    
                    float avg_quality = 0.0f;
                    for(float q : enroll_qualities_)
                        avg_quality += q;
                    if(!enroll_qualities_.empty())
                        avg_quality /= enroll_qualities_.size();
                    
                    wait_msg.average_quality = avg_quality;
                    wait_msg.diversity_score = 0.0f;
                    wait_msg.status = "countdown";
                    
                    // Countdown message with number emoji
                    char wait_buffer[256];
                    std::string emoji;
                    if(countdown == 5) emoji = "5️⃣";
                    else if(countdown == 4) emoji = "4️⃣";
                    else if(countdown == 3) emoji = "3️⃣";
                    else if(countdown == 2) emoji = "2️⃣";
                    else if(countdown == 1) emoji = "1️⃣";
                    else emoji = "⏳";
                    
                    snprintf(wait_buffer, sizeof(wait_buffer),
                        "Ready in %d...",
                        countdown);
                    wait_msg.feedback_message = wait_buffer;
                    
                    // Keep the guidance from last sample
                    int progress_pct = (enroll_samples_ * 100) / enroll_target_samples_;
                    if(progress_pct < 30)
                        wait_msg.guidance = "💡 Turn your head slightly LEFT";
                    else if(progress_pct < 50)
                        wait_msg.guidance = "💡 Turn your head slightly RIGHT";
                    else if(progress_pct < 70)
                        wait_msg.guidance = "💡 Tilt your head slightly UP";
                    else if(progress_pct < 90)
                        wait_msg.guidance = "💡 Tilt your head slightly DOWN";
                    else
                        wait_msg.guidance = "💡 Almost done! One more varied angle";
                    
                    wait_msg.quality_level = "GOOD";
                    wait_msg.emoji = emoji;
                    enrollment_progress_pub_->publish(wait_msg);
                    
                    RCLCPP_INFO(this->get_logger(),
                        "%s seconds remaining before next capture",
                        emoji.c_str());
                }
                
                return;  // Skip processing until interval has passed
            }
            else
            {
                // Reset countdown for next cycle
                last_countdown_value_ = -1;
            }
        }
        
        // Check quality before accepting sample
        float quality = face->quality;
        std::string quality_str;
        std::string quality_emoji;

        if(quality > 0.95)
        {
            quality_str = "EXCELLENT";
            quality_emoji = "✅";
        }
        else if(quality > 0.85)
        {
            quality_str = "GOOD";
            quality_emoji = "👍";
        }
        else if(quality > 0.7)
        {
            quality_str = "ACCEPTABLE";
            quality_emoji = "⚠️";
        }
        else
        {
            // Quality too low, reject sample
            RCLCPP_WARN(
                this->get_logger(),
                "❌ Sample rejected: quality too low (%.3f) - Please face the camera directly",
                quality);

            qbo_msgs::msg::EnrollmentProgress progress_msg;
            progress_msg.person_name = enroll_name_;
            progress_msg.current_sample = enroll_samples_;
            progress_msg.target_samples = enroll_target_samples_;
            progress_msg.current_quality = quality;
            progress_msg.average_quality = 0.0f;
            progress_msg.diversity_score = 0.0f;
            progress_msg.status = "rejected";
            progress_msg.feedback_message = "Sample rejected: quality too low";
            progress_msg.guidance = "Face the camera directly and ensure good lighting";
            progress_msg.quality_level = "LOW";
            progress_msg.emoji = "❌";
            enrollment_progress_pub_->publish(progress_msg);
            return;
        }

        // Calculate diversity with existing samples
        float min_similarity = 1.0f;
        std::string diversity_feedback;

        if(!enroll_embeddings_.empty())
        {
            min_similarity = calculateMinSimilarity(embedding, enroll_embeddings_);

            if(min_similarity > 0.95)
            {
                // Too similar to existing samples
                RCLCPP_WARN(
                    this->get_logger(),
                    "⚠️  Sample too similar to previous ones (%.3f) - Try a different angle!",
                    min_similarity);

                float avg_quality = 0.0f;
                for(float q : enroll_qualities_)
                    avg_quality += q;
                if(!enroll_qualities_.empty())
                    avg_quality /= enroll_qualities_.size();

                qbo_msgs::msg::EnrollmentProgress progress_msg;
                progress_msg.person_name = enroll_name_;
                progress_msg.current_sample = enroll_samples_;
                progress_msg.target_samples = enroll_target_samples_;
                progress_msg.current_quality = quality;
                progress_msg.average_quality = avg_quality;
                progress_msg.diversity_score = 1.0f - min_similarity;
                progress_msg.status = "rejected";
                progress_msg.feedback_message = "Sample too similar to previous ones";
                progress_msg.guidance = "Move your head to a different position";
                progress_msg.quality_level = quality_str;
                progress_msg.emoji = "⚠️";
                enrollment_progress_pub_->publish(progress_msg);
                return;
            }
            else if(min_similarity > 0.85)
            {
                diversity_feedback = "Similar angle - vary more";
            }
            else
            {
                diversity_feedback = "Good diversity! ✓";
            }
        }
        else
        {
            diversity_feedback = "First sample captured";
        }

        // Accept the sample
        enroll_embeddings_.push_back(embedding.clone());
        enroll_qualities_.push_back(quality);
        enroll_samples_++;
        last_enrollment_sample_time_ = now;  // Record time of this sample

        // Calculate average quality
        float avg_quality = 0.0f;
        for(float q : enroll_qualities_)
            avg_quality += q;
        avg_quality /= enroll_qualities_.size();

        // Provide feedback
        RCLCPP_INFO(
            this->get_logger(),
            "%s Sample %d/%d captured | Quality: %s %.3f | Diversity: %s | Avg: %.3f",
            quality_emoji.c_str(),
            enroll_samples_,
            enroll_target_samples_,
            quality_str.c_str(),
            quality,
            diversity_feedback.c_str(),
            avg_quality);

        // Publish detailed progress with structured message
        qbo_msgs::msg::EnrollmentProgress progress_msg;
        progress_msg.person_name = enroll_name_;
        progress_msg.current_sample = enroll_samples_;
        progress_msg.target_samples = enroll_target_samples_;
        progress_msg.current_quality = quality;
        progress_msg.average_quality = avg_quality;
        progress_msg.diversity_score = enroll_embeddings_.size() > 1 ? (1.0f - min_similarity) : 0.0f;
        progress_msg.status = "capturing";

        char buffer[256];
        snprintf(buffer, sizeof(buffer),
            "Sample %d/%d captured | Quality: %s (%.2f) | %s",
            enroll_samples_,
            enroll_target_samples_,
            quality_str.c_str(),
            quality,
            diversity_feedback.c_str());
        progress_msg.feedback_message = buffer;
        progress_msg.quality_level = quality_str;
        progress_msg.emoji = quality_emoji;

        enrollment_progress_pub_->publish(progress_msg);

        // Provide guidance for next sample
        if(enroll_samples_ < enroll_target_samples_)
        {
            int remaining = enroll_target_samples_ - enroll_samples_;
            std::string guidance;

            // Suggest specific movements based on progress
            int progress_pct = (enroll_samples_ * 100) / enroll_target_samples_;

            if(progress_pct < 30)
                guidance = "💡 Turn your head slightly LEFT";
            else if(progress_pct < 50)
                guidance = "💡 Turn your head slightly RIGHT";
            else if(progress_pct < 70)
                guidance = "💡 Tilt your head slightly UP";
            else if(progress_pct < 90)
                guidance = "💡 Tilt your head slightly DOWN";
            else
                guidance = "💡 Almost done! One more varied angle";

            RCLCPP_INFO(this->get_logger(),
                "%s (%d more needed)",
                guidance.c_str(), remaining);

            // Publish guidance as separate message
            qbo_msgs::msg::EnrollmentProgress guidance_msg;
            guidance_msg.person_name = enroll_name_;
            guidance_msg.current_sample = enroll_samples_;
            guidance_msg.target_samples = enroll_target_samples_;
            guidance_msg.current_quality = quality;
            guidance_msg.average_quality = avg_quality;
            guidance_msg.diversity_score = enroll_embeddings_.size() > 1 ? (1.0f - min_similarity) : 0.0f;
            guidance_msg.status = "capturing";
            guidance_msg.feedback_message = "Continue capturing...";
            guidance_msg.guidance = guidance;
            guidance_msg.quality_level = quality_str;
            guidance_msg.emoji = "💡";
            enrollment_progress_pub_->publish(guidance_msg);
        }

        // Check if enrollment is complete
        if(enroll_samples_ >= enroll_target_samples_)
        {
            // Calculate final statistics
            float final_avg_quality = 0.0f;
            for(float q : enroll_qualities_)
                final_avg_quality += q;
            final_avg_quality /= enroll_qualities_.size();

            float avg_similarity = calculateAverageSimilarity(enroll_embeddings_);

            RCLCPP_INFO(this->get_logger(),
                "📊 Enrollment statistics: Avg Quality=%.3f, Avg Diversity=%.3f",
                final_avg_quality, 1.0f - avg_similarity);

            // Save embeddings to disk
            if(database_.savePerson(enroll_name_, enroll_embeddings_, db_path_, this->get_logger()))
            {
                qbo_msgs::msg::EnrollmentProgress success_msg;
                success_msg.person_name = enroll_name_;
                success_msg.current_sample = enroll_samples_;
                success_msg.target_samples = enroll_target_samples_;
                success_msg.current_quality = final_avg_quality;
                success_msg.average_quality = final_avg_quality;
                success_msg.diversity_score = 1.0f - avg_similarity;
                success_msg.status = "completed";

                char success_buffer[256];
                snprintf(success_buffer, sizeof(success_buffer),
                    "Enrollment completed! %d samples captured with %.0f%% average quality",
                    enroll_samples_,
                    final_avg_quality * 100.0f);
                success_msg.feedback_message = success_buffer;
                success_msg.guidance = "Enrollment successful. Ready for recognition!";
                success_msg.quality_level = "EXCELLENT";
                success_msg.emoji = "✅";
                enrollment_progress_pub_->publish(success_msg);

                RCLCPP_INFO(
                    this->get_logger(),
                    "✅ Enrollment completed successfully for %s (%d samples, avg quality: %.3f)",
                    enroll_name_.c_str(),
                    enroll_samples_,
                    final_avg_quality);
            }
            else
            {
                qbo_msgs::msg::EnrollmentProgress error_msg;
                error_msg.person_name = enroll_name_;
                error_msg.current_sample = enroll_samples_;
                error_msg.target_samples = enroll_target_samples_;
                error_msg.current_quality = final_avg_quality;
                error_msg.average_quality = final_avg_quality;
                error_msg.diversity_score = 1.0f - avg_similarity;
                error_msg.status = "error";
                error_msg.feedback_message = "Failed to save enrollment data";
                error_msg.guidance = "Please try again";
                error_msg.quality_level = "LOW";
                error_msg.emoji = "❌";
                enrollment_progress_pub_->publish(error_msg);

                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to save enrollment for %s",
                    enroll_name_.c_str());
            }

            enroll_mode_ = false;
            enroll_embeddings_.clear();
            enroll_qualities_.clear();
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

    // Log detailed matching info to help diagnose multi-person recognition
    RCLCPP_INFO(this->get_logger(),
        "🔍 Match result: best_match=%s, similarity=%.4f, quality=%.4f, threshold=%.2f",
        name.c_str(), similarity, face->quality, min_similarity_threshold_);

    // Update statistics
    total_recognitions_++;
    current_face_name_ = name;
    current_face_similarity_ = similarity;

    auto & history = identity_history_[face->face_id];

    history.push_back(name);

    if(history.size() > static_cast<size_t>(identity_window_))
        history.pop_front();

    RCLCPP_DEBUG(this->get_logger(),
        "Matching result: name=%s, similarity=%.3f, history_size=%zu",
        name.c_str(), similarity, history.size());

    // Publish result
    qbo_msgs::msg::FaceRecognitionResult result;

    result.header = face->header;
    result.face_id = face->face_id;

    // Require both good similarity AND good quality to accept recognition
    // Also require minimum samples in history for stable identification
    if(similarity > min_similarity_threshold_ &&
       face->quality >= min_quality_threshold_ &&
       history.size() >= static_cast<size_t>(min_samples_for_publication_))
    {
        result.known = true;
        result.name = majorityVote(history);
        result.similarity = similarity;

        // Track last positive recognition
        successful_recognitions_++;
        last_positive_recognition_[result.name] = now;

        RCLCPP_INFO(this->get_logger(),
            "✅ RECOGNIZED: name=%s (from history vote), raw_match=%s, similarity=%.4f, quality=%.4f, samples=%zu/%d",
            result.name.c_str(), name.c_str(), similarity, face->quality,
            history.size(), identity_window_);
    }
    else
    {
        result.known = false;
        result.name = "unknown";
        result.similarity = similarity;

        // Detailed rejection reason
        std::string reason;
        if(similarity <= min_similarity_threshold_)
            reason = "similarity too low (" + std::to_string(similarity) + " <= " + std::to_string(min_similarity_threshold_) + ")";
        else if(face->quality < min_quality_threshold_)
            reason = "quality too low (" + std::to_string(face->quality) + " < " + std::to_string(min_quality_threshold_) + ")";
        else if(history.size() < static_cast<size_t>(min_samples_for_publication_))
            reason = "not enough samples (" + std::to_string(history.size()) + " < " + std::to_string(min_samples_for_publication_) + ")";

        RCLCPP_INFO(this->get_logger(),
            "❌ UNKNOWN: best_match=%s, reason=%s",
            name.c_str(), reason.c_str());
    }

    result.distance = face->distance;
    result.quality = face->quality;

    result_pub_->publish(result);

    // Update diagnostics
    diagnostic_updater_.force_update();
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
    enroll_qualities_.clear();
    last_enrollment_sample_time_ = this->now();  // Initialize enrollment timer
    last_countdown_value_ = -1;  // Reset countdown

    res->success = true;

    // Publish enrollment start message
    qbo_msgs::msg::EnrollmentProgress progress_msg;
    progress_msg.person_name = enroll_name_;
    progress_msg.current_sample = 0;
    progress_msg.target_samples = enroll_target_samples_;
    progress_msg.current_quality = 0.0f;
    progress_msg.average_quality = 0.0f;
    progress_msg.diversity_score = 0.0f;
    progress_msg.status = "started";
    progress_msg.feedback_message = "Enrollment started. Please look at the camera.";
    progress_msg.guidance = "Move your head slowly (left/right/up/down) to capture different angles";
    progress_msg.quality_level = "GOOD";
    progress_msg.emoji = "🎬";
    enrollment_progress_pub_->publish(progress_msg);

    RCLCPP_INFO(
        this->get_logger(),
        "🎬 Starting enrollment for: %s (target: %d samples)",
        enroll_name_.c_str(),
        enroll_target_samples_);

    RCLCPP_INFO(
        this->get_logger(),
        "💡 TIPS: Move your head slowly (left/right/up/down) to capture different angles");
}

void FaceRecognitionNode::listPersonsCallback(
    const std::shared_ptr<qbo_msgs::srv::ListPersons::Request> /*req*/,
    std::shared_ptr<qbo_msgs::srv::ListPersons::Response> res)
{
    res->names = database_.getPersonNames();

    RCLCPP_INFO(this->get_logger(), "Listed %zu persons in database", res->names.size());
}

void FaceRecognitionNode::removePersonCallback(
    const std::shared_ptr<qbo_msgs::srv::RemovePerson::Request> req,
    std::shared_ptr<qbo_msgs::srv::RemovePerson::Response> res)
{
    if(req->name.empty())
    {
        res->success = false;
        RCLCPP_ERROR(this->get_logger(), "Person name cannot be empty");
        return;
    }

    // Check if currently in enrollment mode for this person
    if(enroll_mode_ && enroll_name_ == req->name)
    {
        RCLCPP_WARN(this->get_logger(), 
            "Cannot remove person '%s' - currently enrolling. Please wait for enrollment to complete.",
            req->name.c_str());
        res->success = false;
        return;
    }

    // Remove person from database
    if(database_.removePerson(req->name, db_path_, this->get_logger()))
    {
        res->success = true;

        // Clear identity history for all faces that were recognized as this person
        for(auto & entry : identity_history_)
        {
            auto & history = entry.second;
            // Remove all occurrences of this person from history
            history.erase(
                std::remove(history.begin(), history.end(), req->name),
                history.end());
        }

        // Remove from last positive recognition map
        last_positive_recognition_.erase(req->name);

        RCLCPP_INFO(this->get_logger(), 
            "Successfully removed person '%s' from database",
            req->name.c_str());
    }
    else
    {
        res->success = false;
        RCLCPP_ERROR(this->get_logger(), 
            "Failed to remove person '%s' from database",
            req->name.c_str());
    }
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
    stat.add("Min quality threshold", min_quality_threshold_);
    stat.add("Min similarity threshold", min_similarity_threshold_);
    stat.add("Min samples for publication", min_samples_for_publication_);
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

float FaceRecognitionNode::calculateMinSimilarity(
    const cv::Mat & new_embedding, 
    const std::vector<cv::Mat> & existing)
{
    float min_sim = 1.0f;

    for(const auto & emb : existing)
    {
        float sim = new_embedding.dot(emb) / (cv::norm(new_embedding) * cv::norm(emb));
        if(sim < min_sim)
            min_sim = sim;
    }

    return min_sim;
}

float FaceRecognitionNode::calculateAverageSimilarity(const std::vector<cv::Mat> & embeddings)
{
    if(embeddings.size() < 2)
        return 0.0f;

    float total_sim = 0.0f;
    int count = 0;

    for(size_t i = 0; i < embeddings.size(); ++i)
    {
        for(size_t j = i + 1; j < embeddings.size(); ++j)
        {
            float sim = embeddings[i].dot(embeddings[j]) / 
                       (cv::norm(embeddings[i]) * cv::norm(embeddings[j]));
            total_sim += sim;
            count++;
        }
    }

    return (count > 0) ? (total_sim / count) : 0.0f;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FaceRecognitionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}