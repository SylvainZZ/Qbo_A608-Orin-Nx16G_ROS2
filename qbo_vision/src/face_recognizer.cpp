#include "face_recognizer.hpp"
#include <filesystem>

namespace fs = std::filesystem;

FaceRecognizerLBPH::FaceRecognizerLBPH(const rclcpp::NodeOptions & options)
: Node("face_recognizer", options)
{
    RCLCPP_INFO(this->get_logger(), "🧠 Initialisation du noeud FaceRecognizerLBPH");

    // 📁 Paramètres configurables
    faces_dir_              = this->declare_parameter<std::string>("faces_dir", "faces");
    model_file_             = this->declare_parameter<std::string>("model_file", "face_model.xml");
    confidence_threshold_   = this->declare_parameter<double>("confidence_threshold", 55.0);
    min_images_per_person_  = this->declare_parameter<int>("min_images_per_person", 3);

    // 🧠 Création du modèle LBPH
    model_ = cv::face::LBPHFaceRecognizer::create();
    model_->setThreshold(confidence_threshold_);

    // 📦 Chargement ou apprentissage initial du modèle
    if (std::filesystem::exists(model_file_)) {
        try {
            model_->read(model_file_);
            RCLCPP_INFO(this->get_logger(), "📥 Modèle chargé depuis : %s", model_file_.c_str());
        } catch (const cv::Exception &e) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Échec du chargement du modèle. Réentrainement nécessaire.");
            loadFaceDatabase(faces_dir_);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "📂 Aucun modèle trouvé, chargement des images pour apprentissage...");
        loadFaceDatabase(faces_dir_);
    }

    // 🔌 Services ROS2
    recognize_service_ = this->create_service<qbo_msgs::srv::RecognizeFace>(
        "recognize_face",
        std::bind(&FaceRecognizerLBPH::handleRecognize, this, std::placeholders::_1, std::placeholders::_2));

    get_name_service_ = this->create_service<qbo_msgs::srv::GetName>(
        "get_name",
        std::bind(&FaceRecognizerLBPH::handleGetName, this, std::placeholders::_1, std::placeholders::_2));

    learn_faces_service_ = this->create_service<qbo_msgs::srv::LearnFaces>(
    "learn_faces",
    std::bind(&FaceRecognizerLBPH::handleLearnFaces, this, std::placeholders::_1, std::placeholders::_2));

    train_service_ = this->create_service<qbo_msgs::srv::Train>(
        "train",
        std::bind(&FaceRecognizerLBPH::handleTrain, this, std::placeholders::_1, std::placeholders::_2));


    RCLCPP_INFO(this->get_logger(), "✅ Service de reconnaissance prêt");
}


void FaceRecognizerLBPH::loadFaceDatabase(const std::string &directory)
{
    std::vector<cv::Mat> images;
    std::vector<int> labels;
    int current_label = 0;

    for (const auto &entry : fs::directory_iterator(directory))
    {
        if (!entry.is_directory()) continue;

        std::string name = entry.path().filename().string();
        label_to_name_[current_label] = name;

        for (const auto &img_entry : fs::directory_iterator(entry.path()))
        {
            if (img_entry.path().extension() == ".png" || img_entry.path().extension() == ".jpg")
            {
                cv::Mat img = cv::imread(img_entry.path().string(), cv::IMREAD_GRAYSCALE);
                if (!img.empty())
                {
                    images.push_back(img);
                    labels.push_back(current_label);
                }
            }
        }

        current_label++;
    }

    if (images.empty())
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ Aucune image trouvée dans %s", directory.c_str());
        return;
    }

    model_->train(images, labels);
    RCLCPP_INFO(this->get_logger(), "✔️ Entraînement du modèle LBPH terminé (%zu visages)", images.size());
}

cv::Mat FaceRecognizerLBPH::convertMsgToGrayscaleMat(const sensor_msgs::msg::Image &img_msg)
{
    auto cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
    return gray;
}

void FaceRecognizerLBPH::handleRecognize(
    const std::shared_ptr<qbo_msgs::srv::RecognizeFace::Request> request,
    std::shared_ptr<qbo_msgs::srv::RecognizeFace::Response> response)
{
    cv::Mat face = convertMsgToGrayscaleMat(request->face);

    int predicted_label = -1;
    double confidence = 0.0;

    model_->predict(face, predicted_label, confidence);

    if (label_to_name_.count(predicted_label) > 0)
    {
        response->recognized = true;
        response->name = label_to_name_[predicted_label];
        RCLCPP_INFO(this->get_logger(), "🎯 Reconnu: %s (confiance=%.2f)", response->name.c_str(), confidence);
    }
    else
    {
        response->recognized = false;
        response->name = "";
        RCLCPP_WARN(this->get_logger(), "❌ Visage non reconnu (label=%d, confiance=%.2f)", predicted_label, confidence);
    }
}

void FaceRecognizerLBPH::handleGetName(
    const std::shared_ptr<qbo_msgs::srv::GetName::Request> request,
    std::shared_ptr<qbo_msgs::srv::GetName::Response> response)
{
    if (label_to_name_.count(request->label) > 0)
    {
        response->name = label_to_name_[request->label];
    }
    else
    {
        response->name = "unknown";
    }
}

void FaceRecognizerLBPH::handleLearnFaces(
    const std::shared_ptr<qbo_msgs::srv::LearnFaces::Request> request,
    std::shared_ptr<qbo_msgs::srv::LearnFaces::Response> response)
{
    if (request->person_name.empty()) {
        RCLCPP_WARN(this->get_logger(), "Nom vide reçu dans LearnFaces");
        response->learned = false;
        return;
    }

    // Déterminer le dossier cible
    std::string person_dir = faces_dir_ + "/" + request->person_name;
    if (!fs::exists(person_dir)) {
        fs::create_directories(person_dir);
        RCLCPP_INFO(this->get_logger(), "Création du dossier : %s", person_dir.c_str());
    }

    // Convertir ROS Image -> OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(request->face, sensor_msgs::image_encodings::MONO8);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erreur conversion image: %s", e.what());
        response->learned = false;
        return;
    }

    // Nom de fichier unique
    std::string timestamp = std::to_string(this->get_clock()->now().nanoseconds());
    std::string filename = person_dir + "/" + timestamp + ".png";

    if (cv::imwrite(filename, cv_ptr->image)) {
        RCLCPP_INFO(this->get_logger(), "Image sauvegardée : %s", filename.c_str());
        response->learned = true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Échec sauvegarde de l'image.");
        response->learned = false;
    }
}

void FaceRecognizerLBPH::handleTrain(
    const std::shared_ptr<qbo_msgs::srv::Train::Request>,
    std::shared_ptr<qbo_msgs::srv::Train::Response> response)
{
    try {
        loadFaceDatabase(faces_dir_);
        RCLCPP_INFO(this->get_logger(), "Réentrainement réussi depuis %s", faces_dir_.c_str());
        response->taught  = true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erreur réentrainement: %s", e.what());
        response->taught  = false;
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);

    auto node = std::make_shared<FaceRecognizerLBPH>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
