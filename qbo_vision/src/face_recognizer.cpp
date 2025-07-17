#include "face_recognizer.hpp"

#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <chrono>

namespace fs = std::filesystem;

FaceRecognizerLBPH::FaceRecognizerLBPH(const rclcpp::NodeOptions & options)
: Node("qbo_face_recognizer", options)
{
    RCLCPP_INFO(this->get_logger(), "🧠 Initialisation du noeud Qbo Face Recognizer");

    path_faces_learn_     = this->declare_parameter<std::string>("path_faces_learn", "faces/learn");
    path_model_file_      = this->declare_parameter<std::string>("path_model_file", "faces/model");
    model_file_           = this->declare_parameter<std::string>("model_file", "face_model.xml");
    confidence_threshold_ = this->declare_parameter<double>("confidence_threshold", 40.0);

    model_ = cv::face::LBPHFaceRecognizer::create(
        1,     // radius
        8,     // neighbors
        8, 8,  // grid_x, grid_y
        100.0  // threshold (désactivé si 0 ou > confidence_threshold)
    );

    prepareTrainingIfNeeded();
    loadTrainedModelIfAvailable();

    recognize_service_ = this->create_service<qbo_msgs::srv::RecognizeFace>(
        "qbo_face_recognizer/recognize_face",
        std::bind(&FaceRecognizerLBPH::handleRecognize, this, std::placeholders::_1, std::placeholders::_2)
    );
    last_save_time_ = this->now();  // Compatible avec get_clock()

    RCLCPP_INFO(this->get_logger(), "✅ Service recognize_face actif");
}

void FaceRecognizerLBPH::loadTrainedModelIfAvailable()
{
    auto full_path = fs::path(path_model_file_) / model_file_;
    if (fs::exists(full_path)) {
        model_->read(full_path.string());
        model_trained_ = true;
        RCLCPP_INFO(this->get_logger(), "📂 Modèle chargé depuis %s", full_path.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Aucun modèle trouvé à %s", full_path.c_str());
    }
}

void FaceRecognizerLBPH::prepareTrainingIfNeeded()
{
    if (!fs::exists(path_faces_learn_))
        return;

    std::vector<cv::Mat> images;
    std::vector<int> labels;
    std::map<std::string, int> name_to_label;

    int label_count = 0;

    for (const auto & entry : fs::directory_iterator(path_faces_learn_)) {
        if (!entry.is_directory())
            continue;

        std::string folder_name = entry.path().filename().string();

        // Extraire le nom après "_" si présent, sinon ignorer
        std::string label_name;
        size_t pos = folder_name.find_last_of('_');
        if (pos != std::string::npos) {
            label_name = folder_name.substr(pos + 1);  // prend "alice" dans "..._alice"
        } else {
            continue;  // Skip dossiers non nommés
        }

        // Attribution de label unique
        if (!name_to_label.count(label_name)) {
            name_to_label[label_name] = static_cast<int>(name_to_label.size());
            label_to_name_[name_to_label[label_name]] = label_name;
        }

        for (const auto & file : fs::directory_iterator(entry.path())) {
            cv::Mat img = cv::imread(file.path().string(), cv::IMREAD_GRAYSCALE);
            if (!img.empty()) {
                if (img.rows < 80 || img.cols < 80) continue;
                // Étape 1 : resize pour normaliser la taille
                cv::resize(img, img, cv::Size(100, 100));

                // Étape 2 : amélioration de l’histogramme (contraste)
                cv::equalizeHist(img, img);

                images.push_back(img);
                labels.push_back(name_to_label[label_name]);
            }
        }
    }

    if (!images.empty()) {
        model_->train(images, labels);
        fs::create_directories(path_model_file_);
        model_->save((fs::path(path_model_file_) / model_file_).string());
        RCLCPP_INFO(this->get_logger(), "🏋️ Modèle entraîné avec %lu visages.", images.size());
    }
}

void FaceRecognizerLBPH::handleRecognize(
    const std::shared_ptr<qbo_msgs::srv::RecognizeFace::Request> request,
    std::shared_ptr<qbo_msgs::srv::RecognizeFace::Response> response)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(request->face, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erreur de conversion d'image : %s", e.what());
        response->recognized = false;
        response->name = "invalid_image";
        return;
    }

    if (!cv_ptr->image.data) {
        RCLCPP_WARN(this->get_logger(), "Image vide reçue");
        response->recognized = false;
        response->name = "invalid_image";
        return;
    }

    int label = -1;
    double confidence = 0.0;

    if (model_trained_) {
        model_->predict(cv_ptr->image, label, confidence);

        if (confidence < confidence_threshold_ && label_to_name_.count(label)) {
            response->recognized = true;
            response->name = label_to_name_[label];
            RCLCPP_INFO(this->get_logger(), "🙂 Visage reconnu: %s (score %.2f)", response->name.c_str(), confidence);
            return;
        } else {
            RCLCPP_WARN(this->get_logger(), "🤔 Visage non reconnu (score %.2f >= seuil %.2f)", confidence, confidence_threshold_);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ Aucun modèle entraîné. Enregistrement de la nouvelle image.");
    }

    // Cas visage inconnu ou pas de modèle → Sauvegarde
    response->recognized = false;
    response->name = "unknown";

    // Throttle : ignorer si trop récent
    if ((this->now() - last_save_time_).seconds() < 2.0) {
        RCLCPP_WARN(this->get_logger(), "⏱️ Trop tôt depuis la dernière sauvegarde, on ignore ce visage.");
        return;
    }

    // Créer un timestamp unique
    std::string timestamp = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

    // Dossier d'enregistrement
    std::string save_dir = (fs::path(path_faces_learn_) / timestamp).string();
    fs::create_directories(save_dir);

    // Base filename (dans le dossier) avec même timestamp
    std::string base_filename = (fs::path(save_dir) / ("face_" + timestamp)).string();

    // Recadrage léger
    // int crop = 8; // pixels à retirer de chaque bord
    // cv::Rect roi(crop, crop, cv_ptr->image.cols - 2 * crop, cv_ptr->image.rows - 2 * crop);
    // cv::Mat cropped_face = cv_ptr->image(roi).clone();

    // Sauvegarde de la version originale recadrée
    // cv::imwrite(base_filename + ".jpg", cropped_face);
    cv::imwrite(base_filename + ".jpg", cv_ptr->image);


    // // Génération de variations
    // cv::Mat flip_img, bright_img, dark_img;
    // cv::flip(cropped_face, flip_img, 1);
    // cropped_face.convertTo(bright_img, -1, 1.2, 30);
    // cropped_face.convertTo(dark_img, -1, 0.8, -30);

    // // Sauvegardes supplémentaires
    // cv::imwrite(base_filename + "_flip.jpg", flip_img);
    // cv::imwrite(base_filename + "_bright.jpg", bright_img);

    // Variante : Miroir horizontal
    cv::Mat flipped;
    cv::flip(cv_ptr->image, flipped, 1);
    cv::imwrite(base_filename + "_flip.jpg", flipped);

    // Variante : légèrement plus clair
    cv::Mat brighter;
    cv_ptr->image.convertTo(brighter, -1, 1.1, 15);
    cv::imwrite(base_filename + "_bright.jpg", brighter);

    RCLCPP_WARN(this->get_logger(), "🤔 Visage inconnu. Images sauvegardées dans : %s", save_dir.c_str());
    last_save_time_ = this->get_clock()->now();

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceRecognizerLBPH>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
