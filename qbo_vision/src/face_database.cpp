
#include "face_database.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <unordered_set>

float FaceDatabase::cosineSimilarity(
    const cv::Mat & a,
    const cv::Mat & b)
{
    return a.dot(b) / (cv::norm(a) * cv::norm(b));
}

std::pair<std::string,float> FaceDatabase::match(
    const cv::Mat & embedding)
{
    if(database_.empty())
        return {"unknown", 0.0f};

    float best_score = 0.0f;
    std::string best_name = "unknown";

    for(const auto & e : database_)
    {
        float score = cosineSimilarity(embedding, e.embedding);

        if(score > best_score)
        {
            best_score = score;
            best_name = e.name;
        }
    }

    return {best_name, best_score};
}


bool FaceDatabase::addPerson(
    const std::string & name,
    const cv::Mat & embedding)
{
    if(embedding.empty())
        return false;

    FaceEntry entry;
    entry.name = name;
    entry.embedding = embedding.clone();
    database_.push_back(entry);

    return true;
}

bool FaceDatabase::savePerson(
    const std::string & name,
    const std::vector<cv::Mat> & embeddings,
    const std::string & directory,
    rclcpp::Logger logger)
{
    if(embeddings.empty())
    {
        RCLCPP_ERROR(logger, "No embeddings to save for person: %s", name.c_str());
        return false;
    }

    // Create embeddings directory if it doesn't exist
    std::string embeddings_dir = directory + "/embeddings";
    try
    {
        if(!std::filesystem::exists(embeddings_dir))
        {
            std::filesystem::create_directories(embeddings_dir);
        }
    }
    catch(const std::exception & e)
    {
        RCLCPP_ERROR(logger, "Failed to create embeddings directory: %s", e.what());
        return false;
    }

    // Generate timestamp for unique filenames
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()).count();

    std::vector<std::string> saved_files;

    // Save each embedding
    for(size_t i = 0; i < embeddings.size(); ++i)
    {
        std::string filename = name + "_" + std::to_string(timestamp) + "_" +
                              std::to_string(i) + ".bin";
        std::string filepath = embeddings_dir + "/" + filename;

        std::ofstream f(filepath, std::ios::binary);
        if(!f.is_open())
        {
            RCLCPP_ERROR(logger, "Failed to open file for writing: %s", filepath.c_str());
            continue;
        }

        if(embeddings[i].cols != 128 || embeddings[i].rows != 1)
        {
            RCLCPP_ERROR(logger, "Invalid embedding dimensions: %dx%d",
                embeddings[i].rows, embeddings[i].cols);
            continue;
        }

        if(!embeddings[i].isContinuous())
        {
            RCLCPP_ERROR(logger, "Embedding is not continuous in memory");
            continue;
        }

        f.write(reinterpret_cast<const char*>(embeddings[i].data),
                128 * sizeof(float));

        if(!f)
        {
            RCLCPP_ERROR(logger, "Failed to write embedding to: %s", filepath.c_str());
            continue;
        }

        saved_files.push_back(filename);

        // Add to in-memory database
        database_.push_back({name, embeddings[i].clone()});

        RCLCPP_DEBUG(logger, "Saved embedding: %s", filename.c_str());
    }

    if(saved_files.empty())
    {
        RCLCPP_ERROR(logger, "Failed to save any embeddings for: %s", name.c_str());
        return false;
    }

    // Update persons.yaml
    std::string yaml_file = directory + "/persons.yaml";

    try
    {
        YAML::Node config;

        // Load existing YAML if it exists
        if(std::filesystem::exists(yaml_file))
        {
            config = YAML::LoadFile(yaml_file);
        }

        // Ensure persons node exists
        if(!config["persons"])
        {
            config["persons"] = YAML::Node(YAML::NodeType::Sequence);
        }

        // Check if person already exists
        bool person_found = false;
        for(size_t i = 0; i < config["persons"].size(); ++i)
        {
            auto person = config["persons"][i];
            if(person["name"] && person["name"].as<std::string>() == name)
            {
                // Add new embeddings to existing person
                if(!person["embeddings"])
                {
                    config["persons"][i]["embeddings"] = YAML::Node(YAML::NodeType::Sequence);
                }

                for(const auto & file : saved_files)
                {
                    config["persons"][i]["embeddings"].push_back(file);
                }

                person_found = true;
                break;
            }
        }

        // If person doesn't exist, create new entry
        if(!person_found)
        {
            YAML::Node new_person;
            new_person["name"] = name;
            new_person["embeddings"] = YAML::Node(YAML::NodeType::Sequence);

            for(const auto & file : saved_files)
            {
                new_person["embeddings"].push_back(file);
            }

            config["persons"].push_back(new_person);
        }

        // Write back to file
        std::ofstream fout(yaml_file);
        if(!fout.is_open())
        {
            RCLCPP_ERROR(logger, "Failed to open persons.yaml for writing");
            return false;
        }

        fout << config;
        fout.close();

        RCLCPP_INFO(logger, "Updated persons.yaml with %zu embeddings for %s",
            saved_files.size(), name.c_str());

        return true;
    }
    catch(const YAML::Exception & e)
    {
        RCLCPP_ERROR(logger, "YAML error while updating persons.yaml: %s", e.what());
        return false;
    }
    catch(const std::exception & e)
    {
        RCLCPP_ERROR(logger, "Error updating persons.yaml: %s", e.what());
        return false;
    }
}

std::vector<std::string> FaceDatabase::getPersonNames() const
{
    std::vector<std::string> names;
    std::unordered_set<std::string> unique_names;

    for(const auto & entry : database_)
    {
        if(unique_names.find(entry.name) == unique_names.end())
        {
            unique_names.insert(entry.name);
            names.push_back(entry.name);
        }
    }

    return names;
}

bool FaceDatabase::load(const std::string & directory, rclcpp::Logger logger)
{
    // Clear existing database
    database_.clear();

    // Check if directory exists
    if(!std::filesystem::exists(directory))
    {
        RCLCPP_WARN(logger, "Face database directory does not exist: %s", directory.c_str());
        RCLCPP_INFO(logger, "Starting with empty face database");
        return false;
    }

    std::string yaml_file = directory + "/persons.yaml";

    // Check if YAML file exists
    if(!std::filesystem::exists(yaml_file))
    {
        RCLCPP_WARN(logger, "persons.yaml not found in: %s", directory.c_str());
        RCLCPP_INFO(logger, "Starting with empty face database");
        return false;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(yaml_file);

        if(!config["persons"])
        {
            RCLCPP_WARN(logger, "No 'persons' key found in persons.yaml");
            return false;
        }

        auto persons = config["persons"];

        for(const auto & person : persons)
        {
            if(!person["name"] || !person["embeddings"])
            {
                RCLCPP_WARN(logger, "Invalid person entry in persons.yaml");
                continue;
            }

            std::string name = person["name"].as<std::string>();
            auto embeddings = person["embeddings"];

            int loaded_for_person = 0;

            for(const auto & e : embeddings)
            {
                std::string file = directory + "/embeddings/" + e.as<std::string>();

                if(!std::filesystem::exists(file))
                {
                    RCLCPP_WARN(logger, "Embedding file not found: %s", file.c_str());
                    continue;
                }

                std::ifstream f(file, std::ios::binary);
                if(!f.is_open())
                {
                    RCLCPP_ERROR(logger, "Failed to open embedding file: %s", file.c_str());
                    continue;
                }

                // Get file size
                f.seekg(0, std::ios::end);
                size_t file_size = f.tellg();
                f.seekg(0, std::ios::beg);

                if(file_size != 128 * sizeof(float))
                {
                    RCLCPP_ERROR(logger, "Invalid embedding size in %s: expected %zu, got %zu",
                        file.c_str(), 128 * sizeof(float), file_size);
                    continue;
                }

                std::vector<float> data(128);
                f.read(reinterpret_cast<char*>(data.data()), 128 * sizeof(float));

                if(!f)
                {
                    RCLCPP_ERROR(logger, "Failed to read embedding data from: %s", file.c_str());
                    continue;
                }

                cv::Mat embedding(1, 128, CV_32F, data.data());
                database_.push_back({name, embedding.clone()});
                loaded_for_person++;
            }

            if(loaded_for_person > 0)
            {
                RCLCPP_INFO(logger, "Loaded %d embeddings for person: %s",
                    loaded_for_person, name.c_str());
            }
        }

        RCLCPP_INFO(logger, "Face database loaded: %zu total embeddings from %zu persons",
            database_.size(), persons.size());

        return database_.size() > 0;
    }
    catch(const YAML::Exception & e)
    {
        RCLCPP_ERROR(logger, "YAML parsing error: %s", e.what());
        database_.clear();
        return false;
    }
    catch(const std::exception & e)
    {
        RCLCPP_ERROR(logger, "Error loading face database: %s", e.what());
        database_.clear();
        return false;
    }
}