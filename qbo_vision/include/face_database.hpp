#ifndef QBO_FACE_DATABASE_HPP
#define QBO_FACE_DATABASE_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

struct FaceEntry
{
    std::string name;
    cv::Mat embedding;
};

class FaceDatabase
{
public:
    bool load(const std::string & path, rclcpp::Logger logger);

    bool savePerson(
        const std::string & name,
        const std::vector<cv::Mat> & embeddings,
        const std::string & directory,
        rclcpp::Logger logger);

    bool addPerson(
        const std::string & name,
        const cv::Mat & embedding);

    std::pair<std::string,float> match(
        const cv::Mat & embedding);

    std::vector<std::string> getPersonNames() const;

private:

    std::vector<FaceEntry> database_;

    float cosineSimilarity(
        const cv::Mat & a,
        const cv::Mat & b);
};

#endif