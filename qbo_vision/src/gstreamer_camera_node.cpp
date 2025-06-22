#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class GStreamerCameraNode : public rclcpp::Node {
public:
    GStreamerCameraNode() : Node("gstreamer_camera_node") {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

        // GStreamer pipeline using nvarguscamerasrc (Argus-based)
        std::string pipeline =
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1, format=NV12 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! "
            "video/x-raw, format=BGR ! appsink";

        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 30),
            std::bind(&GStreamerCameraNode::capture_frame, this));

        RCLCPP_INFO(this->get_logger(), "GStreamer camera node started.");
    }

private:
    void capture_frame() {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        image_pub_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GStreamerCameraNode>());
    rclcpp::shutdown();
    return 0;
}
