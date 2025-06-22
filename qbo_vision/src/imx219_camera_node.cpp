#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

#define WIDTH 3280
#define HEIGHT 2464
#define BUFFER_COUNT 4

class Imx219V4L2Node : public rclcpp::Node {
public:
    Imx219V4L2Node() : Node("imx219_v4l2_node") {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

        open_device();
        init_device();
        start_capturing();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / 30),
            std::bind(&Imx219V4L2Node::capture_frame, this)
        );
        RCLCPP_INFO(this->get_logger(), "IMX219 V4L2 capture node started.");
    }

    ~Imx219V4L2Node() {
        stop_capturing();
        uninit_device();
        close(fd_);
    }

private:
    int fd_;
    struct Buffer {
        void *start;
        size_t length;
    } buffers_[BUFFER_COUNT];

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void open_device() {
        fd_ = open("/dev/video0", O_RDWR);
        if (fd_ < 0) throw std::runtime_error("Cannot open /dev/video0");
    }

    void init_device() {
        v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = WIDTH;
        fmt.fmt.pix.height = HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB10;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
            throw std::runtime_error("Failed to set format");

        RCLCPP_INFO(this->get_logger(), "Requested V4L2 format: %dx%d pixfmt=0x%X", WIDTH, HEIGHT, V4L2_PIX_FMT_SRGGB10);
        RCLCPP_INFO(this->get_logger(), "Driver accepted: %dx%d, pixfmt=0x%X", fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat);


        v4l2_requestbuffers req = {};
        req.count = BUFFER_COUNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0)
            throw std::runtime_error("Failed to request buffers");

        for (int i = 0; i < BUFFER_COUNT; ++i) {
            v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
                throw std::runtime_error("Failed to query buffer");

            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);

            if (buffers_[i].start == MAP_FAILED)
                throw std::runtime_error("Failed to mmap buffer");

            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0)
                throw std::runtime_error("Failed to queue buffer");
        }
        v4l2_buffer test_buf = {};
        test_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        test_buf.memory = V4L2_MEMORY_MMAP;
        test_buf.index = 0;
        if (ioctl(fd_, VIDIOC_QUERYBUF, &test_buf) < 0) {
            RCLCPP_WARN(this->get_logger(), "Final test VIDIOC_QUERYBUF failed on buffer 0");
        } else {
            RCLCPP_INFO(this->get_logger(), "Final test buffer[0] status: length=%d, offset=%d", test_buf.length, test_buf.m.offset);
        }
    }

    void start_capturing() {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0)
                throw std::runtime_error("Failed to start streaming");
            RCLCPP_INFO(this->get_logger(), "Streaming started successfully.");
            for (int i = 0; i < BUFFER_COUNT; ++i) {
                RCLCPP_INFO(this->get_logger(), "Buffer %d addr=%p size=%zu", i, buffers_[i].start, buffers_[i].length);
            }

        }

        void stop_capturing() {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
        }

        void uninit_device() {
            for (int i = 0; i < BUFFER_COUNT; ++i)
                munmap(buffers_[i].start, buffers_[i].length);
        }

        void capture_frame() {
        static int frame_count = 0;
        RCLCPP_INFO(this->get_logger(), "[Tick %d] Attempting capture...", ++frame_count);

        v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to dequeue buffer");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Buffer %d dequeued, bytes used: %d", buf.index, buf.bytesused);

        // Process raw Bayer10 image
        cv::Mat raw16(HEIGHT, WIDTH, CV_16UC1, buffers_[buf.index].start);
        cv::Mat raw8, rgb;

        double minVal, maxVal;
        cv::minMaxLoc(raw16, &minVal, &maxVal);
        RCLCPP_INFO(this->get_logger(), "Raw frame: type=%d, min=%.1f, max=%.1f", raw16.type(), minVal, maxVal);

        // 10-bit to 8-bit: scale values from 0–1023 → 0–255
        raw16.convertTo(raw8, CV_8UC1, 1.0 / 4.0);

        try {
            cv::demosaicing(raw8, rgb, cv::COLOR_BayerRG2BGR);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Demosaicing failed: %s", e.what());
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        RCLCPP_INFO(this->get_logger(), "Publishing image: %dx%d", rgb.cols, rgb.rows);
        image_pub_->publish(*msg);

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0)
            RCLCPP_WARN(this->get_logger(), "Failed to requeue buffer");
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Imx219V4L2Node>());
    rclcpp::shutdown();
    return 0;
}
