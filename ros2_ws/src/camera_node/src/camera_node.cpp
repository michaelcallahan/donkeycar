#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#ifdef __arm__
#include <raspicam/raspicam_cv.h>
#endif

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
    : Node("camera_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&CameraNode::timer_callback, this));

#ifdef __arm__
        // Only attempt to open the camera if on ARM architecture
        if (!camera_.open()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening camera");
            rclcpp::shutdown();
            return;
        }
        // Set camera parameters (optional)
        camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        camera_.set(cv::CAP_PROP_FPS, 30);
#else
        RCLCPP_WARN(this->get_logger(), "Running without Raspberry Pi camera hardware");
#endif
    }

private:
    void timer_callback()
    {
#ifdef __arm__
        cv::Mat frame;
        if (!camera_.grab()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to grab frame");
            return;
        }
        camera_.retrieve(frame);

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Captured empty frame");
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
#else
        // If not on ARM, simulate a blank image (for testing purposes)
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 255, 0)); // Green image for testing
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
#endif
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

#ifdef __arm__
    raspicam::RaspiCam_Cv camera_;
#endif
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
