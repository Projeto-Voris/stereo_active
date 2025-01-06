#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class StereoImageSubscriber : public rclcpp::Node {
public:
    StereoImageSubscriber() : Node("stereo_image_subscriber") {
        left_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/SM3/left_image_buffer", 10, std::bind(&StereoImageSubscriber::leftImageCallback, this, std::placeholders::_1));
        right_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/SM3/right_image_buffer", 10, std::bind(&StereoImageSubscriber::rightImageCallback, this, std::placeholders::_1));
        
    }
    ~StereoImageSubscriber() {
        cv::destroyAllWindows();
    }

private:
    int count  = 1;
    
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Showing left image: %d", count);
        cv::imshow("Left Image", cv_ptr->image);
        cv::waitKey(10); // Display each image for 1 second
        count++;
    }

    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        // cv::imshow("Right Image", cv_ptr->image);
        // cv::waitKey(1000); // Display each image for 1 second
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}