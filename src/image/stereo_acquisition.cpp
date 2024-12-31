#include <stereo_acquisition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

StereoAcquisition::StereoAcquisition() : Node("stereo_acquisition"), buffer_size_(10), capture_images_(false) {
    
    this->declare_parameter<int>("buffer_size", 10.0);
    this->get_parameter("buffer_size", buffer_size_);

    
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "left_image");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "right_image");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *left_sub, *right_sub);
    sync_->registerCallback(std::bind(&StereoAcquisition::images_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_images_service_ = this->create_service<std_srvs::srv::Trigger>("capture_images", 
                std::bind(&StereoAcquisition::service_cb, this, std::placeholders::_1, std::placeholders::_2));

    noise_image_client_ = this->create_client<std_srvs::srv::SetBool>("pattern_change");

    // timer_ = this->create_wall_timer(
        // std::chrono::milliseconds(50), std::bind(&StereoAcquisition::check_noise_image_service, this));
}

StereoAcquisition::~StereoAcquisition() {}

void StereoAcquisition::images_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                                  const sensor_msgs::msg::Image::ConstSharedPtr &msgRight) {

    cv_bridge::CvImagePtr cv_ptrLeft;
    cv_bridge::CvImagePtr cv_ptrRight;

    try {
        cv_ptrLeft = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8);
        cv_ptrRight = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat left_image = cv_ptrLeft->image;
    cv::Mat right_image = cv_ptrRight->image;

    if (capture_images_) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        noise_image_client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            if (future.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Noise image service triggered successfully.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Noise image service returned false.");
            }
        });
        image_buffer_.emplace_back(left_image, right_image);
        // RCLCPP_INFO(this->get_logger(), "Image buffer: %d", image_buffer_.size());
    }

    if (image_buffer_.size() >= buffer_size_  && capture_images_) {
        capture_images_ = false;
        RCLCPP_INFO(this->get_logger(), "Buffer is full. Stopping image capture.");
        return;
    }

    // cv::imshow("Left Image", left_image);
    // cv::imshow("Right Image", right_image);
    // cv::waitKey(10);

}

void StereoAcquisition::service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (request) {
        capture_images_ = true;
        image_buffer_.clear();
        response->success = true;
        response->message = "Image capture started.";
    }
}

void StereoAcquisition::check_noise_image_service() {
    if (image_buffer_.size() >= buffer_size_) {
        capture_images_ = false;
        return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto result = noise_image_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Noise image service returned true. Starting image capture.");
            capture_images_ = true;
            image_buffer_.clear();
        } else {
            RCLCPP_WARN(this->get_logger(), "Noise image service returned false.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call noise_image service.");
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoAcquisition>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}