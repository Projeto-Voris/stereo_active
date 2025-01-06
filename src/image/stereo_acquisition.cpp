#include <stereo_acquisition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/image.hpp>

StereoAcquisition::StereoAcquisition() : Node("stereo_acquisition"), buffer_size_(10), capture_images_(false) {
    
    this->declare_parameter<int>("buffer_size", 10.0);
    this->declare_parameter<std::string>("images_path", "/home/jetson/Pictures/SM3/temp");
    this->get_parameter("buffer_size", buffer_size_);
    this->get_parameter("images_path", images_path_);
    RCLCPP_INFO(this->get_logger(), "Image path %s.", images_path_.c_str());

    
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "left_image");
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "right_image");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *left_sub, *right_sub);
    sync_->registerCallback(std::bind(&StereoAcquisition::images_cb, this, std::placeholders::_1, std::placeholders::_2));

    get_images_service_ = this->create_service<std_srvs::srv::Trigger>("capture_images", 
                std::bind(&StereoAcquisition::service_cb, this, std::placeholders::_1, std::placeholders::_2));

    see_images_service_ = this->create_service<std_srvs::srv::Trigger>("show_images", 
                std::bind(&StereoAcquisition::service_see_cb, this, std::placeholders::_1, std::placeholders::_2));

    noise_image_client_ = this->create_client<std_srvs::srv::SetBool>("pattern_change");

    count_ = 0;

    RCLCPP_INFO(this->get_logger(), "Image buffer size set to %ld.", buffer_size_);
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

    // Condition to capture images on buffer
    if (capture_images_ && image_buffer_.size() < buffer_size_) {

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        noise_image_client_->async_send_request(request, [this, cv_ptrLeft, cv_ptrRight](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
            if (result.get()->success && image_buffer_.size() < buffer_size_) {
                count_++;
                if (count_ < 6){
                    return;
                }
                cv::Mat left_image = cv_ptrLeft->image;
                cv::Mat right_image = cv_ptrRight->image;

                image_buffer_.emplace_back(left_image, right_image);

                RCLCPP_INFO(this->get_logger(), "Images captured and added to buffer - %ld.", image_buffer_.size());
                // RCLCPP_INFO(this->get_logger(), "Images published.");

                if (image_buffer_.size() >= buffer_size_) {
                    capture_images_ = false;

                }
                
            
            }
            else {
                if(image_buffer_.size() >= buffer_size_) {
                    capture_images_ = false;
                    // RCLCPP_INFO(this->get_logger(), "Buffer is full. Stopping image capture.");
                    // RCLCPP_INFO(this->get_logger(), "Buffer size: %ld", image_buffer_.size());
                    count_ = 0;
                    // rclcpp::sleep_for(std::chrono::milliseconds(1000));
                    // RCLCPP_INFO(this->get_logger(), "Buffer is full. Stopping image capture.");
                    
                    // Publish the images in the buffer
                    for (const auto &image_pair : image_buffer_) {
                        std::string left_image_path = images_path_ + "/left/L" + std::to_string(count_) + ".jpg";
                        std::string right_image_path = images_path_ + "/right/R" + std::to_string(count_) + ".jpg";
                        cv::imwrite(left_image_path, image_pair.first);
                        cv::imwrite(right_image_path, image_pair.second);
                        count_++;
                    }
                }
                else{
                    RCLCPP_WARN(this->get_logger(), "Noise image service returned false.");
                }
            }
        });
    }


}

void StereoAcquisition::service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (request) {
        capture_images_ = true;
        image_buffer_.clear();                           
    }
        response->success = true;
        response->message = "Image capture started.";

}
void StereoAcquisition::service_see_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (request) {
        RCLCPP_INFO(this->get_logger(), "Showing images: %ld", image_buffer_.size());
        int count= 1;
        for (auto &image_pair : image_buffer_) {
            cv::imshow("Left Image", image_pair.first);
            cv::imshow("Right Image", image_pair.second);
            cv::waitKey(1000);
            RCLCPP_INFO(this->get_logger(), "Showing images number: %d", count);
            count++;
        }
        cv::destroyAllWindows();
        response->success = true;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoAcquisition>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}