#ifndef STEREO_ACQUISITION_HPP
#define STEREO_ACQUISITION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

class StereoAcquisition : public rclcpp::Node {
public:
    StereoAcquisition();
    ~StereoAcquisition();

private:
    void images_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msgLeft,
                   const sensor_msgs::msg::Image::ConstSharedPtr &msgRight);

    void service_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void service_see_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void check_noise_image_service();

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::vector<std::pair<cv::Mat, cv::Mat>> image_buffer_;
    size_t buffer_size_;
    bool capture_images_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_images_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr see_images_service_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr noise_image_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // STEREO_ACQUISITION_HPP
