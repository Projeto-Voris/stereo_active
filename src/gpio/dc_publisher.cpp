#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class MotorPublisher : public rclcpp::Node
{
public:
    MotorPublisher() : Node("motor_publisher"), angle_(0.0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("motor/angle", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MotorPublisher::publishAngle, this));
    }

private:
    void publishAngle()
    {
        auto message = std_msgs::msg::Float32();
        message.data = angle_;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published angle: %f", angle_);

        // Increment the angle for demonstration (loop from 0 to 360)
        angle_ = std::fmod(angle_ + 10.0, 360.0);
    }

    float angle_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPublisher>());
    rclcpp::shutdown();
    return 0;
}
