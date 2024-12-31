#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class MotorPublisherNode : public rclcpp::Node
{
public:
    MotorPublisherNode() : Node("motor_publisher"), motor_value_(0)
    {
        // Create a publisher that publishes Int32 messages to the "motor_values" topic
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("motor_values", 10);

        // Create a timer that periodically publishes the motor value
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MotorPublisherNode::publishMotorValue, this));

        RCLCPP_INFO(this->get_logger(), "Motor Publisher Node has started.");
    }

private:
    void publishMotorValue()
    {
        auto message = std_msgs::msg::Int32();
        message.data = motor_value_;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published motor value: %d", motor_value_);

        // Increment the motor value for demonstration (loop from 0 to 100)
        motor_value_ = (motor_value_ + 10) % 100;
    }

    int motor_value_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
