#include <vector>
#include <thread>
#include <atomic>
#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class MotorControl : public rclcpp::Node
{
public:
    MotorControl(std::string node, const std::vector<int>& gpio_pins, const std::string& gpio_chip);
    ~MotorControl();

    void move_motor(const std_msgs::msg::Float32::SharedPtr msg);

private:
    void stop_motor();

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    
    std::vector<int> gpio_pins_;
    std::string gpio_chip_;
    std::vector<std::vector<int>> step_sequence_;

    struct gpiod_chip *chip_;
    std::vector<struct gpiod_line *> gpio_lines_;

    int delay_;
    int steps_per_revolution_;
    std::string stepping_mode_;
    
    std::atomic<bool> keep_rotating_;
    std::thread rotation_thread_;
};