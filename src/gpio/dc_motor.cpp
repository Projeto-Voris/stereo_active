#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <gpiod.h>
#include <iostream>

class StepperMotor : public rclcpp::Node
{
public:
    StepperMotor() : Node("stepper_motor")
    {
        // Declare parameters
        this->declare_parameter<std::string>("stepping_mode", "half"); // full or half
        this->declare_parameter<int>("steps_per_revolution", 512); // 4096 - half, 2048 - full
        this->declare_parameter<int>("delay", 10); // Delay in milliseconds


        // Get parameters
        this->get_parameter("steps_per_revolution", steps_per_revolution_);
        this->get_parameter("stepping_mode", stepping_mode_);
        this->get_parameter("delay", delay_);

        // Initialize GPIO lines
        chip = gpiod_chip_open(gpio_chip_.c_str());
        if (!chip) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip: %s", gpio_chip_.c_str());
            throw std::runtime_error("Failed to open GPIO chip");
        }

        for (int pin : gpio_pins_)
        {
            struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
            if (!line) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get GPIO line: %d", pin);
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to get GPIO line");
            }
            gpio_lines_.push_back(line);

            int ret = gpiod_line_request_output(line, "stepper_motor", 0);
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to request line as output: %d", pin);
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to request line as output");
            }
        }

        // Subscribe to the topic
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "motor/angle", 10, std::bind(&StepperMotor::move_motor, this, std::placeholders::_1));
    }

    ~StepperMotor()
    {
        for (auto line : gpio_lines_) {
            gpiod_line_release(line);
        }
        gpiod_chip_close(chip);
    }

private:
    void move_motor(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::vector<std::vector<int>> step_sequence;

        if (stepping_mode_ == "full") {
            RCLCPP_INFO(this->get_logger(), "Full stepping mode");
            step_sequence = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Half stepping mode");
            step_sequence = {
                {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
                {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}};
        }

        float angle = msg->data;
        RCLCPP_INFO(this->get_logger(), "Moving motor to angle: %f", angle);
        int steps = static_cast<int>((angle / 360.0) * steps_per_revolution_);
        if (steps == 0)
        {
            stop_motor();
            return;
        }

        bool clockwise = (steps > 0);
        steps = std::abs(steps);    
 

    if (!clockwise) {
        std::reverse(step_sequence.begin(), step_sequence.end());
    }

    for (int i = 0; i < steps; ++i)
    {
        for (const auto& step : step_sequence)
        {
            for (size_t j = 0; j < gpio_lines_.size(); ++j)
            {
                gpiod_line_set_value(gpio_lines_[j], step[j]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
        }
    }
}

    void stop_motor()
    {
        for (auto line : gpio_lines_) {
            gpiod_line_set_value(line, 0);
        }
    }

    struct gpiod_chip *chip;
    std::vector<struct gpiod_line *> gpio_lines_;
    std::vector<int> gpio_pins_ = {105, 106, 41, 43}; // Replace with your actual GPIO pin numbers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    std::string gpio_chip_ = "/dev/gpiochip0";
    std::string stepping_mode_;
    std::vector<std::vector<int>> step_sequence_;
    int delay_;
    int steps_per_revolution_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepperMotor>());
    rclcpp::shutdown();
    return 0;
}
