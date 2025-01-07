#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gpiod.h>

class StepperMotor : public rclcpp::Node
{
public:
    StepperMotor() : Node("stepper_motor")
    {
        // Initialize GPIO lines
        chip = gpiod_chip_open_by_name("gpiochip0");
        for (int pin : gpio_pins_)
        {
            gpio_lines_.push_back(gpiod_chip_get_line(chip, pin));
            gpiod_line_request_output(gpio_lines_.back(), "stepper_motor", 0);
        }

        // Subscribe to the topic
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "stepper_motor/steps", 10, std::bind(&StepperMotor::move_motor, this, std::placeholders::_1));
    }

    ~StepperMotor()
    {
        gpiod_chip_close(chip);
    }

private:
    void move_motor(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int steps = msg->data;
        if (steps == 0)
        {
            stop_motor();
            return;
        }

        bool clockwise = (steps > 0);
        steps = std::abs(steps);

        std::vector<std::vector<int>> step_sequence = {
            {1, 0, 0, 1},
            {0, 1, 1, 0},
            {0, 1, 0, 1},
            {1, 0, 0, 1}};

        for (int i = 0; i < steps; ++i)
        {
            int step_index = clockwise ? i % 8 : (7 - (i % 8));
            set_motor_pins(step_sequence[step_index]);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        stop_motor();
    }

    void set_motor_pins(const std::vector<int> &pin_states)
    {
        for (size_t i = 0; i < gpio_lines_.size(); ++i)
        {
            if (gpiod_line_set_value(gpio_lines_[i], pin_states[i]) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO pin %d to %d.", gpio_pins_[i], pin_states[i]);
            }
        }
    }

    void stop_motor()
    {
        set_motor_pins({0, 0, 0, 0});
    }

    std::vector<int> gpio_pins_ = {105, 41, 106, 43};
    std::vector<struct gpiod_line *> gpio_lines_;
    struct gpiod_chip *chip;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepperMotor>());
    rclcpp::shutdown();
    return 0;
}
