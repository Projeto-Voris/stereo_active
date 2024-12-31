#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <gpiod.h>
#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

class StepperMotorController : public rclcpp::Node
{
public:
    StepperMotorController()
        : Node("stepper_motor_controller"), chip(nullptr)
    {

        // Open GPIO chip
        chip = gpiod_chip_open("/dev/gpiochip0");
        if (!chip) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open GPIO chip.");
            rclcpp::shutdown();
            return;
        }

        // Configure the GPIO pins
        for (int pin : gpio_pins_) {
            struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
            if (!line || gpiod_line_request_output(line, "stepper_motor", 0) < 0) {
                RCLCPP_FATAL(this->get_logger(), "Failed to configure GPIO pin %d.", pin);
                gpiod_chip_close(chip);
                rclcpp::shutdown();
                return;
            }
            gpio_lines_.push_back(line);
        }

        // Subscribe to the topic
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "motor_values", 10,
            std::bind(&StepperMotorController::control_motor, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Stepper Motor Controller Node has started.");
    }

    ~StepperMotorController()
    {
        for (auto line : gpio_lines_) {
            gpiod_line_release(line);
        }
        if (chip) {
            gpiod_chip_close(chip);
        }
    }

private:
    void control_motor(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int steps = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received %d steps to move the stepper motor.", steps);

        if (steps == 0) {
            stop_motor();
            return;
        }

        // Direction of movement
        bool clockwise = (steps > 0);
        steps = std::abs(steps);

        // Step sequence for the 28BYJ-48 motor
        std::vector<std::vector<int>> step_sequence = {
            {1, 0, 0, 1},
            {1, 0, 0, 0},
            {1, 1, 0, 0},
            {0, 1, 0, 0},
            {0, 1, 1, 0},
            {0, 0, 1, 0},
            {0, 0, 1, 1},
            {0, 0, 0, 1}};

        for (int i = 0; i < steps; ++i) {
            int step_index = clockwise ? i % 8 : (7 - (i % 8));
            set_motor_pins(step_sequence[step_index]);
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // Adjust for speed
        }

        stop_motor();
    }

    void set_motor_pins(const std::vector<int> &pin_states)
    {
        for (size_t i = 0; i < gpio_lines_.size(); ++i) {
            if (gpiod_line_set_value(gpio_lines_[i], pin_states[i]) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO pin %d to %d.", gpio_pins_[i], pin_states[i]);
            }
        }
    }

    void stop_motor()
    {
        set_motor_pins({0, 0, 0, 0}); // Turn off all motor phases
    }

    std::vector<int> gpio_pins_ = {29,31,32,33};
    std::vector<struct gpiod_line *> gpio_lines_;
    struct gpiod_chip *chip;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepperMotorController>());
    rclcpp::shutdown();
    return 0;
}
