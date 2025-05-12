#include "motor_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <gpiod.h>

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = "motor_control_node";
    std::vector<int> gpio_pins = {105, 106, 41, 43}; // Replace with your actual GPIO pin numbers
    std::string gpio_chip = "/dev/gpiochip0";

    rclcpp::spin(std::make_shared<MotorControl>(node, gpio_pins, gpio_chip));
    rclcpp::shutdown();
    return 0;
}

MotorControl::MotorControl(std::string node, const std::vector<int>& gpio_pins, const std::string& gpio_chip)
    : Node(node), gpio_pins_(gpio_pins), gpio_chip_(gpio_chip), keep_rotating_(false)
{
    this->declare_parameter<int>("steps_per_revolution", 2048);
    this->declare_parameter<int>("delay", 10);
    this->declare_parameter<std::string>("stepping_mode", "full");

    this->get_parameter("steps_per_revolution", steps_per_revolution_);
    this->get_parameter("delay", delay_);
    this->get_parameter("stepping_mode", stepping_mode_);
    current_step_ = 0;
    chip_ = gpiod_chip_open(gpio_chip_.c_str());
    if (!chip_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip: %s", gpio_chip_.c_str());
        throw std::runtime_error("Failed to open GPIO chip");
    }

    for (int pin : gpio_pins_) {
        struct gpiod_line *line = gpiod_chip_get_line(chip_, pin);
        if (!line) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get GPIO line: %d", pin);
            gpiod_chip_close(chip_);
            throw std::runtime_error("Failed to get GPIO line");
        }
        gpio_lines_.push_back(line);
        int ret = gpiod_line_request_output(line, "stepper_motor", 0);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to request line as output: %d", pin);
            gpiod_chip_close(chip_);
            throw std::runtime_error("Failed to request line as output");
        }
    }
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        "motor/angle", 10,
        std::bind(&MotorControl::move_motor, this, std::placeholders::_1));

    std::ostringstream oss;
    for (size_t i = 0; i < gpio_pins_.size(); ++i) {
        oss << gpio_pins_[i];
        if (i < gpio_pins_.size() - 1) {
            oss << ", ";
        }
    }
    RCLCPP_INFO(this->get_logger(), "MotorControl node initialized with GPIO pins: %s", oss.str().c_str());
}

MotorControl::~MotorControl()
{
    keep_rotating_ = false;
    if (rotation_thread_.joinable()) {
        rotation_thread_.join();
    }
    for (auto line : gpio_lines_) {
        gpiod_line_set_value(line, 0);
        gpiod_line_release(line);
    }
    gpiod_chip_close(chip_);
}

void MotorControl::move_motor(const std_msgs::msg::Float32::SharedPtr msg)
{
// Get parameters
        this->get_parameter("steps_per_revolution", steps_per_revolution_);
        this->get_parameter("stepping_mode", stepping_mode_);
        this->get_parameter("delay", delay_);
        
        std::vector<std::vector<int>> step_sequence;

        if (stepping_mode_ == "full") {
            step_sequence = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        } else {
            step_sequence = {
                {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
                {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}};
        }

        float angle = msg->data;
        RCLCPP_INFO(this->get_logger(), "Moving motor to angle: %f", angle);
        if (angle == 500.0) {
            if (rotation_thread_.joinable()) {
                rotation_thread_.join();
            }
            RCLCPP_INFO(this->get_logger(), "Oscillate rotation mode");
            keep_rotating_ = true;
        
            rotation_thread_ = std::thread([this, step_sequence]() {
                int steps = static_cast<int>((20 / 360.0) * steps_per_revolution_);
                while (keep_rotating_) {
                    // Rotate N degrees clockwise
                    for (int i = 0; i < steps; ++i) {
                        for (const auto& step : step_sequence) {
                            for (size_t j = 0; j < gpio_lines_.size(); ++j) {
                                gpiod_line_set_value(gpio_lines_[j], step[j]);
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
                        }
                    }
        
                    // Rotate N degrees counter-clockwise
                    std::vector<std::vector<int>> step_sequence_ccw = step_sequence;
                    std::reverse(step_sequence_ccw.begin(), step_sequence_ccw.end());
                    for (int i = 0; i < steps; ++i) {
                        for (const auto& step : step_sequence_ccw) {
                            for (size_t j = 0; j < gpio_lines_.size(); ++j) {
                                gpiod_line_set_value(gpio_lines_[j], step[j]);
                            }
                            std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
                        }
                    }
                }
            });
        }
        else {
            keep_rotating_ = false;
            if (rotation_thread_.joinable()) {
                rotation_thread_.join();
            }

            int steps = static_cast<int>((angle / 360.0) * steps_per_revolution_);
            if (steps == 0) {
                stop_motor();
                return;
            }

            bool clockwise = (steps > 0);
            steps = std::abs(steps);

            if (!clockwise) {
                std::reverse(step_sequence.begin(), step_sequence.end());
            }

        for (int step_count = 0; step_count < steps; ++step_count) {
            const auto& step = step_sequence[current_step_ % step_sequence.size()];
            for (size_t j = 0; j < gpio_lines_.size(); ++j) {
                gpiod_line_set_value(gpio_lines_[j], step[j]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_));

            current_step_ = (current_step_ + 1) % step_sequence.size();
        }
    }
}

void MotorControl::stop_motor()
{
    for (auto line : gpio_lines_) {
        gpiod_line_set_value(line, 0);
    }
}