#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <gpiod.h>
#include <iostream>
#include <atomic>

class GpioControl : public rclcpp::Node
{
public:
    GpioControl() : Node("gpio_control_node"), keep_rotating_(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("stepping_mode", "full"); // full or half
        this->declare_parameter<int>("steps_per_revolution", 2048); // 4096 - half, 2048 - full
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
            if (pin == 85 || pin == 144){
                if(pin == 85){ laser_line = line; }
                else{ trigger_line = line; }
            }
            else{ gpio_lines_.push_back(line); }

            int ret = gpiod_line_request_output(line, "stepper_motor", 0);
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to request line as output: %d", pin);
                gpiod_chip_close(chip);
                throw std::runtime_error("Failed to request line as output");
            }
        }

        // Subscribe to the topic
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "motor/angle", 10, std::bind(&GpioControl::move_motor, this, std::placeholders::_1));
    
        trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger", std::bind(&GpioControl::trigger_cb, this, std::placeholders::_1, std::placeholders::_2));

        laser_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "laser", std::bind(&GpioControl::laser_cb, this, std::placeholders::_1, std::placeholders::_2));
    }

    ~GpioControl()
    {
        keep_rotating_ = false;
        if (rotation_thread_.joinable()) {
            rotation_thread_.join();
        }
        for (auto line : gpio_lines_) {
            gpiod_line_release(line);
        }
        gpiod_chip_close(chip);
    }

private:
    void move_motor(const std_msgs::msg::Float32::SharedPtr msg){
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

        if (angle == 400.0) {
            RCLCPP_INFO(this->get_logger(), "Continuous rotation mode");
            keep_rotating_ = false;
            if (rotation_thread_.joinable()) {
                rotation_thread_.join();
            }
            keep_rotating_ = true;
            rotation_thread_ = std::thread([this, step_sequence]() {
                while (keep_rotating_) {
                    for (const auto& step : step_sequence) {
                        for (size_t j = 0; j < gpio_lines_.size(); ++j) {
                            gpiod_line_set_value(gpio_lines_[j], step[j]);
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
                    }
                }
            });
        } else {
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

            for (int i = 0; i < steps; ++i) {
                const auto& step = step_sequence[i % step_sequence.size()];
                for (size_t j = 0; j < gpio_lines_.size(); ++j) {
                    gpiod_line_set_value(gpio_lines_[j], step[j]);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_));
            }
        }
    }

    void stop_motor(){
        for (auto line : gpio_lines_) {
            gpiod_line_set_value(line, 0);
        }
    }

    void trigger_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                    const std_srvs::srv::Trigger::Response::SharedPtr response){
        gpiod_line_set_value(trigger_line, 1);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        gpiod_line_set_value(trigger_line, 0);
        response->success = true;
    }

    void laser_cb(const std_srvs::srv::SetBool::Request::SharedPtr request,
                 const std_srvs::srv::SetBool::Response::SharedPtr response){
        if (request->data){
            gpiod_line_set_value(laser_line, 1);
            response->success = true;
        }
        else if(!request->data){
            gpiod_line_set_value(laser_line, 0);
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }


    std::vector<int> gpio_pins_ = {105, 106, 41, 43, 85, 144}; // Replace with your actual GPIO pin numbers
    std::string gpio_chip_ = "/dev/gpiochip0";
    std::string stepping_mode_;
    std::vector<std::vector<int>> step_sequence_;
    struct gpiod_chip *chip;
    std::vector<struct gpiod_line *> gpio_lines_;
    struct gpiod_line *laser_line;
    struct gpiod_line *trigger_line;
    int delay_;
    int steps_per_revolution_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr laser_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;

    std::atomic<bool> keep_rotating_;
    std::thread rotation_thread_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpioControl>());
    rclcpp::shutdown();
    return 0;
}
