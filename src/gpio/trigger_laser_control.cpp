#include "trigger_laser_control.hpp"

int main(int argc, char *argv []){
    rclcpp::init(argc, argv);
    auto node = "trigger_laser_control_node";
    int trigger_pin = 85; // Replace with your actual GPIO pin number for trigger
    int laser_pin = 144; // Replace with your actual GPIO pin number for laser
    std::string gpio_chip = "/dev/gpiochip0"; // Replace with your actual GPIO chip

    rclcpp::spin(std::make_shared<TriggerLaserControl>(node, trigger_pin, laser_pin, gpio_chip));
    rclcpp::shutdown();
    return 0;
}

TriggerLaserControl::TriggerLaserControl(std::string node, int trigger_pin, int laser_pin, const std::string& gpio_chip)
    : Node(node)
{
    chip_ = gpiod_chip_open(gpio_chip.c_str());
    if (!chip_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip: %s", gpio_chip.c_str());
        throw std::runtime_error("Failed to open GPIO chip");
    }

    trigger_line_ = gpiod_chip_get_line(chip_, trigger_pin);
    laser_line_ = gpiod_chip_get_line(chip_, laser_pin);

    if (!trigger_line_ || !laser_line_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get GPIO lines");
        gpiod_chip_close(chip_);
        throw std::runtime_error("Failed to get GPIO lines");
    }

    gpiod_line_request_output(trigger_line_, "trigger", 0);
    gpiod_line_request_output(laser_line_, "laser", 0);

    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "trigger",
        std::bind(&TriggerLaserControl::trigger_cb, this,
                  std::placeholders::_1, std::placeholders::_2));

    laser_service_ = this->create_service<std_srvs::srv::SetBool>(
        "laser",
        std::bind(&TriggerLaserControl::laser_cb, this,
                  std::placeholders::_1, std::placeholders::_2));
}

TriggerLaserControl::~TriggerLaserControl()
{
    gpiod_line_set_value(trigger_line_, 0);
    gpiod_line_set_value(laser_line_, 0);
    gpiod_line_release(trigger_line_);
    gpiod_line_release(laser_line_);
    gpiod_chip_close(chip_);
}

void TriggerLaserControl::trigger_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                     const std_srvs::srv::Trigger::Response::SharedPtr response)
{
    gpiod_line_set_value(trigger_line_, 1);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    gpiod_line_set_value(trigger_line_, 0);
    response->success = true;
}

void TriggerLaserControl::laser_cb(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                   const std_srvs::srv::SetBool::Response::SharedPtr response)
{
    if (request->data) {
        gpiod_line_set_value(laser_line_, 1);
        RCLCPP_INFO(this->get_logger(), "Laser ON: %d", gpiod_line_get_value(laser_line_));

        response->message = "Laser ON";
        response->success = true;
    } else {
        gpiod_line_set_value(laser_line_, 0);
        RCLCPP_INFO(this->get_logger(), "Laser OFF: %d", gpiod_line_get_value(laser_line_));

        response->message = "Laser OFF";
        response->success = true;
    }
}