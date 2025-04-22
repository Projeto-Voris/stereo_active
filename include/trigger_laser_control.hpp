#include <gpiod.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class TriggerLaserControl : public rclcpp::Node
{
public:
    TriggerLaserControl(std::string node, int trigger_pin, int laser_pin, const std::string& gpio_chip);
    ~TriggerLaserControl();

    void trigger_cb(const std_srvs::srv::Trigger::Request::SharedPtr request,
                    const std_srvs::srv::Trigger::Response::SharedPtr response);

    void laser_cb(const std_srvs::srv::SetBool::Request::SharedPtr request,
                  const std_srvs::srv::SetBool::Response::SharedPtr response);

private:
    struct gpiod_chip *chip_;
    struct gpiod_line *trigger_line_;
    struct gpiod_line *laser_line_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr laser_service_;
};