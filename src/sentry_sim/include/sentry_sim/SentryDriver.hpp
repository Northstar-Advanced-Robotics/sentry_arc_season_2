#ifndef SENTRY_DRIVER_HPP
#define SENTRY_DRIVER_HPP

#include <rclcpp/macros.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/subscription.hpp>

class SentryDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription;
  geometry_msgs::msg::Twist cmd_vel_msg;

  WbDeviceTag wheel0;
  WbDeviceTag wheel1;
  WbDeviceTag wheel2;
  WbDeviceTag wheel3;
};

#endif // SENTRY_DRIVER_HPP
