#include "sentry_sim/SentryDriver.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <rclcpp/qos.hpp>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

void SentryDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  wheel0 = wb_robot_get_device("wheel0");
  wheel1 = wb_robot_get_device("wheel1");
  wheel2 = wb_robot_get_device("wheel2");
  wheel3 = wb_robot_get_device("wheel3");

  const auto setup = [](WbDeviceTag wheel) -> void {
      wb_motor_set_position(wheel, INFINITY);
      wb_motor_set_velocity(wheel, 0);
  };
  setup(wheel0);
  setup(wheel1);
  setup(wheel2);
  setup(wheel3);
;      
  cmd_vel_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist &msg) {
        this->cmd_vel_msg.linear = msg.linear;
        this->cmd_vel_msg.angular = msg.angular;
      });
}

void SentryDriver::step() {
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(wheel0, command_motor_left);
  wb_motor_set_velocity(wheel3, command_motor_right);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(SentryDriver, webots_ros2_driver::PluginInterface)
