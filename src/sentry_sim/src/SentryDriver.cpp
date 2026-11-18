#include "sentry_sim/SentryDriver.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <rclcpp/qos.hpp>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025
#define DIST_TO_CENTER 0.2897

void SentryDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  wheel_fl = wb_robot_get_device("wheel0");
  wheel_bl = wb_robot_get_device("wheel1");
  wheel_br = wb_robot_get_device("wheel2");
  wheel_fr = wb_robot_get_device("wheel3");

  const auto setup = [](WbDeviceTag wheel) -> void {
      wb_motor_set_position(wheel, INFINITY);
      wb_motor_set_velocity(wheel, 0);
  };

  setup(wheel_fl);
  setup(wheel_bl);
  setup(wheel_br);
  setup(wheel_fr);

  cmd_vel_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist &msg) {
        this->cmd_vel_msg.linear = msg.linear;
        this->cmd_vel_msg.angular = msg.angular;
      });
}

void SentryDriver::step() {
  driveBasedOnHeading(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z, 0);
}      

void SentryDriver::driveBasedOnHeading(float forward, float sideways, float rotational, float heading) {
    float cos_theta = cos(heading);
    float sin_theta = sin(heading);

    float vx_local = forward * cos_theta - sideways * sin_theta;
    float vy_local = forward * sin_theta + sideways * cos_theta;

    float rotationalComponent = -rotational * DIST_TO_CENTER;

    // these are in meters/second, convert to motor velocity ??
    float motor_lf_velocity = (vx_local - vy_local) / M_SQRT2 + rotationalComponent;
    float motor_rf_velocity = (-vx_local - vy_local) / M_SQRT2 + rotationalComponent;
    float motor_rb_velocity = (-vx_local + vy_local) / M_SQRT2 + rotationalComponent;
    float motor_lb_velocity = (vx_local + vy_local) / M_SQRT2 + rotationalComponent;

    wb_motor_set_velocity(wheel_fl, motor_lf_velocity);
    wb_motor_set_velocity(wheel_fr, motor_rf_velocity);
    wb_motor_set_velocity(wheel_br, motor_rb_velocity);
    wb_motor_set_velocity(wheel_bl, motor_lb_velocity);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(SentryDriver, webots_ros2_driver::PluginInterface)
