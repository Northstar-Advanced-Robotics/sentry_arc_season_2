#include "sentry_sim/SentryDriver.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/qos.hpp>
#include <webots/motor.h>
#include <webots/robot.h>

// Must match sentry_description/urdf/sentry.urdf.xacro and worlds/my_world.wbt
#define WHEEL_RADIUS 0.075   // m, omni wheel radius
#define DIST_TO_CENTER 0.315 // m, wheel contact point to chassis center (mount_radius)

void SentryDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  wheel_fl = wb_robot_get_device("front_left_wheel_joint");
  wheel_bl = wb_robot_get_device("rear_left_wheel_joint");
  wheel_br = wb_robot_get_device("rear_right_wheel_joint");
  wheel_fr = wb_robot_get_device("front_right_wheel_joint");

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

    // Rim speeds (m/s) -> wheel angular velocities (rad/s)
    float motor_lf_velocity = ((vx_local - vy_local) / M_SQRT2 + rotationalComponent) / WHEEL_RADIUS;
    float motor_rf_velocity = ((-vx_local - vy_local) / M_SQRT2 + rotationalComponent) / WHEEL_RADIUS;
    float motor_rb_velocity = ((-vx_local + vy_local) / M_SQRT2 + rotationalComponent) / WHEEL_RADIUS;
    float motor_lb_velocity = ((vx_local + vy_local) / M_SQRT2 + rotationalComponent) / WHEEL_RADIUS;

    // Saturation scaling: if any wheel exceeds the motor limit, scale all four
    // by the same factor so the commanded direction is preserved (Webots would
    // otherwise clamp each wheel independently and bend the trajectory).
    const float max_wheel = wb_motor_get_max_velocity(wheel_fl);
    const float largest = std::max({std::fabs(motor_lf_velocity), std::fabs(motor_rf_velocity),
                                    std::fabs(motor_rb_velocity), std::fabs(motor_lb_velocity)});
    if (largest > max_wheel) {
        const float scale = max_wheel / largest;
        motor_lf_velocity *= scale;
        motor_rf_velocity *= scale;
        motor_rb_velocity *= scale;
        motor_lb_velocity *= scale;
    }

    wb_motor_set_velocity(wheel_fl, motor_lf_velocity);
    wb_motor_set_velocity(wheel_fr, motor_rf_velocity);
    wb_motor_set_velocity(wheel_br, motor_rb_velocity);
    wb_motor_set_velocity(wheel_bl, motor_lb_velocity);
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(SentryDriver, webots_ros2_driver::PluginInterface)
