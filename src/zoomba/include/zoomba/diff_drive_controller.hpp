#ifndef ZOOMBA_DIFF_DRIVE_CONTROLLER_HPP
#define ZOOMBA_DIFF_DRIVE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class DiffDriveController : public rclcpp::Node
{
public:
  DiffDriveController();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double wheel_radius_;
  double wheel_separation_;
  double max_wheel_speed_;

  double left_speed_;
  double right_speed_;
  double left_wheel_pos_;
  double right_wheel_pos_;

  rclcpp::Time last_time_;
};

#endif