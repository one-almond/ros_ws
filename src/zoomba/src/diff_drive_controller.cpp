#include "zoomba/diff_drive_controller.hpp"
#include <algorithm>
#include <cmath>

DiffDriveController::DiffDriveController()
: Node("diff_drive_controller"),
  left_speed_(0.0), right_speed_(0.0),
  left_wheel_pos_(0.0), right_wheel_pos_(0.0)
{
  this->declare_parameter("wheel_radius", 0.05);
  this->declare_parameter("wheel_separation", 0.3);
  this->declare_parameter("max_wheel_speed", 10.0);

  wheel_radius_    = this->get_parameter("wheel_radius").as_double();
  wheel_separation_ = this->get_parameter("wheel_separation").as_double();
  max_wheel_speed_ = this->get_parameter("max_wheel_speed").as_double();

  last_time_ = this->get_clock()->now();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1)
  );

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speeds", 10);

  // 50hz timer for joint states
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&DiffDriveController::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Diff drive controller started");
}

void DiffDriveController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear  = msg->linear.x;
  double angular = msg->angular.z;

  // Differential drive kinematics
  double left_speed  = (linear - angular * wheel_separation_ / 2.0) / wheel_radius_;
  double right_speed = (linear + angular * wheel_separation_ / 2.0) / wheel_radius_;

  // Scale down if over max speed
  double max_input = std::max(std::abs(left_speed), std::abs(right_speed));
  if (max_input > max_wheel_speed_) {
    double scale = max_wheel_speed_ / max_input;
    left_speed  *= scale;
    right_speed *= scale;
  }

  left_speed_  = left_speed;
  right_speed_ = right_speed;

  // Publish wheel speeds for GPIO driver
  std_msgs::msg::Float32MultiArray speed_msg;
  speed_msg.data = {static_cast<float>(left_speed_), static_cast<float>(right_speed_)};
  wheel_speed_pub_->publish(speed_msg);
}

void DiffDriveController::timer_callback()
{
  auto now = this->get_clock()->now();
  double dt = (now - last_time_).seconds();
  last_time_ = now;

  // Integrate wheel positions
  left_wheel_pos_  += left_speed_  * dt;
  right_wheel_pos_ += right_speed_ * dt;

  // Publish joint states
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now;
  joint_state.name     = {"left_wheel_joint", "right_wheel_joint"};
  joint_state.position = {left_wheel_pos_, right_wheel_pos_};
  joint_state.velocity = {left_speed_, right_speed_};
  joint_state_pub_->publish(joint_state);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}