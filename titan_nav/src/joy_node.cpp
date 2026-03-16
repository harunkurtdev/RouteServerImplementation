// Copyright (c) 2024 titan_nav contributors
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace titan_nav
{

/**
 * @brief JoyNode
 *
 * Translates joystick (sensor_msgs/Joy) messages into differential-drive
 * velocity commands (geometry_msgs/Twist) for the warehouse robot.
 *
 * Axis mapping (defaults)
 * -----------------------
 *   axis_linear   – left stick vertical  (axis 1) → linear.x
 *   axis_angular  – right stick horizontal (axis 3) → angular.z
 *
 * Button mapping (defaults)
 * -------------------------
 *   btn_deadman   – button 4 (LB) must be held to enable output
 *
 * Parameters
 * ----------
 *   joy_topic      – Input joystick topic  (default: joy)
 *   cmd_vel_topic  – Output velocity topic (default: cmd_vel_raw)
 *   axis_linear    – Joystick axis index for linear velocity  (default: 1)
 *   axis_angular   – Joystick axis index for angular velocity (default: 3)
 *   btn_deadman    – Button index for deadman switch (default: 4)
 *   scale_linear   – Scale factor for linear velocity  (default: 0.5)
 *   scale_angular  – Scale factor for angular velocity (default: 1.0)
 */
class JoyNode : public rclcpp::Node
{
public:
  JoyNode()
  : Node("joy_node")
  {
    // Parameters
    this->declare_parameter<std::string>("joy_topic", "joy");
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_raw");
    this->declare_parameter<int>("axis_linear",  1);
    this->declare_parameter<int>("axis_angular", 3);
    this->declare_parameter<int>("btn_deadman",  4);
    this->declare_parameter<double>("scale_linear",  0.5);
    this->declare_parameter<double>("scale_angular", 1.0);

    joy_topic_     = this->get_parameter("joy_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    axis_linear_   = this->get_parameter("axis_linear").as_int();
    axis_angular_  = this->get_parameter("axis_angular").as_int();
    btn_deadman_   = this->get_parameter("btn_deadman").as_int();
    scale_linear_  = this->get_parameter("scale_linear").as_double();
    scale_angular_ = this->get_parameter("scale_angular").as_double();

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS());

    // Subscription
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SensorDataQoS(),
      std::bind(&JoyNode::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "JoyNode started — linear axis %d (×%.2f), angular axis %d (×%.2f), deadman btn %d",
      axis_linear_, scale_linear_, axis_angular_, scale_angular_, btn_deadman_);
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::Twist cmd;

    // Deadman switch: only publish non-zero commands when button is held
    const bool deadman_active =
      btn_deadman_ >= 0 &&
      btn_deadman_ < static_cast<int>(msg->buttons.size()) &&
      msg->buttons[btn_deadman_] == 1;

    if (deadman_active) {
      if (axis_linear_ >= 0 && axis_linear_ < static_cast<int>(msg->axes.size())) {
        cmd.linear.x = scale_linear_ * msg->axes[axis_linear_];
      }
      if (axis_angular_ >= 0 && axis_angular_ < static_cast<int>(msg->axes.size())) {
        cmd.angular.z = scale_angular_ * msg->axes[axis_angular_];
      }
    }

    cmd_pub_->publish(cmd);
  }

  // Subscription / publisher
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // Configuration
  std::string joy_topic_;
  std::string cmd_vel_topic_;
  int    axis_linear_;
  int    axis_angular_;
  int    btn_deadman_;
  double scale_linear_;
  double scale_angular_;
};

}  // namespace titan_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<titan_nav::JoyNode>());
  rclcpp::shutdown();
  return 0;
}
