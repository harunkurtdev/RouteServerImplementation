// Copyright (c) 2024 titan_nav contributors
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace titan_nav
{

/**
 * @brief ControllerDynamicsNode
 *
 * Applies a first-order low-pass filter to incoming velocity commands so that
 * the warehouse robot's differential drive does not receive step changes that
 * could destabilise the physical drive train.  The filtered command is then
 * re-published on the outgoing topic consumed by the hardware interface.
 *
 * Parameters
 * ----------
 * input_topic   – Raw Twist topic (default: cmd_vel_raw)
 * output_topic  – Filtered Twist topic (default: cmd_vel)
 * alpha         – Filter coefficient in [0, 1].  0 = very smooth, 1 = no filter.
 * max_linear_x  – Hard velocity cap, m/s (default: 1.0)
 * max_angular_z – Hard angular velocity cap, rad/s (default: 1.5)
 */
class ControllerDynamicsNode : public rclcpp::Node
{
public:
  ControllerDynamicsNode()
  : Node("controller_dynamics_node"),
    filtered_linear_x_(0.0),
    filtered_angular_z_(0.0)
  {
    // Parameters
    this->declare_parameter<std::string>("input_topic", "cmd_vel_raw");
    this->declare_parameter<std::string>("output_topic", "cmd_vel");
    this->declare_parameter<double>("alpha", 0.5);
    this->declare_parameter<double>("max_linear_x", 1.0);
    this->declare_parameter<double>("max_angular_z", 1.5);

    const std::string input_topic  = this->get_parameter("input_topic").as_string();
    const std::string output_topic = this->get_parameter("output_topic").as_string();
    alpha_         = this->get_parameter("alpha").as_double();
    max_linear_x_  = this->get_parameter("max_linear_x").as_double();
    max_angular_z_ = this->get_parameter("max_angular_z").as_double();

    // Clamp alpha to valid range
    alpha_ = std::clamp(alpha_, 0.0, 1.0);

    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      output_topic, rclcpp::SystemDefaultsQoS());

    // Subscription
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&ControllerDynamicsNode::cmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "ControllerDynamicsNode started (alpha=%.2f, max_lin=%.2f, max_ang=%.2f)",
      alpha_, max_linear_x_, max_angular_z_);
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // First-order low-pass filter
    filtered_linear_x_  = alpha_ * msg->linear.x  + (1.0 - alpha_) * filtered_linear_x_;
    filtered_angular_z_ = alpha_ * msg->angular.z + (1.0 - alpha_) * filtered_angular_z_;

    // Hard-clamp to safe velocity limits
    filtered_linear_x_  = std::clamp(filtered_linear_x_,  -max_linear_x_,  max_linear_x_);
    filtered_angular_z_ = std::clamp(filtered_angular_z_, -max_angular_z_, max_angular_z_);

    geometry_msgs::msg::Twist out;
    out.linear.x  = filtered_linear_x_;
    out.angular.z = filtered_angular_z_;
    cmd_pub_->publish(out);
  }

  // Subscription / publisher
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // Filter state
  double alpha_;
  double max_linear_x_;
  double max_angular_z_;
  double filtered_linear_x_;
  double filtered_angular_z_;
};

}  // namespace titan_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<titan_nav::ControllerDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
