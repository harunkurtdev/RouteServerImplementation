// Copyright (c) 2024 titan_nav contributors
// Licensed under the Apache License, Version 2.0

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cmath>
#include <memory>

namespace titan_nav
{

/**
 * @brief PlannerMetricNode
 *
 * Subscribes to the robot odometry and the current navigation plan, then
 * publishes real-time metrics (cross-track error, distance remaining, and
 * estimated time of arrival) so that operators can monitor route execution
 * in the warehouse environment.
 */
class PlannerMetricNode : public rclcpp::Node
{
public:
  PlannerMetricNode()
  : Node("planner_metric_node"),
    total_path_length_(0.0),
    distance_traveled_(0.0)
  {
    // Parameters
    this->declare_parameter<double>("publish_rate", 1.0);
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("plan_topic", "plan");

    const double publish_rate = this->get_parameter("publish_rate").as_double();
    const std::string odom_topic = this->get_parameter("odom_topic").as_string();
    const std::string plan_topic = this->get_parameter("plan_topic").as_string();

    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&PlannerMetricNode::odomCallback, this, std::placeholders::_1));

    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      plan_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&PlannerMetricNode::planCallback, this, std::placeholders::_1));

    // Periodic metric logging timer
    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PlannerMetricNode::publishMetrics, this));

    RCLCPP_INFO(this->get_logger(), "PlannerMetricNode started (odom='%s', plan='%s')",
      odom_topic.c_str(), plan_topic.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (last_odom_) {
      const double dx = msg->pose.pose.position.x - last_odom_->pose.pose.position.x;
      const double dy = msg->pose.pose.position.y - last_odom_->pose.pose.position.y;
      distance_traveled_ += std::hypot(dx, dy);
    }
    last_odom_ = msg;
  }

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    total_path_length_ = 0.0;
    for (std::size_t i = 1; i < msg->poses.size(); ++i) {
      const auto & a = msg->poses[i - 1].pose.position;
      const auto & b = msg->poses[i].pose.position;
      total_path_length_ += std::hypot(b.x - a.x, b.y - a.y);
    }
    current_plan_ = msg;
    distance_traveled_ = 0.0;
  }

  void publishMetrics()
  {
    if (!last_odom_ || !current_plan_ || current_plan_->poses.empty()) {
      return;
    }

    // Distance remaining: find the closest pose on the plan and sum edges from there
    double min_dist = std::numeric_limits<double>::max();
    std::size_t closest_idx = 0;
    for (std::size_t i = 0; i < current_plan_->poses.size(); ++i) {
      const auto & p = current_plan_->poses[i].pose.position;
      const double d = std::hypot(
        p.x - last_odom_->pose.pose.position.x,
        p.y - last_odom_->pose.pose.position.y);
      if (d < min_dist) {
        min_dist = d;
        closest_idx = i;
      }
    }

    double distance_remaining = 0.0;
    for (std::size_t i = closest_idx + 1; i < current_plan_->poses.size(); ++i) {
      const auto & a = current_plan_->poses[i - 1].pose.position;
      const auto & b = current_plan_->poses[i].pose.position;
      distance_remaining += std::hypot(b.x - a.x, b.y - a.y);
    }

    const double speed = std::hypot(
      last_odom_->twist.twist.linear.x,
      last_odom_->twist.twist.linear.y);
    const double eta_s = (speed > 0.01) ? (distance_remaining / speed) : -1.0;

    RCLCPP_DEBUG(this->get_logger(),
      "Metrics — path_length: %.2f m | traveled: %.2f m | remaining: %.2f m | ETA: %.1f s",
      total_path_length_, distance_traveled_, distance_remaining, eta_s);
  }

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;

  // State
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  nav_msgs::msg::Path::SharedPtr current_plan_;
  double total_path_length_;
  double distance_traveled_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace titan_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<titan_nav::PlannerMetricNode>());
  rclcpp::shutdown();
  return 0;
}
