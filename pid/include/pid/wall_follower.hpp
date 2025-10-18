#pragma once

#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include "pid/pid.hpp"

class WallFollower : public rclcpp::Node {
public:
  explicit WallFollower();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  double estimateRightDistance(const sensor_msgs::msg::LaserScan &scan);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr prox_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr prox_sub_;

  PID pid_;
  double desired_dist_;
  double max_ang_speed_;
  double fwd_speed_;
  // Coordination params
  bool publish_prox_{};
  bool wait_for_follower_{};
  std::string prox_topic_;
  std::string follower_prox_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  double prox_front_deg_{};
  double prox_front_thresh_{};
  double follower_timeout_s_{};
  double wait_speed_{};
  rclcpp::Time last_follower_ok_{};
  bool use_sin_speed_{};
  double sin_amplitude_{};
  double sin_period_{};
  rclcpp::Time start_time_{};
};
