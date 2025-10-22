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
  double estimateDistance(const sensor_msgs::msg::LaserScan &scan, double center_deg, double half_deg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  // Parameters
  PID pid_;
  double desired_dist_;
  double max_ang_speed_;
  double fwd_speed_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;

  bool use_sin_speed_{};
  double sin_amplitude_{};
  double sin_period_{};

  rclcpp::Time start_time_{};

  // Leader-detection (behind) parameters
  double leader_detect_center_deg_{180.0};
  double leader_detect_half_deg_{2.0};
  double leader_detect_threshold_{1.0};
  bool leader_detected_{};
  double wait_speed_{0.0};
  bool is_leader_{};
};
