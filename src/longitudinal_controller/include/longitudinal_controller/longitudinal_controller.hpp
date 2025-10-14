#include <pid/pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>

class LongitudinalController : public rclcpp::Node
{
public:
    LongitudinalController(PID pid_controller, double desired_distance);

    void distance_callback(std_msgs::msg::Float64 msg);

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher;
    PID pid_controller;
    double desired_distance;
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::time_point();
};

    