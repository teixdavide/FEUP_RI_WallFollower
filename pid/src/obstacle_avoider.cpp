#include "pid/obstacle_avoider.hpp"
#include <algorithm>

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider")
{
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            return this->scanCallback(msg);
        });
}

void ObstacleAvoider::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (msg->ranges.empty())
        return;

    // Compute closest obstacle in forward hemisphere (|-90deg..+90deg|)
    constexpr double PI = 3.14159265358979323846;
    const double hemi = 90.0 * PI / 180.0;
    double min_range = msg->range_max;
    double min_angle = 0.0;
    size_t finite_count = 0;
    // Global diagnostics across the entire scan
    size_t finite_count_all = 0;
    double min_all = msg->range_max;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
        double r_all = msg->ranges[i];
        if (std::isfinite(r_all))
        {
            ++finite_count_all;
            if (r_all < min_all)
                min_all = r_all;
        }
        if (ang >= -hemi && ang <= hemi)
        {
            double r = msg->ranges[i];
            if (std::isfinite(r))
            {
                ++finite_count;
                if (r < min_range)
                {
                    min_range = r;
                    min_angle = ang;
                }
            }
        }
    }

        // (scan summary logs removed by request)

    auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
    cmd->linear.x = 0.1;

    // If something is close in the forward hemisphere, rotate away from it
    if (finite_count == 0)
    {
        // No returns in forward hemisphere: rotate to search
        auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
        cmd->linear.x = 0.0;
            cmd->angular.z = 0.8; // slower turning
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000 /* ms */, "decision: search rotate ang=%.2f", cmd->angular.z);
        publisher_->publish(std::move(cmd));
        return;
    }
    else if (min_range < 0.25)
    {
        // Turn away from the closest obstacle angle
            cmd->angular.z = (min_angle >= 0.0) ? -1.0 : 1.0;
        cmd->linear.x = 0.0;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500 /* ms */,
                "decision: avoid min=%.3f at %.1fdeg -> turn %s (ang=%.2f)",
                min_range, (min_angle * 180.0 / PI), (min_angle >= 0.0 ? "RIGHT" : "LEFT"), cmd->angular.z);
    }
    else
    {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000 /* ms */,
                "decision: go straight min=%.3f lin=%.2f", min_range, cmd->linear.x);
    }

    publisher_->publish(std::move(cmd));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto avoider = std::make_shared<ObstacleAvoider>();
    rclcpp::spin(avoider);
    rclcpp::shutdown();
    return 0;
}