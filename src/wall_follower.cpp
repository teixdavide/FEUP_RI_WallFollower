
#include "pid/wall_follower.hpp"

#include <fstream>
#include <chrono>

WallFollower::WallFollower()
    : Node("wall_follower"),
      // Required / primary parameters
      desired_dist_(declare_parameter<double>("desired_distance", 0.5)),
      max_ang_speed_(declare_parameter<double>("max_angular_speed", 0.5)),
      fwd_speed_(declare_parameter<double>("forward_speed", 0.1)),
      pid_(
          declare_parameter<double>("kp", 0.5),
          declare_parameter<double>("ki", 0.0),
          declare_parameter<double>("kd", 0.05),
          declare_parameter<double>("anti_windup", 1.0))
{
    // Topics
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    // follower params
    use_sin_speed_ = declare_parameter<bool>("use_sinusoidal_speed", false);
    sin_amplitude_ = declare_parameter<double>("sin_speed_amplitude", 0.03);
    sin_period_ = declare_parameter<double>("sin_speed_period", 5.0);

    // leader params
    is_leader_ = declare_parameter<bool>("is_leader", false);
    wait_speed_ = declare_parameter<double>("wait_linear_speed", 0.0);
    leader_detect_center_deg_ = declare_parameter<double>("leader_detect_center_deg", 180.0);
    leader_detect_half_deg_ = declare_parameter<double>("leader_detect_half_deg", 2.0);
    leader_detect_threshold_ = declare_parameter<double>("leader_detect_threshold", 2.0);

    // pub subs
    pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WallFollower::scanCallback, this, std::placeholders::_1));

    start_time_ = now();
}

double WallFollower::estimateDistance(const sensor_msgs::msg::LaserScan &scan, double center_deg, double half_deg)
{
    constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
    double center = center_deg * DEG2RAD;
    double half = half_deg * DEG2RAD;
    double sum = 0.0;
    int cnt = 0;
    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        double ang = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        if (ang >= center - half && ang <= center + half)
        {
            double r = scan.ranges[i];
            if (std::isfinite(r))
            {
                sum += r;
                cnt++;
            }
        }
    }
    if (cnt == 0)
        return scan.range_max;
    return sum / cnt;
}

void WallFollower::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    if (msg->ranges.empty())
        return;

    double right_dist = estimateDistance(*msg, -60.0, 5.0);

    // Leader-behind detection
    double behind_dist = estimateDistance(*msg, leader_detect_center_deg_, leader_detect_half_deg_);
    leader_detected_ = (behind_dist < leader_detect_threshold_);
    if (leader_detected_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[WallFollower] Follower detected behind at %.2f m", behind_dist);
    }

    auto cmd = geometry_msgs::msg::Twist();
    double linear = fwd_speed_;
    
    // Apply sinusoidal speed variation if enabled
    if (use_sin_speed_) {
        double elapsed = (now() - start_time_).seconds();
        double sin_factor = std::sin(2.0 * 3.14159265358979323846 * elapsed / sin_period_);
        double candidate = fwd_speed_ + sin_amplitude_ * sin_factor;
        if (candidate > fwd_speed_) candidate = fwd_speed_;
        if (candidate < 0.0) candidate = 0.0;
        linear = candidate;
    }
    
    double ang_cmd = 0.0;

    // If no wall detected (right_dist >= scan.range_max), go straight
    if (right_dist < msg->range_max) {
        // Standard PID: setpoint = desired_dist_, measured = right_dist
        double dt = 0.1;
        if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
        {
            static rclcpp::Time last_time(0, 0);
            rclcpp::Time t(msg->header.stamp);
            if (last_time.nanoseconds() > 0)
            {
                dt = (t - last_time).seconds();
            }
            last_time = t;
        }
        ang_cmd = pid_.calculate_control(desired_dist_, right_dist, dt, is_leader_);
        if (ang_cmd > max_ang_speed_)
            ang_cmd = max_ang_speed_;
        else if (ang_cmd < -max_ang_speed_)
            ang_cmd = -max_ang_speed_;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "[WallFollower] ang_cmd=%.2f",
                         ang_cmd);
    cmd.angular.z = ang_cmd;

    // Leader waiting behaviour: if this node is the leader and no follower is detected behind,
    if (is_leader_ && !leader_detected_) {
        cmd.linear.x = wait_speed_;
        cmd.angular.z = 0.0;
    } else {
        cmd.linear.x = linear;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "[WallFollower] right=%.2f set=%.2f rawPID=%.2f ang=%.2f lin=%.2f",
                         right_dist, desired_dist_, ang_cmd, cmd.angular.z, cmd.linear.x);

    pub_->publish(cmd);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    std::ofstream logfile("loop_time.txt", std::ios::app);
    if (logfile.is_open()) {
        logfile << duration << std::endl;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
