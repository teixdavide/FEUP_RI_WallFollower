
#include "pid/wall_follower.hpp"

WallFollower::WallFollower()
    : Node("wall_follower"),
      desired_dist_(declare_parameter<double>("desired_distance", 0.5)),
      max_ang_speed_(declare_parameter<double>("max_angular_speed", 0.5)),
      fwd_speed_(declare_parameter<double>("forward_speed", 0.1)),
      pid_(
          declare_parameter<double>("kp", 0.5),
          declare_parameter<double>("ki", 0.0),
          declare_parameter<double>("kd", 0.05),
          declare_parameter<double>("anti_windup", 1.0))
{

    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WallFollower::scanCallback, this, std::placeholders::_1));
    // Optional follower proximity coordination
    publish_prox_ = declare_parameter<bool>("publish_proximity", false);
    wait_for_follower_ = declare_parameter<bool>("wait_for_follower", false);
    prox_topic_ = declare_parameter<std::string>("proximity_topic", std::string("proximity_ok"));
    follower_prox_topic_ = declare_parameter<std::string>("follower_proximity_topic", std::string("/follower/proximity_ok"));
    prox_front_deg_ = declare_parameter<double>("proximity_front_deg", 10.0);
    prox_front_thresh_ = declare_parameter<double>("proximity_front_threshold", 0.5);
    follower_timeout_s_ = declare_parameter<double>("follower_timeout", 3.0);
    wait_speed_ = declare_parameter<double>("wait_linear_speed", 0.0);
    use_sin_speed_ = declare_parameter<bool>("use_sinusoidal_speed", false);
    sin_amplitude_ = declare_parameter<double>("sin_speed_amplitude", 0.03);
    sin_period_ = declare_parameter<double>("sin_speed_period", 5.0);
    start_time_ = now();
    if (publish_prox_) {
        prox_pub_ = create_publisher<std_msgs::msg::Bool>(prox_topic_, rclcpp::SystemDefaultsQoS());
    }
    if (wait_for_follower_) {
        prox_sub_ = create_subscription<std_msgs::msg::Bool>(
            follower_prox_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Bool::SharedPtr msg){
                if (msg->data) last_follower_ok_ = this->now();
            }
        );
    }
}

double WallFollower::estimateRightDistance(const sensor_msgs::msg::LaserScan &scan)
{
    constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
    double center = -60.0 * DEG2RAD;
    double half = 10.0 * DEG2RAD;
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
    if (msg->ranges.empty())
        return;

    double right_dist = estimateRightDistance(*msg);
    // Follower proximity detection (front window)
    if (publish_prox_) {
        const double DEG2RAD = 3.14159265358979323846 / 180.0;
        double half = prox_front_deg_ * DEG2RAD;
        double min_front = msg->range_max;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
            if (ang >= -half && ang <= half) {
                double r = msg->ranges[i];
                if (std::isfinite(r) && r >= msg->range_min && r < min_front) {
                    min_front = r;
                }
            }
        }
        std_msgs::msg::Bool b; b.data = (min_front < prox_front_thresh_);
        prox_pub_->publish(b);
    }

    auto cmd = geometry_msgs::msg::Twist();
    // Leader waiting logic if enabled
    double linear = fwd_speed_;
    
    // Apply sinusoidal speed variation if enabled
    if (use_sin_speed_) {
        double elapsed = (now() - start_time_).seconds();
        double sin_factor = std::sin(2.0 * 3.14159265358979323846 * elapsed / sin_period_);
        linear = fwd_speed_ + sin_amplitude_ * sin_factor;
        if (linear < 0.0) linear = 0.0;
    }
    
    if (wait_for_follower_) {
        rclcpp::Time nowt = now();
        if (last_follower_ok_.nanoseconds() == 0 || (nowt - last_follower_ok_).seconds() > follower_timeout_s_) {
            linear = wait_speed_;
        }
    }
    cmd.linear.x = linear;
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
        ang_cmd = pid_.calculate_control(desired_dist_, right_dist, dt);
        if (ang_cmd > max_ang_speed_)
            ang_cmd = max_ang_speed_;
        else if (ang_cmd < -max_ang_speed_)
            ang_cmd = -max_ang_speed_;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "[WallFollower] ang_cmd=%.2f",
                         ang_cmd);
    cmd.angular.z = ang_cmd;

    // Detailed debug log
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "[WallFollower] right=%.2f set=%.2f rawPID=%.2f ang=%.2f lin=%.2f",
                         right_dist, desired_dist_, ang_cmd, cmd.angular.z, cmd.linear.x);

    pub_->publish(cmd);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}
