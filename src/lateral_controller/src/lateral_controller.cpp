#include <lateral_controller/lateral_controller.hpp>

LateralController::LateralController(PID pid_controller, double desired_distance)
: Node("lateral_controller"), pid_controller(pid_controller), desired_distance(desired_distance)
{
    distance_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        "/sonar/distance", 10,
        std::bind(&LateralController::distance_callback, this, std::placeholders::_1));

    yaw_rate_publisher = this->create_publisher<std_msgs::msg::Float64>("/robot/yaw_rate", 10);
}

void LateralController::distance_callback(std_msgs::msg::Float64 msg)
{
    if (last_time == std::chrono::steady_clock::time_point()) {
        last_time = std::chrono::steady_clock::now();
        return;
    }
    
    double current_distance = msg.data;

    // Elapsed time calculation in seconds
    std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = timestamp - last_time;
    last_time = timestamp;
    double dt = elapsed.count();

    double control_signal = pid_controller.calculate_control(desired_distance, current_distance, dt);

    auto yaw_rate_msg = std_msgs::msg::Float64();
    yaw_rate_msg.data = control_signal;
    yaw_rate_publisher->publish(yaw_rate_msg);
}