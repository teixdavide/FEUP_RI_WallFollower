#include <longitudinal_controller/longitudinal_controller.hpp>

LongitudinalController::LongitudinalController(PID pid_controller, double desired_distance)
: Node("longitudinal_controller"), pid_controller(pid_controller), desired_distance(desired_distance)
{
    distance_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        "/sonar/distance", 10,
        std::bind(&LongitudinalController::distance_callback, this, std::placeholders::_1));

    velocity_publisher = this->create_publisher<std_msgs::msg::Float64>("/robot/velocity", 10);
}

void LongitudinalController::distance_callback(std_msgs::msg::Float64 msg)
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

    auto velocity_msg = std_msgs::msg::Float64();
    velocity_msg.data = control_signal;
    velocity_publisher->publish(velocity_msg);
}