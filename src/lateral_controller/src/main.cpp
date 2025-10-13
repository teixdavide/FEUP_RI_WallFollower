#include <lateral_controller/lateral_controller.hpp>

int main(){
    rclcpp::init(0, nullptr);

    config_file = declare_parameter<std::string>("config_file", "config/lateral_controller.yaml");

    auto pid_controller = PID(1.0, 0.1, 0.05, 10.0);
    double desired_distance = 2.0;

    auto lateral_controller_node = std::make_shared<LateralController>(pid_controller, desired_distance);
    rclcpp::spin(lateral_controller_node);
    rclcpp::shutdown();
    return 0;
}

