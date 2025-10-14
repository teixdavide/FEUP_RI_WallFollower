#include <longitudinal_controller/longitudinal_controller.hpp>
#include <string>

int main(){
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<rclcpp::Node>("longitudinal_controller_launcher");
    std::string config_file = node->declare_parameter<std::string>("config_file", "config/longitudinal_controller.yaml");;

    auto pid_controller = PID(1.0, 0.1, 0.05, 10.0);
    double desired_distance = 2.0;

    auto longitudinal_controller_node = std::make_shared<LongitudinalController>(pid_controller, desired_distance);
    rclcpp::spin(longitudinal_controller_node);
    rclcpp::shutdown();
    return 0;
}

