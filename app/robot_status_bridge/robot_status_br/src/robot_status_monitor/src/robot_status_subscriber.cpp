#include "rclcpp/rclcpp.hpp"
#include "robot_status_monitor/msg/robot_status.hpp"

class RobotStatusSubscriber : public rclcpp::Node {
public:
    RobotStatusSubscriber() : Node("robot_status_subscriber") {
        subscription_ = this->create_subscription<robot_status_monitor::msg::RobotStatus>(
            "/SCX_ROG_Zephyrus_G16/robot_status", 10, 
            std::bind(&RobotStatusSubscriber::callback, this, std::placeholders::_1));
    }

private:
    void callback(const robot_status_monitor::msg::RobotStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Battery: %.2f%%, CPU: %.2f%%, Temp: %.2f°C, RAM: %.2f%%",
                    msg->battery_percentage, msg->cpu_usage, msg->cpu_temperature, msg->ram_usage);
    }

    rclcpp::Subscription<robot_status_monitor::msg::RobotStatus>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
