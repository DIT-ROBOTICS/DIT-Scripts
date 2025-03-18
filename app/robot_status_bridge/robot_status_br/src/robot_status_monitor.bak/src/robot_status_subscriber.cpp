#include "rclcpp/rclcpp.hpp"
#include "robot_status_monitor/msg/robot_status.hpp"
#include <unistd.h>
#include <algorithm>

class RobotStatusSubscriber : public rclcpp::Node {
public:
    RobotStatusSubscriber() : Node("robot_status_subscriber") {
        std::string hostname = get_valid_hostname();
        std::string topic_name = "/" + hostname + "/robot_status";

        subscription_ = this->create_subscription<robot_status_monitor::msg::RobotStatus>(
            topic_name, 10, 
            std::bind(&RobotStatusSubscriber::callback, this, std::placeholders::_1));
    }

private:
    std::string get_valid_hostname() {
        char hostname[128];
        if (gethostname(hostname, sizeof(hostname)) != 0) {
            return "unknown_robot";
        }
        std::string valid_hostname(hostname);
        std::replace(valid_hostname.begin(), valid_hostname.end(), '-', '_');
        valid_hostname.erase(std::remove_if(valid_hostname.begin(), valid_hostname.end(),
            [](char c) { return !std::isalnum(c) && c != '_'; }), valid_hostname.end());
        return valid_hostname.empty() ? "unknown_robot" : valid_hostname;
    }

    void callback(const robot_status_monitor::msg::RobotStatus::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
                    "Battery: %d%%, Charging: %s, CPU: %.2f%%, Temp: %.2f°C, RAM: %.2f%%, WiFi Signal: %d%%, WiFi Connected: %s, WiFi SSID: %s, Disk Usage: %d%%, IP: %s, Uptime: %.2f hours",
                    msg->battery_percentage,
                    msg->charging ? "Yes" : "No",
                    msg->cpu_usage, msg->cpu_temperature,
                    msg->ram_usage,
                    msg->wifi_signal_strength,
                    msg->wifi_connected ? "Yes" : "No",
                    msg->wifi_ssid.c_str(),
                    msg->disk_usage,
                    msg->robot_ip.c_str(),
                    msg->uptime);
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
