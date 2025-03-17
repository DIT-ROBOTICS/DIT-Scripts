#include "rclcpp/rclcpp.hpp"
#include "robot_status_monitor/msg/robot_status.hpp"
#include <cstdlib>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <algorithm>

class RobotStatusPublisher : public rclcpp::Node {
public:
    RobotStatusPublisher() : Node("robot_status_publisher") {
        std::string hostname = get_valid_hostname();
        topic_name_ = "/" + hostname + "/robot_status";

        publisher_ = this->create_publisher<robot_status_monitor::msg::RobotStatus>(topic_name_, 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&RobotStatusPublisher::publish_status, this));

        RCLCPP_INFO(this->get_logger(), "Publishing robot status to topic: %s", topic_name_.c_str());
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

    float get_battery_percentage() {
        return execute_command("cat /sys/class/power_supply/BAT1/capacity");
    }

    float get_cpu_usage() {
        return execute_command("top -bn1 | grep 'Cpu(s)' | awk '{print 100 - $8}'");
    }

    float get_cpu_temperature() {
        return execute_command("cat /sys/class/thermal/thermal_zone0/temp") / 1000.0;
    }

    float get_ram_usage() {
        return execute_command("free | grep Mem | awk '{print ($3/$2) * 100.0}'");
    }

    float execute_command(const std::string& command) {
        char buffer[128];
        std::string result;
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) return -1.0;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);
        try {
            return std::stof(result);
        } catch (...) {
            return -1.0;
        }
    }

    void publish_status() {
        auto msg = robot_status_monitor::msg::RobotStatus();
        msg.battery_percentage = get_battery_percentage();
        msg.cpu_usage = get_cpu_usage();
        msg.cpu_temperature = get_cpu_temperature();
        msg.ram_usage = get_ram_usage();

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Battery: %.2f%%, CPU: %.2f%%, Temp: %.2f°C, RAM: %.2f%%",
                    msg.battery_percentage, msg.cpu_usage, msg.cpu_temperature, msg.ram_usage);
    }

    std::string topic_name_;
    rclcpp::Publisher<robot_status_monitor::msg::RobotStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
