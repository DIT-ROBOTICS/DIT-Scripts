#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdlib>
#include <string>
#include <map>
#include <chrono>

class RobotStatusPublisher : public rclcpp::Node {
public:
    RobotStatusPublisher() : Node("robot_status_node") {
std::string hostname = get_valid_hostname();

        // std::string hostname = get_valid_hostname();

        // Set the refresh rate for each topic (seconds)
        topic_refresh_rates_ = {
            {"/robot_status/battery", 5.0},
            {"/robot_status/charging", 10.0},
            {"/robot_status/power", 1.0},
            {"/robot_status/cpu", 1.0},      
            {"/robot_status/cpu_temp", 1.0},
            {"/robot_status/ram", 1.0},
            {"/robot_status/wifi_signal", 1.0},
            {"/robot_status/wifi_connected", 5.0},
            {"/robot_status/wifi_ssid", 5.0},
            {"/robot_status/disk", 60.0},
            {"/robot_status/robot_ip", 5.0},
            {"/robot_status/uptime", 30.0},
            {"/robot_status/usb/mission", 5.0},
            {"/robot_status/usb/chassis", 5.0},
            {"/robot_status/usb/lidar", 5.0},
            {"/robot_status/usb/esp", 5.0},
            {"/robot_status/usb/imu", 5.0},
        };

        // Set the shell commands for each topic
        status_commands_ = {
            // Get battery percentage from BAT0 or BAT1
            {"/robot_status/battery", {"std_msgs::msg::Int32", "cat /sys/class/power_supply/BAT0/capacity 2>/dev/null || cat /sys/class/power_supply/BAT1/capacity 2>/dev/null" }},
            // Check if BAT0 or BAT1 is charging
            {"/robot_status/charging", {"std_msgs::msg::Bool", "cat /sys/class/power_supply/BAT0/status 2>/dev/null | grep -q 'Charging' || cat /sys/class/power_supply/BAT1/status 2>/dev/null | grep -q 'Charging' && echo 1 || echo 0" }},
            // Get real-time power consumption in watts
            {"/robot_status/power", {"std_msgs::msg::Float32", "awk '{getline v < \"/sys/class/power_supply/BAT0/voltage_now\"; printf \"%.1f\", v * $1 / 1000000000000}' /sys/class/power_supply/BAT0/current_now" }},
            // Get CPU usage percentage
            {"/robot_status/cpu", {"std_msgs::msg::Float32", "top -bn1 | grep 'Cpu(s)' | awk '{print 100 - $8}'" }},
            // Get CPU temperature in Celsius
            {"/robot_status/cpu_temp", {"std_msgs::msg::Float32", "cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{print $1/1000}'" }},
            // Get RAM usage percentage
            {"/robot_status/ram", {"std_msgs::msg::Float32", "free | grep Mem | awk '{print ($3/$2) * 100.0}'" }},
            // Get Wi-Fi signal strength
            {"/robot_status/wifi_signal", {"std_msgs::msg::Int32", "awk 'NR==3 {print int($3 * 100 / 70)}' /proc/net/wireless" }},
            // Check if Wi-Fi is connected
            {"/robot_status/wifi_connected", {"std_msgs::msg::Bool", "cat /sys/class/net/wlp2s0/operstate 2>/dev/null | grep -q 'up' && echo 1 || echo 0" }},
            // Get Wi-Fi SSID
            {"/robot_status/wifi_ssid", {"std_msgs::msg::String", "iwgetid -r 2>/dev/null" }},
            // Get disk usage percentage
            {"/robot_status/disk", {"std_msgs::msg::Int32", "df --output=pcent / | tail -1 | tr -d '%'" }},
            // Get robot IP address
            {"/robot_status/robot_ip", {"std_msgs::msg::String", "hostname -I | awk '{print $1}'" }},
            // Get system uptime in hours
            {"/robot_status/uptime", {"std_msgs::msg::Float32", "awk '{print $1/3600}' /proc/uptime" }},
            {"/robot_status/usb/mission", {"std_msgs::msg::Bool", "[ -e /dev/mission ] && echo 1 || echo 0" }},
            {"/robot_status/usb/chassis", {"std_msgs::msg::Bool", "[ -e /dev/chassis ] && echo 1 || echo 0" }},
            {"/robot_status/usb/lidar", {"std_msgs::msg::Bool", "[ -e /dev/lidar ] && echo 1 || echo 0" }},
            {"/robot_status/usb/esp", {"std_msgs::msg::Bool", "[ -e /dev/esp ] && echo 1 || echo 0" }},
            {"/robot_status/usb/imu", {"std_msgs::msg::Bool", "lsusb | grep -E '06c2:00[3-a][0-f]' > /dev/null && echo 1 || echo 0" }},
        };

        // Create publishers and timers based on the data type
        for (const auto& [topic, cmd_info] : status_commands_) {
            double refresh_rate = topic_refresh_rates_[topic];

            if (cmd_info.type == "std_msgs::msg::Float32") {
                float_publishers_[topic] = this->create_publisher<std_msgs::msg::Float32>(topic, 10);
            } else if (cmd_info.type == "std_msgs::msg::Bool") {
                bool_publishers_[topic] = this->create_publisher<std_msgs::msg::Bool>(topic, 10);
            } else if (cmd_info.type == "std_msgs::msg::Int32") {
                int_publishers_[topic] = this->create_publisher<std_msgs::msg::Int32>(topic, 10);
            } else if (cmd_info.type == "std_msgs::msg::String") {
                string_publishers_[topic] = this->create_publisher<std_msgs::msg::String>(topic, 10);
            }

            timers_[topic] = this->create_wall_timer(
                std::chrono::duration<double>(refresh_rate), 
                [this, topic, cmd_info]() { publish_status(topic, cmd_info); }
            );
        }
    }

private:
    struct CommandInfo {
        std::string type;
        std::string command;
    };

    std::map<std::string, CommandInfo> status_commands_;
    std::map<std::string, double> topic_refresh_rates_; 
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> float_publishers_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> bool_publishers_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> int_publishers_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> string_publishers_;
    std::map<std::string, rclcpp::TimerBase::SharedPtr> timers_;

    void publish_status(const std::string& topic, const CommandInfo& cmd_info) {
        std::string result = execute_command(cmd_info.command);

        if (cmd_info.type == "std_msgs::msg::Float32") {
            auto msg = std_msgs::msg::Float32();
            msg.data = std::stof(result);
            float_publishers_[topic]->publish(msg);
        } else if (cmd_info.type == "std_msgs::msg::Bool") {
            auto msg = std_msgs::msg::Bool();
            msg.data = (result == "1");
            bool_publishers_[topic]->publish(msg);
        } else if (cmd_info.type == "std_msgs::msg::Int32") {
            auto msg = std_msgs::msg::Int32();
            msg.data = std::stoi(result);
            int_publishers_[topic]->publish(msg);
        } else if (cmd_info.type == "std_msgs::msg::String") {
            auto msg = std_msgs::msg::String();
            msg.data = result;
            string_publishers_[topic]->publish(msg);
        }

        RCLCPP_INFO(this->get_logger(), "Published %s: %s", topic.c_str(), result.c_str());
    }

    std::string execute_command(const std::string& command) {
        char buffer[128];
        std::string result;
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) return "0";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);
        result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
        return result.empty() ? "0" : result;
    }

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
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
