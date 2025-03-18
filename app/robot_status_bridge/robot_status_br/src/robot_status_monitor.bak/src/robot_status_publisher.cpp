#include "rclcpp/rclcpp.hpp"
#include "robot_status_monitor/msg/robot_status.hpp"
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
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

    int get_battery_percentage() {          // Host battery percentage
        float battery_percentage = execute_command("cat /sys/class/power_supply/BAT0/capacity");
        if (battery_percentage < 0) {
            battery_percentage = execute_command("cat /sys/class/power_supply/BAT1/capacity");
        }
        return battery_percentage;
    }

    bool get_charging() {                   // Host charging status
        std::string status = execute_string_command("cat /sys/class/power_supply/BAT0/status");
        if (status.empty()) {
            status = execute_string_command("cat /sys/class/power_supply/BAT1/status");
        }
        return (status.find("Charging") != std::string::npos);
    }

    float get_cpu_usage() {                 // Host CPU usage percentage
        return execute_command("top -bn1 | grep 'Cpu(s)' | awk '{print 100 - $8}'");
    }

    float get_cpu_temperature() {           // Host CPU temperature in Celsius
        return execute_command("cat /sys/class/thermal/thermal_zone0/temp") / 1000.0;
    }

    float get_ram_usage() {                 // Host RAM usage percentage
        return execute_command("free | grep Mem | awk '{print ($3/$2) * 100.0}'");
    }

    int get_wifi_signal_strength() {        // Host Wi-Fi signal strength in percentage
        return execute_command("awk 'NR==3 {print int($3 * 100 / 70)}' /proc/net/wireless");
    }

    bool get_wifi_connected() {             // Host Wi-Fi connection status
        std::string result = execute_string_command("cat /sys/class/net/wlp2s0/operstate");
        if (result.find("up") != std::string::npos) {
            return true;
        }
        return false;
    }

    std::string get_wifi_ssid() {           // Host Wi-Fi SSID
        return execute_string_command("iwgetid -r");
    }

    int get_disk_usage() {                  // Host disk usage percentage
        return execute_command("df --output=pcent / | tail -1 | tr -d '%'");
    }

    std::string get_robot_ip() {            // Host IP address
        return execute_string_command("hostname -I | awk '{print $1}'");
    }

    float get_uptime() {                    // Host uptime in hours
        return execute_command("awk '{print $1/3600}' /proc/uptime");
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

    std::string execute_string_command(const std::string& command) {
        char buffer[128];
        std::string result;
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) return "";
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);
        return result;
    }

    void publish_status() {
        auto msg = robot_status_monitor::msg::RobotStatus();
        msg.battery_percentage = get_battery_percentage();
        msg.charging = get_charging();
        msg.cpu_usage = get_cpu_usage();
        msg.cpu_temperature = get_cpu_temperature();
        msg.ram_usage = get_ram_usage();
        msg.wifi_signal_strength = get_wifi_signal_strength();
        msg.wifi_connected = get_wifi_connected();
        msg.wifi_ssid = get_wifi_ssid();
        msg.disk_usage = get_disk_usage();
        msg.robot_ip = get_robot_ip();
        msg.uptime = get_uptime();

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Battery: %d%%, Charging: %s, CPU: %.2f%%, Temp: %.2f°C, RAM: %.2f%%, WiFi Signal: %d%%, WiFi Connected: %s, WiFi SSID: %s, Disk Usage: %d%%, IP: %s, Uptime: %.2f hours",
                msg.battery_percentage,
                msg.charging ? "Yes" : "No",
                msg.cpu_usage, msg.cpu_temperature,
                msg.ram_usage,
                msg.wifi_signal_strength,
                msg.wifi_connected ? "Yes" : "No",
                msg.wifi_ssid.c_str(),
                msg.disk_usage,
                msg.robot_ip.c_str(),
                msg.uptime);
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
