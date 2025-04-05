#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <fstream>
#include <string>

class BatteryMonitor : public rclcpp::Node {
public:
    BatteryMonitor() : Node("battery_monitor") {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&BatteryMonitor::publish_battery_status, this));
    }

private:
    void publish_battery_status() {
        float battery_percentage = get_battery_percentage();
        if (battery_percentage < 0.0) {
            RCLCPP_WARN(this->get_logger(), "Failed to read battery percentage!");
            return;
        }

        auto msg = std_msgs::msg::Float32();
        msg.data = battery_percentage;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published Battery Percentage: %.2f%%", battery_percentage);
    }

    float get_battery_percentage() {
        
        std::ifstream file("/sys/class/power_supply/BAT0/capacity");
        if (!file.is_open()) {
            file.open("/sys/class/power_supply/BAT1/capacity");
        }
        if (!file.is_open()) {
            return -1.0;
        }

        std::string line;
        std::getline(file, line);
        file.close();

        try {
            return std::stof(line);
        } catch (...) {
            return -1.0;
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
