#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class BatterySubscriber : public rclcpp::Node {
public:
    BatterySubscriber() : Node("battery_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "battery_percentage", 10, std::bind(&BatterySubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Battery Percentage: %.2f%%", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatterySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
