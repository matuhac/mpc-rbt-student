#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("battery_node")
    {
    	this->declare_parameter("MaxVoltage", 42.0);
    	this->declare_parameter("MinVoltage", 32.0);
    
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float32>("battery_voltage", 10, std::bind(&MyNode::battery_callback, this, _1));

    }

private:
    
    void battery_callback(const std_msgs::msg::Float32 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.data);
      auto percentage_msg = std_msgs::msg::Float32();
      
      percentage_msg.data = 100 * (msg.data - this->get_parameter("MaxVoltage").as_double())/(this->get_parameter("MaxVoltage").as_double() - this->get_parameter("MinVoltage").as_double());
      publisher_->publish(percentage_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
