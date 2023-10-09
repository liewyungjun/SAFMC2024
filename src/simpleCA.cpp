#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"


using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class SimpleCA : public rclcpp::Node
{
  public:
    SimpleCA()
    : Node("simple_ca"), count_(0)
    {
      //declare drone name parameter
      //publish to movement topic
      //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      //subscribe to range topic
      
      subscription_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
            "/cf15/topic_name1", 10, std::bind(&SimpleCA::range_callback, this, _1));
      RCLCPP_INFO_STREAM(this->get_logger(), "initialised!");   
      RCLCPP_DEBUG(this->get_logger(), "test log debug");
      RCLCPP_INFO(this->get_logger(), "test log info");
      RCLCPP_WARN(this->get_logger(), "test log warn");
      RCLCPP_ERROR(this->get_logger(), "test log error");  
    }

  private:
    //to change msg type
    void range_callback(const crazyflie_interfaces::msg::LogDataGeneric & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard something");
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", (char)(msg.values[0]));
    }
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr subscription_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   
  rclcpp::spin(std::make_shared<SimpleCA>());
  rclcpp::shutdown();
  return 0;
}