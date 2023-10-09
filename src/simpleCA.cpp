#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


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
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1, std::bind(&SimpleCA::range_callback, this, _1));
    }

  private:
    //to change msg type
    void range_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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