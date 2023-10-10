#include <chrono>#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"


using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

const float THRESHOLD = 100;
const float HEIGHT = 0.5;

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
            "/cf13/topic_name1", 10, std::bind(&SimpleCA::range_callback, this, _1));
      publisher_ = this->create_publisher<crazyflie_interfaces::msg::Hover>(
            "/cf13/cmd_hover", 10);
      RCLCPP_INFO_STREAM(this->get_logger(), "initialised!");   
    }

  private:
    //to change msg type
    void range_callback(const crazyflie_interfaces::msg::LogDataGeneric & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Front: '%f' Back: '%f' Left: '%f' Right: '%f'", (msg.values[0]),(msg.values[1]),(msg.values[2]),(msg.values[3]));
      auto message = crazyflie_interfaces::msg::Hover();
      if (msg.values[0]<THRESHOLD){
        //publish move back message
        RCLCPP_INFO_STREAM(this->get_logger(), "moving back!");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = -0.5;
        message.vy = 0.0;
        message.yaw_rate = 0.0;
        message.z_distance = HEIGHT;
      } else {
        //publish stop message
        RCLCPP_INFO_STREAM(this->get_logger(), "stopped");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = 0.0;
        message.vy = 0.0;
        message.yaw_rate = 0.0;
        message.z_distance = HEIGHT;
        
      }
      publisher_->publish(message);
    }
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr subscription_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr publisher_;
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