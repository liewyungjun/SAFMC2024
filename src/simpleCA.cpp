#include <chrono>#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
//TODO: add takeoff service call

using std::placeholders::_1;  

class SimpleCA : public rclcpp::Node
{
  public:
    SimpleCA()
    : Node("simple_ca"), count_(0)
    {

      this->declare_parameter("threshold", 100.0);
      this->declare_parameter("height", 0.5);
      this->declare_parameter("speed", 0.2);
      this->declare_parameter("drone", "cf13");

      this->get_parameter("threshold",threshold_);
      this->get_parameter("height",height_);
      this->get_parameter("speed",speed_);
      this->get_parameter("drone",drone_);
  
      subscription_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
            "/" + drone_ + "/topic_name1", 10, std::bind(&SimpleCA::range_callback, this, _1));
      publisher_ = this->create_publisher<crazyflie_interfaces::msg::Hover>(
            "/" + drone_ + "/cmd_hover", 10);
      RCLCPP_INFO_STREAM(this->get_logger(), "initialised!");   
    }

  private:
    //to change msg type
    void range_callback(const crazyflie_interfaces::msg::LogDataGeneric & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Front: '%f' Back: '%f' Left: '%f' Right: '%f'", (msg.values[0]),(msg.values[1]),(msg.values[2]),(msg.values[3]));
      auto message = crazyflie_interfaces::msg::Hover();
      if (msg.values[0]<threshold_){
        //publish move back message
        RCLCPP_INFO_STREAM(this->get_logger(), "moving back!");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = -speed_;
        message.vy = 0.0;
        message.yaw_rate = 0.0;
        message.z_distance = height_;
      } else if (msg.values[0]>threshold_+50){
        RCLCPP_INFO_STREAM(this->get_logger(), "moving forward!");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = speed_;
        message.vy = 0.0;
        message.yaw_rate = 0.0;
        message.z_distance = height_;
      } else {
        //publish stop message
        RCLCPP_INFO_STREAM(this->get_logger(), "stopped");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = 0.0;
        message.vy = 0.0;
        message.yaw_rate = 0.0;
        message.z_distance = height_;
        
      }
      publisher_->publish(message);
    }
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr subscription_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr publisher_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    float threshold_;
    float height_;
    float speed_;
    std::string drone_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   
  rclcpp::spin(std::make_shared<SimpleCA>());
  rclcpp::shutdown();
  return 0;
}