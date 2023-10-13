#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"


using std::placeholders::_1;

//maybe these should be ROSparameters
static const float THRESHOLD = 100;
static const float HEIGHT = 0.5;
static const float SPEED = 0.2;
static const std::string Drone_1 = "cf13";
static const std::string Drone_2 = "cf15";

class interDroneCATest : public rclcpp::Node
{
  public:
    interDroneCATest()
    : Node("interdroneCA_test"), count_(0)
    {
      //declare drone name parameter
      //publish to movement topic
      //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      //subscribe to range topic
      
      subscription1_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
            "/"+ Drone_1 +"/topic_name1", 10, std::bind(&interDroneCATest::range_callback1_, this, _1));
      publisher1_ = this->create_publisher<crazyflie_interfaces::msg::Hover>(
            "/"+ Drone_1 +"/cmd_hover", 10);
      subscription2_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
            "/"+ Drone_2 +"/topic_name1", 10, std::bind(&interDroneCATest::range_callback2_, this, _1));
      publisher2_ = this->create_publisher<crazyflie_interfaces::msg::Hover>(
            "/"+ Drone_2 +"/cmd_hover", 10);
      RCLCPP_INFO_STREAM(this->get_logger(), "initialised!");   
    }

  private:
    //to change msg type
    void range_callback1_(const crazyflie_interfaces::msg::LogDataGeneric & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Front: '%f' Back: '%f' Left: '%f' Right: '%f'", (msg.values[0]),(msg.values[1]),(msg.values[2]),(msg.values[3]));
      auto message = crazyflie_interfaces::msg::Hover();
      if (msg.values[0]<THRESHOLD){
        //publish move back message
        RCLCPP_INFO_STREAM(this->get_logger(), "moving back!");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = -SPEED;
        message.vy = 0.0;
        message.yaw_rate = 0.0;
        message.z_distance = HEIGHT;
      } else if (msg.values[0]>THRESHOLD+50){
        RCLCPP_INFO_STREAM(this->get_logger(), "moving forward!");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = SPEED;
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
      publisher1_->publish(message);
    }
    void range_callback2_(const crazyflie_interfaces::msg::LogDataGeneric & msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "Front: '%f' Back: '%f' Left: '%f' Right: '%f'", (msg.values[0]),(msg.values[1]),(msg.values[2]),(msg.values[3]));
      auto message = crazyflie_interfaces::msg::Hover();
      if (msg.values[0]<THRESHOLD){
        //publish move back message
        RCLCPP_INFO_STREAM(this->get_logger(), "moving back!");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = -SPEED;
        message.vy = 0.0;
        message.yaw_rate = 0.0;
        message.z_distance = HEIGHT;
      } else if (msg.values[0]>THRESHOLD+50){
        RCLCPP_INFO_STREAM(this->get_logger(), "moving forward!");   
        message.header.stamp = rclcpp::Clock().now();
        message.vx = SPEED;
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
      publisher2_->publish(message);
    }
    
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr subscription1_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr publisher1_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr subscription2_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr publisher2_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   
  rclcpp::spin(std::make_shared<interDroneCATest>());
  rclcpp::shutdown();
  return 0;
}