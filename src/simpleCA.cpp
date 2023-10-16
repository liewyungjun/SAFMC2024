#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "builtin_interfaces/msg/duration.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
//TODO: add takeoff service call



using std::placeholders::_1;  

class SimpleCA : public rclcpp::Node
{
  public:
    SimpleCA()
    : Node("simple_ca")
    {

      this->declare_parameter("threshold", 100.0);
      this->declare_parameter("height", 1.0);
      this->declare_parameter("speed", 0.2);
      this->declare_parameter("drone", "cf13");
      this->declare_parameter("timer_period_ms", 1000);


      this->get_parameter("threshold",threshold_);
      this->get_parameter("height",height_);
      this->get_parameter("speed",speed_);
      this->get_parameter("drone",drone_);
      this->get_parameter("timer_period_ms", timer_period_ms_);

  
      subscription_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
            "/" + drone_ + "/topic_name1", 10, std::bind(&SimpleCA::range_callback, this, _1));
      publisher_ = this->create_publisher<crazyflie_interfaces::msg::Hover>(
            "/" + drone_ + "/cmd_hover", 10);
      // Create a timer to trigger the range callback
      timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms_), std::bind(&SimpleCA::timer_callback, this));
      timer_ ->cancel();
      getchar();
      timer_->reset();
      
      // Register the SIGINT handler
      //std::signal(SIGINT, &SimpleCA::sigint_handler);

      // Initialize the range vector
      range_vector_.reserve(10);
      RCLCPP_INFO_STREAM(this->get_logger(), "initialised!"); 

      takeoff_client_ = this->create_client<crazyflie_interfaces::srv::Takeoff>("/" + drone_ + "/takeoff");
      // Wait for the takeoff service to be available
      while (!takeoff_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      }

      // Call the takeoff service
      auto request = std::make_shared<crazyflie_interfaces::srv::Takeoff::Request>();
      request->height = height_;
      builtin_interfaces::msg::Duration duration;
      duration.sec = 2;
      request->duration = duration;
      auto result = takeoff_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Takeoff service succeeded");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Takeoff service failed");
      }
    }

  private:
    void timer_callback(){
      // Calculate the average range
      float sum = 0.0;
      for (auto range : range_vector_) {
        sum += range;
      }
      float average_range = sum / range_vector_.size();
      RCLCPP_INFO_STREAM(this->get_logger(), "Average range: " << average_range);

      auto message = crazyflie_interfaces::msg::Hover();
        if (average_range<threshold_){
          //publish move back message
          RCLCPP_INFO_STREAM(this->get_logger(), "moving back!");   
          message.header.stamp = rclcpp::Clock().now();
          message.vx = -speed_;
          message.vy = 0.0;
          message.yaw_rate = 0.0;
          message.z_distance = height_;
        } else if (average_range>threshold_+50){
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
        range_vector_.clear();
    }
    void range_callback(const crazyflie_interfaces::msg::LogDataGeneric & msg)
    {
      range_vector_.push_back(msg.values[0]);
      //RCLCPP_INFO(this->get_logger(), "Front: '%f' Back: '%f' Left: '%f' Right: '%f'", (msg.values[0]),(msg.values[1]),(msg.values[2]),(msg.values[3]));
    }
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr subscription_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr publisher_;
    rclcpp::Client<crazyflie_interfaces::srv::Takeoff>::SharedPtr takeoff_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> range_vector_;
    float threshold_;
    float height_;
    float speed_;
    std::string drone_;
    int timer_period_ms_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
   
  rclcpp::spin(std::make_shared<SimpleCA>());
  rclcpp::shutdown();
  return 0;
}