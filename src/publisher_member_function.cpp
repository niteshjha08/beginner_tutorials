/**
 * @file publisher_member_function.cpp
 * @author Nitesh Jha (niteshj@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-11-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
// #include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpp_pubsub/srv/modify_string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    pub_message = "Fear the turtle!";
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initial string: " << pub_message);
    modify_string_service_ = this->create_service<cpp_pubsub::srv::ModifyString>("/modify_string",std::bind(&MinimalPublisher::modify, this,
    std::placeholders::_1, std::placeholders::_2));
    RCLCPP_DEBUG(this->get_logger(), "Initialized publisher");
  }

 private:

  void modify(const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,
            std::shared_ptr<cpp_pubsub::srv::ModifyString::Response>  response) {
          
              response->b = request->a;
              pub_message = response->b;
              RCLCPP_WARN_STREAM(this->get_logger(), "Modified string to: " << pub_message);

  }

  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = pub_message + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    RCLCPP_ERROR(this->get_logger(), "Count: %ld", count_);
    


    publisher_->publish(message);
  }
  std::string pub_message;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::ModifyString>::SharedPtr modify_string_service_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
