/**
MIT License

Copyright (c) 2022 Nitesh Jha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

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
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include "cpp_pubsub/srv/modify_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
 /**
  * @brief Constructor for ROS2 node
  * 
  */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // get launch arg for start of count
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Count starts from this parameter value";

    int count;
    this->declare_parameter("count", count, param_desc);

    count =
    this->get_parameter("count").get_parameter_value().get<int>();
    count_ = count;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Count begins from: " << count_);
    // timer for publishing at regular intervals
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    pub_message = "Fear the turtle!";
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initial string: " << pub_message);
    // create callback for service
    modify_string_service_ =
        this->create_service<cpp_pubsub::srv::ModifyString>(
            "/modify_string",
            std::bind(&MinimalPublisher::modify, this, std::placeholders::_1,
                      std::placeholders::_2));
    RCLCPP_DEBUG(this->get_logger(), "Initialized publisher");
  }

 private:
 /**
  * @brief Service callback function which modifies the published string
  * 
  * @param request string
  * @param response string
  */
  void modify(
      const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,
      std::shared_ptr<cpp_pubsub::srv::ModifyString::Response> response) {
    response->b = request->a;
    pub_message = response->b;
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Modified string to: " << pub_message);
  }
  // publish string at regular intervals 
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
  rclcpp::Service<cpp_pubsub::srv::ModifyString>::SharedPtr
      modify_string_service_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
