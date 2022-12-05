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
 * @file publisher_member_function.hpp
 * @author Nitesh Jha (niteshj@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-5
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef INCLUDE_CPP_PUBSUB_PUBLISHER_MEMBER_FUNCTION_HPP_
#define INCLUDE_CPP_PUBSUB_PUBLISHER_MEMBER_FUNCTION_HPP_

#include <string>
#include <memory>

#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"

#include "cpp_pubsub/srv/modify_string.hpp"

/**
 * @brief Class for a minimal publisher node that publishes a string on topic 'topic', and a tf frame.
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher();

 private:
 /**
  * @brief Callback function for the service ModifyString
  * 
  * @param request 
  * @param response 
  */
  void modify(
      const std::shared_ptr<cpp_pubsub::srv::ModifyString::Request> request,
      std::shared_ptr<cpp_pubsub::srv::ModifyString::Response> response);

    /**
     * @brief Callback function for the timer
     * 
     */
  void timer_callback();
  // timer member variable
  rclcpp::TimerBase::SharedPtr timer_;
  // publisher member variable
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // service member variable
  rclcpp::Service<cpp_pubsub::srv::ModifyString>::SharedPtr
      modify_string_service_;
    // message to be published
  std::string pub_message_;
    // count of the number of times the timer has been called
  size_t count_;
    // tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // transform
  geometry_msgs::msg::TransformStamped t;
};

#endif  // INCLUDE_CPP_PUBSUB_PUBLISHER_MEMBER_FUNCTION_HPP_
