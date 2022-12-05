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
 * @file main.cpp
 * @author Nitesh Jha (niteshj@umd.edu)
 * @brief
 * @version 0.1
 * @date 2022-12-5
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <gtest/gtest.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include "cpp_pubsub/publisher_member_function.hpp"

using namespace std::chrono_literals;

class TalkerTest : public ::testing::Test {
 public:
  TalkerTest() {}
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node = std::make_shared<MinimalPublisher>();
    clock_ = std::make_unique<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(clock_);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

  void TearDown() override { rclcpp::shutdown(); }

 protected:
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<MinimalPublisher> node;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

TEST(demo, test1) { ASSERT_EQ(1, 1); }

TEST_F(TalkerTest, test2) {
  geometry_msgs::msg::TransformStamped transform;
  auto start = clock_->now();
  double duration_sec = 0.0;

  while (duration_sec < 5.0) {
    rclcpp::spin_some(node);
    duration_sec = (clock_->now() - start).seconds();
  }
  try {
    transform = tf_buffer->lookupTransform("world", "talk", rclcpp::Time(0));
    std::cout << "Transform: " << transform.transform.translation.x
              << std::endl;
  } catch (tf2::TransformException& ex) {
    std::cout << "Transform not available" << std::endl;
    FAIL();
  }

  auto tx = transform.transform.translation.x;
  EXPECT_FLOAT_EQ(tx, 1.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
