// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "nao_lola_client/nao_lola_client.hpp"

// class TestNaoLolaClient : public ::testing::Test
// {
// protected:
//   static void SetUpTestCase()
//   {
//     rclcpp::init(0, nullptr);
//   }

//   static void TearDownTestCase()
//   {
//     rclcpp::shutdown();
//   }

//   void test_parameter(
//     const std::string & parameter_name,
//     const rclcpp::ParameterValue & expected_value)
//   {
//     ASSERT_TRUE(node.has_parameter(parameter_name));
//     auto parameter = node.get_parameter(parameter_name);
//     EXPECT_EQ(parameter.get_type(), expected_value.get_type());
//     EXPECT_EQ(parameter.get_parameter_value(), expected_value);
//   }

//   void test_publisher(const std::string & topic_name, const std::string & topic_type)
//   {
//     auto publishers_info = node.get_publishers_info_by_topic(topic_name);
//     ASSERT_EQ(publishers_info.size(), 1);
//     EXPECT_EQ(publishers_info[0].topic_type(), topic_type);
//   }

//   void test_subscription(const std::string & topic_name, const std::string & topic_type)
//   {
//     auto subscriptions_info = node.get_subscriptions_info_by_topic(topic_name);
//     EXPECT_EQ(subscriptions_info.size(), 1);
//     EXPECT_EQ(subscriptions_info[0].topic_type(), topic_type);
//   }

//   NaoLolaClient node;
// };

// TEST_F(TestNaoLolaClient, TestParameters)
// {
//   SCOPED_TRACE("TestParameters");
//   test_parameter("publish_joint_states", rclcpp::ParameterValue{true});
// }

// TEST_F(TestNaoLolaClient, TestPublishers)
// {
//   SCOPED_TRACE("TestPublishers");
//   test_publisher("sensors/accelerometer", "nao_lola_sensor_msgs/msg/Accelerometer");
//   test_publisher("sensors/angle", "nao_lola_sensor_msgs/msg/Angle");
//   test_publisher("sensors/buttons", "nao_lola_sensor_msgs/msg/Buttons");
//   test_publisher("sensors/fsr", "nao_lola_sensor_msgs/msg/FSR");
//   test_publisher("sensors/gyroscope", "nao_lola_sensor_msgs/msg/Gyroscope");
//   test_publisher("sensors/joint_positions", "nao_lola_sensor_msgs/msg/JointPositions");
//   test_publisher("sensors/joint_stiffnesses", "nao_lola_sensor_msgs/msg/JointStiffnesses");
//   test_publisher("sensors/joint_temperatures", "nao_lola_sensor_msgs/msg/JointTemperatures");
//   test_publisher("sensors/joint_currents", "nao_lola_sensor_msgs/msg/JointCurrents");
//   test_publisher("sensors/joint_statuses", "nao_lola_sensor_msgs/msg/JointStatuses");
//   test_publisher("sensors/sonar", "nao_lola_sensor_msgs/msg/Sonar");
//   test_publisher("sensors/touch", "nao_lola_sensor_msgs/msg/Touch");
//   test_publisher("sensors/battery", "nao_lola_sensor_msgs/msg/Battery");
//   test_publisher("sensors/robot_config", "nao_lola_sensor_msgs/msg/RobotConfig");
// }

// TEST_F(TestNaoLolaClient, TestSubscriptions)
// {
//   SCOPED_TRACE("TestSubscriptions");
//   test_subscription("effectors/joint_positions", "nao_lola_command_msgs/msg/JointPositions");
//   test_subscription("effectors/joint_stiffnesses", "nao_lola_command_msgs/msg/JointStiffnesses");
//   test_subscription("effectors/chest_led", "nao_lola_command_msgs/msg/ChestLed");
//   test_subscription("effectors/left_ear_leds", "nao_lola_command_msgs/msg/LeftEarLeds");
//   test_subscription("effectors/right_ear_leds", "nao_lola_command_msgs/msg/RightEarLeds");
//   test_subscription("effectors/left_eye_leds", "nao_lola_command_msgs/msg/LeftEyeLeds");
//   test_subscription("effectors/right_eye_leds", "nao_lola_command_msgs/msg/RightEyeLeds");
//   test_subscription("effectors/left_foot_led", "nao_lola_command_msgs/msg/LeftFootLed");
//   test_subscription("effectors/right_foot_led", "nao_lola_command_msgs/msg/RightFootLed");
//   test_subscription("effectors/head_leds", "nao_lola_command_msgs/msg/HeadLeds");
//   test_subscription("effectors/sonar_usage", "nao_lola_command_msgs/msg/SonaUsage");
// }

// TEST(TestNaoLolaClient, TestJointStatePublisherDisabled)
// {
//   rclcpp::init(0, nullptr);
//   auto node = NaoLolaClient(
//     rclcpp::NodeOptions().parameter_overrides({{"publish_joint_states", false}}));
//   EXPECT_TRUE(node.get_publishers_info_by_topic("joint_states").empty());
//   rclcpp::shutdown();
// }


struct Topic
{
  Topic(const std::string & topic_name, const std::string & topic_type)
  : topic_name(topic_name), topic_type(topic_type) {}
  std::string topic_name;
  std::string topic_type;
};

// Test Publishers
class TestPublisher : public ::testing::TestWithParam<Topic> {};
TEST_P(TestPublisher, ) {
  auto p = GetParam();
  rclcpp::init(0, nullptr);
  NaoLolaClient node;
  auto publishers_info = node.get_publishers_info_by_topic(p.topic_name);
  ASSERT_EQ(publishers_info.size(), 1);
  EXPECT_EQ(publishers_info[0].topic_type(), p.topic_type);
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(
  ,
  TestPublisher,
  ::testing::Values(
    Topic{"sensors/accelerometer", "nao_lola_sensor_msgs/msg/Accelerometer"},
    Topic{"sensors/angle", "nao_lola_sensor_msgs/msg/Angle"},
    Topic{"sensors/buttons", "nao_lola_sensor_msgs/msg/Buttons"},
    Topic{"sensors/fsr", "nao_lola_sensor_msgs/msg/FSR"},
    Topic{"sensors/gyroscope", "nao_lola_sensor_msgs/msg/Gyroscope"},
    Topic{"sensors/joint_positions", "nao_lola_sensor_msgs/msg/JointPositions"},
    Topic{"sensors/joint_stiffnesses", "nao_lola_sensor_msgs/msg/JointStiffnesses"},
    Topic{"sensors/joint_temperatures", "nao_lola_sensor_msgs/msg/JointTemperatures"},
    Topic{"sensors/joint_currents", "nao_lola_sensor_msgs/msg/JointCurrents"},
    Topic{"sensors/joint_statuses", "nao_lola_sensor_msgs/msg/JointStatuses"},
    Topic{"sensors/sonar", "nao_lola_sensor_msgs/msg/Sonar"},
    Topic{"sensors/touch", "nao_lola_sensor_msgs/msg/Touch"},
    Topic{"sensors/battery", "nao_lola_sensor_msgs/msg/Battery"},
    Topic{"sensors/robot_config", "nao_lola_sensor_msgs/msg/RobotConfig"})
);

// Test subscriptions
class TestSubscriptions : public ::testing::TestWithParam<Topic> {};
TEST_P(TestSubscriptions, ) {
  auto p = GetParam();
  rclcpp::init(0, nullptr);
  NaoLolaClient node;
  auto subscriptions_info = node.get_subscriptions_info_by_topic(p.topic_name);
  ASSERT_EQ(subscriptions_info.size(), 1);
  EXPECT_EQ(subscriptions_info[0].topic_type(), p.topic_type);
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(
  ,
  TestSubscriptions,
  ::testing::Values(
    Topic{"effectors/joint_positions", "nao_lola_command_msgs/msg/JointPositions"},
    Topic{"effectors/joint_stiffnesses", "nao_lola_command_msgs/msg/JointStiffnesses"},
    Topic{"effectors/chest_led", "nao_lola_command_msgs/msg/ChestLed"},
    Topic{"effectors/left_ear_leds", "nao_lola_command_msgs/msg/LeftEarLeds"},
    Topic{"effectors/right_ear_leds", "nao_lola_command_msgs/msg/RightEarLeds"},
    Topic{"effectors/left_eye_leds", "nao_lola_command_msgs/msg/LeftEyeLeds"},
    Topic{"effectors/right_eye_leds", "nao_lola_command_msgs/msg/RightEyeLeds"},
    Topic{"effectors/left_foot_led", "nao_lola_command_msgs/msg/LeftFootLed"},
    Topic{"effectors/right_foot_led", "nao_lola_command_msgs/msg/RightFootLed"},
    Topic{"effectors/head_leds", "nao_lola_command_msgs/msg/HeadLeds"},
    Topic{"effectors/sonar_usage", "nao_lola_command_msgs/msg/SonarUsage"})
);
