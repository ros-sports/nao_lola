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

#ifndef NAO_LOLA_CLIENT__NAO_LOLA_CLIENT_HPP_
#define NAO_LOLA_CLIENT__NAO_LOLA_CLIENT_HPP_

#include <thread>
#include <memory>
#include <mutex>
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "nao_lola_sensor_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_sensor_msgs/msg/joint_temperatures.hpp"
#include "nao_lola_sensor_msgs/msg/joint_currents.hpp"
#include "nao_lola_sensor_msgs/msg/joint_statuses.hpp"
#include "nao_lola_sensor_msgs/msg/buttons.hpp"
#include "nao_lola_sensor_msgs/msg/accelerometer.hpp"
#include "nao_lola_sensor_msgs/msg/gyroscope.hpp"
#include "nao_lola_sensor_msgs/msg/angle.hpp"
#include "nao_lola_sensor_msgs/msg/sonar.hpp"
#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nao_lola_sensor_msgs/msg/touch.hpp"
#include "nao_lola_sensor_msgs/msg/battery.hpp"
#include "nao_lola_sensor_msgs/msg/robot_config.hpp"
#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/left_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/right_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/left_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/right_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/left_foot_led.hpp"
#include "nao_lola_command_msgs/msg/right_foot_led.hpp"
#include "nao_lola_command_msgs/msg/head_leds.hpp"
#include "nao_lola_command_msgs/msg/sonar_usage.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_client/connection.hpp"
#include "nao_lola_client/msgpack_packer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class NaoLolaClient : public rclcpp::Node
{
public:
  explicit NaoLolaClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~NaoLolaClient() {}

private:
  void createPublishers();
  void createSubscriptions();
  void declareParameters();

  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Accelerometer>::SharedPtr accelerometer_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Angle>::SharedPtr angle_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Buttons>::SharedPtr buttons_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::FSR>::SharedPtr fsr_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Gyroscope>::SharedPtr gyroscope_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr joint_positions_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointStiffnesses>::SharedPtr joint_stiffnesses_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointTemperatures>::SharedPtr joint_temperatures_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointCurrents>::SharedPtr joint_currents_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointStatuses>::SharedPtr joint_statuses_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Sonar>::SharedPtr sonar_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Touch>::SharedPtr touch_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Battery>::SharedPtr battery_pub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::RobotConfig>::SharedPtr robot_config_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

  rclcpp::Subscription<nao_lola_command_msgs::msg::JointPositions>::SharedPtr joint_positions_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr
    joint_stiffnesses_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::ChestLed>::SharedPtr chest_led_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::LeftEarLeds>::SharedPtr left_ear_leds_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::RightEarLeds>::SharedPtr right_ear_leds_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::LeftEyeLeds>::SharedPtr left_eye_leds_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::RightEyeLeds>::SharedPtr right_eye_leds_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::LeftFootLed>::SharedPtr left_foot_led_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::RightFootLed>::SharedPtr right_foot_led_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::HeadLeds>::SharedPtr head_leds_sub;
  rclcpp::Subscription<nao_lola_command_msgs::msg::SonarUsage>::SharedPtr sonar_usage_sub;

  std::thread receive_thread_;
  Connection connection;

  MsgpackPacker packer;
  std::mutex packer_mutex;

  bool publish_imu_;
  bool publish_joint_states_;
};

#endif  // NAO_LOLA_CLIENT__NAO_LOLA_CLIENT_HPP_
