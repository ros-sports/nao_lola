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

#ifndef NAO_LOLA__NAO_LOLA_HPP_
#define NAO_LOLA__NAO_LOLA_HPP_

#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nao_interfaces/msg/joint_positions.hpp"
#include "nao_interfaces/msg/joint_stiffnesses.hpp"
#include "nao_interfaces/msg/joint_temperatures.hpp"
#include "nao_interfaces/msg/joint_currents.hpp"
#include "nao_interfaces/msg/joint_statuses.hpp"
#include "nao_interfaces/msg/buttons.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/angle.hpp"
#include "nao_interfaces/msg/sonar.hpp"
#include "nao_interfaces/msg/fsr.hpp"
#include "nao_interfaces/msg/touch.hpp"
#include "nao_interfaces/msg/battery.hpp"
#include "nao_interfaces/msg/robot_config.hpp"
#include "nao_interfaces/msg/chest_led.hpp"
#include "nao_interfaces/msg/left_ear_leds.hpp"
#include "nao_interfaces/msg/right_ear_leds.hpp"
#include "nao_interfaces/msg/left_eye_leds.hpp"
#include "nao_interfaces/msg/right_eye_leds.hpp"
#include "nao_interfaces/msg/left_foot_led.hpp"
#include "nao_interfaces/msg/right_foot_led.hpp"
#include "nao_interfaces/msg/head_leds.hpp"
#include "nao_interfaces/msg/sonar_usage.hpp"
#include "nao_lola/connection.hpp"
#include "nao_lola/msgpack_packer.hpp"

class NaoLola : public rclcpp::Node
{
public:
  NaoLola();
  virtual ~NaoLola() {}

private:
  void createPublishers();
  void createSubscriptions();

  rclcpp::Publisher<nao_interfaces::msg::Accelerometer>::SharedPtr accelerometer_pub;
  rclcpp::Publisher<nao_interfaces::msg::Angle>::SharedPtr angle_pub;
  rclcpp::Publisher<nao_interfaces::msg::Buttons>::SharedPtr buttons_pub;
  rclcpp::Publisher<nao_interfaces::msg::FSR>::SharedPtr fsr_pub;
  rclcpp::Publisher<nao_interfaces::msg::Gyroscope>::SharedPtr gyroscope_pub;
  rclcpp::Publisher<nao_interfaces::msg::JointPositions>::SharedPtr joint_positions_pub;
  rclcpp::Publisher<nao_interfaces::msg::JointStiffnesses>::SharedPtr joint_stiffnesses_pub;
  rclcpp::Publisher<nao_interfaces::msg::JointTemperatures>::SharedPtr joint_temperatures_pub;
  rclcpp::Publisher<nao_interfaces::msg::JointCurrents>::SharedPtr joint_currents_pub;
  rclcpp::Publisher<nao_interfaces::msg::JointStatuses>::SharedPtr joint_statuses_pub;
  rclcpp::Publisher<nao_interfaces::msg::Sonar>::SharedPtr sonar_pub;
  rclcpp::Publisher<nao_interfaces::msg::Touch>::SharedPtr touch_pub;
  rclcpp::Publisher<nao_interfaces::msg::Battery>::SharedPtr battery_pub;
  rclcpp::Publisher<nao_interfaces::msg::RobotConfig>::SharedPtr robot_config_pub;

  rclcpp::Subscription<nao_interfaces::msg::JointPositions>::SharedPtr joint_positions_sub;
  rclcpp::Subscription<nao_interfaces::msg::JointStiffnesses>::SharedPtr joint_stiffnesses_sub;
  rclcpp::Subscription<nao_interfaces::msg::ChestLed>::SharedPtr chest_led_sub;
  rclcpp::Subscription<nao_interfaces::msg::LeftEarLeds>::SharedPtr left_ear_leds_sub;
  rclcpp::Subscription<nao_interfaces::msg::RightEarLeds>::SharedPtr right_ear_leds_sub;
  rclcpp::Subscription<nao_interfaces::msg::LeftEyeLeds>::SharedPtr left_eye_leds_sub;
  rclcpp::Subscription<nao_interfaces::msg::RightEyeLeds>::SharedPtr right_eye_leds_sub;
  rclcpp::Subscription<nao_interfaces::msg::LeftFootLed>::SharedPtr left_foot_led_sub;
  rclcpp::Subscription<nao_interfaces::msg::RightFootLed>::SharedPtr right_foot_led_sub;
  rclcpp::Subscription<nao_interfaces::msg::HeadLeds>::SharedPtr head_leds_sub;
  rclcpp::Subscription<nao_interfaces::msg::SonarUsage>::SharedPtr sonar_usage_sub;

  std::thread receive_thread_;
  Connection connection;

  std::shared_ptr<MsgpackPacker> packer = std::make_shared<MsgpackPacker>();
};

#endif  // NAO_LOLA__NAO_LOLA_HPP_
