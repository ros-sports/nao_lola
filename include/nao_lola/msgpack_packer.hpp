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

#ifndef NAO_LOLA__MSGPACK_PACKER_HPP_
#define NAO_LOLA__MSGPACK_PACKER_HPP_

#include <string>
#include <vector>
#include <memory>
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_command_msgs/msg/chest_led.hpp"
#include "nao_command_msgs/msg/left_ear_leds.hpp"
#include "nao_command_msgs/msg/right_ear_leds.hpp"
#include "nao_command_msgs/msg/left_eye_leds.hpp"
#include "nao_command_msgs/msg/right_eye_leds.hpp"
#include "nao_command_msgs/msg/left_foot_led.hpp"
#include "nao_command_msgs/msg/right_foot_led.hpp"
#include "nao_command_msgs/msg/head_leds.hpp"
#include "nao_command_msgs/msg/sonar_usage.hpp"
#include "nao_lola/lola_enums.hpp"
#include "rclcpp/logger.hpp"


class MsgpackPacker
{
public:
  MsgpackPacker()
  : logger(rclcpp::get_logger("msgpack packer")) {}
  std::string getPacked();

  void setJointPositions(const nao_command_msgs::msg::JointPositions & jointPositions);
  void setJointStiffnesses(const nao_command_msgs::msg::JointStiffnesses & jointStiffnesses);
  void setChestLed(const nao_command_msgs::msg::ChestLed & chestLed);
  void setLeftEarLeds(const nao_command_msgs::msg::LeftEarLeds & leftEarLeds);
  void setRightEarLeds(const nao_command_msgs::msg::RightEarLeds & rightEarLeds);
  void setLeftEyeLeds(const nao_command_msgs::msg::LeftEyeLeds & leftEyeLeds);
  void setRightEyeLeds(const nao_command_msgs::msg::RightEyeLeds & rightEyeLeds);
  void setLeftFootLed(const nao_command_msgs::msg::LeftFootLed & leftFootLed);
  void setRightFootLed(const nao_command_msgs::msg::RightFootLed & rightFootLed);
  void setHeadLeds(const nao_command_msgs::msg::HeadLeds & headLeds);
  void setSonarUsage(const nao_command_msgs::msg::SonarUsage & sonarUsage);

private:
  std::array<float, static_cast<int>(LolaEnums::Joint::NUM_JOINTS)> position;
  std::array<float, static_cast<int>(LolaEnums::Joint::NUM_JOINTS)> stiffness;
  std::array<float, 3> chest;
  std::array<float, static_cast<int>(nao_command_msgs::msg::LeftEarLeds::NUM_LEDS)> l_ear;
  std::array<float, static_cast<int>(nao_command_msgs::msg::RightEarLeds::NUM_LEDS)> r_ear;
  std::array<float, 3 * static_cast<int>(nao_command_msgs::msg::LeftEyeLeds::NUM_LEDS)> l_eye;
  std::array<float, 3 * static_cast<int>(nao_command_msgs::msg::RightEyeLeds::NUM_LEDS)> r_eye;
  std::array<float, 3> l_foot;
  std::array<float, 3> r_foot;
  std::array<float, static_cast<int>(nao_command_msgs::msg::HeadLeds::NUM_LEDS)> skull;
  std::array<bool, 2> sonar;

  rclcpp::Logger logger;
};

#endif  // NAO_LOLA__MSGPACK_PACKER_HPP_
