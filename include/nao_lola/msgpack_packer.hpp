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
#include "rclcpp/logger.hpp"


class MsgpackPacker
{
public:
  MsgpackPacker()
  : logger(rclcpp::get_logger("msgpack packer")) {}
  std::string getPacked();

  void setJointPositions(std::shared_ptr<nao_command_msgs::msg::JointPositions> jointPositions);
  void setJointStiffnesses(
    std::shared_ptr<nao_command_msgs::msg::JointStiffnesses> jointStiffnesses);
  void setChestLed(std::shared_ptr<nao_command_msgs::msg::ChestLed> chestLed);
  void setLeftEarLeds(std::shared_ptr<nao_command_msgs::msg::LeftEarLeds> leftEarLeds);
  void setRightEarLeds(std::shared_ptr<nao_command_msgs::msg::RightEarLeds> rightEarLeds);
  void setLeftEyeLeds(std::shared_ptr<nao_command_msgs::msg::LeftEyeLeds> leftEyeLeds);
  void setRightEyeLeds(std::shared_ptr<nao_command_msgs::msg::RightEyeLeds> rightEyeLeds);
  void setLeftFootLed(std::shared_ptr<nao_command_msgs::msg::LeftFootLed> leftFootLed);
  void setRightFootLed(std::shared_ptr<nao_command_msgs::msg::RightFootLed> rightFootLed);
  void setHeadLeds(std::shared_ptr<nao_command_msgs::msg::HeadLeds> headLeds);
  void setSonarUsage(std::shared_ptr<nao_command_msgs::msg::SonarUsage> sonarUsage);

private:
  std::shared_ptr<std::vector<float>> position;
  std::shared_ptr<std::vector<float>> stiffness;
  std::shared_ptr<std::vector<float>> chest;
  std::shared_ptr<std::vector<float>> l_ear;
  std::shared_ptr<std::vector<float>> r_ear;
  std::shared_ptr<std::vector<float>> l_eye;
  std::shared_ptr<std::vector<float>> r_eye;
  std::shared_ptr<std::vector<float>> l_foot;
  std::shared_ptr<std::vector<float>> r_foot;
  std::shared_ptr<std::vector<float>> skull;
  std::shared_ptr<std::vector<bool>> sonar;

  rclcpp::Logger logger;
};

#endif  // NAO_LOLA__MSGPACK_PACKER_HPP_
