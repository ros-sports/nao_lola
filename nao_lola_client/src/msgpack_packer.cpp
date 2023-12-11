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

#include <string>
#include <map>
#include <utility>
#include <memory>
#include <vector>
#include "nao_lola_client/msgpack_packer.hpp"
#include "msgpack/object.hpp"
#include "msgpack/pack.hpp"
#include "msgpack/adaptor/bool.hpp"
#include "msgpack/adaptor/cpp11/array.hpp"
#include "msgpack/adaptor/float.hpp"
#include "msgpack/adaptor/map.hpp"
#include "msgpack/adaptor/string.hpp"
#include "msgpack/zone.hpp"
#include "nao_lola_client/command_index_conversion.hpp"
#include "rclcpp/logging.hpp"

std::string MsgpackPacker::getPacked()
{
  msgpack::zone z;
  std::map<std::string, msgpack::object> map;

  map.insert(std::make_pair("Position", msgpack::object(position, z)));
  map.insert(std::make_pair("Stiffness", msgpack::object(stiffness, z)));
  map.insert(std::make_pair("Chest", msgpack::object(chest, z)));
  map.insert(std::make_pair("LEar", msgpack::object(l_ear, z)));
  map.insert(std::make_pair("REar", msgpack::object(r_ear, z)));
  map.insert(std::make_pair("LEye", msgpack::object(l_eye, z)));
  map.insert(std::make_pair("REye", msgpack::object(r_eye, z)));
  map.insert(std::make_pair("LFoot", msgpack::object(l_foot, z)));
  map.insert(std::make_pair("RFoot", msgpack::object(r_foot, z)));
  map.insert(std::make_pair("Skull", msgpack::object(skull, z)));
  map.insert(std::make_pair("Sonar", msgpack::object(sonar, z)));

  std::stringstream buffer;
  msgpack::pack(buffer, map);
  std::string packed = buffer.str();

  return packed;
}

void MsgpackPacker::setJointPositions(
  const nao_lola_command_msgs::msg::JointPositions & jointPositions)
{
  if (jointPositions.indexes.size() != jointPositions.positions.size()) {
    RCLCPP_ERROR(
      logger,
      "Incorrect message received for nao_lola_command_msgs::msg::JointPositions. "
      "Angles and Indexes vector must have the same length. "
      "Angles vector has length %zu, while indexes vector has length %zu",
      jointPositions.positions.size(), jointPositions.indexes.size());
    return;
  }

  for (unsigned i = 0; i < jointPositions.indexes.size(); ++i) {
    int msg_joint_index = jointPositions.indexes[i];
    float joint_angle = jointPositions.positions[i];
    LolaEnums::Joint lola_joint_index = IndexConversion::joint_msg_to_lola.at(msg_joint_index);
    position.at(static_cast<int>(lola_joint_index)) = joint_angle;
  }
}

void MsgpackPacker::setJointStiffnesses(
  const nao_lola_command_msgs::msg::JointStiffnesses & jointStiffnesses)
{
  if (jointStiffnesses.indexes.size() != jointStiffnesses.stiffnesses.size()) {
    RCLCPP_ERROR(
      logger,
      "Incorrect message received for nao_lola_command_msgs::msg::JointStiffnesses. "
      "Stiffnesses and Indexes vector must have the same length. "
      "Stiffnesses vector has length %zu, while indexes vector has length %zu",
      jointStiffnesses.stiffnesses.size(), jointStiffnesses.indexes.size());
    return;
  }

  for (unsigned i = 0; i < jointStiffnesses.indexes.size(); ++i) {
    int msg_joint_index = jointStiffnesses.indexes[i];
    float joint_stiffness = jointStiffnesses.stiffnesses[i];
    LolaEnums::Joint lola_joint_index = IndexConversion::joint_msg_to_lola.at(msg_joint_index);
    stiffness.at(static_cast<int>(lola_joint_index)) = joint_stiffness;
  }
}

void MsgpackPacker::setChestLed(const nao_lola_command_msgs::msg::ChestLed & chestLed)
{
  chest.at(0) = chestLed.color.r;
  chest.at(1) = chestLed.color.g;
  chest.at(2) = chestLed.color.b;
}

void MsgpackPacker::setLeftEarLeds(const nao_lola_command_msgs::msg::LeftEarLeds & leftEarLeds)
{
  for (unsigned i = 0; i < leftEarLeds.NUM_LEDS; ++i) {
    LolaEnums::LeftEarLeds lola_index = IndexConversion::left_ear_leds_msg_to_lola.at(i);
    l_ear.at(static_cast<int>(lola_index)) = leftEarLeds.intensities[i];
  }
}

void MsgpackPacker::setRightEarLeds(
  const nao_lola_command_msgs::msg::RightEarLeds & rightEarLeds)
{
  for (unsigned i = 0; i < rightEarLeds.NUM_LEDS; ++i) {
    LolaEnums::RightEarLeds lola_index = IndexConversion::right_ear_leds_msg_to_lola.at(i);
    r_ear.at(static_cast<int>(lola_index)) = rightEarLeds.intensities[i];
  }
}

void MsgpackPacker::setLeftEyeLeds(const nao_lola_command_msgs::msg::LeftEyeLeds & leftEyeLeds)
{
  for (unsigned i = 0; i < leftEyeLeds.NUM_LEDS; ++i) {
    LolaEnums::LeftEyeLeds lola_index = IndexConversion::left_eye_leds_msg_to_lola.at(i);
    l_eye.at(static_cast<int>(lola_index)) = leftEyeLeds.colors[i].r;
    l_eye.at(static_cast<int>(lola_index) + 8) = leftEyeLeds.colors[i].g;
    l_eye.at(static_cast<int>(lola_index) + 16) = leftEyeLeds.colors[i].b;
  }
}

void MsgpackPacker::setRightEyeLeds(
  const nao_lola_command_msgs::msg::RightEyeLeds & rightEyeLeds)
{
  for (unsigned i = 0; i < rightEyeLeds.NUM_LEDS; ++i) {
    LolaEnums::RightEyeLeds lola_index = IndexConversion::right_eye_leds_msg_to_lola.at(i);
    r_eye.at(static_cast<int>(lola_index)) = rightEyeLeds.colors[i].r;
    r_eye.at(static_cast<int>(lola_index) + 8) = rightEyeLeds.colors[i].g;
    r_eye.at(static_cast<int>(lola_index) + 16) = rightEyeLeds.colors[i].b;
  }
}

void MsgpackPacker::setLeftFootLed(const nao_lola_command_msgs::msg::LeftFootLed & leftFootLed)
{
  l_foot.at(0) = leftFootLed.color.r;
  l_foot.at(1) = leftFootLed.color.g;
  l_foot.at(2) = leftFootLed.color.b;
}

void MsgpackPacker::setRightFootLed(
  const nao_lola_command_msgs::msg::RightFootLed & rightFootLed)
{
  r_foot.at(0) = rightFootLed.color.r;
  r_foot.at(1) = rightFootLed.color.g;
  r_foot.at(2) = rightFootLed.color.b;
}

void MsgpackPacker::setHeadLeds(const nao_lola_command_msgs::msg::HeadLeds & headLeds)
{
  for (unsigned i = 0; i < headLeds.NUM_LEDS; ++i) {
    LolaEnums::SkullLeds lola_index = IndexConversion::head_leds_msg_to_lola.at(i);
    skull.at(static_cast<int>(lola_index)) = headLeds.intensities[i];
  }
}

void MsgpackPacker::setSonarUsage(const nao_lola_command_msgs::msg::SonarUsage & sonarUsage)
{
  sonar.at(0) = sonarUsage.left;
  sonar.at(1) = sonarUsage.right;
}
