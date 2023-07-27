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

#ifndef NAO_LOLA_CLIENT__SENSOR_INDEX_CONVERSION_HPP_
#define NAO_LOLA_CLIENT__SENSOR_INDEX_CONVERSION_HPP_

#include <map>
#include "nao_lola_client/lola_enums.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

namespace IndexConversion
{
std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in);

static const std::map<LolaEnums::Joint, int> joint_lola_to_msg = {
  {LolaEnums::Joint::HeadYaw, nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW},
  {LolaEnums::Joint::HeadPitch, nao_lola_sensor_msgs::msg::JointIndexes::HEADPITCH},
  {LolaEnums::Joint::LShoulderPitch, nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERPITCH},
  {LolaEnums::Joint::LShoulderRoll, nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERROLL},
  {LolaEnums::Joint::LElbowYaw, nao_lola_sensor_msgs::msg::JointIndexes::LELBOWYAW},
  {LolaEnums::Joint::LElbowRoll, nao_lola_sensor_msgs::msg::JointIndexes::LELBOWROLL},
  {LolaEnums::Joint::LWristYaw, nao_lola_sensor_msgs::msg::JointIndexes::LWRISTYAW},
  {LolaEnums::Joint::LHipYawPitch, nao_lola_sensor_msgs::msg::JointIndexes::LHIPYAWPITCH},
  {LolaEnums::Joint::LHipRoll, nao_lola_sensor_msgs::msg::JointIndexes::LHIPROLL},
  {LolaEnums::Joint::LHipPitch, nao_lola_sensor_msgs::msg::JointIndexes::LHIPPITCH},
  {LolaEnums::Joint::LKneePitch, nao_lola_sensor_msgs::msg::JointIndexes::LKNEEPITCH},
  {LolaEnums::Joint::LAnklePitch, nao_lola_sensor_msgs::msg::JointIndexes::LANKLEPITCH},
  {LolaEnums::Joint::LAnkleRoll, nao_lola_sensor_msgs::msg::JointIndexes::LANKLEROLL},
  {LolaEnums::Joint::RHipRoll, nao_lola_sensor_msgs::msg::JointIndexes::RHIPROLL},
  {LolaEnums::Joint::RHipPitch, nao_lola_sensor_msgs::msg::JointIndexes::RHIPPITCH},
  {LolaEnums::Joint::RKneePitch, nao_lola_sensor_msgs::msg::JointIndexes::RKNEEPITCH},
  {LolaEnums::Joint::RAnklePitch, nao_lola_sensor_msgs::msg::JointIndexes::RANKLEPITCH},
  {LolaEnums::Joint::RAnkleRoll, nao_lola_sensor_msgs::msg::JointIndexes::RANKLEROLL},
  {LolaEnums::Joint::RShoulderPitch, nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERPITCH},
  {LolaEnums::Joint::RShoulderRoll, nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERROLL},
  {LolaEnums::Joint::RElbowYaw, nao_lola_sensor_msgs::msg::JointIndexes::RELBOWYAW},
  {LolaEnums::Joint::RElbowRoll, nao_lola_sensor_msgs::msg::JointIndexes::RELBOWROLL},
  {LolaEnums::Joint::RWristYaw, nao_lola_sensor_msgs::msg::JointIndexes::RWRISTYAW},
  {LolaEnums::Joint::LHand, nao_lola_sensor_msgs::msg::JointIndexes::LHAND},
  {LolaEnums::Joint::RHand, nao_lola_sensor_msgs::msg::JointIndexes::RHAND},
};

static const std::map<int, LolaEnums::Joint> joint_msg_to_lola = flip(joint_lola_to_msg);

std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in)
{
  std::map<int, LolaEnums::Joint> flipped;
  for (std::map<LolaEnums::Joint, int>::iterator i = in.begin(); i != in.end(); ++i) {
    flipped[i->second] = i->first;
  }

  return flipped;
}

}  // namespace IndexConversion

#endif  // NAO_LOLA_CLIENT__SENSOR_INDEX_CONVERSION_HPP_
