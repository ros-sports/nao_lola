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

#ifndef NAO_LOLA__JOINT_INDEX_CONVERSION_HPP_
#define NAO_LOLA__JOINT_INDEX_CONVERSION_HPP_

#include <map>
#include "nao_lola/lola_enums.hpp"

namespace JointIndexConversion
{
std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in);

static const std::map<LolaEnums::Joint, int> index_lola_to_msg = {
  {LolaEnums::Joint::HeadYaw, nao_interfaces::msg::Joints::HEADYAW},
  {LolaEnums::Joint::HeadPitch, nao_interfaces::msg::Joints::HEADPITCH},
  {LolaEnums::Joint::LShoulderPitch, nao_interfaces::msg::Joints::LSHOULDERPITCH},
  {LolaEnums::Joint::LShoulderRoll, nao_interfaces::msg::Joints::LSHOULDERROLL},
  {LolaEnums::Joint::LElbowYaw, nao_interfaces::msg::Joints::LELBOWYAW},
  {LolaEnums::Joint::LElbowRoll, nao_interfaces::msg::Joints::LELBOWROLL},
  {LolaEnums::Joint::LWristYaw, nao_interfaces::msg::Joints::LWRISTYAW},
  {LolaEnums::Joint::LHipYawPitch, nao_interfaces::msg::Joints::LHIPYAWPITCH},
  {LolaEnums::Joint::LHipRoll, nao_interfaces::msg::Joints::LHIPROLL},
  {LolaEnums::Joint::LHipPitch, nao_interfaces::msg::Joints::LHIPPITCH},
  {LolaEnums::Joint::LKneePitch, nao_interfaces::msg::Joints::LKNEEPITCH},
  {LolaEnums::Joint::LAnklePitch, nao_interfaces::msg::Joints::LANKLEPITCH},
  {LolaEnums::Joint::LAnkleRoll, nao_interfaces::msg::Joints::LANKLEROLL},
  {LolaEnums::Joint::RHipRoll, nao_interfaces::msg::Joints::RHIPROLL},
  {LolaEnums::Joint::RHipPitch, nao_interfaces::msg::Joints::RHIPPITCH},
  {LolaEnums::Joint::RKneePitch, nao_interfaces::msg::Joints::RKNEEPITCH},
  {LolaEnums::Joint::RAnklePitch, nao_interfaces::msg::Joints::RANKLEPITCH},
  {LolaEnums::Joint::RAnkleRoll, nao_interfaces::msg::Joints::RANKLEROLL},
  {LolaEnums::Joint::RShoulderPitch, nao_interfaces::msg::Joints::RSHOULDERPITCH},
  {LolaEnums::Joint::RShoulderRoll, nao_interfaces::msg::Joints::RSHOULDERROLL},
  {LolaEnums::Joint::RElbowYaw, nao_interfaces::msg::Joints::RELBOWYAW},
  {LolaEnums::Joint::RElbowRoll, nao_interfaces::msg::Joints::RELBOWROLL},
  {LolaEnums::Joint::RWristYaw, nao_interfaces::msg::Joints::RWRISTYAW},
  {LolaEnums::Joint::LHand, nao_interfaces::msg::Joints::LHAND},
  {LolaEnums::Joint::RHand, nao_interfaces::msg::Joints::RHAND},
};

static const std::map<int, LolaEnums::Joint> index_msg_to_lola = flip(index_lola_to_msg);

std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in)
{
  std::map<int, LolaEnums::Joint> flipped;
  for (std::map<LolaEnums::Joint, int>::iterator i = in.begin(); i != in.end(); ++i)
    flipped[i->second] = i->first;

  return flipped;
}

}


#endif  // NAO_LOLA__JOINT_INDEX_CONVERSION_HPP_