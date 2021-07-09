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

#ifndef NAO_LOLA__INDEX_CONVERSION_HPP_
#define NAO_LOLA__INDEX_CONVERSION_HPP_

#include <map>
#include "nao_lola/lola_enums.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "nao_interfaces/msg/left_eye_leds.hpp"
#include "nao_interfaces/msg/right_eye_leds.hpp"

namespace IndexConversion
{
std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in);

static const std::map<LolaEnums::Joint, int> joint_lola_to_msg = {
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

static const std::map<int, LolaEnums::Joint> joint_msg_to_lola = flip(joint_lola_to_msg);

std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in)
{
  std::map<int, LolaEnums::Joint> flipped;
  for (std::map<LolaEnums::Joint, int>::iterator i = in.begin(); i != in.end(); ++i) {
    flipped[i->second] = i->first;
  }

  return flipped;
}

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
static const std::map<int, LolaEnums::LeftEyeLeds> left_eye_leds_msg_to_lola
{
  {nao_interfaces::msg::LeftEyeLeds::L0, LolaEnums::LeftEyeLeds::DEG_45},
  {nao_interfaces::msg::LeftEyeLeds::L1, LolaEnums::LeftEyeLeds::DEG_0},
  {nao_interfaces::msg::LeftEyeLeds::L2, LolaEnums::LeftEyeLeds::DEG_315},
  {nao_interfaces::msg::LeftEyeLeds::L3, LolaEnums::LeftEyeLeds::DEG_270},
  {nao_interfaces::msg::LeftEyeLeds::L4, LolaEnums::LeftEyeLeds::DEG_225},
  {nao_interfaces::msg::LeftEyeLeds::L5, LolaEnums::LeftEyeLeds::DEG_180},
  {nao_interfaces::msg::LeftEyeLeds::L6, LolaEnums::LeftEyeLeds::DEG_135},
  {nao_interfaces::msg::LeftEyeLeds::L7, LolaEnums::LeftEyeLeds::DEG_90},
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
static const std::map<int, LolaEnums::RightEyeLeds> right_eye_leds_msg_to_lola
{
  {nao_interfaces::msg::RightEyeLeds::R0, LolaEnums::RightEyeLeds::DEG_315},
  {nao_interfaces::msg::RightEyeLeds::R1, LolaEnums::RightEyeLeds::DEG_270},
  {nao_interfaces::msg::RightEyeLeds::R2, LolaEnums::RightEyeLeds::DEG_225},
  {nao_interfaces::msg::RightEyeLeds::R3, LolaEnums::RightEyeLeds::DEG_180},
  {nao_interfaces::msg::RightEyeLeds::R4, LolaEnums::RightEyeLeds::DEG_135},
  {nao_interfaces::msg::RightEyeLeds::R5, LolaEnums::RightEyeLeds::DEG_90},
  {nao_interfaces::msg::RightEyeLeds::R6, LolaEnums::RightEyeLeds::DEG_45},
  {nao_interfaces::msg::RightEyeLeds::R7, LolaEnums::RightEyeLeds::DEG_0},
};

}  // namespace IndexConversion

#endif  // NAO_LOLA__INDEX_CONVERSION_HPP_
