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
#include "nao_interfaces/msg/joint_indexes.hpp"
#include "nao_interfaces/msg/left_ear_leds.hpp"
#include "nao_interfaces/msg/right_ear_leds.hpp"
#include "nao_interfaces/msg/left_eye_leds.hpp"
#include "nao_interfaces/msg/right_eye_leds.hpp"
#include "nao_interfaces/msg/head_leds.hpp"

namespace IndexConversion
{
std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in);

static const std::map<LolaEnums::Joint, int> joint_lola_to_msg = {
  {LolaEnums::Joint::HeadYaw, nao_interfaces::msg::JointIndexes::HEADYAW},
  {LolaEnums::Joint::HeadPitch, nao_interfaces::msg::JointIndexes::HEADPITCH},
  {LolaEnums::Joint::LShoulderPitch, nao_interfaces::msg::JointIndexes::LSHOULDERPITCH},
  {LolaEnums::Joint::LShoulderRoll, nao_interfaces::msg::JointIndexes::LSHOULDERROLL},
  {LolaEnums::Joint::LElbowYaw, nao_interfaces::msg::JointIndexes::LELBOWYAW},
  {LolaEnums::Joint::LElbowRoll, nao_interfaces::msg::JointIndexes::LELBOWROLL},
  {LolaEnums::Joint::LWristYaw, nao_interfaces::msg::JointIndexes::LWRISTYAW},
  {LolaEnums::Joint::LHipYawPitch, nao_interfaces::msg::JointIndexes::LHIPYAWPITCH},
  {LolaEnums::Joint::LHipRoll, nao_interfaces::msg::JointIndexes::LHIPROLL},
  {LolaEnums::Joint::LHipPitch, nao_interfaces::msg::JointIndexes::LHIPPITCH},
  {LolaEnums::Joint::LKneePitch, nao_interfaces::msg::JointIndexes::LKNEEPITCH},
  {LolaEnums::Joint::LAnklePitch, nao_interfaces::msg::JointIndexes::LANKLEPITCH},
  {LolaEnums::Joint::LAnkleRoll, nao_interfaces::msg::JointIndexes::LANKLEROLL},
  {LolaEnums::Joint::RHipRoll, nao_interfaces::msg::JointIndexes::RHIPROLL},
  {LolaEnums::Joint::RHipPitch, nao_interfaces::msg::JointIndexes::RHIPPITCH},
  {LolaEnums::Joint::RKneePitch, nao_interfaces::msg::JointIndexes::RKNEEPITCH},
  {LolaEnums::Joint::RAnklePitch, nao_interfaces::msg::JointIndexes::RANKLEPITCH},
  {LolaEnums::Joint::RAnkleRoll, nao_interfaces::msg::JointIndexes::RANKLEROLL},
  {LolaEnums::Joint::RShoulderPitch, nao_interfaces::msg::JointIndexes::RSHOULDERPITCH},
  {LolaEnums::Joint::RShoulderRoll, nao_interfaces::msg::JointIndexes::RSHOULDERROLL},
  {LolaEnums::Joint::RElbowYaw, nao_interfaces::msg::JointIndexes::RELBOWYAW},
  {LolaEnums::Joint::RElbowRoll, nao_interfaces::msg::JointIndexes::RELBOWROLL},
  {LolaEnums::Joint::RWristYaw, nao_interfaces::msg::JointIndexes::RWRISTYAW},
  {LolaEnums::Joint::LHand, nao_interfaces::msg::JointIndexes::LHAND},
  {LolaEnums::Joint::RHand, nao_interfaces::msg::JointIndexes::RHAND},
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

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#left-ear
static const std::map<int, LolaEnums::LeftEarLeds> left_ear_leds_msg_to_lola
{
  {nao_interfaces::msg::LeftEarLeds::L0, LolaEnums::LeftEarLeds::Deg_0},
  {nao_interfaces::msg::LeftEarLeds::L1, LolaEnums::LeftEarLeds::Deg_36},
  {nao_interfaces::msg::LeftEarLeds::L2, LolaEnums::LeftEarLeds::Deg_72},
  {nao_interfaces::msg::LeftEarLeds::L3, LolaEnums::LeftEarLeds::Deg_108},
  {nao_interfaces::msg::LeftEarLeds::L4, LolaEnums::LeftEarLeds::Deg_144},
  {nao_interfaces::msg::LeftEarLeds::L5, LolaEnums::LeftEarLeds::Deg_180},
  {nao_interfaces::msg::LeftEarLeds::L6, LolaEnums::LeftEarLeds::Deg_216},
  {nao_interfaces::msg::LeftEarLeds::L7, LolaEnums::LeftEarLeds::Deg_252},
  {nao_interfaces::msg::LeftEarLeds::L8, LolaEnums::LeftEarLeds::Deg_288},
  {nao_interfaces::msg::LeftEarLeds::L9, LolaEnums::LeftEarLeds::Deg_324}
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#right-ear
static const std::map<int, LolaEnums::RightEarLeds> right_ear_leds_msg_to_lola
{
  {nao_interfaces::msg::RightEarLeds::R0, LolaEnums::RightEarLeds::Deg_0},
  {nao_interfaces::msg::RightEarLeds::R1, LolaEnums::RightEarLeds::Deg_36},
  {nao_interfaces::msg::RightEarLeds::R2, LolaEnums::RightEarLeds::Deg_72},
  {nao_interfaces::msg::RightEarLeds::R3, LolaEnums::RightEarLeds::Deg_108},
  {nao_interfaces::msg::RightEarLeds::R4, LolaEnums::RightEarLeds::Deg_144},
  {nao_interfaces::msg::RightEarLeds::R5, LolaEnums::RightEarLeds::Deg_180},
  {nao_interfaces::msg::RightEarLeds::R6, LolaEnums::RightEarLeds::Deg_216},
  {nao_interfaces::msg::RightEarLeds::R7, LolaEnums::RightEarLeds::Deg_252},
  {nao_interfaces::msg::RightEarLeds::R8, LolaEnums::RightEarLeds::Deg_288},
  {nao_interfaces::msg::RightEarLeds::R9, LolaEnums::RightEarLeds::Deg_324}
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
static const std::map<int, LolaEnums::LeftEyeLeds> left_eye_leds_msg_to_lola
{
  {nao_interfaces::msg::LeftEyeLeds::L0, LolaEnums::LeftEyeLeds::Deg_45},
  {nao_interfaces::msg::LeftEyeLeds::L1, LolaEnums::LeftEyeLeds::Deg_0},
  {nao_interfaces::msg::LeftEyeLeds::L2, LolaEnums::LeftEyeLeds::Deg_315},
  {nao_interfaces::msg::LeftEyeLeds::L3, LolaEnums::LeftEyeLeds::Deg_270},
  {nao_interfaces::msg::LeftEyeLeds::L4, LolaEnums::LeftEyeLeds::Deg_225},
  {nao_interfaces::msg::LeftEyeLeds::L5, LolaEnums::LeftEyeLeds::Deg_180},
  {nao_interfaces::msg::LeftEyeLeds::L6, LolaEnums::LeftEyeLeds::Deg_135},
  {nao_interfaces::msg::LeftEyeLeds::L7, LolaEnums::LeftEyeLeds::Deg_90},
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
static const std::map<int, LolaEnums::RightEyeLeds> right_eye_leds_msg_to_lola
{
  {nao_interfaces::msg::RightEyeLeds::R0, LolaEnums::RightEyeLeds::Deg_315},
  {nao_interfaces::msg::RightEyeLeds::R1, LolaEnums::RightEyeLeds::Deg_270},
  {nao_interfaces::msg::RightEyeLeds::R2, LolaEnums::RightEyeLeds::Deg_225},
  {nao_interfaces::msg::RightEyeLeds::R3, LolaEnums::RightEyeLeds::Deg_180},
  {nao_interfaces::msg::RightEyeLeds::R4, LolaEnums::RightEyeLeds::Deg_135},
  {nao_interfaces::msg::RightEyeLeds::R5, LolaEnums::RightEyeLeds::Deg_90},
  {nao_interfaces::msg::RightEyeLeds::R6, LolaEnums::RightEyeLeds::Deg_45},
  {nao_interfaces::msg::RightEyeLeds::R7, LolaEnums::RightEyeLeds::Deg_0},
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#head-tactile-sensor-led-locations
static const std::map<int, LolaEnums::SkullLeds> head_leds_msg_to_lola
{
  {nao_interfaces::msg::HeadLeds::B0, LolaEnums::SkullLeds::Front_Right_1},
  {nao_interfaces::msg::HeadLeds::B1, LolaEnums::SkullLeds::Front_Right_0},
  {nao_interfaces::msg::HeadLeds::B2, LolaEnums::SkullLeds::Middle_Right_0},
  {nao_interfaces::msg::HeadLeds::B3, LolaEnums::SkullLeds::Rear_Right_0},
  {nao_interfaces::msg::HeadLeds::B4, LolaEnums::SkullLeds::Rear_Right_1},
  {nao_interfaces::msg::HeadLeds::B5, LolaEnums::SkullLeds::Rear_Right_2},
  {nao_interfaces::msg::HeadLeds::B6, LolaEnums::SkullLeds::Rear_Left_2},
  {nao_interfaces::msg::HeadLeds::B7, LolaEnums::SkullLeds::Rear_Left_1},
  {nao_interfaces::msg::HeadLeds::B8, LolaEnums::SkullLeds::Rear_Left_0},
  {nao_interfaces::msg::HeadLeds::B9, LolaEnums::SkullLeds::Middle_Left_0},
  {nao_interfaces::msg::HeadLeds::B10, LolaEnums::SkullLeds::Front_Left_0},
  {nao_interfaces::msg::HeadLeds::B11, LolaEnums::SkullLeds::Front_Left_1},
};

}  // namespace IndexConversion

#endif  // NAO_LOLA__INDEX_CONVERSION_HPP_
