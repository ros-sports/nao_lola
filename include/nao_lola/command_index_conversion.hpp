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

#ifndef NAO_LOLA__COMMAND_INDEX_CONVERSION_HPP_
#define NAO_LOLA__COMMAND_INDEX_CONVERSION_HPP_

#include <map>
#include "nao_lola/lola_enums.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "nao_command_msgs/msg/left_ear_leds.hpp"
#include "nao_command_msgs/msg/right_ear_leds.hpp"
#include "nao_command_msgs/msg/left_eye_leds.hpp"
#include "nao_command_msgs/msg/right_eye_leds.hpp"
#include "nao_command_msgs/msg/head_leds.hpp"

namespace IndexConversion
{
std::map<int, LolaEnums::Joint> flip(std::map<LolaEnums::Joint, int> in);

static const std::map<LolaEnums::Joint, int> joint_lola_to_msg = {
  {LolaEnums::Joint::HeadYaw, nao_command_msgs::msg::JointIndexes::HEADYAW},
  {LolaEnums::Joint::HeadPitch, nao_command_msgs::msg::JointIndexes::HEADPITCH},
  {LolaEnums::Joint::LShoulderPitch, nao_command_msgs::msg::JointIndexes::LSHOULDERPITCH},
  {LolaEnums::Joint::LShoulderRoll, nao_command_msgs::msg::JointIndexes::LSHOULDERROLL},
  {LolaEnums::Joint::LElbowYaw, nao_command_msgs::msg::JointIndexes::LELBOWYAW},
  {LolaEnums::Joint::LElbowRoll, nao_command_msgs::msg::JointIndexes::LELBOWROLL},
  {LolaEnums::Joint::LWristYaw, nao_command_msgs::msg::JointIndexes::LWRISTYAW},
  {LolaEnums::Joint::LHipYawPitch, nao_command_msgs::msg::JointIndexes::LHIPYAWPITCH},
  {LolaEnums::Joint::LHipRoll, nao_command_msgs::msg::JointIndexes::LHIPROLL},
  {LolaEnums::Joint::LHipPitch, nao_command_msgs::msg::JointIndexes::LHIPPITCH},
  {LolaEnums::Joint::LKneePitch, nao_command_msgs::msg::JointIndexes::LKNEEPITCH},
  {LolaEnums::Joint::LAnklePitch, nao_command_msgs::msg::JointIndexes::LANKLEPITCH},
  {LolaEnums::Joint::LAnkleRoll, nao_command_msgs::msg::JointIndexes::LANKLEROLL},
  {LolaEnums::Joint::RHipRoll, nao_command_msgs::msg::JointIndexes::RHIPROLL},
  {LolaEnums::Joint::RHipPitch, nao_command_msgs::msg::JointIndexes::RHIPPITCH},
  {LolaEnums::Joint::RKneePitch, nao_command_msgs::msg::JointIndexes::RKNEEPITCH},
  {LolaEnums::Joint::RAnklePitch, nao_command_msgs::msg::JointIndexes::RANKLEPITCH},
  {LolaEnums::Joint::RAnkleRoll, nao_command_msgs::msg::JointIndexes::RANKLEROLL},
  {LolaEnums::Joint::RShoulderPitch, nao_command_msgs::msg::JointIndexes::RSHOULDERPITCH},
  {LolaEnums::Joint::RShoulderRoll, nao_command_msgs::msg::JointIndexes::RSHOULDERROLL},
  {LolaEnums::Joint::RElbowYaw, nao_command_msgs::msg::JointIndexes::RELBOWYAW},
  {LolaEnums::Joint::RElbowRoll, nao_command_msgs::msg::JointIndexes::RELBOWROLL},
  {LolaEnums::Joint::RWristYaw, nao_command_msgs::msg::JointIndexes::RWRISTYAW},
  {LolaEnums::Joint::LHand, nao_command_msgs::msg::JointIndexes::LHAND},
  {LolaEnums::Joint::RHand, nao_command_msgs::msg::JointIndexes::RHAND},
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
  {nao_command_msgs::msg::LeftEarLeds::L0, LolaEnums::LeftEarLeds::Deg_0},
  {nao_command_msgs::msg::LeftEarLeds::L1, LolaEnums::LeftEarLeds::Deg_36},
  {nao_command_msgs::msg::LeftEarLeds::L2, LolaEnums::LeftEarLeds::Deg_72},
  {nao_command_msgs::msg::LeftEarLeds::L3, LolaEnums::LeftEarLeds::Deg_108},
  {nao_command_msgs::msg::LeftEarLeds::L4, LolaEnums::LeftEarLeds::Deg_144},
  {nao_command_msgs::msg::LeftEarLeds::L5, LolaEnums::LeftEarLeds::Deg_180},
  {nao_command_msgs::msg::LeftEarLeds::L6, LolaEnums::LeftEarLeds::Deg_216},
  {nao_command_msgs::msg::LeftEarLeds::L7, LolaEnums::LeftEarLeds::Deg_252},
  {nao_command_msgs::msg::LeftEarLeds::L8, LolaEnums::LeftEarLeds::Deg_288},
  {nao_command_msgs::msg::LeftEarLeds::L9, LolaEnums::LeftEarLeds::Deg_324}
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#right-ear
static const std::map<int, LolaEnums::RightEarLeds> right_ear_leds_msg_to_lola
{
  {nao_command_msgs::msg::RightEarLeds::R0, LolaEnums::RightEarLeds::Deg_0},
  {nao_command_msgs::msg::RightEarLeds::R1, LolaEnums::RightEarLeds::Deg_36},
  {nao_command_msgs::msg::RightEarLeds::R2, LolaEnums::RightEarLeds::Deg_72},
  {nao_command_msgs::msg::RightEarLeds::R3, LolaEnums::RightEarLeds::Deg_108},
  {nao_command_msgs::msg::RightEarLeds::R4, LolaEnums::RightEarLeds::Deg_144},
  {nao_command_msgs::msg::RightEarLeds::R5, LolaEnums::RightEarLeds::Deg_180},
  {nao_command_msgs::msg::RightEarLeds::R6, LolaEnums::RightEarLeds::Deg_216},
  {nao_command_msgs::msg::RightEarLeds::R7, LolaEnums::RightEarLeds::Deg_252},
  {nao_command_msgs::msg::RightEarLeds::R8, LolaEnums::RightEarLeds::Deg_288},
  {nao_command_msgs::msg::RightEarLeds::R9, LolaEnums::RightEarLeds::Deg_324}
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
static const std::map<int, LolaEnums::LeftEyeLeds> left_eye_leds_msg_to_lola
{
  {nao_command_msgs::msg::LeftEyeLeds::L0, LolaEnums::LeftEyeLeds::Deg_45},
  {nao_command_msgs::msg::LeftEyeLeds::L1, LolaEnums::LeftEyeLeds::Deg_0},
  {nao_command_msgs::msg::LeftEyeLeds::L2, LolaEnums::LeftEyeLeds::Deg_315},
  {nao_command_msgs::msg::LeftEyeLeds::L3, LolaEnums::LeftEyeLeds::Deg_270},
  {nao_command_msgs::msg::LeftEyeLeds::L4, LolaEnums::LeftEyeLeds::Deg_225},
  {nao_command_msgs::msg::LeftEyeLeds::L5, LolaEnums::LeftEyeLeds::Deg_180},
  {nao_command_msgs::msg::LeftEyeLeds::L6, LolaEnums::LeftEyeLeds::Deg_135},
  {nao_command_msgs::msg::LeftEyeLeds::L7, LolaEnums::LeftEyeLeds::Deg_90},
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
static const std::map<int, LolaEnums::RightEyeLeds> right_eye_leds_msg_to_lola
{
  {nao_command_msgs::msg::RightEyeLeds::R0, LolaEnums::RightEyeLeds::Deg_315},
  {nao_command_msgs::msg::RightEyeLeds::R1, LolaEnums::RightEyeLeds::Deg_270},
  {nao_command_msgs::msg::RightEyeLeds::R2, LolaEnums::RightEyeLeds::Deg_225},
  {nao_command_msgs::msg::RightEyeLeds::R3, LolaEnums::RightEyeLeds::Deg_180},
  {nao_command_msgs::msg::RightEyeLeds::R4, LolaEnums::RightEyeLeds::Deg_135},
  {nao_command_msgs::msg::RightEyeLeds::R5, LolaEnums::RightEyeLeds::Deg_90},
  {nao_command_msgs::msg::RightEyeLeds::R6, LolaEnums::RightEyeLeds::Deg_45},
  {nao_command_msgs::msg::RightEyeLeds::R7, LolaEnums::RightEyeLeds::Deg_0},
};

// See http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#head-tactile-sensor-led-locations
static const std::map<int, LolaEnums::SkullLeds> head_leds_msg_to_lola
{
  {nao_command_msgs::msg::HeadLeds::B0, LolaEnums::SkullLeds::Front_Right_1},
  {nao_command_msgs::msg::HeadLeds::B1, LolaEnums::SkullLeds::Front_Right_0},
  {nao_command_msgs::msg::HeadLeds::B2, LolaEnums::SkullLeds::Middle_Right_0},
  {nao_command_msgs::msg::HeadLeds::B3, LolaEnums::SkullLeds::Rear_Right_0},
  {nao_command_msgs::msg::HeadLeds::B4, LolaEnums::SkullLeds::Rear_Right_1},
  {nao_command_msgs::msg::HeadLeds::B5, LolaEnums::SkullLeds::Rear_Right_2},
  {nao_command_msgs::msg::HeadLeds::B6, LolaEnums::SkullLeds::Rear_Left_2},
  {nao_command_msgs::msg::HeadLeds::B7, LolaEnums::SkullLeds::Rear_Left_1},
  {nao_command_msgs::msg::HeadLeds::B8, LolaEnums::SkullLeds::Rear_Left_0},
  {nao_command_msgs::msg::HeadLeds::B9, LolaEnums::SkullLeds::Middle_Left_0},
  {nao_command_msgs::msg::HeadLeds::B10, LolaEnums::SkullLeds::Front_Left_0},
  {nao_command_msgs::msg::HeadLeds::B11, LolaEnums::SkullLeds::Front_Left_1},
};

}  // namespace IndexConversion

#endif  // NAO_LOLA__COMMAND_INDEX_CONVERSION_HPP_
