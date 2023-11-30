// Copyright 2023 Kenji Brameld
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

#include "conversion.hpp"

#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

namespace conversion
{

std::vector<std::string> joint_names = {
  "HeadYaw",
  "HeadPitch",
  "LShoulderPitch",
  "LShoulderRoll",
  "LElbowYaw",
  "LElbowRoll",
  "LWristYaw",
  "LHipYawPitch",
  "LHipRoll",
  "LHipPitch",
  "LKneePitch",
  "LAnklePitch",
  "LAnkleRoll",
  "RHipRoll",
  "RHipPitch",
  "RKneePitch",
  "RAnklePitch",
  "RAnkleRoll",
  "RShoulderPitch",
  "RShoulderRoll",
  "RElbowYaw",
  "RElbowRoll",
  "RWristYaw",
  "LHand",
  "RHand",
};

sensor_msgs::msg::JointState toJointState(
  const nao_lola_sensor_msgs::msg::JointPositions& joint_positions)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names;

  for (unsigned i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    joint_state.position.push_back(joint_positions.positions[i]);
  }

  return joint_state;
}

}  // namespace conversion
