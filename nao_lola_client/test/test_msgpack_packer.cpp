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

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "msgpack/object.hpp"
#include "msgpack/adaptor/bool.hpp"
#include "msgpack/adaptor/float.hpp"
#include "msgpack/adaptor/map.hpp"
#include "msgpack/adaptor/string.hpp"
#include "msgpack/adaptor/vector.hpp"
#include "msgpack/adaptor/vector_bool.hpp"
#include "msgpack/unpack.hpp"
#include "nao_lola_client/msgpack_packer.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_client/lola_enums.hpp"


static std::vector<float> getFloatVector(std::string packed, std::string mapKey);
static std::vector<bool> getBoolVector(std::string packed, std::string mapKey);
static std::map<std::string, msgpack::object> unpack(std::string packed);

class TestMsgpackPacker : public ::testing::Test
{
public:
  MsgpackPacker packer;
};

TEST_F(TestMsgpackPacker, TestJointPositions)
{
  nao_lola_command_msgs::msg::JointPositions command;
  command.indexes.push_back(nao_lola_command_msgs::msg::JointIndexes::HEADYAW);
  command.positions.push_back(1.01);
  command.indexes.push_back(nao_lola_command_msgs::msg::JointIndexes::RHAND);
  command.positions.push_back(2.0);

  packer.setJointPositions(command);
  std::string packed = packer.getPacked();

  std::vector<float> position(static_cast<int>(LolaEnums::Joint::NUM_JOINTS), 0);
  position.at(static_cast<int>(LolaEnums::Joint::HeadYaw)) = 1.01;
  position.at(static_cast<int>(LolaEnums::Joint::RHand)) = 2.0;
  EXPECT_EQ(getFloatVector(packed, "Position"), position);
}

TEST_F(TestMsgpackPacker, TestJointStiffnesses)
{
  nao_lola_command_msgs::msg::JointStiffnesses command;
  command.indexes.push_back(nao_lola_command_msgs::msg::JointIndexes::HEADPITCH);
  command.stiffnesses.push_back(0.3);
  command.indexes.push_back(nao_lola_command_msgs::msg::JointIndexes::LHAND);
  command.stiffnesses.push_back(0.7);

  packer.setJointStiffnesses(command);
  std::string packed = packer.getPacked();

  std::vector<float> stiffness(static_cast<int>(LolaEnums::Joint::NUM_JOINTS), 0);
  stiffness.at(static_cast<int>(LolaEnums::Joint::HeadPitch)) = 0.3;
  stiffness.at(static_cast<int>(LolaEnums::Joint::LHand)) = 0.7;
  EXPECT_EQ(getFloatVector(packed, "Stiffness"), stiffness);
}

TEST_F(TestMsgpackPacker, TestChestLed)
{
  nao_lola_command_msgs::msg::ChestLed chestLed;
  chestLed.color.r = 0.1;
  chestLed.color.g = 0.5;
  chestLed.color.b = 1.0;

  packer.setChestLed(chestLed);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.1, 0.5, 1.0};
  EXPECT_EQ(getFloatVector(packed, "Chest"), expected);
}

TEST_F(TestMsgpackPacker, TestLeftEarLeds)
{
  nao_lola_command_msgs::msg::LeftEarLeds leftEarLeds;
  leftEarLeds.intensities[leftEarLeds.L0] = 0.1;
  leftEarLeds.intensities[leftEarLeds.L1] = 0.2;
  leftEarLeds.intensities[leftEarLeds.L2] = 0.3;
  leftEarLeds.intensities[leftEarLeds.L3] = 0.4;
  leftEarLeds.intensities[leftEarLeds.L4] = 0.5;
  leftEarLeds.intensities[leftEarLeds.L5] = 0.6;
  leftEarLeds.intensities[leftEarLeds.L6] = 0.7;
  leftEarLeds.intensities[leftEarLeds.L7] = 0.8;
  leftEarLeds.intensities[leftEarLeds.L8] = 0.9;
  leftEarLeds.intensities[leftEarLeds.L9] = 1.0;

  packer.setLeftEarLeds(leftEarLeds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{
    0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
  EXPECT_EQ(getFloatVector(packed, "LEar"), expected);
}


TEST_F(TestMsgpackPacker, TestRightEarLeds)
{
  nao_lola_command_msgs::msg::RightEarLeds rightEarLeds;
  rightEarLeds.intensities[rightEarLeds.R0] = 0.1;
  rightEarLeds.intensities[rightEarLeds.R1] = 0.2;
  rightEarLeds.intensities[rightEarLeds.R2] = 0.3;
  rightEarLeds.intensities[rightEarLeds.R3] = 0.4;
  rightEarLeds.intensities[rightEarLeds.R4] = 0.5;
  rightEarLeds.intensities[rightEarLeds.R5] = 0.6;
  rightEarLeds.intensities[rightEarLeds.R6] = 0.7;
  rightEarLeds.intensities[rightEarLeds.R7] = 0.8;
  rightEarLeds.intensities[rightEarLeds.R8] = 0.9;
  rightEarLeds.intensities[rightEarLeds.R9] = 1.0;

  packer.setRightEarLeds(rightEarLeds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{
    1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
  EXPECT_EQ(getFloatVector(packed, "REar"), expected);
}

TEST_F(TestMsgpackPacker, TestLeftEyeLeds)
{
  // Explanation of eye correspondence: http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
  nao_lola_command_msgs::msg::LeftEyeLeds leftEyeLeds;
  leftEyeLeds.colors[leftEyeLeds.L0].r = 0.01;
  leftEyeLeds.colors[leftEyeLeds.L0].g = 0.02;
  leftEyeLeds.colors[leftEyeLeds.L0].b = 0.03;
  leftEyeLeds.colors[leftEyeLeds.L1].r = 0.04;
  leftEyeLeds.colors[leftEyeLeds.L1].g = 0.05;
  leftEyeLeds.colors[leftEyeLeds.L1].b = 0.06;
  leftEyeLeds.colors[leftEyeLeds.L2].r = 0.07;
  leftEyeLeds.colors[leftEyeLeds.L2].g = 0.08;
  leftEyeLeds.colors[leftEyeLeds.L2].b = 0.09;
  leftEyeLeds.colors[leftEyeLeds.L3].r = 0.10;
  leftEyeLeds.colors[leftEyeLeds.L3].g = 0.11;
  leftEyeLeds.colors[leftEyeLeds.L3].b = 0.12;
  leftEyeLeds.colors[leftEyeLeds.L4].r = 0.13;
  leftEyeLeds.colors[leftEyeLeds.L4].g = 0.14;
  leftEyeLeds.colors[leftEyeLeds.L4].b = 0.15;
  leftEyeLeds.colors[leftEyeLeds.L5].r = 0.16;
  leftEyeLeds.colors[leftEyeLeds.L5].g = 0.17;
  leftEyeLeds.colors[leftEyeLeds.L5].b = 0.18;
  leftEyeLeds.colors[leftEyeLeds.L6].r = 0.19;
  leftEyeLeds.colors[leftEyeLeds.L6].g = 0.20;
  leftEyeLeds.colors[leftEyeLeds.L6].b = 0.21;
  leftEyeLeds.colors[leftEyeLeds.L7].r = 0.22;
  leftEyeLeds.colors[leftEyeLeds.L7].g = 0.23;
  leftEyeLeds.colors[leftEyeLeds.L7].b = 0.24;

  packer.setLeftEyeLeds(leftEyeLeds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{
    0.01, 0.04, 0.07, 0.10, 0.13, 0.16, 0.19, 0.22,
    0.02, 0.05, 0.08, 0.11, 0.14, 0.17, 0.20, 0.23,
    0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24};
  EXPECT_EQ(getFloatVector(packed, "LEye"), expected);
}

TEST_F(TestMsgpackPacker, TestRightEyeLeds)
{
  // Explanation of eye correspondence: http://doc.aldebaran.com/2-5/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
  nao_lola_command_msgs::msg::RightEyeLeds rightEyeLeds;
  rightEyeLeds.colors[rightEyeLeds.R0].r = 0.01;
  rightEyeLeds.colors[rightEyeLeds.R0].g = 0.02;
  rightEyeLeds.colors[rightEyeLeds.R0].b = 0.03;
  rightEyeLeds.colors[rightEyeLeds.R1].r = 0.04;
  rightEyeLeds.colors[rightEyeLeds.R1].g = 0.05;
  rightEyeLeds.colors[rightEyeLeds.R1].b = 0.06;
  rightEyeLeds.colors[rightEyeLeds.R2].r = 0.07;
  rightEyeLeds.colors[rightEyeLeds.R2].g = 0.08;
  rightEyeLeds.colors[rightEyeLeds.R2].b = 0.09;
  rightEyeLeds.colors[rightEyeLeds.R3].r = 0.10;
  rightEyeLeds.colors[rightEyeLeds.R3].g = 0.11;
  rightEyeLeds.colors[rightEyeLeds.R3].b = 0.12;
  rightEyeLeds.colors[rightEyeLeds.R4].r = 0.13;
  rightEyeLeds.colors[rightEyeLeds.R4].g = 0.14;
  rightEyeLeds.colors[rightEyeLeds.R4].b = 0.15;
  rightEyeLeds.colors[rightEyeLeds.R5].r = 0.16;
  rightEyeLeds.colors[rightEyeLeds.R5].g = 0.17;
  rightEyeLeds.colors[rightEyeLeds.R5].b = 0.18;
  rightEyeLeds.colors[rightEyeLeds.R6].r = 0.19;
  rightEyeLeds.colors[rightEyeLeds.R6].g = 0.20;
  rightEyeLeds.colors[rightEyeLeds.R6].b = 0.21;
  rightEyeLeds.colors[rightEyeLeds.R7].r = 0.22;
  rightEyeLeds.colors[rightEyeLeds.R7].g = 0.23;
  rightEyeLeds.colors[rightEyeLeds.R7].b = 0.24;

  packer.setRightEyeLeds(rightEyeLeds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{
    0.22, 0.19, 0.16, 0.13, 0.10, 0.07, 0.04, 0.01,
    0.23, 0.20, 0.17, 0.14, 0.11, 0.08, 0.05, 0.02,
    0.24, 0.21, 0.18, 0.15, 0.12, 0.09, 0.06, 0.03};
  EXPECT_EQ(getFloatVector(packed, "REye"), expected);
}

TEST_F(TestMsgpackPacker, TestLeftFootLed)
{
  nao_lola_command_msgs::msg::LeftFootLed leftFootLed;
  leftFootLed.color.r = 0.2;
  leftFootLed.color.g = 0.3;
  leftFootLed.color.b = 0.4;

  packer.setLeftFootLed(leftFootLed);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.2, 0.3, 0.4};
  EXPECT_EQ(getFloatVector(packed, "LFoot"), expected);
}

TEST_F(TestMsgpackPacker, TestRightFootLed)
{
  nao_lola_command_msgs::msg::RightFootLed rightFootLed;
  rightFootLed.color.r = 0.5;
  rightFootLed.color.g = 0.6;
  rightFootLed.color.b = 0.7;

  packer.setRightFootLed(rightFootLed);
  std::string packed = packer.getPacked();

  std::vector<float> expected{0.5, 0.6, 0.7};
  EXPECT_EQ(getFloatVector(packed, "RFoot"), expected);
}

TEST_F(TestMsgpackPacker, TestHeadLeds)
{
  nao_lola_command_msgs::msg::HeadLeds headLeds;
  headLeds.intensities[headLeds.B0] = 0.00;
  headLeds.intensities[headLeds.B1] = 0.01;
  headLeds.intensities[headLeds.B2] = 0.02;
  headLeds.intensities[headLeds.B3] = 0.03;
  headLeds.intensities[headLeds.B4] = 0.04;
  headLeds.intensities[headLeds.B5] = 0.05;
  headLeds.intensities[headLeds.B6] = 0.06;
  headLeds.intensities[headLeds.B7] = 0.07;
  headLeds.intensities[headLeds.B8] = 0.08;
  headLeds.intensities[headLeds.B9] = 0.09;
  headLeds.intensities[headLeds.B10] = 0.10;
  headLeds.intensities[headLeds.B11] = 0.11;

  packer.setHeadLeds(headLeds);
  std::string packed = packer.getPacked();

  std::vector<float> expected{
    0.11, 0.10, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03, 0.02, 0.01, 0.00};
  EXPECT_EQ(getFloatVector(packed, "Skull"), expected);
}

TEST_F(TestMsgpackPacker, TestSonarUsage)
{
  nao_lola_command_msgs::msg::SonarUsage sonarUsage;
  sonarUsage.left = true;
  sonarUsage.right = false;

  packer.setSonarUsage(sonarUsage);
  std::string packed = packer.getPacked();

  std::vector<bool> expected{true, false};
  EXPECT_EQ(getBoolVector(packed, "Sonar"), expected);
}

// Helper functions
static std::vector<float> getFloatVector(std::string packed, std::string mapKey)
{
  std::map<std::string, msgpack::object> unpacked = unpack(packed);
  return unpacked.at(mapKey).as<std::vector<float>>();
}

static std::vector<bool> getBoolVector(std::string packed, std::string mapKey)
{
  std::map<std::string, msgpack::object> unpacked = unpack(packed);
  return unpacked.at(mapKey).as<std::vector<bool>>();
}

static std::map<std::string, msgpack::object> unpack(std::string packed)
{
  msgpack::object_handle oh =
    msgpack::unpack(packed.data(), packed.size());
  return oh.get().as<std::map<std::string, msgpack::object>>();
}
