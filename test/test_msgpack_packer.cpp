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
#include "msgpack.hpp"
#include "nao_lola/msgpack_packer.hpp"
#include "nao_interfaces/msg/joints.hpp"

class TestMsgpackPacker : public ::testing::Test
{
public:
  MsgpackPacker packer;

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

private:
  static std::map<std::string, msgpack::object> unpack(std::string packed)
  {
    msgpack::object_handle oh =
      msgpack::unpack(packed.data(), packed.size());
    return oh.get().as<std::map<std::string, msgpack::object>>();
  }
};

// TEST_F(TestMsgpackPacker, TestPackPosition)
// {
//   std::vector<float> position = {
//     1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0, 2.0};
//   packer.setPosition(position);
//   std::string packed = packer.getPacked();

//   EXPECT_EQ(getFloatVector(packed, "Position"), position);
// }

// TEST_F(TestMsgpackPacker, TestStiffness)
// {
//   std::vector<float> stiffness = {
//     0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.1, 0.1, 0.1, 0.1, 0.1};
//   nao_interfaces::msg::Joints joints;
//   joints.angles[nao_interfaces::msg::Joints::LELBOWYAW] = 0.5;
//   joints.stiffnesses[nao_interfaces::msg::Joints::LELBOWYAW] = 0.7;

//   packer.setJoints(std::make_shared(joints));
//   std::string packed = packer.getPacked();

//   EXPECT_EQ(getFloatVector(packed, "Stiffness"), stiffness);
// }

// TEST_F(TestMsgpackPacker, TestChest)
// {
//   std::vector<float> chest = {0.3, 0.3, 0.3};
//   packer.setChest(chest);
//   std::string packed = packer.getPacked();

//   EXPECT_EQ(getFloatVector(packed, "Chest"), chest);
// }
