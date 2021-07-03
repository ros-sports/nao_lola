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

#include "nao_lola/msgpack_parser.hpp"
#include <iostream>

enum class Accelerometer {x, y, z};
enum class Angles {x, y};
enum class Touch {
  chest,
  head_front,
  head_middle,
  head_rear,
  l_foot_bumper_left,
  l_foot_bumper_right,
  r_foot_bumper_left,
  r_foot_bumper_right
};

MsgpackParser::MsgpackParser(std::string packed)
{
  msgpack::object_handle oh =
      msgpack::unpack(packed.data(), packed.size());

  unpacked = oh.get().as<std::map<std::string, msgpack::object>>();
}

nao_interfaces::msg::Accelerometer MsgpackParser::getAccelerometer()
{
  nao_interfaces::msg::Accelerometer acc;
  std::vector<float> vec = unpacked.at("Accelerometer").as<std::vector<float>>();
  acc.x = vec.at(static_cast<int>(Accelerometer::x));
  acc.y = vec.at(static_cast<int>(Accelerometer::y));
  acc.z = vec.at(static_cast<int>(Accelerometer::z));
  return acc;
}

nao_interfaces::msg::Angle MsgpackParser::getAngle()
{
  nao_interfaces::msg::Angle ang;
  std::vector<float> vec = unpacked.at("Angles").as<std::vector<float>>();
  ang.x = vec.at(static_cast<int>(Angles::x));
  ang.y = vec.at(static_cast<int>(Angles::y));
  return ang;
}

nao_interfaces::msg::Buttons MsgpackParser::getButtons()
{
  nao_interfaces::msg::Buttons but;
  std::vector<float> vec = unpacked.at("Touch").as<std::vector<float>>();
  but.chest = vec.at(static_cast<int>(Touch::chest));
  but.l_foot_bumper_left = vec.at(static_cast<int>(Touch::l_foot_bumper_left));
  but.l_foot_bumper_right = vec.at(static_cast<int>(Touch::l_foot_bumper_right));
  but.r_foot_bumper_left = vec.at(static_cast<int>(Touch::r_foot_bumper_left));
  but.r_foot_bumper_right = vec.at(static_cast<int>(Touch::r_foot_bumper_right));
  return but;
}

std::vector<std::string> MsgpackParser::getRobotConfig()
{
  return unpacked.at("RobotConfig").as<std::vector<std::string>>();
}
