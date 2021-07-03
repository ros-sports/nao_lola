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
#include "nao_lola/lola_enums.hpp"


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
  acc.x = vec.at(static_cast<int>(Accelerometer::X));
  acc.y = vec.at(static_cast<int>(Accelerometer::Y));
  acc.z = vec.at(static_cast<int>(Accelerometer::Z));
  return acc;
}

nao_interfaces::msg::Angle MsgpackParser::getAngle()
{
  nao_interfaces::msg::Angle ang;
  std::vector<float> vec = unpacked.at("Angles").as<std::vector<float>>();
  ang.x = vec.at(static_cast<int>(Angles::X));
  ang.y = vec.at(static_cast<int>(Angles::Y));
  return ang;
}

nao_interfaces::msg::Buttons MsgpackParser::getButtons()
{
  nao_interfaces::msg::Buttons but;
  std::vector<float> vec = unpacked.at("Touch").as<std::vector<float>>();
  but.chest = vec.at(static_cast<int>(Touch::ChestBoard_Button));
  but.l_foot_bumper_left = vec.at(static_cast<int>(Touch::LFoot_Bumper_Left));
  but.l_foot_bumper_right = vec.at(static_cast<int>(Touch::LFoot_Bumper_Right));
  but.r_foot_bumper_left = vec.at(static_cast<int>(Touch::RFoot_Bumper_Left));
  but.r_foot_bumper_right = vec.at(static_cast<int>(Touch::RFoot_Bumper_Right));
  return but;
}

std::vector<std::string> MsgpackParser::getRobotConfig()
{
  return unpacked.at("RobotConfig").as<std::vector<std::string>>();
}

nao_interfaces::msg::FSR MsgpackParser::getFSR()
{
  nao_interfaces::msg::FSR fsr;
  std::vector<float> vec = unpacked.at("FSR").as<std::vector<float>>();
  fsr.l_foot_front_left = vec.at(static_cast<int>(FSR::LFoot_FrontLeft));
  fsr.l_foot_front_right = vec.at(static_cast<int>(FSR::LFoot_FrontRight));
  fsr.l_foot_back_left = vec.at(static_cast<int>(FSR::LFoot_RearLeft));
  fsr.l_foot_back_right = vec.at(static_cast<int>(FSR::LFoot_RearRight));
  fsr.r_foot_front_left = vec.at(static_cast<int>(FSR::RFoot_FrontLeft));
  fsr.r_foot_front_right = vec.at(static_cast<int>(FSR::RFoot_FrontRight));
  fsr.r_foot_back_left = vec.at(static_cast<int>(FSR::RFoot_RearLeft));
  fsr.r_foot_back_right = vec.at(static_cast<int>(FSR::RFoot_RearRight));
  return fsr;
}