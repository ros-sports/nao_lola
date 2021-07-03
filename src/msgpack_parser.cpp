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

MsgpackParser::MsgpackParser(std::string packed)
{
  msgpack::object_handle oh =
      msgpack::unpack(packed.data(), packed.size());

  unpacked = oh.get().as<std::map<std::string, msgpack::object>>();
}

nao_interfaces::msg::Accelerometer MsgpackParser::getAccelerometer()
{
  nao_interfaces::msg::Accelerometer acc;
  std::vector<float> accs = unpacked.at("Accelerometer").as<std::vector<float>>();
  acc.x = accs.at(0);
  acc.y = accs.at(1);
  acc.z = accs.at(2);
  return acc;
}

std::vector<std::string> MsgpackParser::getRobotConfig()
{
  return unpacked.at("RobotConfig").as<std::vector<std::string>>();
}
