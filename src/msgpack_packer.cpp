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

#include <string>
#include <map>
#include <utility>
#include "nao_lola/msgpack_packer.hpp"
#include "msgpack.hpp"

std::string MsgpackPacker::getPacked()
{
  msgpack::zone z;
  std::map<std::string, msgpack::object> map;

  if (position) {
    map.insert(std::make_pair("Position", msgpack::object(position, z)));
  }

  if (stiffness) {
    map.insert(std::make_pair("Stiffness", msgpack::object(stiffness, z)));
  }

  if (chest) {
    map.insert(std::make_pair("Chest", msgpack::object(chest, z)));
  }

  if (l_ear) {
    map.insert(std::make_pair("LEar", msgpack::object(l_ear, z)));
  }

  if (r_ear) {
    map.insert(std::make_pair("REar", msgpack::object(r_ear, z)));
  }

  if (l_eye) {
    map.insert(std::make_pair("LEye", msgpack::object(l_eye, z)));
  }

  if (r_eye) {
    map.insert(std::make_pair("REye", msgpack::object(r_eye, z)));
  }

  if (l_foot) {
    map.insert(std::make_pair("LFoot", msgpack::object(l_foot, z)));
  }

  if (r_foot) {
    map.insert(std::make_pair("RFoot", msgpack::object(r_foot, z)));
  }

  if (skull) {
    map.insert(std::make_pair("Skull", msgpack::object(skull, z)));
  }

  if (sonar) {
    map.insert(std::make_pair("Sonar", msgpack::object(sonar, z)));
  }

  std::stringstream buffer;
  msgpack::pack(buffer, map);
  std::string packed = buffer.str();

  return packed;
}

// void MsgpackPacker::setPosition(std::shared_ptr<std::array<float>> position)
// {
//   this->position = position;
// }

// void MsgpackPacker::setStiffness(std::shared_ptr<std::array<float>> stiffness)
// {
//   this->stiffness = stiffness;
// }

// void MsgpackPacker::setChest(std::shared_ptr<std::array<float>> chest)
// {
//   this->chest = chest;
// }

// void MsgpackPacker::setLEar(std::shared_ptr<std::array<float>> l_ear)
// {
//   this->l_ear = l_ear;
// }

// void MsgpackPacker::setREar(std::shared_ptr<std::array<float>> r_ear)
// {
//   this->r_ear = r_ear;
// }

// void MsgpackPacker::setLEye(std::shared_ptr<std::array<float>> l_eye)
// {
//   this->l_eye = l_eye;
// }

// void MsgpackPacker::setREye(std::shared_ptr<std::array<float>> r_eye)
// {
//   this->r_eye = r_eye;
// }

// void MsgpackPacker::setLFoot(std::shared_ptr<std::array<float>> l_foot)
// {
//   this->l_foot = l_foot;
// }

// void MsgpackPacker::setRFoot(std::shared_ptr<std::array<float>> r_foot)
// {
//   this->r_foot = r_foot;
// }

// void MsgpackPacker::setSkull(std::shared_ptr<std::array<float>> skull)
// {
//   this->skull = skull;
// }

// void MsgpackPacker::setSonar(std::shared_ptr<std::array<bool>> sonar)
// {
//   this->sonar = sonar;
// }
