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

#ifndef NAO_LOLA__MSGPACK_PACKER_HPP_
#define NAO_LOLA__MSGPACK_PACKER_HPP_

#include <string>
#include <vector>
#include <memory>
#include "nao_interfaces/msg/joints.hpp"

class MsgpackPacker
{
public:
  std::string getPacked();

  void setJoints(std::shared_ptr<nao_interfaces::msg::Joints> joints);
  // void setChest(std::shared_ptr<std::vector<float>> chest);
  // void setLEar(std::shared_ptr<std::vector<float>> l_ear);
  // void setREar(std::shared_ptr<std::vector<float>> r_ear);
  // void setLEye(std::shared_ptr<std::vector<float>> l_eye);
  // void setREye(std::shared_ptr<std::vector<float>> r_eye);
  // void setLFoot(std::shared_ptr<std::vector<float>> l_foot);
  // void setRFoot(std::shared_ptr<std::vector<float>> r_foot);
  // void setSkull(std::shared_ptr<std::vector<float>> skull);
  // void setSonar(std::shared_ptr<std::vector<bool>> sonar);

private:
  std::shared_ptr<std::vector<float>> position;
  std::shared_ptr<std::vector<float>> stiffness;
  std::shared_ptr<std::vector<float>> chest;
  std::shared_ptr<std::vector<float>> l_ear;
  std::shared_ptr<std::vector<float>> r_ear;
  std::shared_ptr<std::vector<float>> l_eye;
  std::shared_ptr<std::vector<float>> r_eye;
  std::shared_ptr<std::vector<float>> l_foot;
  std::shared_ptr<std::vector<float>> r_foot;
  std::shared_ptr<std::vector<float>> skull;
  std::shared_ptr<std::vector<bool>> sonar;
};

#endif  // NAO_LOLA__MSGPACK_PACKER_HPP_
