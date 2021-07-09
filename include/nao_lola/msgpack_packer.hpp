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
#include "nao_interfaces/msg/chest_led.hpp"
#include "nao_interfaces/msg/left_ear_leds.hpp"
#include "nao_interfaces/msg/right_ear_leds.hpp"
#include "nao_interfaces/msg/left_eye_leds.hpp"
#include "nao_interfaces/msg/right_eye_leds.hpp"
#include "nao_interfaces/msg/left_foot_led.hpp"
#include "nao_interfaces/msg/right_foot_led.hpp"
#include "nao_interfaces/msg/skull_leds.hpp"
#include "nao_interfaces/msg/sonar_usage.hpp"


class MsgpackPacker
{
public:
  std::string getPacked();

  void setJoints(std::shared_ptr<nao_interfaces::msg::Joints> joints);
  void setChestLed(std::shared_ptr<nao_interfaces::msg::ChestLed> chestLed);
  void setLeftEarLeds(std::shared_ptr<nao_interfaces::msg::LeftEarLeds> leftEarLeds);
  void setRightEarLeds(std::shared_ptr<nao_interfaces::msg::RightEarLeds> rightEarLeds);
  void setLeftEyeLeds(std::shared_ptr<nao_interfaces::msg::LeftEyeLeds> leftEyeLeds);
  void setRightEyeLeds(std::shared_ptr<nao_interfaces::msg::RightEyeLeds> rightEyeLeds);
  void setLeftFootLeds(std::shared_ptr<nao_interfaces::msg::LeftFootLed> leftFootLed);
  void setRightFootLeds(std::shared_ptr<nao_interfaces::msg::RightFootLed> rightFootLed);
  void setSkullLeds(std::shared_ptr<nao_interfaces::msg::SkullLeds> skullLeds);
  void setSonarUsage(std::shared_ptr<nao_interfaces::msg::SonarUsage> sonarUsage);

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
