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

#ifndef NAO_LOLA__MSGPACK_PARSER_HPP_
#define NAO_LOLA__MSGPACK_PARSER_HPP_

#include <map>
#include <string>
#include <vector>
#include "msgpack.hpp"
#include "nao_interfaces/msg/joint_currents.hpp"
#include "nao_interfaces/msg/joint_positions.hpp"
#include "nao_interfaces/msg/joint_stiffnesses.hpp"
#include "nao_interfaces/msg/joint_temperatures.hpp"
#include "nao_interfaces/msg/joint_statuses.hpp"
#include "nao_interfaces/msg/buttons.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/angle.hpp"
#include "nao_interfaces/msg/sonar.hpp"
#include "nao_interfaces/msg/fsr.hpp"
#include "nao_interfaces/msg/touch.hpp"
#include "nao_interfaces/msg/battery.hpp"
#include "nao_interfaces/msg/robot_config.hpp"

class MsgpackParser
{
public:
  explicit MsgpackParser(char data[896]);

  nao_interfaces::msg::Accelerometer getAccelerometer();
  nao_interfaces::msg::Angle getAngle();
  nao_interfaces::msg::Buttons getButtons();
  nao_interfaces::msg::FSR getFSR();
  nao_interfaces::msg::Gyroscope getGyroscope();
  nao_interfaces::msg::JointCurrents getJointCurrents();
  nao_interfaces::msg::JointPositions getJointPositions();
  nao_interfaces::msg::JointStiffnesses getJointStiffnesses();
  nao_interfaces::msg::JointTemperatures getJointTemperatures();
  nao_interfaces::msg::JointStatuses getJointStatuses();
  nao_interfaces::msg::Sonar getSonar();
  nao_interfaces::msg::Touch getTouch();
  nao_interfaces::msg::Battery getBattery();
  nao_interfaces::msg::RobotConfig getRobotConfig();

private:
  std::map<std::string, msgpack::object> unpacked;
};


#endif  // NAO_LOLA__MSGPACK_PARSER_HPP_
