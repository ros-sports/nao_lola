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
#include "nao_sensor_msgs/msg/joint_currents.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/joint_stiffnesses.hpp"
#include "nao_sensor_msgs/msg/joint_temperatures.hpp"
#include "nao_sensor_msgs/msg/joint_statuses.hpp"
#include "nao_sensor_msgs/msg/buttons.hpp"
#include "nao_sensor_msgs/msg/accelerometer.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/sonar.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "nao_sensor_msgs/msg/touch.hpp"
#include "nao_sensor_msgs/msg/battery.hpp"
#include "nao_sensor_msgs/msg/robot_config.hpp"

class MsgpackParser
{
public:
  explicit MsgpackParser(char data[], int size);

  nao_sensor_msgs::msg::Accelerometer getAccelerometer();
  nao_sensor_msgs::msg::Angle getAngle();
  nao_sensor_msgs::msg::Buttons getButtons();
  nao_sensor_msgs::msg::FSR getFSR();
  nao_sensor_msgs::msg::Gyroscope getGyroscope();
  nao_sensor_msgs::msg::JointCurrents getJointCurrents();
  nao_sensor_msgs::msg::JointPositions getJointPositions();
  nao_sensor_msgs::msg::JointStiffnesses getJointStiffnesses();
  nao_sensor_msgs::msg::JointTemperatures getJointTemperatures();
  nao_sensor_msgs::msg::JointStatuses getJointStatuses();
  nao_sensor_msgs::msg::Sonar getSonar();
  nao_sensor_msgs::msg::Touch getTouch();
  nao_sensor_msgs::msg::Battery getBattery();
  nao_sensor_msgs::msg::RobotConfig getRobotConfig();

private:
  msgpack::object_handle oh;  // Keep this variable throughout the lifetime of this object
  std::map<std::string, msgpack::object> unpacked;
};


#endif  // NAO_LOLA__MSGPACK_PARSER_HPP_
