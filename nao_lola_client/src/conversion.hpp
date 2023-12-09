// Copyright 2023 Kenji Brameld
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

#ifndef CONVERSION_HPP_
#define CONVERSION_HPP_

#include "nao_lola_sensor_msgs/msg/accelerometer.hpp"
#include "nao_lola_sensor_msgs/msg/gyroscope.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace conversion
{

sensor_msgs::msg::Imu toImu(
  const nao_lola_sensor_msgs::msg::Accelerometer & accelerometer,
  const nao_lola_sensor_msgs::msg::Gyroscope & gyroscope);

sensor_msgs::msg::JointState toJointState(
  const nao_lola_sensor_msgs::msg::JointPositions & joint_positions);

}  // namespace conversion

#endif  // CONVERSION_HPP_
