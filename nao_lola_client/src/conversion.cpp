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

#include "conversion.hpp"

#include <string>
#include <vector>

#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

namespace conversion
{

static const std::vector<std::string> joint_names = {
  "HeadYaw",
  "HeadPitch",
  "LShoulderPitch",
  "LShoulderRoll",
  "LElbowYaw",
  "LElbowRoll",
  "LWristYaw",
  "LHipYawPitch",
  "LHipRoll",
  "LHipPitch",
  "LKneePitch",
  "LAnklePitch",
  "LAnkleRoll",
  "RHipRoll",
  "RHipPitch",
  "RKneePitch",
  "RAnklePitch",
  "RAnkleRoll",
  "RShoulderPitch",
  "RShoulderRoll",
  "RElbowYaw",
  "RElbowRoll",
  "RWristYaw",
  "LHand",
  "RHand",
};

sensor_msgs::msg::Imu toImu(
  const nao_lola_sensor_msgs::msg::Accelerometer & accelerometer,
  const nao_lola_sensor_msgs::msg::Gyroscope & gyroscope)
{
  sensor_msgs::msg::Imu imu;

  // Frame id is set to the same as the accelerometer frame id. Technically, there is a very small
  // difference between the accelerometer and gyroscope frame positions (2mm), but this is
  // negligible.
  imu.header.frame_id = "ImuTorsoAccelerometer_frame";

  // Orientation is not available from the robot.
  // The robot provides an AngleX and AngleY, but there is no easy way to store this in the imu msg.
  // According to the documentation in sensor_msgs::msg::Imu.msg, if there is no orientation
  // estimation, the orientation covariance should be set to -1.
  imu.orientation_covariance[0] = -1;

  // Linear Acceleration
  imu.linear_acceleration.x = accelerometer.x;
  imu.linear_acceleration.y = accelerometer.y;
  imu.linear_acceleration.z = accelerometer.z;

  // Angular Velocity
  imu.angular_velocity.x = gyroscope.x;
  imu.angular_velocity.y = gyroscope.y;
  imu.angular_velocity.z = gyroscope.z;

  return imu;
}

sensor_msgs::msg::JointState toJointState(
  const nao_lola_sensor_msgs::msg::JointPositions & joint_positions)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names;

  for (unsigned i = 0; i < nao_lola_sensor_msgs::msg::JointIndexes::NUMJOINTS; ++i) {
    joint_state.position.push_back(joint_positions.positions[i]);
  }

  return joint_state;
}

}  // namespace conversion
