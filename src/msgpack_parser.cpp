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

#include <vector>
#include <map>
#include <string>
#include "nao_lola/msgpack_parser.hpp"
#include "nao_lola/lola_enums.hpp"

static const std::map<Joint, int> index_lola_to_msg = {
  {Joint::HeadYaw, nao_interfaces::msg::Joints::HEADYAW},
  {Joint::HeadPitch, nao_interfaces::msg::Joints::HEADPITCH},
  {Joint::LShoulderPitch, nao_interfaces::msg::Joints::LSHOULDERPITCH},
  {Joint::LShoulderRoll, nao_interfaces::msg::Joints::LSHOULDERROLL},
  {Joint::LElbowYaw, nao_interfaces::msg::Joints::LELBOWYAW},
  {Joint::LElbowRoll, nao_interfaces::msg::Joints::LELBOWROLL},
  {Joint::LWristYaw, nao_interfaces::msg::Joints::LWRISTYAW},
  {Joint::LHipYawPitch, nao_interfaces::msg::Joints::LHIPYAWPITCH},
  {Joint::LHipRoll, nao_interfaces::msg::Joints::LHIPROLL},
  {Joint::LHipPitch, nao_interfaces::msg::Joints::LHIPPITCH},
  {Joint::LKneePitch, nao_interfaces::msg::Joints::LKNEEPITCH},
  {Joint::LAnklePitch, nao_interfaces::msg::Joints::LANKLEPITCH},
  {Joint::LAnkleRoll, nao_interfaces::msg::Joints::LANKLEROLL},
  {Joint::RHipRoll, nao_interfaces::msg::Joints::RHIPROLL},
  {Joint::RHipPitch, nao_interfaces::msg::Joints::RHIPPITCH},
  {Joint::RKneePitch, nao_interfaces::msg::Joints::RKNEEPITCH},
  {Joint::RAnklePitch, nao_interfaces::msg::Joints::RANKLEPITCH},
  {Joint::RAnkleRoll, nao_interfaces::msg::Joints::RANKLEROLL},
  {Joint::RShoulderPitch, nao_interfaces::msg::Joints::RSHOULDERPITCH},
  {Joint::RShoulderRoll, nao_interfaces::msg::Joints::RSHOULDERROLL},
  {Joint::RElbowYaw, nao_interfaces::msg::Joints::RELBOWYAW},
  {Joint::RElbowRoll, nao_interfaces::msg::Joints::RELBOWROLL},
  {Joint::RWristYaw, nao_interfaces::msg::Joints::RWRISTYAW},
  {Joint::LHand, nao_interfaces::msg::Joints::LHAND},
  {Joint::RHand, nao_interfaces::msg::Joints::RHAND},
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

nao_interfaces::msg::Gyroscope MsgpackParser::getGyroscope()
{
  nao_interfaces::msg::Gyroscope gyr;
  std::vector<float> vec = unpacked.at("Gyroscope").as<std::vector<float>>();
  gyr.x = vec.at(static_cast<int>(Gyroscope::X));
  gyr.y = vec.at(static_cast<int>(Gyroscope::Y));
  gyr.z = vec.at(static_cast<int>(Gyroscope::Z));
  return gyr;
}

nao_interfaces::msg::Joints MsgpackParser::getJoints()
{
  nao_interfaces::msg::Joints jts;

  std::vector<float> positions = unpacked.at("Position").as<std::vector<float>>();
  std::vector<float> stiffnesses = unpacked.at("Stiffness").as<std::vector<float>>();
  std::vector<float> temperatures = unpacked.at("Temperature").as<std::vector<float>>();
  std::vector<float> currents = unpacked.at("Current").as<std::vector<float>>();

  for (int i = 0; i < static_cast<int>(Joint::NUM_JOINTS); ++i) {
    int msg_index = index_lola_to_msg.at(static_cast<Joint>(i));
    jts.angles.at(msg_index) = positions.at(i);
    jts.stiffnesses.at(msg_index) = stiffnesses.at(i);
    jts.temperatures.at(msg_index) = temperatures.at(i);
    jts.currents.at(msg_index) = currents.at(i);
  }
  return jts;
}

nao_interfaces::msg::Sonar MsgpackParser::getSonar()
{
  nao_interfaces::msg::Sonar snr;
  std::vector<float> vec = unpacked.at("Sonar").as<std::vector<float>>();
  snr.left = vec.at(static_cast<int>(Sonar::Left));
  snr.right = vec.at(static_cast<int>(Sonar::Right));
  return snr;
}


nao_interfaces::msg::Touch MsgpackParser::getTouch()
{
  nao_interfaces::msg::Touch tch;
  std::vector<float> vec = unpacked.at("Touch").as<std::vector<float>>();
  tch.head_front = vec.at(static_cast<int>(Touch::Head_Touch_Front));
  tch.head_middle = vec.at(static_cast<int>(Touch::Head_Touch_Middle));
  tch.head_rear = vec.at(static_cast<int>(Touch::Head_Touch_Rear));
  return tch;
}

std::vector<std::string> MsgpackParser::getRobotConfig()
{
  return unpacked.at("RobotConfig").as<std::vector<std::string>>();
}
