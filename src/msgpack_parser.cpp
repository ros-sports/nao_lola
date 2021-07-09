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
#include "nao_lola/joint_index_conversion.hpp"


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
  acc.x = vec.at(static_cast<int>(LolaEnums::Accelerometer::X));
  acc.y = vec.at(static_cast<int>(LolaEnums::Accelerometer::Y));
  acc.z = vec.at(static_cast<int>(LolaEnums::Accelerometer::Z));
  return acc;
}

nao_interfaces::msg::Angle MsgpackParser::getAngle()
{
  nao_interfaces::msg::Angle ang;
  std::vector<float> vec = unpacked.at("Angles").as<std::vector<float>>();
  ang.x = vec.at(static_cast<int>(LolaEnums::Angles::X));
  ang.y = vec.at(static_cast<int>(LolaEnums::Angles::Y));
  return ang;
}

nao_interfaces::msg::Buttons MsgpackParser::getButtons()
{
  nao_interfaces::msg::Buttons but;
  std::vector<float> vec = unpacked.at("Touch").as<std::vector<float>>();
  but.chest = vec.at(static_cast<int>(LolaEnums::Touch::ChestBoard_Button));
  but.l_foot_bumper_left = vec.at(static_cast<int>(LolaEnums::Touch::LFoot_Bumper_Left));
  but.l_foot_bumper_right = vec.at(static_cast<int>(LolaEnums::Touch::LFoot_Bumper_Right));
  but.r_foot_bumper_left = vec.at(static_cast<int>(LolaEnums::Touch::RFoot_Bumper_Left));
  but.r_foot_bumper_right = vec.at(static_cast<int>(LolaEnums::Touch::RFoot_Bumper_Right));
  return but;
}

nao_interfaces::msg::FSR MsgpackParser::getFSR()
{
  nao_interfaces::msg::FSR fsr;
  std::vector<float> vec = unpacked.at("FSR").as<std::vector<float>>();
  fsr.l_foot_front_left = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_FrontLeft));
  fsr.l_foot_front_right = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_FrontRight));
  fsr.l_foot_back_left = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_RearLeft));
  fsr.l_foot_back_right = vec.at(static_cast<int>(LolaEnums::FSR::LFoot_RearRight));
  fsr.r_foot_front_left = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_FrontLeft));
  fsr.r_foot_front_right = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_FrontRight));
  fsr.r_foot_back_left = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_RearLeft));
  fsr.r_foot_back_right = vec.at(static_cast<int>(LolaEnums::FSR::RFoot_RearRight));
  return fsr;
}

nao_interfaces::msg::Gyroscope MsgpackParser::getGyroscope()
{
  nao_interfaces::msg::Gyroscope gyr;
  std::vector<float> vec = unpacked.at("Gyroscope").as<std::vector<float>>();
  gyr.x = vec.at(static_cast<int>(LolaEnums::Gyroscope::X));
  gyr.y = vec.at(static_cast<int>(LolaEnums::Gyroscope::Y));
  gyr.z = vec.at(static_cast<int>(LolaEnums::Gyroscope::Z));
  return gyr;
}

nao_interfaces::msg::Joints MsgpackParser::getJoints()
{
  nao_interfaces::msg::Joints jts;

  std::vector<float> positions = unpacked.at("Position").as<std::vector<float>>();
  std::vector<float> stiffnesses = unpacked.at("Stiffness").as<std::vector<float>>();
  std::vector<float> temperatures = unpacked.at("Temperature").as<std::vector<float>>();
  std::vector<float> currents = unpacked.at("Current").as<std::vector<float>>();

  // The LoLA RoboCupper docs say "status" is an integer data type, that's wrong.
  std::vector<float> statuses = unpacked.at("Status").as<std::vector<float>>();

  for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
    int msg_index = JointIndexConversion::index_lola_to_msg.at(static_cast<LolaEnums::Joint>(i));
    jts.angles.at(msg_index) = positions.at(i);
    jts.stiffnesses.at(msg_index) = stiffnesses.at(i);
    jts.temperatures.at(msg_index) = temperatures.at(i);
    jts.currents.at(msg_index) = currents.at(i);
    jts.statuses.at(msg_index) = statuses.at(i);
  }
  return jts;
}

nao_interfaces::msg::Sonar MsgpackParser::getSonar()
{
  nao_interfaces::msg::Sonar snr;
  std::vector<float> vec = unpacked.at("Sonar").as<std::vector<float>>();
  snr.left = vec.at(static_cast<int>(LolaEnums::Sonar::Left));
  snr.right = vec.at(static_cast<int>(LolaEnums::Sonar::Right));
  return snr;
}


nao_interfaces::msg::Touch MsgpackParser::getTouch()
{
  nao_interfaces::msg::Touch tch;
  std::vector<float> vec = unpacked.at("Touch").as<std::vector<float>>();
  tch.head_front = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Front));
  tch.head_middle = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Middle));
  tch.head_rear = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Rear));
  return tch;
}

nao_interfaces::msg::Battery MsgpackParser::getBattery()
{
  nao_interfaces::msg::Battery btr;
  std::vector<float> vec = unpacked.at("Battery").as<std::vector<float>>();
  btr.charge = vec.at(static_cast<int>(LolaEnums::Battery::Charge)) * 100.0;  // Convert to [0% - 100%]
  btr.current = vec.at(static_cast<int>(LolaEnums::Battery::Current));
  btr.temperature = vec.at(static_cast<int>(LolaEnums::Battery::Temperature));


  // Check whether robot is charging, with BHuman's equation used as reference:
  // https://github.com/bhuman/BHumanCodeRelease/blob/d7deadc6f1a4c445c4bbd2e9f256bf058b80a24c/Src/Modules/Infrastructure/NaoProvider/NaoProvider.cpp#L320
  float status = vec.at(static_cast<int>(LolaEnums::Battery::Status));
  btr.charging = ((static_cast<int16_t>(status) & 0x80) != 0);

  return btr;
}

nao_interfaces::msg::RobotConfig MsgpackParser::getRobotConfig()
{
  nao_interfaces::msg::RobotConfig cfg;
  std::vector<std::string> vec = unpacked.at("RobotConfig").as<std::vector<std::string>>();
  cfg.body_id = vec.at(static_cast<int>(LolaEnums::RobotConfig::Body_BodyId));
  cfg.body_version = vec.at(static_cast<int>(LolaEnums::RobotConfig::Body_Version));
  cfg.head_id = vec.at(static_cast<int>(LolaEnums::RobotConfig::Head_FullHeadId));
  cfg.head_version = vec.at(static_cast<int>(LolaEnums::RobotConfig::Head_Version));
  return cfg;
}
