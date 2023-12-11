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
#include "msgpack/object.hpp"
#include "msgpack/adaptor/float.hpp"
#include "msgpack/adaptor/map.hpp"
#include "msgpack/adaptor/string.hpp"
#include "msgpack/adaptor/vector.hpp"
#include "msgpack/unpack.hpp"
#include "nao_lola_client/msgpack_parser.hpp"
#include "nao_lola_client/lola_enums.hpp"
#include "nao_lola_client/sensor_index_conversion.hpp"


MsgpackParser::MsgpackParser(char data[], int size)
{
  oh = msgpack::unpack(data, size);

  unpacked = oh.get().as<std::map<std::string, msgpack::object>>();
}

nao_lola_sensor_msgs::msg::Accelerometer MsgpackParser::getAccelerometer()
{
  nao_lola_sensor_msgs::msg::Accelerometer acc;
  std::vector<float> vec = unpacked.at("Accelerometer").as<std::vector<float>>();
  acc.x = vec.at(static_cast<int>(LolaEnums::Accelerometer::X));
  acc.y = vec.at(static_cast<int>(LolaEnums::Accelerometer::Y));
  acc.z = vec.at(static_cast<int>(LolaEnums::Accelerometer::Z));
  return acc;
}

nao_lola_sensor_msgs::msg::Angle MsgpackParser::getAngle()
{
  nao_lola_sensor_msgs::msg::Angle ang;
  std::vector<float> vec = unpacked.at("Angles").as<std::vector<float>>();
  ang.x = vec.at(static_cast<int>(LolaEnums::Angles::X));
  ang.y = vec.at(static_cast<int>(LolaEnums::Angles::Y));
  return ang;
}

nao_lola_sensor_msgs::msg::Buttons MsgpackParser::getButtons()
{
  nao_lola_sensor_msgs::msg::Buttons but;
  std::vector<float> vec = unpacked.at("Touch").as<std::vector<float>>();
  but.chest = vec.at(static_cast<int>(LolaEnums::Touch::ChestBoard_Button));
  but.l_foot_bumper_left = vec.at(static_cast<int>(LolaEnums::Touch::LFoot_Bumper_Left));
  but.l_foot_bumper_right = vec.at(static_cast<int>(LolaEnums::Touch::LFoot_Bumper_Right));
  but.r_foot_bumper_left = vec.at(static_cast<int>(LolaEnums::Touch::RFoot_Bumper_Left));
  but.r_foot_bumper_right = vec.at(static_cast<int>(LolaEnums::Touch::RFoot_Bumper_Right));
  return but;
}

nao_lola_sensor_msgs::msg::FSR MsgpackParser::getFSR()
{
  nao_lola_sensor_msgs::msg::FSR fsr;
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

nao_lola_sensor_msgs::msg::Gyroscope MsgpackParser::getGyroscope()
{
  nao_lola_sensor_msgs::msg::Gyroscope gyr;
  std::vector<float> vec = unpacked.at("Gyroscope").as<std::vector<float>>();
  gyr.x = vec.at(static_cast<int>(LolaEnums::Gyroscope::X));
  gyr.y = vec.at(static_cast<int>(LolaEnums::Gyroscope::Y));
  gyr.z = vec.at(static_cast<int>(LolaEnums::Gyroscope::Z));
  return gyr;
}

nao_lola_sensor_msgs::msg::JointPositions MsgpackParser::getJointPositions()
{
  nao_lola_sensor_msgs::msg::JointPositions jointPositions;

  std::vector<float> positions = unpacked.at("Position").as<std::vector<float>>();
  for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
    int msg_index = IndexConversion::joint_lola_to_msg.at(static_cast<LolaEnums::Joint>(i));
    jointPositions.positions.at(msg_index) = positions.at(i);
  }
  return jointPositions;
}

nao_lola_sensor_msgs::msg::JointStiffnesses MsgpackParser::getJointStiffnesses()
{
  nao_lola_sensor_msgs::msg::JointStiffnesses jointStiffnesses;

  std::vector<float> stiffnesses = unpacked.at("Stiffness").as<std::vector<float>>();
  for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
    int msg_index = IndexConversion::joint_lola_to_msg.at(static_cast<LolaEnums::Joint>(i));
    jointStiffnesses.stiffnesses.at(msg_index) = stiffnesses.at(i);
  }
  return jointStiffnesses;
}

nao_lola_sensor_msgs::msg::JointTemperatures MsgpackParser::getJointTemperatures()
{
  nao_lola_sensor_msgs::msg::JointTemperatures jointTemperatures;

  std::vector<float> temperatures = unpacked.at("Temperature").as<std::vector<float>>();
  for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
    int msg_index = IndexConversion::joint_lola_to_msg.at(static_cast<LolaEnums::Joint>(i));
    jointTemperatures.temperatures.at(msg_index) = temperatures.at(i);
  }
  return jointTemperatures;
}

nao_lola_sensor_msgs::msg::JointCurrents MsgpackParser::getJointCurrents()
{
  nao_lola_sensor_msgs::msg::JointCurrents jointCurrents;

  std::vector<float> currents = unpacked.at("Current").as<std::vector<float>>();
  for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
    int msg_index = IndexConversion::joint_lola_to_msg.at(static_cast<LolaEnums::Joint>(i));
    jointCurrents.currents.at(msg_index) = currents.at(i);
  }
  return jointCurrents;
}

nao_lola_sensor_msgs::msg::JointStatuses MsgpackParser::getJointStatuses()
{
  nao_lola_sensor_msgs::msg::JointStatuses jointStatuses;

  // The LoLA RoboCupper docs say "status" is an integer data type, that's wrong.
  std::vector<float> statuses = unpacked.at("Status").as<std::vector<float>>();
  for (int i = 0; i < static_cast<int>(LolaEnums::Joint::NUM_JOINTS); ++i) {
    int msg_index = IndexConversion::joint_lola_to_msg.at(static_cast<LolaEnums::Joint>(i));
    jointStatuses.statuses.at(msg_index) = statuses.at(i);
  }
  return jointStatuses;
}

nao_lola_sensor_msgs::msg::Sonar MsgpackParser::getSonar()
{
  nao_lola_sensor_msgs::msg::Sonar snr;
  std::vector<float> vec = unpacked.at("Sonar").as<std::vector<float>>();
  snr.left = vec.at(static_cast<int>(LolaEnums::Sonar::Left));
  snr.right = vec.at(static_cast<int>(LolaEnums::Sonar::Right));
  return snr;
}


nao_lola_sensor_msgs::msg::Touch MsgpackParser::getTouch()
{
  nao_lola_sensor_msgs::msg::Touch tch;
  std::vector<float> vec = unpacked.at("Touch").as<std::vector<float>>();
  tch.head_front = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Front));
  tch.head_middle = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Middle));
  tch.head_rear = vec.at(static_cast<int>(LolaEnums::Touch::Head_Touch_Rear));
  return tch;
}

nao_lola_sensor_msgs::msg::Battery MsgpackParser::getBattery()
{
  nao_lola_sensor_msgs::msg::Battery btr;
  std::vector<float> vec = unpacked.at("Battery").as<std::vector<float>>();
  // Convert charge to [0% - 100%]
  btr.charge = vec.at(static_cast<int>(LolaEnums::Battery::Charge)) * 100.0;
  btr.current = vec.at(static_cast<int>(LolaEnums::Battery::Current));
  btr.temperature = vec.at(static_cast<int>(LolaEnums::Battery::Temperature));


  // Check whether robot is charging, with BHuman's equation used as reference:
  // https://github.com/bhuman/BHumanCodeRelease/tree/coderelease2019/Src/Modules/Infrastructure/NaoProvider/NaoProvider.cpp#L320
  float status = vec.at(static_cast<int>(LolaEnums::Battery::Status));
  btr.charging = ((static_cast<int16_t>(status) & 0x80) != 0);

  return btr;
}

nao_lola_sensor_msgs::msg::RobotConfig MsgpackParser::getRobotConfig()
{
  nao_lola_sensor_msgs::msg::RobotConfig cfg;
  std::vector<std::string> vec = unpacked.at("RobotConfig").as<std::vector<std::string>>();
  cfg.body_id = vec.at(static_cast<int>(LolaEnums::RobotConfig::Body_BodyId));
  cfg.body_version = vec.at(static_cast<int>(LolaEnums::RobotConfig::Body_Version));
  cfg.head_id = vec.at(static_cast<int>(LolaEnums::RobotConfig::Head_FullHeadId));
  cfg.head_version = vec.at(static_cast<int>(LolaEnums::RobotConfig::Head_Version));
  return cfg;
}
