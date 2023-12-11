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

#include <gtest/gtest.h>
#include <vector>
#include <map>
#include <string>
#include <memory>
#include "msgpack/object.hpp"
#include "msgpack/adaptor/float.hpp"
#include "msgpack/adaptor/int.hpp"
#include "msgpack/adaptor/vector.hpp"
#include "msgpack/adaptor/map.hpp"
#include "msgpack/adaptor/string.hpp"
#include "msgpack/pack.hpp"
#include "msgpack/zone.hpp"
#include "nao_lola_client/msgpack_parser.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"

std::vector<int> status =
{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3};
std::vector<float> stiffness =
{0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8};
std::vector<float> accelerometer = {-3.0656251907348633, -0.39278322458267212, -9.3214168548583984};
std::vector<float> battery = {0.9, 0.5, 0.0, 37.0};
std::vector<float> current =
{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2};
std::vector<float> touch = {1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0};
std::vector<float> fsr =
{0.014380865730345249, 0.29265055060386658, 0.47892898321151733, 0.62120223045349121,
  0.28502300381660461, 0.70163685083389282, 0.40598109364509583, 0.086648054420948029};
std::vector<float> angles = {0.037582572549581528, -0.35991066694259644};
std::vector<float> position =
{0.59361600875854492, 0.49544000625610352, 1.2133520841598511, 0.33283615112304688,
  0.76849198341369629, -0.16869807243347168, -0.39427995681762695, -0.50617814064025879,
  0.297637939453125, -0.34050607681274414, 2.1506261825561523, -1.1137261390686035,
  -0.062852144241333008, -0.10733795166015625, -0.23014187812805176, 2.1399722099304199,
  -1.2056820392608643, -0.10120201110839844, 1.0600361824035645, 0.2039799690246582,
  -0.46791195869445801, 0.066004037857055664, 0.47089600563049316, 0.010400056838989258,
  0.011199951171875};
std::vector<float> sonar = {0.3, 1.3};
std::vector<float> gyroscope =
{-0.00026631611399352551, -0.001065264455974102, 0.001065264455974102};
std::vector<float> temperature =
{38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 33.0, 27.0, 27.0, 27.0, 27.0,
  27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 39.0};
std::vector<std::string> robotConfig =
{"P0000073A07S94700012", "6.0.0", "P0000074A05S93M00061", "6.0.0"};

class TestMsgpackParser : public ::testing::Test
{
public:
  std::shared_ptr<MsgpackParser> parser;

protected:
  virtual void SetUp()
  {
    // A way of packing a hashmap containing floats and strings and other
    // data types are explained in this comment:
    // https://github.com/msgpack/msgpack-c/issues/651#issuecomment-365197261
    msgpack::zone z;

    std::map<std::string, msgpack::object> map{
      {"Status", msgpack::object(status, z)},
      {"Stiffness", msgpack::object(stiffness, z)},
      {"Accelerometer", msgpack::object(accelerometer, z)},
      {"Battery", msgpack::object(battery, z)},
      {"Current", msgpack::object(current, z)},
      {"Touch", msgpack::object(touch, z)},
      {"FSR", msgpack::object(fsr, z)},
      {"Angles", msgpack::object(angles, z)},
      {"Position", msgpack::object(position, z)},
      {"Sonar", msgpack::object(sonar, z)},
      {"Gyroscope", msgpack::object(gyroscope, z)},
      {"Temperature", msgpack::object(temperature, z)},
      {"RobotConfig", msgpack::object(robotConfig, z)}
    };

    // serialize the buffer
    std::stringstream buffer;
    msgpack::pack(buffer, map);

    // deserialize the buffer
    // DO NOT use a std::string because serialized data may contain null characters
    char c;
    std::vector<char> packed;
    packed.reserve(1000);
    while (buffer.get(c)) {
      packed.push_back(c);
    }
    parser = std::make_shared<MsgpackParser>(packed.data(), packed.size());
  }
};

TEST_F(TestMsgpackParser, TestAccelerometer)
{
  nao_lola_sensor_msgs::msg::Accelerometer acc = parser->getAccelerometer();
  EXPECT_NEAR(acc.x, -3.0656251907348633, 0.000001);
  EXPECT_NEAR(acc.y, -0.39278322458267212, 0.000001);
  EXPECT_NEAR(acc.z, -9.3214168548583984, 0.000001);
}

TEST_F(TestMsgpackParser, TestAngle)
{
  nao_lola_sensor_msgs::msg::Angle ang = parser->getAngle();
  EXPECT_NEAR(ang.x, 0.037582572549581528, 0.000001);
  EXPECT_NEAR(ang.y, -0.35991066694259644, 0.000001);
}

TEST_F(TestMsgpackParser, TestButtons)
{
  nao_lola_sensor_msgs::msg::Buttons but = parser->getButtons();
  EXPECT_TRUE(but.chest);
  EXPECT_TRUE(but.l_foot_bumper_left);
  EXPECT_FALSE(but.l_foot_bumper_right);
  EXPECT_FALSE(but.r_foot_bumper_left);
  EXPECT_TRUE(but.r_foot_bumper_right);
}

TEST_F(TestMsgpackParser, TestFSR)
{
  nao_lola_sensor_msgs::msg::FSR fsr = parser->getFSR();
  EXPECT_NEAR(fsr.l_foot_front_left, 0.014380865730345249, 0.000001);
  EXPECT_NEAR(fsr.l_foot_front_right, 0.29265055060386658, 0.000001);
  EXPECT_NEAR(fsr.l_foot_back_left, 0.47892898321151733, 0.000001);
  EXPECT_NEAR(fsr.l_foot_back_right, 0.62120223045349121, 0.000001);
  EXPECT_NEAR(fsr.r_foot_front_left, 0.28502300381660461, 0.000001);
  EXPECT_NEAR(fsr.r_foot_front_right, 0.70163685083389282, 0.000001);
  EXPECT_NEAR(fsr.r_foot_back_left, 0.40598109364509583, 0.000001);
  EXPECT_NEAR(fsr.r_foot_back_right, 0.086648054420948029, 0.000001);
}

TEST_F(TestMsgpackParser, TestGyroscope)
{
  nao_lola_sensor_msgs::msg::Gyroscope gyr = parser->getGyroscope();
  EXPECT_NEAR(gyr.x, -0.00026631611399352551, 0.000001);
  EXPECT_NEAR(gyr.y, -0.001065264455974102, 0.000001);
  EXPECT_NEAR(gyr.z, 0.001065264455974102, 0.000001);
}

TEST_F(TestMsgpackParser, TestJointPositions)
{
  nao_lola_sensor_msgs::msg::JointPositions jointPositions = parser->getJointPositions();
  EXPECT_NEAR(
    jointPositions.positions.at(
      nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW), 0.59361600875854492, 0.000001);
  EXPECT_NEAR(
    jointPositions.positions.at(
      nao_lola_sensor_msgs::msg::JointIndexes::RHAND), 0.011199951171875, 0.00001);
}

TEST_F(TestMsgpackParser, TestJointStiffnesses)
{
  nao_lola_sensor_msgs::msg::JointStiffnesses jointStiffnesses = parser->getJointStiffnesses();
  EXPECT_NEAR(
    jointStiffnesses.stiffnesses.at(
      nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW), 0.3, 0.000001);
  EXPECT_NEAR(
    jointStiffnesses.stiffnesses.at(
      nao_lola_sensor_msgs::msg::JointIndexes::RHAND), 0.8, 0.00001);
}

TEST_F(TestMsgpackParser, TestJointTemperatures)
{
  nao_lola_sensor_msgs::msg::JointTemperatures jointTemperatures = parser->getJointTemperatures();
  EXPECT_NEAR(
    jointTemperatures.temperatures.at(
      nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW), 38.0, 0.000001);
  EXPECT_NEAR(
    jointTemperatures.temperatures.at(
      nao_lola_sensor_msgs::msg::JointIndexes::RHAND), 39.0, 0.00001);
}

TEST_F(TestMsgpackParser, TestJointCurrents)
{
  nao_lola_sensor_msgs::msg::JointCurrents jointCurrents = parser->getJointCurrents();
  EXPECT_NEAR(
    jointCurrents.currents.at(nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW), 0.1,
    0.000001);
  EXPECT_NEAR(
    jointCurrents.currents.at(
      nao_lola_sensor_msgs::msg::JointIndexes::RHAND), 0.2, 0.00001);
}

TEST_F(TestMsgpackParser, TestJointStatuses)
{
  nao_lola_sensor_msgs::msg::JointStatuses jointStatuses = parser->getJointStatuses();
  EXPECT_EQ(jointStatuses.statuses.at(nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW), 1);
  EXPECT_EQ(jointStatuses.statuses.at(nao_lola_sensor_msgs::msg::JointIndexes::RHAND), 3);
}

TEST_F(TestMsgpackParser, TestSonar)
{
  nao_lola_sensor_msgs::msg::Sonar snr = parser->getSonar();
  EXPECT_NEAR(snr.left, 0.3, 0.000001);
  EXPECT_NEAR(snr.right, 1.3, 0.000001);
}

TEST_F(TestMsgpackParser, TestTouch)
{
  nao_lola_sensor_msgs::msg::Touch tch = parser->getTouch();
  EXPECT_FALSE(tch.head_front);
  EXPECT_TRUE(tch.head_middle);
  EXPECT_FALSE(tch.head_rear);
}

TEST_F(TestMsgpackParser, TestBattery)
{
  nao_lola_sensor_msgs::msg::Battery btr = parser->getBattery();
  // From Lola, charge has range 0.0 - 1.0, but in Battery msg, we have 0.0% - 100.0%.
  // So, we multiply by 100 below.
  EXPECT_NEAR(btr.charge, 0.9 * 100, 0.000001);
  EXPECT_NEAR(btr.current, 0.5, 0.000001);
  EXPECT_FALSE(btr.charging);
  EXPECT_NEAR(btr.temperature, 37.0, 0.000001);
}


TEST_F(TestMsgpackParser, TestRobotConfig)
{
  nao_lola_sensor_msgs::msg::RobotConfig robotConfig = parser->getRobotConfig();
  EXPECT_EQ(robotConfig.body_id, "P0000073A07S94700012");
  EXPECT_EQ(robotConfig.body_version, "6.0.0");
  EXPECT_EQ(robotConfig.head_id, "P0000074A05S93M00061");
  EXPECT_EQ(robotConfig.head_version, "6.0.0");
}
