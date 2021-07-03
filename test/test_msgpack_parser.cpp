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
#include "nao_lola/msgpack_parser.hpp"



std::vector<float> status = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
std::vector<float> stiffness = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> accelerometer = {-3.0656251907348633, -0.39278322458267212, -9.3214168548583984};
std::vector<float> battery = {1.0, 0.0, 0.0, 0.0};
std::vector<float> current = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> touch = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<float> fsr = {0.014380865730345249, 0.29265055060386658, 0.47892898321151733, 0.62120223045349121,
      0.28502300381660461, 0.70163685083389282, 0.40598109364509583, 0.086648054420948029};
std::vector<float> angles = {0.037582572549581528, -0.35991066694259644};
std::vector<float> position = {0.59361600875854492, 0.49544000625610352, 1.2133520841598511, 0.33283615112304688,
      0.76849198341369629, -0.16869807243347168, -0.39427995681762695, -0.50617814064025879,
      0.297637939453125, -0.34050607681274414, 2.1506261825561523, -1.1137261390686035,
      -0.062852144241333008, -0.10733795166015625, -0.23014187812805176, 2.1399722099304199,
      -1.2056820392608643, -0.10120201110839844, 1.0600361824035645, 0.2039799690246582,
      -0.46791195869445801, 0.066004037857055664, 0.47089600563049316, 0.010400056838989258,
      0.011199951171875};
std::vector<float> sonar = {0.0, 1.0};
std::vector<float> gyroscope = {-0.00026631611399352551, -0.001065264455974102, 0.001065264455974102};
std::vector<float> temperature = {38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 33.0, 27.0, 27.0, 27.0, 27.0,
      27.0, 27.0, 27.0, 27.0, 27.0, 27.0, 38.0, 38.0, 38.0, 38.0, 38.0, 38.0, 38.0};
std::vector<std::string> robotConfig = {"P0000073A07S94700012", "6.0.0", "P0000074A05S93M00061", "6.0.0"};

class TestMsgpackParser : public ::testing::Test
{
public:
  std::string packed;

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

    std::stringstream buffer;
    msgpack::pack(buffer, map);

    // send the buffer ...
    buffer.seekg(0);

    // deserialize the buffer into msgpack::object instance.
    packed = buffer.str();
  }
};

TEST_F(TestMsgpackParser, TestAccelerometer)
{
  MsgpackParser parser(packed);
  nao_interfaces::msg::Accelerometer acc = parser.getAccelerometer();
  EXPECT_NEAR(acc.x, -3.0656251907348633, 0.01);
  EXPECT_NEAR(acc.y, -0.39278322458267212, 0.01);
  EXPECT_NEAR(acc.z, -9.3214168548583984, 0.01);
}

TEST_F(TestMsgpackParser, TestRobotConfig)
{
  MsgpackParser parser(packed);
  std::vector<std::string> robotConfig = parser.getRobotConfig();
  EXPECT_EQ(robotConfig.at(0), "P0000073A07S94700012");
  EXPECT_EQ(robotConfig.at(1), "6.0.0");
  EXPECT_EQ(robotConfig.at(2), "P0000074A05S93M00061");
  EXPECT_EQ(robotConfig.at(3), "6.0.0");
}
