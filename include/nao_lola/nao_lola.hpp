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

#ifndef NAO_LOLA__NAO_LOLA_HPP_
#define NAO_LOLA__NAO_LOLA_HPP_

#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "nao_interfaces/msg/joints.hpp"
#include "nao_interfaces/msg/buttons.hpp"
#include "nao_interfaces/msg/accelerometer.hpp"
#include "nao_interfaces/msg/gyroscope.hpp"
#include "nao_interfaces/msg/angle.hpp"
#include "nao_interfaces/msg/sonar.hpp"
#include "nao_interfaces/msg/fsr.hpp"
#include "nao_interfaces/msg/touch.hpp"
#include "nao_interfaces/msg/eye_leds.hpp"
#include "nao_interfaces/msg/battery.hpp"
#include "nao_interfaces/msg/robot_config.hpp"
#include "nao_lola/connection.hpp"

class NaoLola : public rclcpp::Node
{
public:
  NaoLola();
  virtual ~NaoLola() {}

private:
  rclcpp::Publisher<nao_interfaces::msg::Accelerometer>::SharedPtr accelerometer_pub;
  rclcpp::Publisher<nao_interfaces::msg::Angle>::SharedPtr angle_pub;
  rclcpp::Publisher<nao_interfaces::msg::Buttons>::SharedPtr buttons_pub;
  rclcpp::Publisher<nao_interfaces::msg::FSR>::SharedPtr fsr_pub;
  rclcpp::Publisher<nao_interfaces::msg::Gyroscope>::SharedPtr gyroscope_pub;
  rclcpp::Publisher<nao_interfaces::msg::Joints>::SharedPtr joints_pub;
  rclcpp::Publisher<nao_interfaces::msg::Sonar>::SharedPtr sonar_pub;
  rclcpp::Publisher<nao_interfaces::msg::Touch>::SharedPtr touch_pub;
  rclcpp::Publisher<nao_interfaces::msg::Battery>::SharedPtr battery_pub;
  rclcpp::Publisher<nao_interfaces::msg::RobotConfig>::SharedPtr robot_config_pub;

  std::thread receive_thread_;
  Connection connection;
};

#endif  // NAO_LOLA__NAO_LOLA_HPP_
