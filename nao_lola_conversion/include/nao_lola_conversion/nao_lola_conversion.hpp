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

#ifndef NAO_LOLA_CONVERSION__NAO_LOLA_CONVERSION_HPP_
#define NAO_LOLA_CONVERSION__NAO_LOLA_CONVERSION_HPP_

#include <memory>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "nao_lola_sensor_msgs/msg/accelerometer.hpp"
#include "nao_lola_sensor_msgs/msg/gyroscope.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace nao_lola_conversion
{

class NaoLolaConversion : public rclcpp::Node
{
public:
  explicit NaoLolaConversion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  // Subscriptions
  message_filters::Subscriber<nao_lola_sensor_msgs::msg::Accelerometer> accelerometer_sub_;
  message_filters::Subscriber<nao_lola_sensor_msgs::msg::Gyroscope> gyroscope_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Synchronizer
  std::shared_ptr<message_filters::TimeSynchronizer<nao_lola_sensor_msgs::msg::Accelerometer,
    nao_lola_sensor_msgs::msg::Gyroscope>> synchronizer_;

  // Callbacks
  void synchronizerCallback(
    const nao_lola_sensor_msgs::msg::Accelerometer::SharedPtr & accelerometer,
    const nao_lola_sensor_msgs::msg::Gyroscope::SharedPtr & gyroscope);
};

}  // namespace nao_lola_conversion

#endif  // NAO_LOLA_CONVERSION__NAO_LOLA_CONVERSION_HPP_
