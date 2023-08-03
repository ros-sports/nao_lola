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

#include "nao_lola_conversion/nao_lola_conversion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nao_lola_conversion
{

using nao_lola_sensor_msgs::msg::Accelerometer;
using nao_lola_sensor_msgs::msg::Gyroscope;
using sensor_msgs::msg::Imu;

NaoLolaConversion::NaoLolaConversion(const rclcpp::NodeOptions & options)
: rclcpp::Node{"nao_lola_conversion", options}
{
  // Message filter subscribers
  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  accelerometer_sub_.subscribe(this, "sensors/accelerometer", rmw_qos_profile);
  gyroscope_sub_.subscribe(this, "sensors/gyroscope", rmw_qos_profile);

  // Publishers
  imu_pub_ = create_publisher<Imu>("sensors/imu", 10);

  // Synchronizer
  synchronizer_ =
    std::make_shared<message_filters::TimeSynchronizer<Accelerometer, Gyroscope>>(
    accelerometer_sub_, gyroscope_sub_, 10);
  synchronizer_->registerCallback(&NaoLolaConversion::synchronizerCallback, this);
}

void NaoLolaConversion::synchronizerCallback(
  const Accelerometer::SharedPtr & accelerometer,
  const Gyroscope::SharedPtr & gyroscope)
{
  Imu imu;
  imu.header.frame_id = "ImuTorsoAccelerometer_frame";

  // Linear Acceleration
  imu.linear_acceleration.x = accelerometer->x;
  imu.linear_acceleration.y = accelerometer->y;
  imu.linear_acceleration.z = accelerometer->z;

  // Angular Velocity
  imu.angular_velocity.x = gyroscope->x;
  imu.angular_velocity.y = gyroscope->y;
  imu.angular_velocity.z = gyroscope->z;

  imu_pub_->publish(imu);
}

}  // namespace nao_lola_conversion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nao_lola_conversion::NaoLolaConversion)
