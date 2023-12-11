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

#include <string>
#include <memory>

#include "nao_lola_client/nao_lola_client.hpp"
#include "nao_lola_client/msgpack_parser.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

#include "conversion.hpp"

NaoLolaClient::NaoLolaClient(const rclcpp::NodeOptions & options)
: Node("NaoLolaClient", options)
{
  declareParameters();
  createPublishers();
  createSubscriptions();

  // Start receive and send loop
  receive_thread_ = std::thread(
    [this]() {
      while (rclcpp::ok()) {
        RecvData recvData;
        try {
          recvData = connection.receive();
        } catch (const std::runtime_error & e) {
          RCLCPP_ERROR_STREAM_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000, e.what());
          continue;
        }

        MsgpackParser parsed(recvData.data(), recvData.size());

        accelerometer_pub->publish(parsed.getAccelerometer());
        angle_pub->publish(parsed.getAngle());
        buttons_pub->publish(parsed.getButtons());
        fsr_pub->publish(parsed.getFSR());
        gyroscope_pub->publish(parsed.getGyroscope());
        joint_positions_pub->publish(parsed.getJointPositions());
        joint_stiffnesses_pub->publish(parsed.getJointStiffnesses());
        joint_temperatures_pub->publish(parsed.getJointTemperatures());
        joint_currents_pub->publish(parsed.getJointCurrents());
        joint_statuses_pub->publish(parsed.getJointStatuses());
        sonar_pub->publish(parsed.getSonar());
        touch_pub->publish(parsed.getTouch());
        battery_pub->publish(parsed.getBattery());
        robot_config_pub->publish(parsed.getRobotConfig());

        auto stamp = now();

        if (publish_imu_) {
          auto imu = conversion::toImu(parsed.getAccelerometer(), parsed.getGyroscope());
          imu.header.stamp = stamp;
          imu_pub->publish(imu);
        }

        if (publish_joint_states_) {
          auto joint_state = conversion::toJointState(parsed.getJointPositions());
          joint_state.header.stamp = stamp;
          joint_states_pub->publish(joint_state);
        }

        // In mutex, copy packer
        // Do the pack and send outside mutex to avoid retain lock for a long time
        MsgpackPacker packerCopy;
        {
          std::lock_guard<std::mutex> guard(packer_mutex);
          packerCopy = packer;
        }
        connection.send(packerCopy.getPacked());
      }
    });
}

void NaoLolaClient::createPublishers()
{
  RCLCPP_DEBUG(get_logger(), "Initialise publishers");
  accelerometer_pub = create_publisher<nao_lola_sensor_msgs::msg::Accelerometer>(
    "sensors/accelerometer", 10);
  angle_pub = create_publisher<nao_lola_sensor_msgs::msg::Angle>("sensors/angle", 10);
  buttons_pub = create_publisher<nao_lola_sensor_msgs::msg::Buttons>("sensors/buttons", 10);
  fsr_pub = create_publisher<nao_lola_sensor_msgs::msg::FSR>("sensors/fsr", 10);
  gyroscope_pub = create_publisher<nao_lola_sensor_msgs::msg::Gyroscope>("sensors/gyroscope", 10);
  joint_positions_pub = create_publisher<nao_lola_sensor_msgs::msg::JointPositions>(
    "sensors/joint_positions", 10);
  joint_stiffnesses_pub = create_publisher<nao_lola_sensor_msgs::msg::JointStiffnesses>(
    "sensors/joint_stiffnesses", 10);
  joint_temperatures_pub = create_publisher<nao_lola_sensor_msgs::msg::JointTemperatures>(
    "sensors/joint_temperatures", 10);
  joint_currents_pub = create_publisher<nao_lola_sensor_msgs::msg::JointCurrents>(
    "sensors/joint_currents", 10);
  joint_statuses_pub = create_publisher<nao_lola_sensor_msgs::msg::JointStatuses>(
    "sensors/joint_statuses", 10);
  sonar_pub = create_publisher<nao_lola_sensor_msgs::msg::Sonar>("sensors/sonar", 10);
  touch_pub = create_publisher<nao_lola_sensor_msgs::msg::Touch>("sensors/touch", 10);
  battery_pub = create_publisher<nao_lola_sensor_msgs::msg::Battery>("sensors/battery", 10);
  robot_config_pub =
    create_publisher<nao_lola_sensor_msgs::msg::RobotConfig>("sensors/robot_config", 10);

  if (publish_joint_states_) {
    joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }

  if (publish_imu_) {
    imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  }

  RCLCPP_DEBUG(get_logger(), "Finished initialising publishers");
}

void NaoLolaClient::createSubscriptions()
{
  RCLCPP_DEBUG(get_logger(), "Initialise subscriptions");
  joint_positions_sub =
    create_subscription<nao_lola_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 1,
    [this](const nao_lola_command_msgs::msg::JointPositions & jointPositions) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setJointPositions(jointPositions);
    }
    );

  joint_stiffnesses_sub =
    create_subscription<nao_lola_command_msgs::msg::JointStiffnesses>(
    "effectors/joint_stiffnesses", 1,
    [this](const nao_lola_command_msgs::msg::JointStiffnesses & jointStiffnesses) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setJointStiffnesses(jointStiffnesses);
    }
    );

  chest_led_sub =
    create_subscription<nao_lola_command_msgs::msg::ChestLed>(
    "effectors/chest_led", 1,
    [this](const nao_lola_command_msgs::msg::ChestLed & chestLed) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setChestLed(chestLed);
    }
    );

  left_ear_leds_sub =
    create_subscription<nao_lola_command_msgs::msg::LeftEarLeds>(
    "effectors/left_ear_leds", 1,
    [this](const nao_lola_command_msgs::msg::LeftEarLeds & leftEarLeds) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setLeftEarLeds(leftEarLeds);
    }
    );

  right_ear_leds_sub =
    create_subscription<nao_lola_command_msgs::msg::RightEarLeds>(
    "effectors/right_ear_leds", 1,
    [this](const nao_lola_command_msgs::msg::RightEarLeds & rightEarLeds) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setRightEarLeds(rightEarLeds);
    }
    );

  left_eye_leds_sub =
    create_subscription<nao_lola_command_msgs::msg::LeftEyeLeds>(
    "effectors/left_eye_leds", 1,
    [this](const nao_lola_command_msgs::msg::LeftEyeLeds & leftEyeLeds) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setLeftEyeLeds(leftEyeLeds);
    }
    );

  right_eye_leds_sub =
    create_subscription<nao_lola_command_msgs::msg::RightEyeLeds>(
    "effectors/right_eye_leds", 1,
    [this](const nao_lola_command_msgs::msg::RightEyeLeds & rightEyeLeds) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setRightEyeLeds(rightEyeLeds);
    }
    );

  left_foot_led_sub =
    create_subscription<nao_lola_command_msgs::msg::LeftFootLed>(
    "effectors/left_foot_led", 1,
    [this](const nao_lola_command_msgs::msg::LeftFootLed & leftFootLed) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setLeftFootLed(leftFootLed);
    }
    );

  right_foot_led_sub =
    create_subscription<nao_lola_command_msgs::msg::RightFootLed>(
    "effectors/right_foot_led", 1,
    [this](const nao_lola_command_msgs::msg::RightFootLed & rightFootLed) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setRightFootLed(rightFootLed);
    }
    );

  head_leds_sub =
    create_subscription<nao_lola_command_msgs::msg::HeadLeds>(
    "effectors/head_leds", 1,
    [this](const nao_lola_command_msgs::msg::HeadLeds & headLeds) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setHeadLeds(headLeds);
    }
    );

  sonar_usage_sub =
    create_subscription<nao_lola_command_msgs::msg::SonarUsage>(
    "effectors/sonar_usage", 1,
    [this](const nao_lola_command_msgs::msg::SonarUsage & sonarUsage) {
      std::lock_guard<std::mutex> guard(packer_mutex);
      packer.setSonarUsage(sonarUsage);
    }
    );
  RCLCPP_DEBUG(get_logger(), "Finished creating subscriptions");
}

void NaoLolaClient::declareParameters()
{
  publish_joint_states_ = declare_parameter(
    "publish_joint_states", rclcpp::ParameterValue(true),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__name("publish_joint_states")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    .set__description(
      "Whether to convert nao_lola sensor_msgs/JointPositions to sensor_msgs/JointState and "
      "publish it on topic 'joint_states'")
    .set__read_only(true)
  ).get<bool>();

  publish_imu_ = declare_parameter(
    "publish_imu", rclcpp::ParameterValue(true),
    rcl_interfaces::msg::ParameterDescriptor()
    .set__name("publish_imu")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    .set__description(
      "Whether to convert nao_lola sensor_msgs/JointPositions and nao_lola_sensor_msgs/Gyroscope to"
      " sensor_msgs/Imu and publish it on topic 'imu'")
    .set__read_only(true)
  ).get<bool>();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NaoLolaClient)
