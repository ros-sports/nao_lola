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
#include "nao_lola/nao_lola.hpp"
#include "nao_lola/msgpack_parser.hpp"

NaoLola::NaoLola()
: Node("NaoLola")
{
  RCLCPP_DEBUG(get_logger(), "Initialise publishers");
  createPublishers();
  createSubscriptions();

  // Start receive and send loop
  receive_thread_ = std::thread(
    [this]() {
      while (rclcpp::ok()) {
        std::string received = connection.receive();
        MsgpackParser parsed(received);

        accelerometer_pub->publish(parsed.getAccelerometer());
        angle_pub->publish(parsed.getAngle());
        buttons_pub->publish(parsed.getButtons());
        fsr_pub->publish(parsed.getFSR());
        gyroscope_pub->publish(parsed.getGyroscope());
        joints_pub->publish(parsed.getJoints());
        sonar_pub->publish(parsed.getSonar());
        touch_pub->publish(parsed.getTouch());
        battery_pub->publish(parsed.getBattery());
        robot_config_pub->publish(parsed.getRobotConfig());

        connection.send(packer->getPacked());

        // Reset packer
        packer = std::make_shared<MsgpackPacker>();
      }
    });
}

void NaoLola::createPublishers()
{
  accelerometer_pub = create_publisher<nao_interfaces::msg::Accelerometer>(
    "sensors/accelerometer", 10);
  angle_pub = create_publisher<nao_interfaces::msg::Angle>("sensors/angle", 10);
  buttons_pub = create_publisher<nao_interfaces::msg::Buttons>("sensors/buttons", 10);
  fsr_pub = create_publisher<nao_interfaces::msg::FSR>("sensors/fsr", 10);
  gyroscope_pub = create_publisher<nao_interfaces::msg::Gyroscope>("sensors/gyroscope", 10);
  joints_pub = create_publisher<nao_interfaces::msg::Joints>("sensors/joints", 10);
  sonar_pub = create_publisher<nao_interfaces::msg::Sonar>("sensors/sonar", 10);
  touch_pub = create_publisher<nao_interfaces::msg::Touch>("sensors/touch", 10);
  battery_pub = create_publisher<nao_interfaces::msg::Battery>("sensors/battery", 10);
  robot_config_pub = create_publisher<nao_interfaces::msg::RobotConfig>("sensors/robot_config", 10);
}

void NaoLola::createSubscriptions()
{
  joints_sub =
    create_subscription<nao_interfaces::msg::Joints>(
    "effectors/joints", 1,
    [this](nao_interfaces::msg::Joints::SharedPtr joints) {
      packer->setJoints(joints);
    }
    );

  chest_led_sub =
    create_subscription<nao_interfaces::msg::ChestLed>(
    "effectors/chest_led", 1,
    [this](nao_interfaces::msg::ChestLed::SharedPtr chestLed) {
      packer->setChestLed(chestLed);
    }
    );

  left_ear_leds_sub =
    create_subscription<nao_interfaces::msg::LeftEarLeds>(
    "effectors/left_ear_leds", 1,
    [this](nao_interfaces::msg::LeftEarLeds::SharedPtr leftEarLeds) {
      packer->setLeftEarLeds(leftEarLeds);
    }
    );

  right_ear_leds_sub =
    create_subscription<nao_interfaces::msg::RightEarLeds>(
    "effectors/right_ear_leds", 1,
    [this](nao_interfaces::msg::RightEarLeds::SharedPtr rightEarLeds) {
      packer->setRightEarLeds(rightEarLeds);
    }
    );

  left_eye_leds_sub =
    create_subscription<nao_interfaces::msg::LeftEyeLeds>(
    "effectors/left_eye_leds", 1,
    [this](nao_interfaces::msg::LeftEyeLeds::SharedPtr leftEyeLeds) {
      packer->setLeftEyeLeds(leftEyeLeds);
    }
    );

  right_eye_leds_sub =
    create_subscription<nao_interfaces::msg::RightEyeLeds>(
    "effectors/right_eye_leds", 1,
    [this](nao_interfaces::msg::RightEyeLeds::SharedPtr rightEyeLeds) {
      packer->setRightEyeLeds(rightEyeLeds);
    }
    );

  left_foot_led_sub =
    create_subscription<nao_interfaces::msg::LeftFootLed>(
    "effectors/left_foot_led", 1,
    [this](nao_interfaces::msg::LeftFootLed::SharedPtr leftFootLed) {
      packer->setLeftFootLed(leftFootLed);
    }
    );

  right_foot_led_sub =
    create_subscription<nao_interfaces::msg::RightFootLed>(
    "effectors/right_foot_led", 1,
    [this](nao_interfaces::msg::RightFootLed::SharedPtr rightFootLed) {
      packer->setRightFootLed(rightFootLed);
    }
    );

  head_leds_sub =
    create_subscription<nao_interfaces::msg::HeadLeds>(
    "effectors/head_leds", 1,
    [this](nao_interfaces::msg::HeadLeds::SharedPtr headLeds) {
      packer->setHeadLeds(headLeds);
    }
    );

  sonar_usage_sub =
    create_subscription<nao_interfaces::msg::SonarUsage>(
    "effectors/sonar_usage", 1,
    [this](nao_interfaces::msg::SonarUsage::SharedPtr sonarUsage) {
      packer->setSonarUsage(sonarUsage);
    }
    );
}
