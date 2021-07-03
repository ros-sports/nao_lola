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

#ifndef NAO_LOLA__LOLA_ENUMS_HPP_
#define NAO_LOLA__LOLA_ENUMS_HPP_

enum class Joint {
  HeadYaw,
  HeadPitch,
  LShoulderPitch,
  LShoulderRoll,
  LElbowYaw,
  LElbowRoll,
  LWristYaw,
  LHipYawPitch,
  LHipRoll,
  LHipPitch,
  LKneePitch,
  LAnklePitch,
  LAnkleRoll,
  RHipRoll,
  RHipPitch,
  RKneePitch,
  RAnklePitch,
  RAnkleRoll,
  RShoulderPitch,
  RShoulderRoll,
  RElbowYaw,
  RElbowRoll,
  RWristYaw,
  LHand,
  RHand,
  NUM_JOINTS};

enum class Battery {Charge, Current, Status, Temperature};

enum class Accelerometer {X, Y, Z};

enum class Gyroscope {X, Y, Z};

enum class Angles {X, Y};

enum class Sonar {Left, Right};

enum class FSR {
  LFoot_FrontLeft,
  LFoot_FrontRight,
  LFoot_RearLeft,
  LFoot_RearRight,
  RFoot_FrontLeft,
  RFoot_FrontRight,
  RFoot_RearLeft,
  RFoot_RearRight};

enum class Touch {
  ChestBoard_Button,
  Head_Touch_Front,
  Head_Touch_Middle,
  Head_Touch_Rear,
  LFoot_Bumper_Left,
  LFoot_Bumper_Right,
  LHand_Touch_Back,
  LHand_Touch_Left,
  LHand_Touch_Right,
  RFoot_Bumper_Left,
  RFoot_Bumper_Right,
  RHand_Touch_Back,
  RHand_Touch_Left,
  RHand_Touch_Right};

enum class RobotConfig {
  Body_BodyId,
  Body_Version,
  Head_FullHeadId,
  Head_Version};

#endif  // NAO_LOLA__LOLA_ENUMS_HPP_