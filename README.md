# nao_lola

This ROS2 galactic package is currently under heavy development and not ready for use.
Ideally, this is a service started from systemd, so that any number of processes can listen to the topics.

## Installation

In your ROS2 workspace, clone the repository:
```
git clone --recursive https://github.com/ijnek/nao_lola.git src/nao_lola
vcs import src < src/nao_lola/dependencies.repos
```

and build it
```
colcon build
```

## Running

After sourcing the setup file, run
```
ros2 run nao_lola nao_lola
```

## Publishing Topics

* `sensors/accelerometer` (`nao_interfaces::msg::Accelerometer`)
* `sensors/angle` (`nao_interfaces::msg::Angle`)
* `sensors/buttons` (`nao_interfaces::msg::Buttons`)
* `sensors/fsr` (`nao_interfaces::msg::FSR`)
* `sensors/gyroscope` (`nao_interfaces::msg::Gyroscope`)
* `sensors/joint_positions` (`nao_interfaces::msg::JointPositions`)
* `sensors/joint_stiffnesses` (`nao_interfaces::msg::JointStiffnesses`)
* `sensors/joint_temperatures` (`nao_interfaces::msg::JointTemperatures`)
* `sensors/joint_currents` (`nao_interfaces::msg::JointCurrents`)
* `sensors/joint_statuses` (`nao_interfaces::msg::JointStatuses`)
* `sensors/sonar` (`nao_interfaces::msg::Sonar`)
* `sensors/touch` (`nao_interfaces::msg::Touch`)
* `sensors/battery` (`nao_interfaces::msg::Battery`)
* `sensors/robot_config` (`nao_interfaces::msg::RobotConfig`)

## Subscription Topics

* `effectors/joint_positions` (`nao_interfaces::msg::JointPositions`)
* `effectors/joint_stiffnesses` (`nao_interfaces::msg::JointStiffnesses`)
* `effectors/chest_led` (`nao_interfaces::msg::ChestLed`)
* `effectors/left_ear_leds` (`nao_interfaces::msg::LeftEarLeds`)
* `effectors/right_ear_leds` (`nao_interfaces::msg::RightEarLeds`)
* `effectors/left_eye_leds` (`nao_interfaces::msg::LeftEyeLeds`)
* `effectors/right_eye_leds` (`nao_interfaces::msg::RightEyeLeds`)
* `effectors/left_foot_led` (`nao_interfaces::msg::LeftFootLed`)
* `effectors/right_foot_led` (`nao_interfaces::msg::RightFootLed`)
* `effectors/head_leds` (`nao_interfaces::msg::HeadLeds`)
* `effectors/sonar_usage` (`nao_interfaces::msg::SonarUsage`)
