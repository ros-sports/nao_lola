# nao_lola

This ROS2 galactic package is currently under heavy development and not ready for use.
Ideally, this is a service started from systemd, so that any number of processes can listen to the topics.

## Installation

In your ROS2 workspace, clone the repository:
```
git clone --recursive https://github.com/ijnek/naosoccer_sim.git src/naosoccer_sim
vcs import src < src/naosoccer_sim/dependencies.repos
```

and build it
```
colcon build
```


## Publishing Topics

* `sensors/accelerometer` (`nao_interfaces::msg::Accelerometer`)
* `sensors/angle` (`nao_interfaces::msg::Angle`)
* `sensors/buttons` (`nao_interfaces::msg::Buttons`)
* `sensors/fsr` (`nao_interfaces::msg::FSR`)
* `sensors/gyroscope` (`nao_interfaces::msg::Gyroscope`)
* `sensors/joints` (`nao_interfaces::msg::Joints`)
* `sensors/sonar` (`nao_interfaces::msg::Sonar`)
* `sensors/touch` (`nao_interfaces::msg::Touch`)
* `sensors/battery` (`nao_interfaces::msg::Battery`)
* `sensors/robot_config` (`nao_interfaces::msg::RobotConfig`)

## Subscription Topics

** not implemented yet **
