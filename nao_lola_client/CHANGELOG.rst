^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_lola_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2023-12-09)
------------------
* Publish imu and joint state messages. These features are enabled by default, but can optionally be disabled through the "publish_imu" and "publish_joint_states" parameters.
* Modify nao_lola_client node to be an rclcpp component to allow composition.
* Allow socket connection to retry until it connects, to allow nao_lola_client to be launched before the Lola agent itself. This is useful when bringing up simulation (eg. webots) after nao_lola_client.
* Prevent segmentation faults caused by invalid JointPositions and JointStiffnesses incoming messages.
* Contributors: Kenji Brameld, ijnek

1.1.1 (2023-08-23)
------------------
* revert msgpack-c commit to match those in the iron branch, because newer version of msgpack-c is crashing LoLA
* Contributors: ijnek

1.1.0 (2023-08-03)
------------------

1.0.0 (2023-08-02)
------------------
* Migrate nao_lola across to nao_lola/nao_lola_client
* Contributors: ijnek

0.3.1 (2023-05-20)
------------------

0.3.0 (2023-04-28)
------------------

0.2.0 (2022-07-21)
------------------

0.0.4 (2022-02-04)
------------------

0.0.3 (2021-07-29)
------------------

0.0.2 (2021-07-20)
------------------

0.0.1 (2021-07-17)
------------------
