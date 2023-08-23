^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_lola
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2023-08-23)
------------------
* revert msgpack-c commit to match those in the iron branch, because newer version of msgpack-c is crashing LoLA
* Contributors: ijnek

1.1.0 (2023-08-03)
------------------

1.0.0 (2023-08-02)
------------------
* Deprecate nao_lola package in favor of nao_lola_client package.
* Contributors: ijnek

0.3.1 (2023-05-20)
------------------
* Retain state of all effectors
* Send effectors in every cycle
* Contributors: ijnek

0.3.0 (2023-04-28)
------------------
* Fix race condition on packer
* Contributors: ijnek

0.2.0 (2022-07-21)
------------------
* Split off galactic and humble branches
* Fix status test to use int rather than float
* Contributors: Kenji Brameld

0.0.4 (2022-02-04)
------------------
* fix cpplint warning
* add changes to ci to test all distros
* Contributors: Kenji Brameld, ijnek

0.0.3 (2021-07-29)
------------------
* add ament_cmake_gtest and boost dependency in package.xml and CMakeLists.txt
* Contributors: Kenji Brameld

0.0.2 (2021-07-20)
------------------
* specify license and update package description
* Contributors: Kenji Brameld

0.0.1 (2021-07-17)
------------------
* Initial impelementation
* Contributors: Kenji Brameld
