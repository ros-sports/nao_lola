cmake_minimum_required(VERSION 3.5)
project(nao_lola_sensor_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Accelerometer.msg"
  "msg/Angle.msg"
  "msg/Battery.msg"
  "msg/Buttons.msg"
  "msg/FSR.msg"
  "msg/Gyroscope.msg"
  "msg/JointCurrents.msg"
  "msg/JointIndexes.msg"
  "msg/JointPositions.msg"
  "msg/JointStatuses.msg"
  "msg/JointStiffnesses.msg"
  "msg/JointTemperatures.msg"
  "msg/RobotConfig.msg"
  "msg/Sonar.msg"
  "msg/Touch.msg"
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
