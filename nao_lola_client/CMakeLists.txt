cmake_minimum_required(VERSION 3.8)
project(nao_lola_client)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  external/msgpack-c/include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  nao_lola_sensor_msgs
  nao_lola_command_msgs
  sensor_msgs)

# build msgpack_parser_lib
add_library(msgpack_parser_lib SHARED
  src/msgpack_parser.cpp)

ament_target_dependencies(msgpack_parser_lib
  rclcpp
  nao_lola_sensor_msgs
  Boost)

# build msgpack_packer_lib
add_library(msgpack_packer_lib SHARED
  src/msgpack_packer.cpp)

ament_target_dependencies(msgpack_packer_lib
  rclcpp
  nao_lola_command_msgs
  Boost)

# build nao_lola_client_node
add_library(nao_lola_client_node SHARED
  src/nao_lola_client.cpp
  src/connection.cpp
  src/conversion.cpp)
target_link_libraries(nao_lola_client_node
  msgpack_parser_lib
  msgpack_packer_lib)
ament_target_dependencies(nao_lola_client_node rcl_interfaces sensor_msgs rclcpp_components)

rclcpp_components_register_node(nao_lola_client_node
  PLUGIN "NaoLolaClient"
  EXECUTABLE nao_lola_client)
ament_export_targets(export_nao_lola_client HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install
install(TARGETS
  nao_lola_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS msgpack_parser_lib msgpack_packer_lib nao_lola_client_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
