cmake_minimum_required(VERSION 3.8)
project(nao_lola)

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
find_package(nao_sensor_msgs REQUIRED)
find_package(nao_command_msgs REQUIRED)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

# build msgpack_parser_lib
add_library(msgpack_parser_lib SHARED
  src/msgpack_parser.cpp)

ament_target_dependencies(msgpack_parser_lib
  rclcpp
  nao_sensor_msgs
  Boost)

# build msgpack_packer_lib
add_library(msgpack_packer_lib SHARED
  src/msgpack_packer.cpp)

ament_target_dependencies(msgpack_packer_lib
  rclcpp
  nao_command_msgs
  Boost)

# build nao_lola
add_executable(nao_lola
  src/nao_lola_node.cpp
  src/nao_lola.cpp
  src/connection.cpp)

target_link_libraries(nao_lola
  msgpack_parser_lib
  msgpack_packer_lib)

# Install
install(TARGETS
  nao_lola
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS msgpack_parser_lib msgpack_packer_lib
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
