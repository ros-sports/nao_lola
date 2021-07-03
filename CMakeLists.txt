cmake_minimum_required(VERSION 3.8)
project(nao_lola)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nao_interfaces REQUIRED)

add_library(msgpack_parser_lib SHARED
  src/msgpack_parser.cpp)

ament_target_dependencies(msgpack_parser_lib
  rclcpp
  nao_interfaces)

# Install
install(DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS msgpack_parser_lib
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
