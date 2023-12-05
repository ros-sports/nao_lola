# Build test_msgpack_parser
ament_add_gtest(test_msgpack_parser
  test_msgpack_parser.cpp)

target_link_libraries(test_msgpack_parser
  msgpack_parser_lib)

# Build test_msgpack_packer
ament_add_gtest(test_msgpack_packer
  test_msgpack_packer.cpp)

target_link_libraries(test_msgpack_packer
  msgpack_packer_lib)
