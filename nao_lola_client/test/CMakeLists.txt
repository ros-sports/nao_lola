# Build test_nao_lola_client
ament_add_gtest(test_nao_lola_client
  test_nao_lola_client.cpp)

target_link_libraries(test_nao_lola_client
  nao_lola_client_node)

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
