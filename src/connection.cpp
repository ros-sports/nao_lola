// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "nao_lola/connection.hpp"

#define ENDPOINT "/tmp/robocup"

Connection::Connection()
: io_service(), socket(io_service), logger(rclcpp::get_logger("lola connection"))
{
  boost::system::error_code ec;
  socket.connect(ENDPOINT, ec);
  if (ec) {
    RCLCPP_ERROR(logger, (std::string{"Could not connect to LoLA: "} + ec.message()).c_str());
  }
}

std::array<char, MSGPACK_READ_LENGTH> Connection::receive()
{
  boost::system::error_code ec;
  std::array<char, MSGPACK_READ_LENGTH> data;
  socket.receive(boost::asio::buffer(data), 0, ec);
  if (ec) {
    RCLCPP_ERROR(logger, (std::string{"Could not read from LoLA: "} + ec.message()).c_str());
  }
  return data;
}

void Connection::send(std::string data)
{
  socket.send(boost::asio::buffer(data));
}
