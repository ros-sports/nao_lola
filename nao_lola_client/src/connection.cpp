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
#include "nao_lola_client/connection.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

#define ENDPOINT "/tmp/robocup"

Connection::Connection()
: io_service(), socket(io_service), logger(rclcpp::get_logger("lola connection"))
{
  boost::system::error_code ec;
  rclcpp::Clock clock;
  do {
    socket.connect(ENDPOINT, ec);
    if (ec) {
      RCLCPP_WARN_STREAM_SKIPFIRST_THROTTLE(
        logger, clock, 1000,
        "Could not connect to LoLA, retrying: " << ec.message());
    }
  } while (ec && rclcpp::ok());
}

std::array<char, MSGPACK_READ_LENGTH> Connection::receive()
{
  boost::system::error_code ec;
  std::array<char, MSGPACK_READ_LENGTH> data;
  socket.receive(boost::asio::buffer(data), 0, ec);
  if (ec) {
    throw std::runtime_error(std::string{"Could not read from LoLA: "} + ec.message());
  }
  return data;
}

void Connection::send(std::string data)
{
  socket.send(boost::asio::buffer(data));
}
