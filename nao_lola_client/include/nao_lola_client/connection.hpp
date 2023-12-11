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

#ifndef NAO_LOLA_CLIENT__CONNECTION_HPP_
#define NAO_LOLA_CLIENT__CONNECTION_HPP_

#include <array>
#include <string>
#include "boost/asio.hpp"
#include "rclcpp/logger.hpp"

#define MSGPACK_READ_LENGTH 896

using RecvData = std::array<char, MSGPACK_READ_LENGTH>;

class Connection
{
public:
  Connection();
  RecvData receive();
  void send(std::string data);

private:
  boost::asio::io_service io_service;
  boost::asio::local::stream_protocol::socket socket;
  rclcpp::Logger logger;
};

#endif  // NAO_LOLA_CLIENT__CONNECTION_HPP_
