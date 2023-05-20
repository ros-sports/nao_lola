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

#ifndef NAO_LOLA__NAO_LOLA_NODE_HPP_
#define NAO_LOLA__NAO_LOLA_NODE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

class NaoLolaNode : public rclcpp::Node
{
public:
  NaoLolaNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~NaoLolaNode() {}

private:
  class Impl;
  std::shared_ptr<Impl> impl;
};

#endif  // NAO_LOLA__NAO_LOLA_NODE_HPP_
