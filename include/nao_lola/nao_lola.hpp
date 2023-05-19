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

#ifndef NAO_LOLA__NAO_LOLA_HPP_
#define NAO_LOLA__NAO_LOLA_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

class NaoLola : public rclcpp::Node
{
public:
  NaoLola();
  virtual ~NaoLola() {}

private:
  class Impl;
  std::shared_ptr<Impl> impl;
};

#endif  // NAO_LOLA__NAO_LOLA_HPP_
