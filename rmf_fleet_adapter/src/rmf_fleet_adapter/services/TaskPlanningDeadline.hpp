/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_FLEET_ADAPTER__SERVICES__TASKPLANNINGDEADLINE_HPP
#define RMF_FLEET_ADAPTER__SERVICES__TASKPLANNINGDEADLINE_HPP

#include <atomic>
#include <memory>
#include <chrono>
#include <functional>
#include <utility>

namespace rmf_fleet_adapter {
namespace services {

class TaskPlanningDeadline
{
public:
  using Clock = std::chrono::steady_clock;

  TaskPlanningDeadline(
    Clock::duration budget,
    std::function<bool()> interrupter = nullptr)
  : _deadline(Clock::now() + budget),
    _interrupter(std::move(interrupter))
  {
  }

  bool interrupted() const
  {
    if (Clock::now() >= _deadline || (_interrupter && _interrupter()))
      _interrupted->store(true);
    return _interrupted->load();
  }

private:
  std::shared_ptr<std::atomic_bool> _interrupted =
    std::make_shared<std::atomic_bool>(false);
  Clock::time_point _deadline;
  std::function<bool()> _interrupter;
};

} // namespace services
} // namespace rmf_fleet_adapter

#endif
