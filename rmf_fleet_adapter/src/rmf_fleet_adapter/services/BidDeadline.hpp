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

#ifndef RMF_FLEET_ADAPTER__SERVICES__BIDDEADLINE_HPP
#define RMF_FLEET_ADAPTER__SERVICES__BIDDEADLINE_HPP

#include <algorithm>
#include <chrono>

namespace rmf_fleet_adapter {
namespace services {

class BidDeadline
{
public:
  using Clock = std::chrono::steady_clock;

  BidDeadline(Clock::time_point received, Clock::duration window)
  {
    window = std::max(window, Clock::duration::zero());
    // Reserve up to 500ms each for result processing and response transport.
    // For short auctions, reserve a quarter of the window for each stage.
    const auto margin = std::min<Clock::duration>(
      std::chrono::milliseconds(500), window / 4);
    _response = received + window - margin;
    _planning = std::min(received + std::chrono::seconds(10), _response - margin);
    // BidNotice does not carry the auctioneer's start time. These margins are
    // best effort; the auctioneer must also reject late responses safely.
  }

  Clock::time_point planning_deadline() const
  {
    return _planning;
  }

  bool can_plan(Clock::time_point now = Clock::now()) const
  {
    return now < _planning;
  }

  bool can_respond(Clock::time_point now = Clock::now()) const
  {
    return now < _response;
  }

private:
  Clock::time_point _planning;
  Clock::time_point _response;
};

} // namespace services
} // namespace rmf_fleet_adapter

#endif
