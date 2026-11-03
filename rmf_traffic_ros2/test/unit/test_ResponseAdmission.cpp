/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <rmf_utils/catch.hpp>
#include "../../src/rmf_traffic_ros2/schedule/ResponseAdmission.hpp"

#include <vector>

using rmf_traffic_ros2::schedule::detail::ResponseAdmission;
using namespace std::chrono_literals;

TEST_CASE("Negotiation admission bounds planning across robots", "[admission]")
{
  ResponseAdmission admission(2, 3);
  const auto now = ResponseAdmission::Clock::now();
  std::vector<ResponseAdmission::Lease> running;
  std::vector<int> started;
  std::vector<int> expired;
  auto request = [&](int id)
  {
    return ResponseAdmission::Request{
    {1, static_cast<uintptr_t>(id), 0}, now + 15s,
    [] { return true; },
    [&, id](auto lease, auto deadline)
    {
      CHECK(deadline == now + 15s);
      started.push_back(id);
      running.push_back(std::move(lease));
    },
    [&, id] { expired.push_back(id); }};
  };

  admission.enqueue(request(1), now);
  admission.enqueue(request(2), now);
  admission.drain(now);
  REQUIRE(started == std::vector<int>{1, 2});

  // Duplicate active and queued requests must not start or forfeit anything.
  admission.enqueue(request(1), now);
  admission.enqueue(request(3), now);
  admission.enqueue(request(3), now);
  admission.enqueue(request(4), now);
  admission.enqueue(request(5), now);
  admission.enqueue(request(6), now);
  REQUIRE(expired == std::vector<int>{6});
  admission.drain(now);
  REQUIRE(started.size() == 2);

  // A timeout notification alone does not free a slot: outstanding work must
  // release the responder/lease first.
  admission.drain(now + 1s);
  REQUIRE(started.size() == 2);
  running[0].reset();
  admission.drain(now + 2s);
  REQUIRE(started == std::vector<int>{1, 2, 3});

  // Waiting consumes the original deadline, even with both slots occupied.
  admission.drain(now + 15s);
  REQUIRE(expired == std::vector<int>{6, 4, 5});
  running.clear();
  admission.drain(now + 16s);
  REQUIRE(started.size() == 3);
}

TEST_CASE("Obsolete requests leave the bounded negotiation queue",
  "[admission]")
{
  ResponseAdmission admission(1, 1);
  const auto now = ResponseAdmission::Clock::now();
  ResponseAdmission::Lease running;
  bool obsolete = false;
  int started = 0;
  int expired = 0;
  auto request = [&](int id, std::function<bool()> valid)
  {
    return ResponseAdmission::Request{
    {1, static_cast<uintptr_t>(id), 0}, now + 15s, std::move(valid),
    [&](auto lease, auto)
    {
      ++started;
      running = std::move(lease);
    },
    [&] { ++expired; }};
  };
  admission.enqueue(request(1, [] { return true; }), now);
  admission.drain(now);
  admission.enqueue(request(2, [&] { return !obsolete; }), now);
  obsolete = true;
  admission.enqueue(request(3, [] { return true; }), now);
  running.reset();
  admission.drain(now);
  CHECK(started == 2);
  CHECK(expired == 0);
}

TEST_CASE("Admission callbacks may reenter and leases outlive the queue",
  "[admission]")
{
  ResponseAdmission::Lease running;
  const auto now = ResponseAdmission::Clock::now();
  int started = 0;
  {
    ResponseAdmission admission(1, 1);
    const auto second = ResponseAdmission::Request{
      {1, 2, 0}, now + 15s, [] { return true; },
      [&](auto lease, auto) { ++started; running = std::move(lease); },
      [] { FAIL("Unexpected expiry"); }};
    admission.enqueue({{1, 1, 0}, now + 15s, [] { return true; },
        [&](auto, auto)
        {
          ++started;
          admission.enqueue(second, now);
        }, [] { FAIL("Unexpected expiry"); }}, now);
    admission.drain(now);
    REQUIRE(started == 1);
    admission.drain(now);
    REQUIRE(started == 2);
  }
  running.reset();
}
