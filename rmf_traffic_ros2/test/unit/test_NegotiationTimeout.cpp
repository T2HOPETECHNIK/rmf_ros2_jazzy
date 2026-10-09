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

#include "../../src/rmf_traffic_ros2/schedule/internal_Node.hpp"

#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_utils/catch.hpp>

using namespace std::chrono_literals;
using rmf_traffic_ros2::schedule::ScheduleNode;

TEST_CASE("Schedule culling expires only elapsed negotiation deadlines",
  "[negotiation_timeout]")
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  auto node = std::make_shared<ScheduleNode>(
    database, rclcpp::NodeOptions().context(context),
    ScheduleNode::NoAutomaticSetup{});
  node->conflict_conclusion_pub =
    node->create_publisher<ScheduleNode::ConflictConclusion>("test_conclusion", 10);
  node->negotiation_states_pub =
    node->create_publisher<ScheduleNode::NegotiationStates>("test_states", 10);
  node->negotiation_stasuses_pub =
    node->create_publisher<ScheduleNode::NegotiationStatuses>("test_status", 10);

  const auto now = rmf_traffic_ros2::convert(node->now());
  auto& conflicts = node->active_conflicts;

  SECTION("Acknowledgments retain their full grace period")
  {
    using Wait = ScheduleNode::ConflictRecord::Wait;
    conflicts._waiting.emplace(18, Wait{86, std::nullopt, now - 161ms});
    conflicts._waiting.emplace(25, Wait{86, std::nullopt, now - 30s});
    conflicts._waiting.emplace(21, Wait{86, std::nullopt, now - 31s});
    conflicts._waiting.emplace(35, Wait{86, std::nullopt, now + 5s});

    node->cull();
    CHECK(conflicts._waiting.count(18) == 1);
    CHECK(conflicts._waiting.at(18).negotiation_version == 86);
    CHECK(conflicts._waiting.count(25) == 0);
    CHECK(conflicts._waiting.count(21) == 0);
    CHECK(conflicts._waiting.count(35) == 1);
    conflicts.acknowledge(*node, 86, 18, 42);
    REQUIRE(conflicts._waiting.at(18).itinerary_update_version.has_value());
    CHECK(*conflicts._waiting.at(18).itinerary_update_version == 42);
    conflicts.check(18, 42);
    CHECK(conflicts._waiting.count(18) == 0);
  }

  SECTION("Active negotiations expire based on last activity")
  {
    using Description = rmf_traffic::schedule::ParticipantDescription;
    const auto shape = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.5);
    const auto a = database->register_participant(Description(
      "a", "test", Description::Rx::Responsive,
      rmf_traffic::Profile{shape})).id();
    const auto b = database->register_participant(Description(
      "b", "test", Description::Rx::Responsive,
      rmf_traffic::Profile{shape})).id();

    const auto entry = conflicts.insert({a, b}, now - 60s, *database);
    REQUIRE(entry.has_value());
    const auto version = entry->first;
    auto* open = conflicts.negotiation(version);
    REQUIRE(open != nullptr);
    bool expires = false;

    SECTION("A recent update keeps an old negotiation alive")
    {
      open->last_active_time = now - 161ms;
    }
    SECTION("The deadline itself has elapsed")
    {
      open->last_active_time = now - 30s;
      expires = true;
    }
    SECTION("An inactive negotiation is removed")
    {
      open->last_active_time = now - 31s;
      expires = true;
    }
    SECTION("A future activity time is not expired")
    {
      open->last_active_time = now + 5s;
    }

    node->cull();
    CHECK((conflicts.negotiation(version) == nullptr) == expires);
    CHECK((conflicts._version.count(a) == 0) == expires);
    CHECK((conflicts._version.count(b) == 0) == expires);
  }

  node.reset();
  context->shutdown("test complete");
}
