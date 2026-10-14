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

TEST_CASE("Conflicts blocked by negotiation waits remain retryable",
  "[deferred_conflict]")
{
  const bool expire_wait = GENERATE(false, true);
  // Also cover a waiting participant added through dependency expansion.
  const int waiting_index = GENERATE(0, 1, 2);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  auto node = std::make_shared<ScheduleNode>(
    database, rclcpp::NodeOptions().context(context),
    ScheduleNode::NoAutomaticSetup{});
  const auto now = rmf_traffic_ros2::convert(node->now());
  using Description = rmf_traffic::schedule::ParticipantDescription;
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.5);
  std::vector<rmf_traffic::ParticipantId> ids;
  rmf_traffic::schedule::ParticipantDescriptionsMap descriptions;
  for (const auto& name : {"a", "b", "dependent"})
  {
    Description description(name, "test", Description::Rx::Responsive,
      rmf_traffic::Profile{shape});
    const auto id = database->register_participant(description).id();
    ids.push_back(id);
    descriptions.emplace(id, description);
    rmf_traffic::Trajectory trajectory;
    trajectory.insert(now, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    trajectory.insert(now + 60s,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    rmf_traffic::Route route("test_map", std::move(trajectory));
    if (ids.size() == 3)
      route.add_dependency(0, {ids[0], 0, 0, 1});
    database->set(id, 0, {route}, 0, 1);
  }

  rmf_traffic::schedule::Mirror mirror;
  mirror.update_participants_info(descriptions);
  REQUIRE(mirror.update(database->changes(
      rmf_traffic::schedule::query_all(), std::nullopt)));
  const auto version = mirror.latest_version();
  const auto waiting = ids[waiting_index];
  auto& record = node->active_conflicts;
  record._waiting.emplace(waiting, ScheduleNode::ConflictRecord::Wait{
      86, std::nullopt, now - 31s});
  const auto pair = std::make_pair(ids[0], ids[1]);
  std::set<std::pair<rmf_traffic::ParticipantId, rmf_traffic::ParticipantId>>
    deferred;

  // Both changed-route detection and horizon promotion hand off an imminent
  // pair after removing it from the deferred set. Failed insertion must put
  // the original pair back, even when the blocker is a dependent participant.
  CHECK(node->insert_conflicts({{ids[0], ids[1]}}, mirror, deferred).empty());
  REQUIRE(deferred.size() == 1);
  CHECK(deferred.count(pair) == 1);
  CHECK(record._negotiations.empty());
  CHECK(node->insert_conflicts({{ids[0], ids[1]}}, mirror, deferred).empty());
  CHECK(deferred.size() == 1);

  if (expire_wait)
  {
    node->cull();
  }
  else
  {
    ScheduleNode::ConflictAck ack;
    ack.conflict_version = 86;
    ack.acknowledgments.emplace_back();
    ack.acknowledgments.back().participant = waiting;
    ack.acknowledgments.back().updating = false;
    node->receive_conclusion_ack(ack);
  }
  REQUIRE(record._waiting.empty());
  CHECK(mirror.latest_version() == version);

  // The periodic retry uses the same mirror: no itinerary update is needed.
  const auto entry = node->insert_conflicts(
    {{deferred.begin()->first, deferred.begin()->second}}, mirror, deferred);
  REQUIRE(entry.size() == 1);
  CHECK(deferred.empty());
  CHECK(entry.begin()->second->participants().size() == 3);

  // Nullopt for an already active negotiation is not a blocked insertion.
  deferred.insert(pair);
  CHECK(node->insert_conflicts({{ids[0], ids[1]}}, mirror, deferred).empty());
  CHECK(deferred.empty());
  CHECK(record._negotiations.size() == 1);
  node.reset();
  context->shutdown("test complete");
}
