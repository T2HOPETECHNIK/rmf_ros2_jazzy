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

#include <events/ExecutePlan.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_utils/catch.hpp>

#include <algorithm>
#include <chrono>

SCENARIO("Lane closures ignore reached execution waypoints", "[lane_closure]")
{
  using namespace rmf_traffic;
  using rmf_fleet_adapter::events::ExecutePlan;

  const Profile profile{geometry::make_final_convex<geometry::Circle>(0.2)};
  agv::Graph graph;
  graph.add_waypoint("map", {0.0, 0.0});
  graph.add_waypoint("map", {2.0, 0.0});
  graph.add_waypoint("map", {2.0, 2.0});
  graph.add_waypoint("map", {4.0, 2.0});
  graph.add_lane(0, 1);
  graph.add_lane(1, 2);
  graph.add_lane(2, 3);

  const agv::VehicleTraits traits{{0.7, 0.3}, {1.0, 0.45}, profile};
  agv::Planner planner{
    agv::Planner::Configuration{graph, traits},
    agv::Planner::Options{nullptr}};
  // An old planned time alone must not make an unreported waypoint "passed".
  const auto start_time = std::chrono::steady_clock::now()
    - std::chrono::hours(1);
  const auto result = planner.plan(
    agv::Planner::Start{start_time, 0, 0.0}, agv::Planner::Goal{3});
  REQUIRE(result);

  auto database = std::make_shared<schedule::Database>();
  auto participant = schedule::make_participant(
    schedule::ParticipantDescription{
      "robot", "lane_closure_test",
      schedule::ParticipantDescription::Rx::Responsive, profile},
    database);
  const auto plan_id = participant.assign_plan_id();
  REQUIRE(participant.set(plan_id, result->get_itinerary()));
  ExecutePlan execution{
    *result, std::make_shared<PlanId>(plan_id), plan_id,
    result->get_waypoints().back().time(), nullptr};

  const auto& waypoints = execution.plan.get_waypoints();
  const auto first_arrival = std::find_if(
    waypoints.begin(), waypoints.end(),
    [](const auto& wp)
    {
      const auto& lanes = wp.approach_lanes();
      return std::find(lanes.begin(), lanes.end(), 0) != lanes.end();
    });
  REQUIRE(first_arrival != waypoints.end());
  REQUIRE_FALSE(first_arrival->arrival_checkpoints().empty());

  SECTION("Unreached and unrelated lanes")
  {
    CHECK(execution.uses_closed_lanes({0}, participant));
    CHECK(execution.uses_closed_lanes({1}, participant));
    CHECK_FALSE(execution.uses_closed_lanes({99}, participant));
    CHECK_FALSE(execution.uses_closed_lanes({}, participant));
  }

  SECTION("A reached arrival no longer makes its approach lane relevant")
  {
    for (const auto& c : first_arrival->arrival_checkpoints())
    {
      REQUIRE(c.checkpoint_id > 0);
      participant.reached(plan_id, c.route_id, c.checkpoint_id - 1);
    }
    CHECK(execution.uses_closed_lanes({0}, participant));

    for (const auto& c : first_arrival->arrival_checkpoints())
      participant.reached(plan_id, c.route_id, c.checkpoint_id);

    CHECK_FALSE(execution.uses_closed_lanes({0}, participant));
    CHECK(execution.uses_closed_lanes({1}, participant));
    CHECK(execution.uses_closed_lanes({0, 1}, participant));
  }

  SECTION("Missing checkpoint progress remains conservative")
  {
    auto empty = schedule::make_participant(
      schedule::ParticipantDescription{
        "empty", "lane_closure_test",
        schedule::ParticipantDescription::Rx::Responsive, profile},
      database);
    execution.progress_plan_id = empty.current_plan_id();
    REQUIRE(empty.reached().empty());
    CHECK(execution.uses_closed_lanes({0}, empty));
  }

  SECTION("Replacement itinerary progress cannot skip the original plan")
  {
    const auto replacement_id = participant.assign_plan_id();
    REQUIRE(participant.set(replacement_id, result->get_itinerary()));
    for (std::size_t i = 0; i < participant.itinerary().size(); ++i)
    {
      participant.reached(
        replacement_id, i, participant.itinerary()[i].trajectory().size() - 1);
    }

    // Lift and mutex phases update this shared ID when replacing the schedule.
    *execution.plan_id = replacement_id;
    CHECK(execution.uses_closed_lanes({0}, participant));
    CHECK(execution.uses_closed_lanes({2}, participant));
  }
}
