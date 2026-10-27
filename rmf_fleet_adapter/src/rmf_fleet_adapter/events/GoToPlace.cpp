/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "GoToPlace.hpp"
#include "../project_itinerary.hpp"

#include <rmf_traffic/schedule/StubbornNegotiator.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <algorithm>
#include <queue>
#include <unordered_set>

namespace rmf_fleet_adapter {
namespace events {

//==============================================================================
void GoToPlace::add(rmf_task_sequence::Event::Initializer& initializer)
{
  initializer.add<Description>(
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      std::function<void()> update) -> StandbyPtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update));
    },
    [](
      const AssignIDPtr& id,
      const std::function<rmf_task::State()>& get_state,
      const rmf_task::ConstParametersPtr& parameters,
      const Description& description,
      const nlohmann::json&,
      std::function<void()> update,
      std::function<void()> checkpoint,
      std::function<void()> finished) -> ActivePtr
    {
      return Standby::make(
        id, get_state, parameters, description, std::move(update))
      ->begin(std::move(checkpoint), std::move(finished));
    });
}

//==============================================================================
auto GoToPlace::Standby::make(
  const AssignIDPtr& id,
  const std::function<rmf_task::State()>& get_state,
  const rmf_task::ConstParametersPtr& parameters,
  const rmf_task_sequence::events::GoToPlace::Description& description,
  std::function<void()> update,
  std::optional<rmf_traffic::Duration> tail_period)
-> std::shared_ptr<Standby>
{
  const auto state = get_state();
  const auto context = state.get<agv::GetContext>()->value;
  const auto header = description.generate_header(state, *parameters);

  auto standby = std::make_shared<Standby>(Standby{description});
  standby->_assign_id = id;
  standby->_context = context;
  standby->_time_estimate = header.original_duration_estimate();
  standby->_tail_period = tail_period;
  standby->_update = std::move(update);
  standby->_state = rmf_task::events::SimpleEventState::make(
    id->assign(),
    header.category(),
    header.detail(),
    rmf_task::Event::Status::Standby,
    {},
    context->clock());

  return standby;
}

//==============================================================================
GoToPlace::Standby::Standby(Description description)
: _description(std::move(description))
{
  // Do nothin
}

//==============================================================================
auto GoToPlace::Standby::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration GoToPlace::Standby::duration_estimate() const
{
  return _time_estimate;
}

//==============================================================================
auto GoToPlace::Standby::begin(
  std::function<void()>,
  std::function<void()> finished) -> ActivePtr
{
  if (!_active)
  {
    _active = Active::make(
      _assign_id,
      _context,
      _description,
      _tail_period,
      _state,
      _update,
      std::move(finished));
  }

  return _active;
}

//==============================================================================
auto GoToPlace::Active::make(
  const AssignIDPtr& id,
  agv::RobotContextPtr context,
  Description description,
  std::optional<rmf_traffic::Duration> tail_period,
  rmf_task::events::SimpleEventStatePtr state,
  std::function<void()> update,
  std::function<void()> finished) -> std::shared_ptr<Active>
{
  auto active = std::make_shared<Active>(Active(std::move(description)));
  active->_assign_id = id;
  active->_context = std::move(context);
  active->_tail_period = tail_period;
  active->_update = std::move(update);
  active->_finished = std::move(finished);
  active->_state = std::move(state);

  if (active->_description.one_of().empty())
  {
    active->_state->update_status(Status::Error);
    active->_state->update_log().error(
      "No destination option was provided to go_to_place. There is nowhere to "
      "go, so we will proceed to the next step in the task.");

    RCLCPP_ERROR(
      active->_context->node()->get_logger(),
      "No destination option was provided for a go_to_place for [%s]. There is "
      "nowhere to go, so we will proceed to the next step in the task.",
      active->_context->requester_id().c_str());

    active->_context->worker().schedule(
      [finished = active->_finished](const auto&)
      {
        finished();
      });
    return active;
  }

  active->_negotiator =
    Negotiator::make(
    active->_context,
    [w = active->weak_from_this()](
      const auto& t, const auto& r) -> Negotiator::NegotiatePtr
    {
      if (const auto self = w.lock())
        return self->_respond(t, r);

      r->forfeit({});
      return nullptr;
    });

  active->_replan_request_subscription =
    active->_context->observe_replan_request()
    .observe_on(rxcpp::identity_same_worker(active->_context->worker()))
    .subscribe(
    [w = active->weak_from_this()](const auto&)
    {
      const auto self = w.lock();
      if (self && !self->_find_path_service)
      {
        RCLCPP_INFO(
          self->_context->node()->get_logger(),
          "Replanning requested for [%s]",
          self->_context->requester_id().c_str());

        if (const auto c = self->_context->command())
          c->stop();

        self->_find_plan();
      }
    });

  active->_graph_change_subscription =
    active->_context->observe_graph_change()
    .observe_on(rxcpp::identity_same_worker(active->_context->worker()))
    .subscribe(
    [w = active->weak_from_this()](
      const agv::RobotContext::GraphChange& changes)
    {
      const auto self = w.lock();
      if (!self)
      {
        return;
      }

      if (self->_find_path_service)
      {
        // If we're currently replanning, we should restart the replanning
        // because the upcoming solution might involve a closed lane
        RCLCPP_INFO(
          self->_context->node()->get_logger(),
          "Requesting replan for [%s] to account for a newly closed lane",
          self->_context->requester_id().c_str());
        self->_context->request_replan();
        return;
      }

      if (self->_execution.has_value())
      {
        // Lanes the robot has already driven past do not affect it.
        const auto closed_ahead = self->_execution->closed_lanes_ahead(
          changes.closed_lanes, self->_context->itinerary());
        if (!closed_ahead.empty())
        {
          const auto& requesting = self->_context->requesting_mutex_groups();

          // A closed lane ahead that touches a mutex group the robot is
          // waiting for (e.g. the lane into a lift that another robot just
          // took) means the wait itself is for a path that has closed. That
          // mutex may not be released for a long time, so replan now in case
          // another route, like a different lift, is open.
          const auto& graph = self->_context->navigation_graph();
          const bool waiting_on_closed = std::any_of(
            closed_ahead.begin(), closed_ahead.end(),
            [&](const std::size_t l)
            {
              if (l >= graph.num_lanes())
                return false;

              const auto& lane = graph.get_lane(l);
              for (const auto* group : {
                  &lane.properties().in_mutex_group(),
                  &graph.get_waypoint(lane.entry().waypoint_index())
                  .in_mutex_group(),
                  &graph.get_waypoint(lane.exit().waypoint_index())
                  .in_mutex_group()})
              {
                if (!group->empty() && requesting.count(*group) > 0)
                  return true;
              }
              return false;
            });

          if (!requesting.empty() && !waiting_on_closed)
          {
            // The robot is holding in place until it locks a mutex group, so
            // there is no need to replan yet. The plan is checked against the
            // closed lanes once the mutex groups are locked.
            RCLCPP_INFO(
              self->_context->node()->get_logger(),
              "Deferring replan for [%s] around a newly closed lane until it "
              "locks the mutex groups it is waiting for",
              self->_context->requester_id().c_str());
            self->_context->defer_replan_until_mutex_locked();
            return;
          }

          RCLCPP_INFO(
            self->_context->node()->get_logger(),
            "Requesting replan for [%s] to avoid a newly closed lane",
            self->_context->requester_id().c_str());
          self->_context->request_replan();
          return;
        }
      }
      else
      {
        // Strange that there isn't an execution and also isn't a
        // _find_path_service, but let's just request a replan.
        RCLCPP_INFO(
          self->_context->node()->get_logger(),
          "Requesting replan for [%s] to account for a newly closed lane (v2)",
          self->_context->requester_id().c_str());
        self->_context->request_replan();
      }
    });

  active->_find_plan();
  return active;
}

//==============================================================================
auto GoToPlace::Active::state() const -> ConstStatePtr
{
  return _state;
}

//==============================================================================
rmf_traffic::Duration GoToPlace::Active::remaining_time_estimate() const
{
  if (_execution.has_value())
  {
    const auto finish = _execution->finish_time_estimate;
    const auto now = _context->now();

    const auto& itin = _context->itinerary();
    if (_execution->plan_id)
    {
      if (const auto delay = itin.cumulative_delay(*_execution->plan_id))
        return finish - now + *delay;
    }
    else
    {
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "Missing plan_id for go_to_place of robot [%s]. Please report this "
        "critical bug to the maintainers of RMF.",
        _context->requester_id().c_str());
    }
  }

  if (!_chosen_goal.has_value())
  {
    return rmf_traffic::Duration(0);
  }

  const auto& estimate =
    _context->planner()->setup(_context->location(), _chosen_goal->waypoint());

  if (estimate.ideal_cost().has_value())
    return rmf_traffic::time::from_seconds(*estimate.ideal_cost());

  // It would be very suspicious if this happens... probably indicates that the
  // task is impossible.
  return rmf_traffic::Duration(0);
}

//==============================================================================
auto GoToPlace::Active::backup() const -> Backup
{
  // GoToPlace doesn't need to be backed up
  return Backup::make(0, nlohmann::json());
}

//==============================================================================
auto GoToPlace::Active::interrupt(std::function<void()> task_is_interrupted)
-> Resume
{
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Interrupting go_to_place for robot [%s]",
    _context->requester_id().c_str());
  _negotiator->clear_license();
  _is_interrupted = true;
  _stop_and_clear();

  _state->update_status(Status::Standby);
  _state->update_log().info("Going into standby for an interruption");
  _state->update_dependencies({});

  _context->worker().schedule(
    [task_is_interrupted](const auto&)
    {
      task_is_interrupted();
    });

  return Resume::make(
    [w = weak_from_this()]()
    {
      if (const auto self = w.lock())
      {
        self->_negotiator->claim_license();
        self->_is_interrupted = false;
        self->_find_plan();
      }
    });
}

//==============================================================================
void GoToPlace::Active::cancel()
{
  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Canceling go_to_place for robot [%s]",
    _context->requester_id().c_str());
  _stop_and_clear();
  _state->update_status(Status::Canceled);
  _state->update_log().info("Received signal to cancel");
  _finished();
}

//==============================================================================
void GoToPlace::Active::kill()
{
  _stop_and_clear();
  _state->update_status(Status::Killed);
  _state->update_log().info("Received signal to kill");
  _finished();
}

//==============================================================================
std::string wp_name(
  const agv::RobotContext& context,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  const auto& g = context.planner()->get_configuration().graph();
  const auto& wp = g.get_waypoint(goal.waypoint());
  if (wp.name())
    return *wp.name();

  return "#" + std::to_string(goal.waypoint());
}

//==============================================================================
std::string wp_name(const agv::RobotContext& context)
{
  const auto& g = context.planner()->get_configuration().graph();
  const auto& locations = context.location();
  for (const auto& l : locations)
  {
    const auto& wp = g.get_waypoint(l.waypoint());
    if (wp.name())
      return *wp.name();
  }

  if (locations.empty())
    return "<null>";

  return "#" + std::to_string(locations.front().waypoint());
}

//==============================================================================
std::optional<rmf_traffic::agv::Plan::Goal> GoToPlace::Active::_choose_goal(
  bool only_same_map) const
{
  auto current_location = _context->location();
  auto graph = _context->navigation_graph();
  if (current_location.size() == 0)
  {
    //unable to get location. We should return some form of error stste.
    RCLCPP_ERROR(
      _context->node()->get_logger(),
      "Robot [%s] can't get location",
      _context->requester_id().c_str());
    return std::nullopt;
  }

  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Selecting a new go_to_place location from [%lu] choices for robot [%s]",
    _description.one_of().size(),
    _context->requester_id().c_str());

  auto lowest_cost = std::numeric_limits<double>::infinity();
  std::optional<std::size_t> selected_idx;
  for (std::size_t i = 0; i < _description.one_of().size(); ++i)
  {
    const auto wp_idx = _description.one_of()[i].waypoint();
    if (only_same_map)
    {
      const auto& wp = graph.get_waypoint(wp_idx);

      // Check if same map. If not don't consider location. This is to ensure
      // the robot does not try to board a lift.
      if (wp.get_map_name() != _context->map())
      {
        RCLCPP_INFO(
          _context->node()->get_logger(),
          "Skipping [%lu] as it is on map [%s] but robot is on map [%s].",
          wp_idx,
          wp.get_map_name().c_str(),
          _context->map().c_str());
        continue;
      }
    }

    // Find distance to said point
    auto result = _context->planner()->quickest_path(current_location, wp_idx);
    if (result.has_value())
    {
      RCLCPP_INFO(
        _context->node()->get_logger(),
        "Got distance from [%lu] as %f",
        wp_idx,
        result->cost());

      if (result->cost() < lowest_cost)
      {
        selected_idx = i;
        lowest_cost = result->cost();
      }
    }
    else
    {
      RCLCPP_ERROR(
        _context->node()->get_logger(),
        "No path found for robot [%s] to waypoint [%lu]",
        _context->requester_id().c_str(),
        wp_idx);
    }
  }

  if (selected_idx.has_value())
  {
    return _description.one_of()[*selected_idx];
  }

  return std::nullopt;
}

//==============================================================================
void GoToPlace::Active::_find_plan()
{
  if (_is_interrupted)
    return;

  // A fresh plan accounts for any lane closures whose replan was deferred.
  _context->take_deferred_replan();

  // A staging waypoint is never the event's goal. Choose again so a replan
  // aims for the real goal, or stages again if it is still unreachable.
  if (_staging)
  {
    _chosen_goal = std::nullopt;
    _staging = false;
  }

  if (!_chosen_goal.has_value() && _description.prefer_same_map())
  {
    _chosen_goal = _choose_goal(true);
  }

  if (!_chosen_goal.has_value())
  {
    _chosen_goal = _choose_goal(false);
  }

  _staging = false;
  if (!_chosen_goal.has_value())
  {
    std::string error_msg = "Unable to find a path to any of the goal options ["
      + _description.destination_name(*_context->task_parameters())
      + "]";

    _state->update_log().error(error_msg);
    RCLCPP_ERROR(
      _context->node()->get_logger(),
      "%s for [%s]",
      error_msg.c_str(),
      _context->requester_id().c_str());

    _chosen_goal = _choose_staging_goal();
    if (!_chosen_goal.has_value())
    {
      // The robot waits where it is, so it should not keep mutex groups for
      // the path ahead (e.g. the lift lobby) that block other robots.
      const auto& graph = _context->navigation_graph();
      std::unordered_set<std::string> retain;
      for (const auto& start : _context->location())
      {
        retain.insert(graph.get_waypoint(start.waypoint()).in_mutex_group());
        if (start.lane().has_value())
        {
          const auto& lane = graph.get_lane(*start.lane());
          retain.insert(lane.properties().in_mutex_group());
          retain.insert(
            graph.get_waypoint(lane.entry().waypoint_index()).in_mutex_group());
        }
      }
      retain.erase("");
      _context->retain_mutex_groups(retain);

      // A replan can get here with the previous (now stopped) execution still
      // set, and the retry does nothing while an execution exists.
      _execution = std::nullopt;
      _schedule_retry();
      return;
    }

    _staging = true;
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Moving [%s] to staging waypoint [%s] until a path to its goal opens",
      _context->requester_id().c_str(),
      wp_name(*_context, *_chosen_goal).c_str());
  }

  _state->update_status(Status::Underway);
  const auto start_name = wp_name(*_context);
  const auto goal_name = wp_name(*_context, *_chosen_goal);
  _state->update_log().info(
    "Generating plan to move from [" + start_name + "] to [" + goal_name + "]");

  const auto& graph = _context->navigation_graph();
  std::stringstream ss;
  ss << "Planning for [" << _context->requester_id()
     << "] to [" << goal_name << "] from one of these locations:"
     << agv::print_starts(_context->location(), graph);

  RCLCPP_INFO(
    _context->node()->get_logger(),
    "%s",
    ss.str().c_str());

  // TODO(MXG): Make the planning time limit configurable
  _find_path_service = std::make_shared<services::FindPath>(
    _context->planner(), _context->location(), *_chosen_goal,
    _context->schedule()->snapshot(), _context->itinerary().id(),
    _context->profile(),
    std::chrono::seconds(5));

  _plan_subscription = rmf_rxcpp::make_job<services::FindPath::Result>(
    _find_path_service)
    .observe_on(rxcpp::identity_same_worker(_context->worker()))
    .subscribe(
    [w = weak_from_this(), start_name, goal_name, goal = *_chosen_goal](
      const services::FindPath::Result& result)
    {
      const auto self = w.lock();
      if (!self)
        return;

      if (!result)
      {
        // The planner could not find a way to reach the goal
        self->_state->update_status(Status::Error);
        self->_state->update_log().error(
          "Failed to find a plan to move from ["
          + start_name + "] to [" + goal_name + "]. Will retry soon.");

        // Reset the chosen goal in case this goal has become impossible to
        // reach
        self->_chosen_goal = std::nullopt;
        self->_execution = std::nullopt;
        self->_schedule_retry();

        self->_context->worker()
        .schedule([update = self->_update](const auto&) { update(); });

        return;
      }

      self->_state->update_status(Status::Underway);
      self->_state->update_log().info(
        "Found a plan to move from ["
        + start_name + "] to [" + goal_name + "]");

      auto full_itinerary = project_itinerary(
        *result, self->_description.expected_next_destinations(),
        *self->_context->planner());

      // Clear the old retry first, because _execute_plan may schedule a new
      // one (e.g. when the robot is already at its staging waypoint).
      self->_retry_timer = nullptr;

      self->_execute_plan(
        self->_context->itinerary().assign_plan_id(),
        *std::move(result),
        std::move(full_itinerary),
        goal);

      self->_find_path_service = nullptr;
    });

  _find_path_timeout = _context->node()->try_create_wall_timer(
    std::chrono::seconds(10),
    [
      weak_service = _find_path_service->weak_from_this(),
      weak_self = weak_from_this()
    ]()
    {
      if (const auto service = weak_service.lock())
        service->interrupt();

      if (const auto self = weak_self.lock())
        self->_find_path_timeout = nullptr;
    });

  _update();
}

//==============================================================================
std::optional<rmf_traffic::agv::Plan::Goal>
GoToPlace::Active::_choose_staging_goal() const
{
  const auto& starts = _context->location();
  const auto* closures = _context->get_lane_closures();
  if (starts.empty() || !closures)
    return std::nullopt;

  const auto& graph = _context->navigation_graph();
  const std::size_t N = graph.num_waypoints();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> cost(N, inf);
  std::vector<std::optional<std::size_t>> via_lane(N);

  // Shortest paths over the graph, ignoring lane closures.
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  for (const auto& start : starts)
  {
    cost[start.waypoint()] = 0.0;
    queue.push({0.0, start.waypoint()});
  }

  while (!queue.empty())
  {
    const auto [c, wp] = queue.top();
    queue.pop();
    if (c > cost[wp])
      continue;

    const Eigen::Vector2d p0 = graph.get_waypoint(wp).get_location();
    for (const auto lane_index : graph.lanes_from(wp))
    {
      const auto next = graph.get_lane(lane_index).exit().waypoint_index();
      const Eigen::Vector2d p1 = graph.get_waypoint(next).get_location();
      const double next_cost = c + (p1 - p0).norm();
      if (next_cost < cost[next])
      {
        cost[next] = next_cost;
        via_lane[next] = lane_index;
        queue.push({next_cost, next});
      }
    }
  }

  std::optional<std::size_t> goal_wp;
  for (const auto& option : _description.one_of())
  {
    const auto wp = option.waypoint();
    if (wp < N && cost[wp] < inf && (!goal_wp || cost[wp] < cost[*goal_wp]))
      goal_wp = wp;
  }

  if (!goal_wp.has_value())
    return std::nullopt;

  // Waypoints from the start to the goal
  std::vector<std::size_t> path_wps = {*goal_wp};
  std::vector<std::size_t> path_lanes;
  while (via_lane[path_wps.back()].has_value())
  {
    const auto lane = *via_lane[path_wps.back()];
    path_lanes.push_back(lane);
    path_wps.push_back(graph.get_lane(lane).entry().waypoint_index());
  }
  std::reverse(path_wps.begin(), path_wps.end());
  std::reverse(path_lanes.begin(), path_lanes.end());

  // The lane i connects path_wps[i] to path_wps[i+1]
  std::optional<std::size_t> blocked;
  for (std::size_t i = 0; i < path_lanes.size(); ++i)
  {
    if (closures->is_closed(path_lanes[i]))
    {
      blocked = i;
      break;
    }
  }

  if (!blocked.has_value())
    return std::nullopt;

  // Stop before entering the mutex group of the waypoint in front of the
  // closed lane, e.g. outside the lift lobby, so the robot does not block it.
  std::size_t stage = *blocked;
  const auto& group = graph.get_waypoint(path_wps[stage]).in_mutex_group();
  if (!group.empty())
  {
    while (stage > 0
      && graph.get_waypoint(path_wps[stage - 1]).in_mutex_group() == group)
    {
      --stage;
    }

    if (stage == 0)
      return std::nullopt;

    --stage;
  }

  // A plan cannot end inside a lift cabin, so wait outside the lift instead.
  while (stage > 0 && graph.get_waypoint(path_wps[stage]).in_lift())
    --stage;

  if (stage == 0)
    return std::nullopt;

  return rmf_traffic::agv::Plan::Goal(path_wps[stage]);
}

//==============================================================================
GoToPlace::Active::Active(Description description)
: _description(std::move(description))
{
  // Do nothing
}

//==============================================================================
void GoToPlace::Active::_schedule_retry()
{
  if (_retry_timer)
    return;

  // TODO(MXG): Make the retry timing configurable
  _retry_timer = _context->node()->try_create_wall_timer(
    std::chrono::seconds(5),
    [w = weak_from_this()]()
    {
      const auto self = w.lock();
      if (!self)
        return;

      self->_retry_timer = nullptr;
      if (self->_execution.has_value())
        return;

      self->_find_plan();
    });
}

//==============================================================================
void GoToPlace::Active::_execute_plan(
  const rmf_traffic::PlanId plan_id,
  rmf_traffic::agv::Plan plan,
  rmf_traffic::schedule::Itinerary full_itinerary,
  rmf_traffic::agv::Plan::Goal goal)
{
  if (_is_interrupted)
    return;

  if (_staging && (plan.get_itinerary().empty() || plan.get_waypoints().empty()))
  {
    // Already at the staging waypoint. Keep waiting for the goal to open.
    _chosen_goal = std::nullopt;
    _staging = false;
    _schedule_retry();
    return;
  }

  if (plan.get_itinerary().empty() || plan.get_waypoints().empty())
  {
    _state->update_status(Status::Completed);
    _state->update_log().info(
      "The planner indicates that the robot is already at its goal.");
    RCLCPP_INFO(
      _context->node()->get_logger(),
      "Robot [%s] is already at its goal [%lu]",
      _context->requester_id().c_str(),
      goal.waypoint());

    const auto& graph = _context->navigation_graph();
    _context->retain_mutex_groups(
      {graph.get_waypoint(goal.waypoint()).in_mutex_group()});

    _finished();
    return;
  }

  const auto& graph = _context->navigation_graph();

  RCLCPP_INFO(
    _context->node()->get_logger(),
    "Executing go_to_place [%s] for robot [%s]",
    graph.get_waypoint(plan.get_waypoints().back().graph_index().value())
    .name_or_index().c_str(),
    _context->requester_id().c_str());

  auto finished = _finished;
  if (_staging)
  {
    // Reaching the staging waypoint does not finish this event. Try to plan
    // to the real goal again from there.
    finished = [w = weak_from_this()]()
      {
        const auto self = w.lock();
        if (!self)
          return;

        self->_context->worker().schedule(
          [w](const auto&)
          {
            const auto self = w.lock();
            if (!self)
              return;

            self->_execution = std::nullopt;
            self->_chosen_goal = std::nullopt;
            self->_staging = false;

            // The staging execution shares this event's state and marked it
            // completed, but the event is still waiting for its real goal.
            self->_state->update_status(Status::Underway);
            self->_state->update_log().info(
              "Reached the staging waypoint. Waiting for a path to the goal.");
            self->_update();

            self->_find_plan();
          });
      };
  }

  _execution = ExecutePlan::make(
    _context, plan_id, std::move(plan), std::move(goal),
    std::move(full_itinerary),
    _assign_id, _state, _update, std::move(finished), _tail_period);

  if (!_execution.has_value())
  {
    _state->update_status(Status::Error);
    _state->update_log().error(
      "Invalid (empty) plan generated. Will retry soon. "
      "Please report this incident to the Open-RMF developers.");
    _schedule_retry();
  }
}

//==============================================================================
void GoToPlace::Active::_stop_and_clear()
{
  _execution = std::nullopt;
  if (const auto command = _context->command())
    command->stop();

  _context->itinerary().clear();
}

//==============================================================================
Negotiator::NegotiatePtr GoToPlace::Active::_respond(
  const Negotiator::TableViewerPtr& table_view,
  const Negotiator::ResponderPtr& responder)
{
  if (!_chosen_goal.has_value())
  {
    responder->forfeit({});
    return nullptr;
  }

  auto approval_cb = [w = weak_from_this(), goal = *_chosen_goal](
    const rmf_traffic::PlanId plan_id,
    const rmf_traffic::agv::Plan& plan,
    rmf_traffic::schedule::Itinerary itinerary)
    -> std::optional<rmf_traffic::schedule::ItineraryVersion>
    {
      if (auto self = w.lock())
      {
        self->_execute_plan(plan_id, plan, std::move(itinerary), goal);
        return self->_context->itinerary().version();
      }

      return std::nullopt;
    };

  const auto evaluator = Negotiator::make_evaluator(table_view);
  return services::Negotiate::path(
    _context->itinerary().assign_plan_id(), _context->planner(),
    _context->location(), *_chosen_goal,
    _description.expected_next_destinations(), table_view,
    responder, std::move(approval_cb), std::move(evaluator));
}

} // namespace events
} // namespace rmf_fleet_adapter
