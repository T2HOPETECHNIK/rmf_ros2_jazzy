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

#include "MockAdapterFixture.hpp"

#include <events/LockMutexGroup.hpp>
#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_utils/catch.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>

using namespace std::chrono_literals;
using rmf_fleet_adapter::phases::test::MockAdapterFixture;
using rmf_fleet_adapter::events::LockMutexGroup;

namespace {
template<typename Predicate>
bool wait_for(Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  do
  {
    if (predicate())
      return true;
    std::this_thread::sleep_for(10ms);
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

struct Requests
{
  using Message = rmf_fleet_msgs::msg::MutexGroupRequest;
  std::mutex mutex;
  std::vector<Message> messages;

  bool released(const std::string& group)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& msg : messages)
    {
      if (msg.group == group && msg.mode == Message::MODE_RELEASE)
        return true;
    }
    return false;
  }
};
}

TEST_CASE_METHOD(MockAdapterFixture,
  "Mutex requests follow the lifetime of their execution", "[mutex_lifecycle]")
{
  using namespace rmf_fleet_adapter;
  const auto robot = add_robot();
  const auto context = robot.context;
  auto finished = std::make_shared<std::atomic<int>>(0);
  auto requests = std::make_shared<Requests>();
  auto subscription = data->ros_node->create_subscription<Requests::Message>(
    MutexGroupRequestTopicName, rclcpp::QoS(100).reliable().transient_local(),
    [requests](Requests::Message::ConstSharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(requests->mutex);
      requests->messages.push_back(*msg);
    });
  REQUIRE(wait_for([&]()
    { return context->node()->mutex_group_request()->get_subscription_count() > 0; }));

  auto begin = [&](std::unordered_set<std::string> groups)
  {
    return robot.schedule_and_wait<std::shared_ptr<LockMutexGroup::Active>>(
      [&, groups = std::move(groups)]()
      {
        // These events normally execute under a TaskManager task. Without
        // this, the idle-robot watchdog correctly withdraws every request.
        context->current_task_id("mutex_lifecycle_test");
        auto state = rmf_task::events::SimpleEventState::make(
          0, "mutex test", "", rmf_task::Event::Status::Standby,
          {}, context->clock());
        LockMutexGroup::Data info{
          groups, "test_map", Eigen::Vector3d(0.0, -10.0, 0.0),
          context->now(),
          std::make_shared<rmf_traffic::PlanId>(
            context->itinerary().assign_plan_id()),
          std::make_shared<rmf_traffic::schedule::Itinerary>(), {},
          rmf_traffic::agv::Plan::Goal(0)};
        return LockMutexGroup::Active::make(
          context, std::move(state), [finished]() { ++*finished; },
          std::move(info));
      });
  };

  SECTION("Cancel and kill withdraw pending requests exactly once")
  {
    auto event = begin({"pending"});
    CHECK(robot.schedule_and_wait<bool>([&]()
      { return context->requesting_mutex_groups().count("pending") == 1; }));
    robot.schedule_and_wait<bool>([&]()
      {
        event->cancel();
        event->kill();
        return true;
      });
    REQUIRE(wait_for([&]() { return requests->released("pending"); }));
    CHECK(robot.schedule_and_wait<bool>([&]()
      { return context->requesting_mutex_groups().empty(); }));
    REQUIRE(wait_for([&]() { return finished->load() == 1; }));
    event.reset();
    CHECK(finished->load() == 1);
  }

  SECTION("Discarding an execution withdraws its pending requests")
  {
    auto event = begin({"discarded"});
    event.reset();
    REQUIRE(wait_for([&]() { return requests->released("discarded"); }));
    CHECK(robot.schedule_and_wait<bool>([&]()
      { return context->requesting_mutex_groups().empty(); }));
    CHECK(finished->load() == 0);
  }

  SECTION("Old cleanup cannot withdraw a replacement's request")
  {
    auto old_event = begin({"shared", "obsolete"});
    auto replacement = begin({"shared"});
    old_event.reset();
    REQUIRE(wait_for([&]() { return requests->released("obsolete"); }));
    CHECK(robot.schedule_and_wait<bool>([&]()
      {
        return context->requesting_mutex_groups().size() == 1
          && context->requesting_mutex_groups().count("shared") == 1;
      }));
    CHECK_FALSE(requests->released("shared"));
    replacement.reset();
    REQUIRE(wait_for([&]() { return requests->released("shared"); }));
  }

  SECTION("Cancellation preserves an acquired lock in a partial acquisition")
  {
    auto event = begin({"occupied", "pending"});
    using States = rmf_fleet_msgs::msg::MutexGroupStates;
    auto publisher = data->ros_node->create_publisher<States>(
      MutexGroupStatesTopicName, rclcpp::QoS(10).reliable().transient_local());
    auto states = robot.schedule_and_wait<States>([&]()
      {
        States result;
        result.assignments.emplace_back();
        auto& assignment = result.assignments.back();
        assignment.group = "occupied";
        assignment.claimant = context->participant_id();
        assignment.claim_time =
          context->requesting_mutex_groups().at("occupied");
        return result;
      });
    publisher->publish(states);
    REQUIRE(wait_for([&]()
      {
        return robot.schedule_and_wait<bool>([&]()
          { return context->locked_mutex_groups().count("occupied") == 1; });
      }));
    robot.schedule_and_wait<bool>([&]() { event->cancel(); return true; });
    REQUIRE(wait_for([&]() { return requests->released("pending"); }));
    CHECK(robot.schedule_and_wait<bool>([&]()
      {
        return context->requesting_mutex_groups().empty()
          && context->locked_mutex_groups().count("occupied") == 1;
      }));
    CHECK_FALSE(requests->released("occupied"));
    event.reset();
    CHECK(robot.schedule_and_wait<bool>([&]()
      { return context->locked_mutex_groups().count("occupied") == 1; }));
  }
}
