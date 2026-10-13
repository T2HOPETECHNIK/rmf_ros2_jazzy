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
#include <agv/internal_FleetUpdateHandle.hpp>
#include <services/TaskPlanningDeadline.hpp>
#include <services/BidDeadline.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
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

TEST_CASE("Task allocation deadlines preserve interruption across copies",
  "[task_allocation]")
{
  using rmf_fleet_adapter::services::TaskPlanningDeadline;
  TaskPlanningDeadline expired(0s);
  CHECK(expired.interrupted());
  bool cancel = false;
  TaskPlanningDeadline deadline(10s, [&]() { return cancel; });
  const auto copy = deadline;
  CHECK_FALSE(copy.interrupted());
  cancel = true;
  CHECK(copy.interrupted());
  cancel = false;
  CHECK(deadline.interrupted());
}

TEST_CASE_METHOD(MockAdapterFixture,
  "Task allocation worker does not block mutex heartbeats",
  "[task_allocation]")
{
  using namespace rmf_fleet_adapter;
  const auto robot = add_robot();
  const auto context = robot.context;
  auto messages = std::make_shared<std::atomic<int>>(0);
  auto subscription = data->ros_node->create_subscription<Requests::Message>(
    MutexGroupRequestTopicName, rclcpp::QoS(100).reliable().transient_local(),
    [messages](Requests::Message::ConstSharedPtr msg)
    { if (msg->group == "heartbeat" && msg->mode == Requests::Message::MODE_LOCK) ++*messages; });
  REQUIRE(wait_for([&]()
    { return context->node()->mutex_group_request()->get_subscription_count() > 0; }));
  robot.schedule_and_wait<bool>([&]()
    {
      context->current_task_id("busy_planner_test");
      context->request_mutex_groups({"heartbeat"}, context->now());
      return true;
    });
  REQUIRE(wait_for([&]() { return messages->load() > 0; }));
  const int before = messages->load();
  auto started = std::make_shared<std::atomic_bool>(false);
  auto done = std::make_shared<std::atomic_bool>(false);
  auto& impl = agv::FleetUpdateHandle::Implementation::get(*data->fleet);
  auto thread_promise = std::make_shared<std::promise<std::thread::id>>();
  auto thread_future = thread_promise->get_future();
  impl.reassignment_worker.schedule([thread_promise](const auto&)
    { thread_promise->set_value(std::this_thread::get_id()); });
  REQUIRE(thread_future.wait_for(5s) == std::future_status::ready);
  const auto allocation_thread = thread_future.get();
  // Exercise every thread in RxCpp's shared pool. A dedicated allocation
  // worker must not coincide with any of these callback threads.
  const auto pool_size = std::max(std::thread::hardware_concurrency(), 4u);
  for (unsigned i = 0; i < pool_size; ++i)
  {
    auto promise = std::make_shared<std::promise<std::thread::id>>();
    auto future = promise->get_future();
    auto pool_worker = rxcpp::schedulers::make_event_loop().create_worker();
    pool_worker.schedule([promise](const auto&)
      { promise->set_value(std::this_thread::get_id()); });
    REQUIRE(future.wait_for(5s) == std::future_status::ready);
    CHECK(future.get() != allocation_thread);
  }
  impl.reassignment_worker.schedule([started, done](const auto&)
    {
      started->store(true);
      // Simulates an expensive estimate that cannot check its interrupter.
      std::this_thread::sleep_for(4s);
      done->store(true);
    });
  REQUIRE(wait_for([&]() { return started->load(); }));
  REQUIRE(wait_for([&]() { return messages->load() > before; }));
  CHECK_FALSE(done->load());
  REQUIRE(wait_for([&]() { return done->load(); }));
}

TEST_CASE("Bid deadlines include queueing and reserve publication time",
  "[bid_deadline]")
{
  using rmf_fleet_adapter::services::BidDeadline;
  using rmf_fleet_adapter::services::TaskPlanningDeadline;
  const auto received = BidDeadline::Clock::now();
  const BidDeadline deadline(received, 5s);
  CHECK(deadline.planning_deadline() == received + 4s);
  CHECK(deadline.can_plan(received + 3s));
  CHECK_FALSE(deadline.can_plan(received + 4s));
  CHECK(deadline.can_respond(received + 4s));
  CHECK_FALSE(deadline.can_respond(received + 4500ms));

  // Constructing a planning interrupter after a queue delay must not reset
  // the clock, even if no search expansions have happened yet.
  const BidDeadline queued(received - 6s, 5s);
  TaskPlanningDeadline expired(queued.planning_deadline());
  CHECK(expired.interrupted());
  CHECK_FALSE(queued.can_respond());
  CHECK_FALSE(BidDeadline(received, 0s).can_plan(received));
  CHECK_FALSE(BidDeadline(received, -1s).can_respond(received));
  CHECK(BidDeadline(received, 100ms).planning_deadline() == received + 50ms);
  CHECK(BidDeadline(received, 60s).planning_deadline() == received + 10s);
}

TEST_CASE_METHOD(MockAdapterFixture,
  "Expired bids never publish or cache an allocation", "[bid_deadline]")
{
  using namespace rmf_fleet_adapter;
  using namespace rmf_battery::agv;
  // 0: timely response, 1: queue behind reassignment, 2: delayed result delivery.
  const int delay_mode = GENERATE(0, 1, 2);
  auto battery = std::make_shared<BatterySystem>(
    *BatterySystem::make(24.0, 40.0, 8.8));
  auto motion = std::make_shared<SimpleMotionPowerSink>(
    *battery, *MechanicalSystem::make(70.0, 40.0, 0.22));
  auto device = std::make_shared<SimpleDevicePowerSink>(
    *battery, *PowerSystem::make(20.0));
  REQUIRE(data->fleet->set_task_planner_params(
      battery, motion, device, device, 0.2, 1.0, false));
  data->fleet->consider_patrol_requests([](const auto&, auto& confirmation)
    { confirmation.accept(); });
  const auto robot = add_robot();
  auto responses = std::make_shared<std::atomic<int>>(0);
  auto proposals = std::make_shared<std::atomic<int>>(0);
  auto& impl = agv::FleetUpdateHandle::Implementation::get(*data->fleet);
  auto release = std::make_shared<std::promise<void>>();
  const auto released = release->get_future().share();
  if (delay_mode == 1)
  {
    impl.reassignment_worker.schedule([released](const auto&)
      { released.wait_for(2s); });
  }

  robot.schedule_and_wait<bool>([&]()
    {
      rmf_task_msgs::msg::BidNotice notice;
      notice.task_id = "expiry_test";
      notice.time_window.sec = delay_mode == 0 ? 5 : 0;
      notice.time_window.nanosec = delay_mode == 0 ? 0 : 100000000;
      notice.request = R"({"category":"patrol","description":{"places":[0,1],"rounds":1}})";
      impl.bid_notice_cb(notice, [responses, proposals](const auto& result)
        {
          ++*responses;
          if (result.proposal)
            ++*proposals;
        });
      if (delay_mode == 2)
      {
        // Keep the result callback queued on the adapter worker until expiry.
        auto planned = std::make_shared<std::promise<void>>();
        auto done = planned->get_future();
        impl.reassignment_worker.schedule([planned](const auto&)
          { planned->set_value(); });
        done.wait_for(2s);
        std::this_thread::sleep_for(200ms);
      }
      return true;
    });
  if (delay_mode == 1)
    std::this_thread::sleep_for(200ms);
  release->set_value();

  // Drain allocation work and then the adapter's result callbacks.
  auto drained = std::make_shared<std::promise<void>>();
  auto done = drained->get_future();
  impl.reassignment_worker.schedule([drained, worker = impl.worker](const auto&)
    {
      worker.schedule([drained](const auto&) { drained->set_value(); });
    });
  REQUIRE(done.wait_for(5s) == std::future_status::ready);
  if (delay_mode == 0)
  {
    REQUIRE(wait_for([&]() { return responses->load() == 1; }));
    CHECK(proposals->load() == 1);
  }
  else
  {
    CHECK(responses->load() == 0);
  }
  CHECK(robot.schedule_and_wait<bool>([&]()
    { return impl.bid_notice_assignments.count("expiry_test")
      == (delay_mode == 0 ? 1u : 0u); }));
}
