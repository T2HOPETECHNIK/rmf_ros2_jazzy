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
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>
#include <rmf_traffic_msgs/msg/negotiation_conclusion.hpp>
#include <rmf_traffic_msgs/msg/negotiation_forfeit.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <deque>
#include <thread>

namespace {
using Negotiation = rmf_traffic_ros2::schedule::Negotiation;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class TestWorker : public Negotiation::Worker
{
public:
  void schedule(std::function<void()> job) final
  {
    std::lock_guard<std::mutex> lock(mutex);
    jobs.push_back(std::move(job));
  }

  void run_one()
  {
    std::function<void()> job;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (jobs.empty())
        return;
      job = std::move(jobs.front());
      jobs.pop_front();
    }
    job();
  }
private:
  std::mutex mutex;
  std::deque<std::function<void()>> jobs;
};
} // anonymous namespace

TEST_CASE("Shared negotiation interface admits seven local robots safely",
  "[admission_integration]")
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "test_negotiation_admission", rclcpp::NodeOptions().context(context));
  rclcpp::ExecutorOptions options;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);
  executor.add_node(node);
  auto worker = std::make_shared<TestWorker>();
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const bool asynchronous = GENERATE(false, true);
  auto manager = std::make_unique<Negotiation>(
    *node, database, asynchronous ? worker : nullptr);
  manager->set_response_limits(2, 32);
  manager->timeout_duration(1500ms);

  std::vector<std::shared_ptr<void>> licenses;
  std::vector<Negotiation::ResponderPtr> running;
  std::vector<Negotiation::TableViewPtr> viewers;
  std::vector<Clock::time_point> deadlines;
  using Notice = rmf_traffic_msgs::msg::NegotiationNotice;
  using Conclusion = rmf_traffic_msgs::msg::NegotiationConclusion;
  using Forfeit = rmf_traffic_msgs::msg::NegotiationForfeit;
  Notice notice;
  notice.conflict_version = 12345;
  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(0.5);
  using Description = rmf_traffic::schedule::ParticipantDescription;
  for (int i = 0; i < 7; ++i)
  {
    const auto id = database->register_participant(Description(
          "robot_" + std::to_string(i), "same_adapter",
          Description::Rx::Responsive, rmf_traffic::Profile{shape})).id();
    notice.participants.push_back(id);
    licenses.push_back(manager->register_negotiator(id,
      [&](Negotiation::TableViewPtr viewer, Negotiation::ResponderPtr responder)
      {
        const auto deadline = Negotiation::response_deadline(responder);
        REQUIRE(deadline.has_value());
        deadlines.push_back(*deadline);
        running.push_back(std::move(responder));
        viewers.push_back(std::move(viewer));
      }, nullptr));
  }

  const auto qos = rclcpp::ServicesQoS().reliable().keep_last(1000);
  auto notices = node->create_publisher<Notice>(
    rmf_traffic_ros2::NegotiationNoticeTopicName, qos);
  auto conclusions = node->create_publisher<Conclusion>(
    rmf_traffic_ros2::NegotiationConclusionTopicName, qos);
  std::size_t forfeits = 0;
  auto forfeits_sub = node->create_subscription<Forfeit>(
    rmf_traffic_ros2::NegotiationForfeitTopicName, qos,
    [&](Forfeit::UniquePtr msg)
    {
      if (msg->conflict_version == notice.conflict_version)
        ++forfeits;
    });
  bool concluded = false;
  manager->on_conclusion([&](uint64_t, bool) { concluded = true; });

  const auto until = [&](const std::function<bool()>& ready,
    Clock::duration duration = 3s)
  {
    const auto stop = Clock::now() + duration;
    while (!ready() && Clock::now() < stop)
    {
      executor.spin_some();
      worker->run_one();
      std::this_thread::sleep_for(1ms);
    }
    return ready();
  };
  const auto advance = [&](Clock::duration duration)
  {
    until([] { return false; }, duration);
  };
  REQUIRE(until([&] { return notices->get_subscription_count() > 0; }));
  notices->publish(notice);
  REQUIRE(until([&] { return running.size() == 2; }));

  SECTION("Duplicate requests and timeout notification retain active slots")
  {
    notices->publish(notice);
    bool heartbeat = false;
    worker->schedule([&] { heartbeat = true; });
    REQUIRE(until([&] { return heartbeat; }));
    advance(80ms);
    REQUIRE(running.size() == 2);
    CHECK(forfeits == 0);

    running[0]->forfeit({});
    advance(80ms);
    REQUIRE(running.size() == 2);
    running[0].reset();
    REQUIRE(until([&] { return running.size() == 3; }));
    // Queue time is not replaced with a fresh 1500 ms allowance.
    CHECK(deadlines[2] <= deadlines[0] + 50ms);

    Conclusion conclusion;
    conclusion.conflict_version = notice.conflict_version;
    conclusion.resolved = false;
    conclusions->publish(conclusion);
    REQUIRE(until([&] { return concluded; }));
    running.clear();
    advance(100ms);
    CHECK(deadlines.size() == 3);
  }

  SECTION("Shared timeout timer preserves the queued response deadline")
  {
    // Keep both slots occupied long enough to distinguish the original deadline
    // from an incorrectly restarted timeout when the third robot is admitted.
    advance(650ms);
    running[0]->forfeit({});
    running[0].reset();
    REQUIRE(until([&] { return running.size() == 3; }));
    REQUIRE(deadlines[2] <= deadlines[0] + 50ms);

    // Allow for the shared timer's 100 ms polling interval. A fresh 1500 ms
    // timeout at admission would leave the third responder alive beyond this.
    const auto stop = deadlines[2] + 350ms;
    REQUIRE(Clock::now() < stop);
    REQUIRE(until([&] { return forfeits == 7; }, stop - Clock::now()));
    CHECK(running.size() == 3);
  }

  SECTION("Expired queued requests never construct a responder")
  {
    REQUIRE(until([&] { return forfeits == 7; }));
    CHECK(running.size() == 2);
    running.clear();
    advance(100ms);
    CHECK(deadlines.size() == 2);
  }

  SECTION("Children of locally submitted proposals use the same admission gate")
  {
    running[0]->submit(1, {}, nullptr);
    advance(80ms);
    CHECK(running.size() == 2);
    running[0].reset();
    REQUIRE(until([&] { return running.size() == 3; }));
    // The remaining original root requests are ahead of newly created children.
    CHECK(viewers.back()->sequence().size() == 1);
    // Release the remaining queued roots, then require a local child response.
    while (running.size() < 8)
    {
      const auto previous = running.size();
      running.back()->forfeit({});
      running.back().reset();
      REQUIRE(until([&] { return running.size() > previous; }));
    }
    CHECK(viewers.back()->sequence().size() == 2);
  }

  SECTION("Queued worker callbacks and responders survive interface teardown")
  {
    manager.reset();
    running[0]->forfeit({});
    running.clear();
    advance(80ms);
    CHECK(deadlines.size() == 2);
  }

  manager.reset();
  running.clear();
  context->shutdown("test completed");
}
