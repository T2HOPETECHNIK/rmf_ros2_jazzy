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

#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include "../../src/rmf_task_ros2/bidding/internal_Auctioneer.hpp"
#include <rmf_utils/catch.hpp>

namespace rmf_task_ros2 {
namespace bidding {

//==============================================================================
auto now = std::chrono::steady_clock::now();

Response::Proposal submission1{
  "fleet1", "", 2.3, 3.4, rmf_traffic::time::apply_offset(now, 5)
};
Response::Proposal submission2{
  "fleet2", "", 3.5, 3.6, rmf_traffic::time::apply_offset(now, 5.5)
};
Response::Proposal submission3{
  "fleet3", "", 0.0, 1.4, rmf_traffic::time::apply_offset(now, 3)
};
Response::Proposal submission4{
  "fleet4", "", 5.0, 5.4, rmf_traffic::time::apply_offset(now, 4)
};
Response::Proposal submission5{
  "fleet5", "", 0.5, 0.8, rmf_traffic::time::apply_offset(now, 3.5)
};

//==============================================================================
SCENARIO("Auctioneer Winner Evaluator", "[Evaluator]")
{
  const auto rcl_context = std::make_shared<rclcpp::Context>();
  rcl_context->init(0, nullptr);
  auto node = rclcpp::Node::make_shared(
    "test_selfbidding", rclcpp::NodeOptions().context(rcl_context));

  auto auctioneer = Auctioneer::make(node,
      [](const auto&, const auto, const auto&) {},
      nullptr);

  WHEN("Least Diff Cost Evaluator")
  {
    auto eval = std::make_shared<LeastFleetDiffCostEvaluator>();
    auctioneer->set_evaluator(eval);

    AND_WHEN("0 submissions")
    {
      std::vector<Response> responses{};
      auto winner = evaluate(*auctioneer, responses);
      REQUIRE(!winner); // no winner
    }
    AND_WHEN("5 submissions")
    {
      std::vector<Response> responses{
        Response{submission1, {}},
        Response{submission2, {}},
        Response{submission3, {}},
        Response{submission4, {}},
        Response{submission5, {}} };
      auto winner = evaluate(*auctioneer, responses);
      REQUIRE(winner->fleet_name == "fleet2"); // least diff cost agent
    }
  }

  WHEN("Least Fleet Cost Evaluator")
  {
    auto eval = std::make_shared<LeastFleetCostEvaluator>();
    auctioneer->set_evaluator(eval);

    AND_WHEN("0 submissions")
    {
      std::vector<Response> submissions{};
      auto winner = evaluate(*auctioneer, submissions);
      REQUIRE(!winner); // no winner
    }
    AND_WHEN("5 submissions")
    {
      std::vector<Response> submissions{
        Response{submission1, {}},
        Response{submission2, {}},
        Response{submission3, {}},
        Response{submission4, {}},
        Response{submission5, {}} };
      auto winner = evaluate(*auctioneer, submissions);
      REQUIRE(winner->fleet_name == "fleet5"); // least diff cost agent
    }
  }

  WHEN("Quickest Finish Time Evaluator")
  {
    auto eval = std::make_shared<QuickestFinishEvaluator>();
    auctioneer->set_evaluator(eval);

    AND_WHEN("0 submissions")
    {
      std::vector<Response> submissions{};
      auto winner = evaluate(*auctioneer, submissions);
      REQUIRE(!winner); // no winner
    }
    AND_WHEN("5 submissions")
    {
      std::vector<Response> submissions{
        Response{submission1, {}},
        Response{submission2, {}},
        Response{submission3, {}},
        Response{submission4, {}},
        Response{submission5, {}} };
      auto winner = evaluate(*auctioneer, submissions);
      REQUIRE(winner->fleet_name == "fleet3"); // least diff cost agent
    }
  }

  rclcpp::shutdown(rcl_context);
}

} // namespace bidding
} // namespace rmf_task_ros2

using namespace rmf_task_ros2::bidding;

TEST_CASE("Auctioneer ignores responses outside the active auction",
  "[late_bid_response]")
{
  const bool has_proposal = GENERATE(false, true);
  const auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  const auto node = rclcpp::Node::make_shared(
    "late_bid_response", rclcpp::NodeOptions().context(context));
  Auctioneer::Implementation auction(
    node->get_node_base_interface(), node->get_node_clock_interface(),
    node->get_node_logging_interface(), node->get_node_timers_interface(),
    node->get_node_topics_interface(), node->get_node_parameters_interface(),
    nullptr, nullptr);
  Response response{std::nullopt, {"planning timed out"}};
  if (has_proposal)
    response = Response{Response::Proposal{
        "fleet", "robot", 0.0, 1.0, rmf_traffic_ros2::convert(node->now())}, {}};
  const auto first_response = convert(response, "first");

  // Includes unsolicited responses and a response after the final bid is popped.
  auction.receive_response(first_response);
  CHECK(auction.open_bid_queue.empty());

  BidNoticeMsg notice;
  notice.task_id = "first";
  notice.time_window.sec = 5;
  auction.request_bid(notice);
  auction.receive_response(first_response);
  CHECK(auction.open_bid_queue.front().responses.empty());
  auction.finish_bidding_process();
  auction.receive_response(convert(response, "unmatched"));
  CHECK(auction.open_bid_queue.front().responses.empty());
  auction.receive_response(first_response);
  CHECK(auction.open_bid_queue.front().responses.size() == 1);

  // Expired, but the periodic closure callback has not run yet.
  auction.open_bid_queue.front().start_time =
    node->now() - rclcpp::Duration::from_seconds(10.0);
  auction.receive_response(first_response);
  CHECK(auction.open_bid_queue.front().responses.size() == 1);
  auction.finish_bidding_process();
  REQUIRE(auction.open_bid_queue.empty());
  auction.receive_response(first_response);
  CHECK(auction.open_bid_queue.empty());

  // An old response must not be attributed to the next auction either.
  notice.task_id = "second";
  auction.request_bid(notice);
  auction.bidding_in_process = false;
  auction.finish_bidding_process();
  auction.receive_response(first_response);
  CHECK(auction.open_bid_queue.front().responses.empty());
  auction.receive_response(convert(response, "second"));
  CHECK(auction.open_bid_queue.front().responses.size() == 1);
  rclcpp::shutdown(context);
}

TEST_CASE("Auctioneer waits for the award ack before the next auction",
  "[award_ack_race]")
{
  const auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  const auto node = rclcpp::Node::make_shared(
    "award_ack_race", rclcpp::NodeOptions().context(context));
  Auctioneer::Implementation auction(
    node->get_node_base_interface(), node->get_node_clock_interface(),
    node->get_node_logging_interface(), node->get_node_timers_interface(),
    node->get_node_topics_interface(), node->get_node_parameters_interface(),
    nullptr, nullptr);

  BidNoticeMsg notice;
  notice.time_window.sec = 5;
  notice.task_id = "first";
  auction.request_bid(notice);
  notice.task_id = "second";
  auction.request_bid(notice);

  // "second" has been queued for longer than its time window.
  const auto old = node->now() - rclcpp::Duration::from_seconds(60.0);
  auction.open_bid_queue.back().start_time = old;

  auction.finish_bidding_process();
  REQUIRE(auction.open_bid_queue.front().started);
  auction.open_bid_queue.front().start_time = old;
  auction.finish_bidding_process();
  REQUIRE(auction.open_bid_queue.size() == 1);

  // The award for "first" is not acknowledged yet, so "second" must neither
  // be judged against its queue time nor announced.
  auction.finish_bidding_process();
  auction.finish_bidding_process();
  REQUIRE(auction.open_bid_queue.size() == 1);
  CHECK_FALSE(auction.open_bid_queue.front().started);

  // The ack arrives: "second" is announced with a fresh start time.
  auction.bidding_in_process = false;
  auction.finish_bidding_process();
  REQUIRE(auction.open_bid_queue.front().started);
  CHECK(auction.bidding_in_process);

  // A duplicate ready_for_next_bid() must not restart the running auction.
  const auto response = convert(
    Response{std::nullopt, {"planning timed out"}}, "second");
  auction.receive_response(response);
  REQUIRE(auction.open_bid_queue.front().responses.size() == 1);
  const auto started_at = rclcpp::Time(auction.open_bid_queue.front().start_time);
  auction.bidding_in_process = false;
  auction.finish_bidding_process();
  CHECK(auction.bidding_in_process);
  CHECK(rclcpp::Time(auction.open_bid_queue.front().start_time) == started_at);
  CHECK(auction.open_bid_queue.front().responses.size() == 1);
  rclcpp::shutdown(context);
}
