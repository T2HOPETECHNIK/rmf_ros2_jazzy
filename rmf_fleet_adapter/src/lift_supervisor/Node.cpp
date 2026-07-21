/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "Node.hpp"

#include <rmf_fleet_adapter/StandardNames.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>

namespace rmf_fleet_adapter {
namespace lift_supervisor {
int publish_count = 0;

//==============================================================================
void Node::_publish_end_session(LiftRequest::UniquePtr& request)
{
  if (!request)
    return;

  if (request->request_type != LiftRequest::REQUEST_END_SESSION)
  {
    auto end_request = std::make_unique<LiftRequest>();
    end_request->lift_name = request->lift_name;
    end_request->destination_floor = request->destination_floor;
    end_request->session_id = request->session_id;
    end_request->request_type = LiftRequest::REQUEST_END_SESSION;
    request = std::move(end_request);
  }

  request->request_time = this->now();
  _lift_request_pub->publish(*request);
}

//==============================================================================
void Node::_publish_pending_session(
  const std::string& lift_name,
  LiftRequest::UniquePtr& request)
{
  auto pending_it = _pending_sessions.find(lift_name);
  if (pending_it == _pending_sessions.end() || !pending_it->second)
    return;

  pending_it->second->request_time = this->now();
  _lift_request_pub->publish(*pending_it->second);
  request = std::move(pending_it->second);
  _pending_sessions.erase(pending_it);

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] Published pending lift request to [%s] from lift supervisor",
    request->session_id.c_str(),
    request->destination_floor.c_str());
}

//==============================================================================
Node::Node()
: rclcpp::Node("rmf_lift_supervisor")
{
  const auto default_qos =
    rclcpp::SystemDefaultsQoS().durability_volatile().keep_last(100).reliable();
  const auto transient_qos = rclcpp::SystemDefaultsQoS()
    .reliable().keep_last(100).transient_local();



  _lift_request_pub = create_publisher<LiftRequest>(
    FinalLiftRequestTopicName, transient_qos);

  _adapter_lift_request_sub = create_subscription<LiftRequest>(
    AdapterLiftRequestTopicName, transient_qos,
    [&](LiftRequest::UniquePtr msg)
    {
      _adapter_lift_request_update(std::move(msg));
    });

  _lift_state_sub = create_subscription<LiftState>(
    LiftStateTopicName, default_qos,
    [&](LiftState::UniquePtr msg)
    {
      _lift_state_update(std::move(msg));
    });

  _emergency_notice_pub = create_publisher<EmergencyNotice>(
    rmf_traffic_ros2::EmergencyTopicName, default_qos);
}

//==============================================================================
void Node::_adapter_lift_request_update(LiftRequest::UniquePtr msg)
{

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] Received adapter lift request to [%s] with request type [%d]",
    msg->session_id.c_str(), msg->destination_floor.c_str(), msg->request_type
  );
  auto& curr_request = _active_sessions.insert(
    std::make_pair(msg->lift_name, nullptr)).first->second;

  if (curr_request)
  {
    if (curr_request->session_id == msg->session_id)
    {
      if (curr_request->request_type == LiftRequest::REQUEST_END_SESSION &&
        msg->request_type != LiftRequest::REQUEST_END_SESSION)
      {
        _publish_end_session(curr_request);
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Ignored lift request because the session is being released",
          curr_request->session_id.c_str());
        return;
      }

      publish_count = 0;
      msg->request_time = this->now();
      _lift_request_pub->publish(*msg);
      if (msg->request_type != LiftRequest::REQUEST_END_SESSION)
      {
        curr_request = std::move(msg);
      }
      else
      {
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Published end lift session from lift supervisor",
          msg->session_id.c_str()
        );  
        curr_request = std::move(msg);
      }
    }
    else
    {
      if (msg->request_type != LiftRequest::REQUEST_END_SESSION)
      {
        auto& pending_request = _pending_sessions.insert(
          std::make_pair(msg->lift_name, nullptr)).first->second;
        msg->request_time = this->now();
        pending_request = std::move(msg);
      }

      publish_count++; 
      if (publish_count > 10)
      { 
        _publish_end_session(curr_request);
        publish_count = 0;
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Published end lift session from lift supervisor as new session more than 10 requests received",
          curr_request->session_id.c_str()
        );
      }
      else
      {
        RCLCPP_INFO(
        this->get_logger(),
        "publish_count is less than 10, get new request"
      );
      }
    }
  }
  else
  {
    RCLCPP_INFO(
    this->get_logger(),
    "[%s] Published adapter lift request to [%s] with request type [%d]",
    msg->session_id.c_str(), msg->destination_floor.c_str(), msg->request_type
    );
    _lift_request_pub->publish(*msg);
    if (msg->request_type != LiftRequest::REQUEST_END_SESSION)
    {
      curr_request = std::move(msg);
    }
  }

  // TODO(MXG): Make this more intelligent by scheduling the lift
}

//==============================================================================
void Node::_lift_state_update(LiftState::UniquePtr msg)
{
  auto& lift_request = _active_sessions.insert(
    std::make_pair(msg->lift_name, nullptr)).first->second;

  if (lift_request)
  {
    if (lift_request->request_type == LiftRequest::REQUEST_END_SESSION)
    {
      if (msg->session_id == lift_request->session_id)
      {
        _publish_end_session(lift_request);
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Republished end lift session from lift supervisor",
          lift_request->session_id.c_str());
      }
      else
      {
        RCLCPP_INFO(
          this->get_logger(),
          "[%s] Lift session released from lift supervisor",
          lift_request->session_id.c_str());
        lift_request = nullptr;
        _publish_pending_session(msg->lift_name, lift_request);
      }

      return;
    }

    if ((lift_request->destination_floor != msg->current_floor) ||
      (lift_request->door_state != msg->door_state))
      lift_request->request_time = this->now();
    _lift_request_pub->publish(*lift_request);
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] Published lift request to [%s] from lift supervisor",
      msg->session_id.c_str(), lift_request->destination_floor.c_str()
    );
  }
  else
  {
    // If there are no active sessions going on, we keep publishing session
    // end requests to ensure that the lift is released
    LiftRequest request;
    request.lift_name = msg->lift_name;
    request.destination_floor = msg->current_floor;
    request.session_id = msg->session_id;
    request.request_time = this->now();
    request.request_type = LiftRequest::REQUEST_END_SESSION;

    _lift_request_pub->publish(request);
    RCLCPP_INFO(
      this->get_logger(),
      "No active session and published end session, releasing lift"
    );

    //_lift_request_pub->publish(request);

  }

  // For now, we do not need to publish this.

//  std_msgs::msg::Bool emergency_msg;
//  emergency_msg.data = false;

//  if (LiftState::MODE_FIRE == msg->current_mode
//      || LiftState::MODE_EMERGENCY == msg->current_mode)
//  {
//    emergency_msg.data = true;
//  }

//  _emergency_notice_pub->publish(emergency_msg);
}

} // namespace lift_supervisor
} // namespace rmf_fleet_adapter
