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

#ifndef RMF_TRAFFIC_ROS2__SCHEDULE__RESPONSEADMISSION_HPP
#define RMF_TRAFFIC_ROS2__SCHEDULE__RESPONSEADMISSION_HPP

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <tuple>
#include <cstdint>
#include <stdexcept>

namespace rmf_traffic_ros2 {
namespace schedule {
namespace detail {

// Bounds construction of expensive asynchronous responders. A lease is retained
// by the responder for its entire lifetime, including work draining after timeout.
class ResponseAdmission
{
public:
  using Clock = std::chrono::steady_clock;
  using Key = std::tuple<uint64_t, uintptr_t, uint64_t>;
  using Lease = std::shared_ptr<void>;
  struct Request
  {
    Key key;
    Clock::time_point deadline;
    std::function<bool()> valid;
    std::function<void(Lease, Clock::time_point)> start;
    std::function<void()> expire;
  };

  ResponseAdmission(std::size_t active_limit, std::size_t queue_limit)
  : _active_limit(active_limit), _queue_limit(queue_limit)
  {
    if (active_limit == 0 || queue_limit == 0)
      throw std::invalid_argument(
              "Negotiation admission limits must be positive");
  }

  // Never invokes user callbacks while holding the queue mutex.
  void enqueue(Request request, Clock::time_point now = Clock::now())
  {
    prune(now);
    bool full = false;
    {
      std::lock_guard<std::mutex> lock(_state->mutex);
      if (_state->keys.count(request.key))
        return;
      full = _pending.size() >= _queue_limit;
      if (!full)
      {
        _state->keys.insert(request.key);
        _pending.push_back(std::make_shared<Request>(std::move(request)));
      }
    }
    if (full && request.valid())
      request.expire();
  }

  bool has_pending() const
  {
    std::lock_guard<std::mutex> lock(_state->mutex);
    return !_pending.empty();
  }

  void drain(Clock::time_point now = Clock::now())
  {
    prune(now);
    // A bounded batch also prevents synchronous responders from monopolizing
    // the worker by continuously appending children during this call.
    for (std::size_t i = 0; i < _active_limit; ++i)
    {
      std::shared_ptr<Request> request;
      std::shared_ptr<Token> token;
      {
        std::lock_guard<std::mutex> lock(_state->mutex);
        if (_state->active >= _active_limit || _pending.empty())
          return;
        request = _pending.front();
        _pending.pop_front();
        token = std::make_shared<Token>(_state, request->key);
        ++_state->active;
      }
      if (!request->valid())
        continue;
      if (request->deadline <= now)
        request->expire();
      else
        request->start(std::move(token), request->deadline);
    }
  }

private:
  struct State
  {
    std::mutex mutex;
    std::set<Key> keys;
    std::size_t active = 0;
  };

  struct Token
  {
    std::shared_ptr<State> state;
    Key key;
    Token(std::shared_ptr<State> state_, Key key_)
    : state(std::move(state_)), key(key_) {}
    ~Token()
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      --state->active;
      state->keys.erase(key);
    }
  };

  void prune(Clock::time_point now)
  {
    std::deque<std::shared_ptr<Request>> snapshot;
    {
      std::lock_guard<std::mutex> lock(_state->mutex);
      snapshot = _pending;
    }
    for (const auto& request : snapshot)
    {
      const bool valid = request->valid();
      if (valid && now < request->deadline)
        continue;
      {
        std::lock_guard<std::mutex> lock(_state->mutex);
        const auto it = std::find(_pending.begin(), _pending.end(), request);
        if (it == _pending.end())
          continue;
        _pending.erase(it);
        _state->keys.erase(request->key);
      }
      if (valid)
        request->expire();
    }
  }

  const std::size_t _active_limit;
  const std::size_t _queue_limit;
  std::shared_ptr<State> _state = std::make_shared<State>();
  std::deque<std::shared_ptr<Request>> _pending;
};

} // namespace detail
} // namespace schedule
} // namespace rmf_traffic_ros2
#endif
