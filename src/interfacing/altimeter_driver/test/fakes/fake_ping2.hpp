// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "altimeter_driver/ping2.hpp"

namespace altimeter_driver::test
{

inline std::vector<uint8_t> to_le(uint32_t value, std::size_t size)
{
  std::vector<uint8_t> bytes(size);
  for (std::size_t i = 0; i < size; ++i) {
    bytes[i] = static_cast<uint8_t>(value >> (8 * i));
  }
  return bytes;
}

inline uint32_t from_le(const std::vector<uint8_t> & bytes, std::size_t offset, std::size_t size)
{
  uint32_t value = 0;
  for (std::size_t i = 0; i < size; ++i) {
    value |= static_cast<uint32_t>(bytes[offset + i]) << (8 * i);
  }
  return value;
}

/// What a simulated Ping2 holds and what it was sent. Shared with the test, which keeps it after the device is gone.
struct SimulatedPing2
{
  std::atomic<bool> answers{true};
  std::atomic<bool> keeps_own_settings{false};
  std::atomic<bool> refuses_distance{false};
  std::atomic<uint16_t> confidence{100};
  std::atomic<uint32_t> speed_of_sound_mm_s{1500000};
  std::atomic<uint16_t> ping_interval_ms{100};
  uint32_t distance_mm = 3210;
  uint32_t scan_start_mm = 300;
  uint32_t scan_length_mm = 9700;
  std::atomic<bool> closed{false};
};

/// A ByteStream that answers like a Ping2.
class FakePing2Stream : public ByteStream
{
public:
  explicit FakePing2Stream(std::shared_ptr<SimulatedPing2> device)
  : device_(std::move(device))
  {}

  ~FakePing2Stream() override
  {
    device_->closed = true;
  }

  void write(const std::vector<uint8_t> & data) override
  {
    for (const uint8_t byte : data) {
      if (auto message = parser_.feed(byte)) {
        handle(*message);
      }
    }
  }

  std::size_t read(uint8_t * data, std::size_t size, std::chrono::milliseconds timeout) override
  {
    std::size_t count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (count < size && !replies_.empty()) {
        data[count++] = replies_.front();
        replies_.pop_front();
      }
    }
    if (count == 0) {
      std::this_thread::sleep_for(std::min(timeout, std::chrono::milliseconds(1)));
    }
    return count;
  }

private:
  void handle(const PingMessage & message)
  {
    switch (message.id) {
      case ping_id::kSetSpeedOfSound:
        if (!device_->keeps_own_settings) {
          device_->speed_of_sound_mm_s = from_le(message.payload, 0, 4);
        }
        break;
      case ping_id::kSetPingInterval:
        if (!device_->keeps_own_settings) {
          device_->ping_interval_ms = static_cast<uint16_t>(from_le(message.payload, 0, 2));
        }
        break;
      case ping_id::kGeneralRequest:
        answer(static_cast<uint16_t>(from_le(message.payload, 0, 2)));
        break;
      default:
        break;
    }
  }

  void answer(uint16_t requested_id)
  {
    if (!device_->answers) {
      return;
    }
    std::vector<uint8_t> payload;
    uint16_t id = requested_id;
    if (requested_id == ping_id::kProtocolVersion) {
      payload = {1, 0, 0, 0};
    } else if (requested_id == ping_id::kSpeedOfSound) {
      payload = to_le(device_->speed_of_sound_mm_s, 4);
    } else if (requested_id == ping_id::kPingInterval) {
      payload = to_le(device_->ping_interval_ms, 2);
    } else if (requested_id == ping_id::kDistance && device_->refuses_distance) {
      id = ping_id::kNack;
      payload = to_le(requested_id, 2);
      const char reason[] = "busy";
      payload.insert(payload.end(), reason, reason + sizeof(reason));  // NUL-terminated, as the device sends it
    } else if (requested_id == ping_id::kDistance) {
      for (const auto & [value, size] : std::vector<std::pair<uint32_t, std::size_t>>{
             {device_->distance_mm, 4},
             {device_->confidence, 2},
             {100, 2},  // transmit duration, us
             {1, 4},  // ping number
             {device_->scan_start_mm, 4},
             {device_->scan_length_mm, 4},
             {0, 4}})  // gain setting
      {
        const auto bytes = to_le(value, size);
        payload.insert(payload.end(), bytes.begin(), bytes.end());
      }
    } else {
      return;
    }
    const auto frame = encode_ping_message(id, payload);
    std::lock_guard<std::mutex> lock(mutex_);
    replies_.insert(replies_.end(), frame.begin(), frame.end());
  }

  std::shared_ptr<SimulatedPing2> device_;
  PingParser parser_;
  std::mutex mutex_;
  std::deque<uint8_t> replies_;
};

}  // namespace altimeter_driver::test
