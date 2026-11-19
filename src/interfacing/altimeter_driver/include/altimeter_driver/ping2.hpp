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

// Blue Robotics Ping2 echosounder over a serial port, using the Ping protocol:
// https://docs.bluerobotics.com/ping-protocol/

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace altimeter_driver
{

/// The Ping protocol message IDs this driver uses.
namespace ping_id
{
constexpr uint16_t kNack = 2;
constexpr uint16_t kProtocolVersion = 5;
constexpr uint16_t kGeneralRequest = 6;
constexpr uint16_t kSetSpeedOfSound = 1002;
constexpr uint16_t kSetPingInterval = 1004;
constexpr uint16_t kSpeedOfSound = 1203;
constexpr uint16_t kPingInterval = 1206;
constexpr uint16_t kDistance = 1212;
}  // namespace ping_id

struct PingMessage
{
  uint16_t id;
  std::vector<uint8_t> payload;
};

/// Frames a message: the "BR" header, payload length, id, source and destination 0, payload, checksum.
std::vector<uint8_t> encode_ping_message(uint16_t id, const std::vector<uint8_t> & payload = {});

/// Reassembles messages from a byte stream, dropping bytes that are not part of a valid frame.
class PingParser
{
public:
  /// Adds one byte; returns the message it completes, if any.
  std::optional<PingMessage> feed(uint8_t byte);

private:
  std::vector<uint8_t> frame_;
};

/// A byte stream to the device. Failures throw std::system_error.
class ByteStream
{
public:
  virtual ~ByteStream() = default;

  virtual void write(const std::vector<uint8_t> & data) = 0;

  /// Reads up to `size` bytes, waiting at most `timeout` for the first; returns how many were read.
  virtual std::size_t read(uint8_t * data, std::size_t size, std::chrono::milliseconds timeout) = 0;
};

/// A raw 8N1 serial port.
class SerialPort : public ByteStream
{
public:
  /// @throws std::invalid_argument for an unsupported baud rate, std::system_error if the port cannot be set up
  SerialPort(const std::string & path, int baudrate);
  ~SerialPort() override;

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void write(const std::vector<uint8_t> & data) override;
  std::size_t read(uint8_t * data, std::size_t size, std::chrono::milliseconds timeout) override;

private:
  int fd_;
};

/// One reading from the Ping2's `distance` message. Distances are in mm along the beam.
struct Ping2Distance
{
  uint32_t distance_mm;
  uint16_t confidence;  // percent
  uint32_t scan_start_mm;
  uint32_t scan_length_mm;
};

/// A Ping2. Construction checks that the device answers.
class Ping2
{
public:
  static constexpr std::chrono::milliseconds kDefaultReplyTimeout{500};

  /// @throws std::runtime_error if the device does not answer, std::system_error on a stream failure
  explicit Ping2(std::unique_ptr<ByteStream> stream, std::chrono::milliseconds reply_timeout = kDefaultReplyTimeout);

  /// Writes the speed of sound (mm/s) and ping interval (ms), and checks the device reports them back.
  /// @throws std::runtime_error if it does not, std::system_error on a stream failure
  void configure(uint32_t speed_of_sound_mm_s, uint16_t ping_interval_ms);

  /// Requests a reading; empty if the device does not reply in time.
  /// @throws std::system_error on a stream failure
  std::optional<Ping2Distance> get_distance();

private:
  /// Asks for message `id` and waits for it. @throws std::runtime_error if the device refuses it
  std::optional<PingMessage> request(uint16_t id);
  void write_setting(const std::string & name, uint16_t set_id, uint16_t get_id, uint32_t value, std::size_t size);

  std::unique_ptr<ByteStream> stream_;
  std::chrono::milliseconds reply_timeout_;
  PingParser parser_;
};

}  // namespace altimeter_driver
