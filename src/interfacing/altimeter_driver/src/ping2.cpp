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

#include "altimeter_driver/ping2.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace altimeter_driver
{
namespace
{

constexpr uint8_t kStart1 = 'B';
constexpr uint8_t kStart2 = 'R';
constexpr std::size_t kHeaderSize = 8;  // start bytes, payload length, id, source, destination
constexpr std::size_t kChecksumSize = 2;
// Larger than any Ping2 message; a bigger length means the parser is out of step with the stream
constexpr std::size_t kMaxPayloadSize = 1024;
constexpr std::size_t kDistancePayloadSize = 24;

uint32_t read_le(const std::vector<uint8_t> & bytes, std::size_t offset, std::size_t size)
{
  uint32_t value = 0;
  for (std::size_t i = 0; i < size; ++i) {
    value |= static_cast<uint32_t>(bytes[offset + i]) << (8 * i);
  }
  return value;
}

std::vector<uint8_t> to_le(uint32_t value, std::size_t size)
{
  std::vector<uint8_t> bytes(size);
  for (std::size_t i = 0; i < size; ++i) {
    bytes[i] = static_cast<uint8_t>(value >> (8 * i));
  }
  return bytes;
}

uint16_t checksum(const std::vector<uint8_t> & bytes, std::size_t size)
{
  uint32_t sum = 0;
  for (std::size_t i = 0; i < size; ++i) {
    sum += bytes[i];
  }
  return static_cast<uint16_t>(sum);
}

speed_t to_speed(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::invalid_argument("unsupported baud rate " + std::to_string(baudrate));
  }
}

std::system_error serial_error(const std::string & what)
{
  return std::system_error(errno, std::generic_category(), what);
}

}  // namespace

std::vector<uint8_t> encode_ping_message(uint16_t id, const std::vector<uint8_t> & payload)
{
  std::vector<uint8_t> frame{kStart1, kStart2};
  const auto length = to_le(static_cast<uint32_t>(payload.size()), 2);
  const auto id_bytes = to_le(id, 2);
  frame.insert(frame.end(), length.begin(), length.end());
  frame.insert(frame.end(), id_bytes.begin(), id_bytes.end());
  frame.push_back(0);  // source device
  frame.push_back(0);  // destination device
  frame.insert(frame.end(), payload.begin(), payload.end());
  const auto sum = to_le(checksum(frame, frame.size()), kChecksumSize);
  frame.insert(frame.end(), sum.begin(), sum.end());
  return frame;
}

std::optional<PingMessage> PingParser::feed(uint8_t byte)
{
  frame_.push_back(byte);
  if (frame_.size() == 1) {
    if (byte != kStart1) {
      frame_.clear();
    }
    return std::nullopt;
  }
  if (frame_.size() == 2) {
    if (byte != kStart2) {
      frame_.clear();
      if (byte == kStart1) {
        frame_.push_back(byte);
      }
    }
    return std::nullopt;
  }
  if (frame_.size() < kHeaderSize) {
    return std::nullopt;
  }

  const std::size_t payload_size = read_le(frame_, 2, 2);
  if (payload_size > kMaxPayloadSize) {
    frame_.clear();
    return std::nullopt;
  }
  const std::size_t checksum_offset = kHeaderSize + payload_size;
  if (frame_.size() < checksum_offset + kChecksumSize) {
    return std::nullopt;
  }

  std::optional<PingMessage> message;
  if (read_le(frame_, checksum_offset, kChecksumSize) == checksum(frame_, checksum_offset)) {
    message = PingMessage{
      static_cast<uint16_t>(read_le(frame_, 4, 2)),
      std::vector<uint8_t>(frame_.begin() + kHeaderSize, frame_.begin() + checksum_offset)};
  }
  frame_.clear();
  return message;
}

SerialPort::SerialPort(const std::string & path, int baudrate)
{
  const speed_t speed = to_speed(baudrate);
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    throw serial_error("cannot open " + path);
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    const auto error = serial_error("cannot read the settings of " + path);
    ::close(fd_);
    throw error;
  }
  ::cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);
  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const auto error = serial_error("cannot configure " + path);
    ::close(fd_);
    throw error;
  }
  ::tcflush(fd_, TCIOFLUSH);
}

SerialPort::~SerialPort()
{
  ::close(fd_);
}

void SerialPort::write(const std::vector<uint8_t> & data)
{
  std::size_t written = 0;
  while (written < data.size()) {
    const ssize_t count = ::write(fd_, data.data() + written, data.size() - written);
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw serial_error("serial write failed");
    }
    written += static_cast<std::size_t>(count);
  }
}

std::size_t SerialPort::read(uint8_t * data, std::size_t size, std::chrono::milliseconds timeout)
{
  pollfd poll_fd{fd_, POLLIN, 0};
  const int ready = ::poll(&poll_fd, 1, static_cast<int>(timeout.count()));
  if (ready < 0) {
    if (errno == EINTR) {
      return 0;
    }
    throw serial_error("serial poll failed");
  }
  if (ready == 0) {
    return 0;
  }
  if (poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::system_error(EIO, std::generic_category(), "the serial port closed");
  }

  const ssize_t count = ::read(fd_, data, size);
  if (count < 0) {
    if (errno == EINTR || errno == EAGAIN) {
      return 0;
    }
    throw serial_error("serial read failed");
  }
  return static_cast<std::size_t>(count);
}

Ping2::Ping2(std::unique_ptr<ByteStream> stream, std::chrono::milliseconds reply_timeout)
: stream_(std::move(stream))
, reply_timeout_(reply_timeout)
{
  if (!request(ping_id::kProtocolVersion)) {
    throw std::runtime_error("the device did not answer");
  }
}

void Ping2::configure(uint32_t speed_of_sound_mm_s, uint16_t ping_interval_ms)
{
  write_setting("speed of sound", ping_id::kSetSpeedOfSound, ping_id::kSpeedOfSound, speed_of_sound_mm_s, 4);
  write_setting("ping interval", ping_id::kSetPingInterval, ping_id::kPingInterval, ping_interval_ms, 2);
}

std::optional<Ping2Distance> Ping2::get_distance()
{
  const auto reply = request(ping_id::kDistance);
  if (!reply || reply->payload.size() < kDistancePayloadSize) {
    return std::nullopt;
  }
  const auto & payload = reply->payload;
  return Ping2Distance{
    read_le(payload, 0, 4),
    static_cast<uint16_t>(read_le(payload, 4, 2)),
    read_le(payload, 12, 4),
    read_le(payload, 16, 4)};
}

std::optional<PingMessage> Ping2::request(uint16_t id)
{
  stream_->write(encode_ping_message(ping_id::kGeneralRequest, to_le(id, 2)));

  // The Ping2 only sends when asked, so nothing that matters follows the reply in the same read
  const auto deadline = std::chrono::steady_clock::now() + reply_timeout_;
  uint8_t buffer[64];
  for (auto now = std::chrono::steady_clock::now(); now < deadline; now = std::chrono::steady_clock::now()) {
    const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
    const std::size_t count = stream_->read(buffer, sizeof(buffer), remaining);
    for (std::size_t i = 0; i < count; ++i) {
      auto message = parser_.feed(buffer[i]);
      if (!message) {
        continue;
      }
      if (message->id == id) {
        return message;
      }
      if (message->id == ping_id::kNack && message->payload.size() >= 2 && read_le(message->payload, 0, 2) == id) {
        std::string reason(message->payload.begin() + 2, message->payload.end());
        reason.erase(std::find(reason.begin(), reason.end(), '\0'), reason.end());
        throw std::runtime_error("the device refused message " + std::to_string(id) + ": " + reason);
      }
    }
  }
  return std::nullopt;
}

void Ping2::write_setting(const std::string & name, uint16_t set_id, uint16_t get_id, uint32_t value, std::size_t size)
{
  stream_->write(encode_ping_message(set_id, to_le(value, size)));

  const auto reply = request(get_id);
  if (!reply) {
    throw std::runtime_error("no reply when setting the " + name);
  }
  if (reply->payload.size() < size) {
    throw std::runtime_error("a malformed reply when setting the " + name);
  }
  const uint32_t reported = read_le(reply->payload, 0, size);
  if (reported != value) {
    throw std::runtime_error(
      "asked for a " + name + " of " + std::to_string(value) + ", the device reports " + std::to_string(reported));
  }
}

}  // namespace altimeter_driver
