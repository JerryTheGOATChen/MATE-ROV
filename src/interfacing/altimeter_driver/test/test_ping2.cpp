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

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "altimeter_driver/ping2.hpp"
#include "fakes/fake_ping2.hpp"

using altimeter_driver::encode_ping_message;
using altimeter_driver::Ping2;
using altimeter_driver::PingMessage;
using altimeter_driver::PingParser;
using altimeter_driver::test::FakePing2Stream;
using altimeter_driver::test::SimulatedPing2;
using altimeter_driver::test::to_le;
using Catch::Matchers::ContainsSubstring;
namespace ping_id = altimeter_driver::ping_id;

namespace
{

constexpr auto kReplyTimeout = std::chrono::milliseconds(50);

std::optional<PingMessage> parse(PingParser & parser, const std::vector<uint8_t> & bytes)
{
  std::optional<PingMessage> message;
  for (const uint8_t byte : bytes) {
    if (auto parsed = parser.feed(byte)) {
      message = parsed;
    }
  }
  return message;
}

}  // namespace

TEST_CASE("encode_ping_message frames a request", "[ping_protocol]")
{
  // A general request for the distance message, 1212 = 0x04BC. The checksum is the sum of the other bytes.
  const std::vector<uint8_t> expected{'B', 'R', 2, 0, 6, 0, 0, 0, 0xBC, 0x04, 0x5C, 0x01};
  REQUIRE(encode_ping_message(ping_id::kGeneralRequest, to_le(ping_id::kDistance, 2)) == expected);
}

TEST_CASE("PingParser reassembles messages from a stream", "[ping_protocol]")
{
  PingParser parser;
  const auto frame = encode_ping_message(ping_id::kSetSpeedOfSound, to_le(1482000, 4));

  SECTION("A frame decodes to its id and payload")
  {
    const auto message = parse(parser, frame);
    REQUIRE(message.has_value());
    REQUIRE(message->id == ping_id::kSetSpeedOfSound);
    REQUIRE(message->payload == to_le(1482000, 4));
  }

  SECTION("Bytes before the frame are skipped")
  {
    std::vector<uint8_t> bytes{'x', 'B', 'B', 'y'};
    bytes.insert(bytes.end(), frame.begin(), frame.end());
    REQUIRE(parse(parser, bytes).has_value());
  }

  SECTION("A frame with a bad checksum is dropped, and the next one still decodes")
  {
    auto corrupt = frame;
    corrupt.back() ^= 0xFF;
    REQUIRE_FALSE(parse(parser, corrupt).has_value());
    REQUIRE(parse(parser, frame).has_value());
  }
}

TEST_CASE("Ping2 talks to the device", "[ping2]")
{
  auto device = std::make_shared<SimulatedPing2>();

  SECTION("It fails to open a device that does not answer")
  {
    device->answers = false;
    REQUIRE_THROWS_WITH(
      Ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout), ContainsSubstring("did not answer"));
  }

  SECTION("configure writes both settings")
  {
    Ping2 ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout);
    ping2.configure(1482000, 50);
    REQUIRE(device->speed_of_sound_mm_s == 1482000);
    REQUIRE(device->ping_interval_ms == 50);
  }

  SECTION("configure fails when the device keeps its own settings")
  {
    device->keeps_own_settings = true;
    Ping2 ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout);
    REQUIRE_THROWS_WITH(ping2.configure(1482000, 50), ContainsSubstring("the device reports 1500000"));
  }

  SECTION("configure fails without a reply")
  {
    Ping2 ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout);
    device->answers = false;
    REQUIRE_THROWS_WITH(ping2.configure(1482000, 50), ContainsSubstring("no reply"));
  }

  SECTION("get_distance returns the reading")
  {
    device->confidence = 93;
    Ping2 ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout);
    const auto reading = ping2.get_distance();
    REQUIRE(reading.has_value());
    REQUIRE(reading->distance_mm == 3210);
    REQUIRE(reading->confidence == 93);
    REQUIRE(reading->scan_start_mm == 300);
    REQUIRE(reading->scan_length_mm == 9700);
  }

  SECTION("get_distance is empty without a reply")
  {
    Ping2 ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout);
    device->answers = false;
    REQUIRE_FALSE(ping2.get_distance().has_value());
  }

  SECTION("A refused request is reported")
  {
    device->refuses_distance = true;
    Ping2 ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout);
    REQUIRE_THROWS_WITH(ping2.get_distance(), ContainsSubstring("refused message 1212: busy"));
  }

  SECTION("Destroying it closes the stream")
  {
    {
      Ping2 ping2(std::make_unique<FakePing2Stream>(device), kReplyTimeout);
    }
    REQUIRE(device->closed);
  }
}
