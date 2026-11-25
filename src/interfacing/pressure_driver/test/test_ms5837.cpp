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

#include <memory>
#include <stdexcept>
#include <system_error>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "fakes/fake_ms5837.hpp"
#include "pressure_driver/ms5837.hpp"

using Catch::Matchers::WithinAbs;
using pressure_driver::compensate;
using pressure_driver::crc4;
using pressure_driver::Ms5837;
using pressure_driver::test::datasheet_calibration;
using pressure_driver::test::FakeMs5837Device;
using pressure_driver::test::kDatasheetRawPressure;
using pressure_driver::test::kDatasheetRawTemperature;
using pressure_driver::test::SimulatedMs5837;

TEST_CASE("compensate reproduces the datasheet example", "[ms5837]")
{
  const auto reading = compensate(datasheet_calibration(), kDatasheetRawPressure, kDatasheetRawTemperature);
  // The datasheet gives 3999.8 mbar and 19.81 C, both truncated
  REQUIRE_THAT(reading.pressure_pa, WithinAbs(399980.0, 10.0));
  REQUIRE_THAT(reading.temperature_c, WithinAbs(19.81, 0.02));
}

TEST_CASE("crc4 ignores the checksum bits it is compared with", "[ms5837]")
{
  auto calibration = datasheet_calibration();
  const uint8_t crc = crc4(calibration);
  calibration[0] ^= 0xF000;
  REQUIRE(crc4(calibration) == crc);

  calibration[3] ^= 0x0001;
  REQUIRE(crc4(calibration) != crc);
}

TEST_CASE("Ms5837 reads the sensor over I2C", "[ms5837]")
{
  auto sensor = std::make_shared<SimulatedMs5837>();

  SECTION("It resets the sensor, then reads the calibration and both conversions")
  {
    Ms5837 ms5837(std::make_unique<FakeMs5837Device>(sensor));
    const auto reading = ms5837.read();
    REQUIRE_THAT(reading.pressure_pa, WithinAbs(399980.0, 10.0));
    REQUIRE(sensor->commands.front() == 0x1E);
    REQUIRE(sensor->commands.size() == 1 + 7 + 4);
  }

  SECTION("A calibration that fails its checksum is rejected")
  {
    sensor->calibration[2] ^= 0x0100;
    REQUIRE_THROWS_AS(Ms5837(std::make_unique<FakeMs5837Device>(sensor)), std::runtime_error);
  }

  SECTION("An I2C failure is reported")
  {
    Ms5837 ms5837(std::make_unique<FakeMs5837Device>(sensor));
    sensor->fail_transfers = true;
    REQUIRE_THROWS_AS(ms5837.read(), std::system_error);
  }

  SECTION("Destroying it releases the device")
  {
    {
      Ms5837 ms5837(std::make_unique<FakeMs5837Device>(sensor));
    }
    REQUIRE(sensor->closed);
  }
}
