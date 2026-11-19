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

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <system_error>
#include <utility>
#include <vector>

#include "pressure_driver/ms5837.hpp"

namespace pressure_driver::test
{

/// The datasheet's example calibration, with its checksum in word 0.
inline Ms5837Calibration datasheet_calibration()
{
  Ms5837Calibration calibration{0, 34982, 36352, 20328, 22354, 26646, 26146};
  calibration[0] = static_cast<uint16_t>(crc4(calibration) << 12);
  return calibration;
}

// The datasheet's example raw pressure (D1) and temperature (D2)
constexpr uint32_t kDatasheetRawPressure = 4958179;
constexpr uint32_t kDatasheetRawTemperature = 6815414;

/// What a simulated MS5837 holds and what it was sent. Shared with the test, which keeps it after the device is gone.
struct SimulatedMs5837
{
  Ms5837Calibration calibration = datasheet_calibration();
  uint32_t raw_pressure = kDatasheetRawPressure;
  uint32_t raw_temperature = kDatasheetRawTemperature;
  std::atomic<bool> fail_transfers{false};
  std::atomic<bool> closed{false};
  std::vector<uint8_t> commands;
};

/// An I2cDevice that answers like an MS5837.
class FakeMs5837Device : public I2cDevice
{
public:
  explicit FakeMs5837Device(std::shared_ptr<SimulatedMs5837> sensor)
  : sensor_(std::move(sensor))
  {}

  ~FakeMs5837Device() override
  {
    sensor_->closed = true;
  }

  void write(const uint8_t * data, std::size_t size) override
  {
    fail_if_asked();
    if (size != 1) {
      throw std::system_error(EINVAL, std::generic_category(), "MS5837 commands are one byte");
    }
    const uint8_t command = data[0];
    sensor_->commands.push_back(command);
    if (command >= 0xA0 && command <= 0xAC) {
      const uint16_t word = sensor_->calibration[(command - 0xA0) / 2];
      reply_ = {static_cast<uint8_t>(word >> 8), static_cast<uint8_t>(word & 0xFF)};
    } else if (command == 0x4A) {
      conversion_ = sensor_->raw_pressure;
    } else if (command == 0x5A) {
      conversion_ = sensor_->raw_temperature;
    } else if (command == 0x00) {
      reply_ = {
        static_cast<uint8_t>(conversion_ >> 16),
        static_cast<uint8_t>(conversion_ >> 8),
        static_cast<uint8_t>(conversion_)};
    }
  }

  void read(uint8_t * data, std::size_t size) override
  {
    fail_if_asked();
    if (size != reply_.size()) {
      throw std::system_error(EIO, std::generic_category(), "unexpected read length");
    }
    for (std::size_t i = 0; i < size; ++i) {
      data[i] = reply_[i];
    }
  }

private:
  void fail_if_asked() const
  {
    if (sensor_->fail_transfers) {
      throw std::system_error(EIO, std::generic_category(), "simulated I2C failure");
    }
  }

  std::shared_ptr<SimulatedMs5837> sensor_;
  std::vector<uint8_t> reply_;
  uint32_t conversion_{0};
};

}  // namespace pressure_driver::test
