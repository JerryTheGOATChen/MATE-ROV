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

// MS5837-30BA pressure sensor (the Blue Robotics Bar30) over Linux I2C.
//
// The compensation and checksum follow the MS5837-30BA datasheet, as do Blue Robotics' MS5837
// libraries (https://github.com/bluerobotics/ms5837-python), which are distributed under this license:
//
// The MIT License (MIT)
//
// Copyright (c) 2017 Blue Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace pressure_driver
{

/// One device on an I2C bus. Every transfer is a complete transaction; failures throw std::system_error.
class I2cDevice
{
public:
  virtual ~I2cDevice() = default;

  virtual void write(const uint8_t * data, std::size_t size) = 0;
  virtual void read(uint8_t * data, std::size_t size) = 0;
};

/// An I2C device reached through Linux's /dev/i2c-<bus>.
class LinuxI2cDevice : public I2cDevice
{
public:
  /// @throws std::system_error if the bus cannot be opened or the address selected
  LinuxI2cDevice(int bus, uint8_t address);
  ~LinuxI2cDevice() override;

  LinuxI2cDevice(const LinuxI2cDevice &) = delete;
  LinuxI2cDevice & operator=(const LinuxI2cDevice &) = delete;

  void write(const uint8_t * data, std::size_t size) override;
  void read(uint8_t * data, std::size_t size) override;

private:
  int fd_;
};

/// The sensor's seven 16-bit factory calibration words (its PROM). The top 4 bits of word 0 are a checksum.
using Ms5837Calibration = std::array<uint16_t, 7>;

struct Ms5837Reading
{
  double pressure_pa;
  double temperature_c;
};

/// Applies the datasheet's first and second order compensation to the raw pressure (D1) and temperature (D2).
Ms5837Reading compensate(const Ms5837Calibration & calibration, uint32_t raw_pressure, uint32_t raw_temperature);

/// The datasheet's 4-bit checksum of the calibration words. A valid calibration has it in word 0's top 4 bits.
uint8_t crc4(const Ms5837Calibration & calibration);

/// An MS5837-30BA. Construction resets the sensor and loads its calibration.
class Ms5837
{
public:
  static constexpr uint8_t kAddress = 0x76;

  /// @throws std::system_error on an I2C failure, std::runtime_error if the calibration fails its checksum
  explicit Ms5837(std::unique_ptr<I2cDevice> device);

  /// Takes one measurement at the highest oversampling, which takes about 41 ms.
  /// @throws std::system_error on an I2C failure
  Ms5837Reading read();

private:
  void write_command(uint8_t command);
  uint32_t convert(uint8_t command);

  std::unique_ptr<I2cDevice> device_;
  Ms5837Calibration calibration_{};
};

}  // namespace pressure_driver
