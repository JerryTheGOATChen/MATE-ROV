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

#include "pressure_driver/ms5837.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include <linux/i2c-dev.h>

namespace pressure_driver
{
namespace
{

constexpr uint8_t kReset = 0x1E;
constexpr uint8_t kAdcRead = 0x00;
constexpr uint8_t kPromRead = 0xA0;
// Conversions at the highest oversampling ratio (8192)
constexpr uint8_t kConvertPressure = 0x4A;
constexpr uint8_t kConvertTemperature = 0x5A;

constexpr auto kResetTime = std::chrono::milliseconds(10);
// The datasheet's maximum conversion time at OSR 8192 is 18.08 ms; this leaves a margin.
constexpr auto kConversionTime = std::chrono::microseconds(20480);

std::system_error i2c_error(const std::string & what)
{
  return std::system_error(errno, std::generic_category(), what);
}

}  // namespace

LinuxI2cDevice::LinuxI2cDevice(int bus, uint8_t address)
{
  const std::string path = "/dev/i2c-" + std::to_string(bus);
  fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw i2c_error("cannot open " + path);
  }
  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    const auto error = i2c_error("cannot select I2C address " + std::to_string(address) + " on " + path);
    ::close(fd_);
    throw error;
  }
}

LinuxI2cDevice::~LinuxI2cDevice()
{
  ::close(fd_);
}

void LinuxI2cDevice::write(const uint8_t * data, std::size_t size)
{
  if (::write(fd_, data, size) != static_cast<ssize_t>(size)) {
    throw i2c_error("I2C write failed");
  }
}

void LinuxI2cDevice::read(uint8_t * data, std::size_t size)
{
  if (::read(fd_, data, size) != static_cast<ssize_t>(size)) {
    throw i2c_error("I2C read failed");
  }
}

Ms5837Reading compensate(const Ms5837Calibration & calibration, uint32_t raw_pressure, uint32_t raw_temperature)
{
  // Datasheet integer arithmetic. C1..C6 are calibration words 1..6.
  const int64_t c1 = calibration[1];
  const int64_t c2 = calibration[2];
  const int64_t c3 = calibration[3];
  const int64_t c4 = calibration[4];
  const int64_t c5 = calibration[5];
  const int64_t c6 = calibration[6];
  const int64_t d1 = raw_pressure;
  const int64_t d2 = raw_temperature;

  const int64_t dt = d2 - c5 * 256;
  const int64_t temp = 2000 + dt * c6 / 8388608;  // hundredths of a degree C
  int64_t off = c2 * 65536 + c4 * dt / 128;
  int64_t sens = c1 * 32768 + c3 * dt / 256;

  // Second order compensation
  int64_t temp_correction = 0;
  int64_t off_correction = 0;
  int64_t sens_correction = 0;
  const int64_t below_20c = temp - 2000;
  if (temp < 2000) {
    temp_correction = 3 * dt * dt / 8589934592;
    off_correction = 3 * below_20c * below_20c / 2;
    sens_correction = 5 * below_20c * below_20c / 8;
    if (temp < -1500) {
      const int64_t below_minus_15c = temp + 1500;
      off_correction += 7 * below_minus_15c * below_minus_15c;
      sens_correction += 4 * below_minus_15c * below_minus_15c;
    }
  } else {
    temp_correction = 2 * dt * dt / 137438953472;
    off_correction = below_20c * below_20c / 16;
  }
  off -= off_correction;
  sens -= sens_correction;

  // The datasheet's pressure is in tenths of a millibar (10 Pa); keep the fraction for a finer depth.
  const double pressure_tenth_mbar = static_cast<double>(d1 * sens / 2097152 - off) / 8192.0;
  return {pressure_tenth_mbar * 10.0, static_cast<double>(temp - temp_correction) / 100.0};
}

uint8_t crc4(const Ms5837Calibration & calibration)
{
  std::array<uint16_t, 8> words{};
  for (std::size_t i = 0; i < calibration.size(); ++i) {
    words[i] = calibration[i];
  }
  words[0] &= 0x0FFF;  // the checksum's own bits

  uint16_t remainder = 0;
  for (std::size_t i = 0; i < 16; ++i) {
    const uint16_t word = words[i >> 1];
    remainder ^= (i % 2 == 1) ? (word & 0x00FF) : (word >> 8);
    for (int bit = 0; bit < 8; ++bit) {
      remainder =
        (remainder & 0x8000) ? static_cast<uint16_t>((remainder << 1) ^ 0x3000) : static_cast<uint16_t>(remainder << 1);
    }
  }
  return static_cast<uint8_t>((remainder >> 12) & 0x0F);
}

Ms5837::Ms5837(std::unique_ptr<I2cDevice> device)
: device_(std::move(device))
{
  write_command(kReset);
  std::this_thread::sleep_for(kResetTime);

  for (std::size_t i = 0; i < calibration_.size(); ++i) {
    write_command(static_cast<uint8_t>(kPromRead + 2 * i));
    uint8_t word[2];
    device_->read(word, sizeof(word));
    calibration_[i] = static_cast<uint16_t>(word[0] << 8 | word[1]);
  }

  if (calibration_[0] >> 12 != crc4(calibration_)) {
    throw std::runtime_error("the MS5837 calibration failed its checksum");
  }
}

Ms5837Reading Ms5837::read()
{
  const uint32_t raw_pressure = convert(kConvertPressure);
  const uint32_t raw_temperature = convert(kConvertTemperature);
  return compensate(calibration_, raw_pressure, raw_temperature);
}

void Ms5837::write_command(uint8_t command)
{
  device_->write(&command, 1);
}

uint32_t Ms5837::convert(uint8_t command)
{
  write_command(command);
  std::this_thread::sleep_for(kConversionTime);
  write_command(kAdcRead);
  uint8_t value[3];
  device_->read(value, sizeof(value));
  return static_cast<uint32_t>(value[0]) << 16 | static_cast<uint32_t>(value[1]) << 8 | value[2];
}

}  // namespace pressure_driver
