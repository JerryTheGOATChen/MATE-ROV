# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""MS5837 pressure sensor (Blue Robotics Bar30 and Bar02) over I2C.

Adapted from https://github.com/bluerobotics/ms5837-python, which is distributed under
this license:

The MIT License (MIT)

Copyright (c) 2017 Blue Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from time import sleep

MODEL_02BA = 0
MODEL_30BA = 1

# Oversampling ratios. A higher ratio is less noisy and takes longer to convert.
OSR_256 = 0
OSR_512 = 1
OSR_1024 = 2
OSR_2048 = 3
OSR_4096 = 4
OSR_8192 = 5

_ADDRESS = 0x76
_RESET = 0x1E
_ADC_READ = 0x00
_PROM_READ = 0xA0
_CONVERT_D1_256 = 0x40
_CONVERT_D2_256 = 0x50

_RESET_TIME_S = 0.01
# The datasheet's maximum conversion time is 2.2 us per oversampling step; 2.5 us leaves a margin.
_CONVERSION_TIME_PER_STEP_S = 2.5e-6


class MS5837:
    """Read compensated pressure and temperature from an MS5837.

    `bus` is an open `smbus2.SMBus`, or any object with the same `write_byte`,
    `read_word_data`, `read_i2c_block_data` and `close` methods. I2C failures surface as
    the `OSError` the bus raises.
    """

    def __init__(self, bus, model=MODEL_30BA):
        if model not in (MODEL_02BA, MODEL_30BA):
            raise ValueError(f"Unknown MS5837 model: {model}")
        self._bus = bus
        self._model = model
        self._calibration = []

    def init(self):
        """Reset the sensor and load its factory calibration.

        Raises `OSError` if the calibration fails its checksum.
        """
        self._bus.write_byte(_ADDRESS, _RESET)
        sleep(_RESET_TIME_S)

        calibration = []
        for i in range(7):
            word = self._bus.read_word_data(_ADDRESS, _PROM_READ + 2 * i)
            # SMBus word transfers are little-endian and the sensor sends the high byte first.
            calibration.append(((word & 0xFF) << 8) | (word >> 8))

        expected_crc = calibration[0] >> 12
        if expected_crc != _crc4(calibration):
            raise OSError("MS5837 calibration failed its checksum")
        self._calibration = calibration

    def read(self, oversampling=OSR_8192):
        """Take one measurement and return `(pressure_pa, temperature_c)`."""
        if not OSR_256 <= oversampling <= OSR_8192:
            raise ValueError(f"Invalid oversampling option: {oversampling}")
        if not self._calibration:
            raise RuntimeError("init() must succeed before read()")

        raw_pressure = self._convert(_CONVERT_D1_256, oversampling)
        raw_temperature = self._convert(_CONVERT_D2_256, oversampling)
        return _compensate(
            self._calibration, self._model, raw_pressure, raw_temperature
        )

    def close(self):
        self._bus.close()

    def _convert(self, command, oversampling):
        self._bus.write_byte(_ADDRESS, command + 2 * oversampling)
        sleep(_CONVERSION_TIME_PER_STEP_S * 2 ** (8 + oversampling))
        data = self._bus.read_i2c_block_data(_ADDRESS, _ADC_READ, 3)
        return data[0] << 16 | data[1] << 8 | data[2]


def _compensate(calibration, model, raw_pressure, raw_temperature):
    """Apply the datasheet's first and second order compensation.

    Returns `(pressure_pa, temperature_c)`.
    """
    c = calibration
    dt = raw_temperature - c[5] * 256
    if model == MODEL_02BA:
        sens = c[1] * 65536 + (c[3] * dt) / 128
        off = c[2] * 131072 + (c[4] * dt) / 64
    else:
        sens = c[1] * 32768 + (c[3] * dt) / 256
        off = c[2] * 65536 + (c[4] * dt) / 128
    temp = 2000 + dt * c[6] / 8388608  # hundredths of a degree C

    temp_correction = 0
    off_correction = 0
    sens_correction = 0
    if model == MODEL_02BA:
        if temp < 2000:
            temp_correction = (11 * dt * dt) / 34359738368
            off_correction = (31 * (temp - 2000) ** 2) / 8
            sens_correction = (63 * (temp - 2000) ** 2) / 32
    elif temp < 2000:
        temp_correction = (3 * dt * dt) / 8589934592
        off_correction = (3 * (temp - 2000) ** 2) / 2
        sens_correction = (5 * (temp - 2000) ** 2) / 8
        if temp < -1500:
            off_correction += 7 * (temp + 1500) ** 2
            sens_correction += 4 * (temp + 1500) ** 2
    else:
        temp_correction = 2 * (dt * dt) / 137438953472
        off_correction = ((temp - 2000) ** 2) / 16

    off -= off_correction
    sens -= sens_correction
    temp -= temp_correction

    if model == MODEL_02BA:
        pressure_mbar = ((raw_pressure * sens) / 2097152 - off) / 32768 / 100.0
    else:
        pressure_mbar = ((raw_pressure * sens) / 2097152 - off) / 8192 / 10.0
    return pressure_mbar * 100.0, temp / 100.0


def _crc4(calibration):
    """Return the datasheet's 4-bit checksum of the seven calibration words."""
    words = [calibration[0] & 0x0FFF, *calibration[1:], 0]
    remainder = 0
    for i in range(16):
        if i % 2 == 1:
            remainder ^= words[i >> 1] & 0x00FF
        else:
            remainder ^= words[i >> 1] >> 8
        for _ in range(8):
            if remainder & 0x8000:
                remainder = (remainder << 1) ^ 0x3000
            else:
                remainder <<= 1
    return (remainder >> 12) & 0x000F
