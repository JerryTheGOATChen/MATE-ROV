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
import pytest

from pressure_driver.ms5837 import MODEL_30BA, MS5837, _compensate, _crc4

# Worked example from the MS5837-30BA datasheet: 3999.8 mbar at 19.81 degrees C.
DATASHEET_CALIBRATION = [0, 34982, 36352, 20328, 22354, 26646, 26146]
DATASHEET_RAW_PRESSURE = 4958179
DATASHEET_RAW_TEMPERATURE = 6815414


def with_crc(calibration):
    """Return the calibration with its checksum stored where the sensor keeps it."""
    words = list(calibration)
    words[0] = (words[0] & 0x0FFF) | (_crc4(words) << 12)
    return words


class FakeBus:
    """Stand in for smbus2.SMBus with a scripted MS5837 behind it."""

    def __init__(self, calibration, raw_pressure, raw_temperature):
        self._calibration = calibration
        self._raw = {0x40: raw_pressure, 0x50: raw_temperature}
        self._pending = None
        self.closed = False

    def write_byte(self, address, command):
        assert address == 0x76
        if command != 0x1E:
            self._pending = self._raw[command & 0xF0]

    def read_word_data(self, address, register):
        word = self._calibration[(register - 0xA0) // 2]
        return ((word & 0xFF) << 8) | (word >> 8)

    def read_i2c_block_data(self, address, register, length):
        return list(self._pending.to_bytes(length, "big"))

    def close(self):
        self.closed = True


def test_compensation_matches_datasheet_example():
    pressure_pa, temperature_c = _compensate(
        DATASHEET_CALIBRATION,
        MODEL_30BA,
        DATASHEET_RAW_PRESSURE,
        DATASHEET_RAW_TEMPERATURE,
    )

    assert pressure_pa == pytest.approx(399980.0, abs=50.0)
    assert temperature_c == pytest.approx(19.81, abs=0.01)


def test_read_decodes_bus_traffic():
    bus = FakeBus(
        with_crc(DATASHEET_CALIBRATION),
        DATASHEET_RAW_PRESSURE,
        DATASHEET_RAW_TEMPERATURE,
    )
    sensor = MS5837(bus)
    sensor.init()

    pressure_pa, temperature_c = sensor.read(oversampling=0)

    assert pressure_pa == pytest.approx(399980.0, abs=50.0)
    assert temperature_c == pytest.approx(19.81, abs=0.01)


def test_init_rejects_corrupt_calibration():
    calibration = with_crc(DATASHEET_CALIBRATION)
    calibration[3] ^= 0x0001
    sensor = MS5837(FakeBus(calibration, 0, 0))

    with pytest.raises(OSError):
        sensor.init()


def test_read_requires_init():
    sensor = MS5837(FakeBus(with_crc(DATASHEET_CALIBRATION), 0, 0))

    with pytest.raises(RuntimeError):
        sensor.read()


def test_close_closes_the_bus():
    bus = FakeBus(with_crc(DATASHEET_CALIBRATION), 0, 0)

    MS5837(bus).close()

    assert bus.closed
