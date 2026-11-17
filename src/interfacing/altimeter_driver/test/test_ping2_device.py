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
"""Check `Ping2` against the real brping library and an in-memory device."""

import struct

import pytest

from altimeter_driver.ping2_node import Ping2

_HEADER_LENGTH = 8
_CHECKSUM_LENGTH = 2


class Serial:
    """An in-memory Ping2 that speaks the Ping protocol.

    brping chooses its serial code path by this class name.
    """

    def __init__(self, brping):
        self._brping = brping
        self.speed_of_sound = 1500000  # mm/s
        self.ping_interval = 100  # ms
        self.ignore_settings = False
        self.silent = False
        self.closed = False
        self._received = bytearray()
        self._replies = bytearray()

    @property
    def in_waiting(self):
        return len(self._replies)

    def read(self, size):
        data = bytes(self._replies[:size])
        del self._replies[:size]
        return data

    def write(self, data):
        self._received += data
        while len(self._received) >= _HEADER_LENGTH:
            length, message_id = struct.unpack_from("<HH", self._received, 2)
            end = _HEADER_LENGTH + length + _CHECKSUM_LENGTH
            if len(self._received) < end:
                break
            payload = bytes(self._received[_HEADER_LENGTH : _HEADER_LENGTH + length])
            del self._received[:end]
            self._handle(message_id, payload)
        return len(data)

    def close(self):
        self.closed = True

    def _handle(self, message_id, payload):
        definitions = self._brping.definitions
        if message_id == definitions.PING1D_SET_SPEED_OF_SOUND:
            if not self.ignore_settings:
                (self.speed_of_sound,) = struct.unpack("<I", payload)
        elif message_id == definitions.PING1D_SET_PING_INTERVAL:
            if not self.ignore_settings:
                (self.ping_interval,) = struct.unpack("<H", payload)
        elif not self.silent:
            # A request is an empty message carrying the id of the message asked for.
            self._reply(message_id)

    def _reply(self, message_id):
        definitions = self._brping.definitions
        values = {
            definitions.PING1D_SPEED_OF_SOUND: [self.speed_of_sound],
            definitions.PING1D_PING_INTERVAL: [self.ping_interval],
            # distance, confidence, transmit_duration, ping_number, scan_start,
            # scan_length, gain_setting
            definitions.PING1D_DISTANCE: [3210, 95, 100, 7, 300, 9700, 1],
        }[message_id]
        msg = self._brping.PingMessage(message_id)
        for name, value in zip(msg.payload_field_names, values):
            setattr(msg, name, value)
        msg.pack_msg_data()
        self._replies += msg.msg_data


@pytest.fixture
def brping():
    # Skip per test, not per module: a module-level skip here hides the other test files
    # in this directory from pytest.
    return pytest.importorskip("brping")


@pytest.fixture
def serial(brping):
    return Serial(brping)


@pytest.fixture
def ping2(brping, serial):
    device = brping.Ping1D()
    device.iodev = serial
    return Ping2(device)


def test_configure_writes_both_settings(ping2, serial):
    ping2.configure(1482000, 50)

    assert serial.speed_of_sound == 1482000
    assert serial.ping_interval == 50


def test_configure_fails_when_the_device_keeps_its_own_value(ping2, serial):
    serial.ignore_settings = True

    with pytest.raises(OSError, match="the device reports 1500000"):
        ping2.configure(1482000, 50)


def test_configure_fails_without_a_reply(ping2, serial):
    serial.silent = True

    with pytest.raises(OSError, match="no reply"):
        ping2.configure(1482000, 50)


def test_get_distance_returns_the_reading(ping2):
    reading = ping2.get_distance()

    assert reading["distance"] == 3210
    assert reading["confidence"] == 95
    assert reading["scan_start"] == 300
    assert reading["scan_length"] == 9700


def test_get_distance_returns_none_without_a_reply(ping2, serial):
    serial.silent = True

    assert ping2.get_distance() is None


def test_close_closes_the_port(ping2, serial):
    ping2.close()

    assert serial.closed
