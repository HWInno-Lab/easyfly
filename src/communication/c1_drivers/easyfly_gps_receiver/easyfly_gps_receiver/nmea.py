# Copyright 2026 EasyFly Project
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

"""Parse bounded, checksummed GGA/RMC observations and pair one original UTC epoch."""

from datetime import datetime, timezone
import math
import re


class InvalidFix(ValueError):
    """Represent an explicit receiver invalidation rather than a corrupted transport line."""


def sentence(data):
    """Decode one bounded ASCII sentence only after its XOR checksum matches."""
    if not 10 <= len(data) <= 128:
        raise ValueError('NMEA sentence size is invalid')
    text = data.decode('ascii').rstrip('\r\n')
    match = re.fullmatch(r'\$([^$*\r\n]+)\*([A-Fa-f0-9]{2})', text)
    if not match:
        raise ValueError('NMEA framing is invalid')
    body, checksum = match.groups()
    actual = 0
    for byte in body.encode('ascii'):
        actual ^= byte
    if actual != int(checksum, 16):
        raise ValueError('NMEA checksum mismatch')
    fields = body.split(',')
    if not re.fullmatch('[A-Z]{2}[A-Z]{3}', fields[0]):
        raise ValueError('NMEA talker/type is invalid')
    return fields


def number(text, minimum, maximum):
    """Reject nonfinite, empty and out-of-range navigation values."""
    if not re.fullmatch(r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)', text):
        raise ValueError('invalid decimal encoding')
    value = float(text)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError('NMEA number outside supported range')
    return value


def coordinate(text, hemisphere, latitude):
    """Convert degrees/minutes with strict hemisphere, minutes and pole validation."""
    digits = 2 if latitude else 3
    if not re.fullmatch(r'\d{%d}\d{2}(?:\.\d{1,9})?' % digits, text):
        raise ValueError('invalid coordinate encoding')
    degrees = int(text[:digits])
    minutes = number(text[digits:], 0, 59.999999999)
    limit = 90 if latitude else 180
    if degrees > limit or (degrees == limit and minutes != 0):
        raise ValueError('coordinate outside globe')
    if hemisphere not in (('N', 'S') if latitude else ('E', 'W')):
        raise ValueError('coordinate hemisphere mismatch')
    return (degrees + minutes / 60) * (-1 if hemisphere in ('S', 'W') else 1)


def time_of_day(text):
    """Keep exact nanosecond fractions; leap-second labels require a separate PPS policy."""
    match = re.fullmatch(r'(\d{2})(\d{2})(\d{2})(?:\.(\d{1,9}))?', text)
    if not match:
        raise ValueError('invalid UTC time')
    hour, minute, second = map(int, match.groups()[:3])
    if hour > 23 or minute > 59 or second > 59:
        raise ValueError('unsupported UTC time or leap-second label')
    nanos = int((match.group(4) or '').ljust(9, '0'))
    return (hour * 3600 + minute * 60 + second) * 10**9 + nanos


def rmc(fields):
    """Read UTC date/time and a receiver-validated geographic position."""
    if len(fields) < 10:
        raise ValueError('truncated RMC')
    if fields[2] != 'A' or (len(fields) > 12 and fields[12] not in ('', 'A', 'D', 'P', 'R', 'F')):
        raise InvalidFix('receiver_rmc_invalid')
    if len(fields) > 13 and fields[13] not in ('', 'A', 'D', 'V'):
        raise InvalidFix('receiver_navigation_integrity_invalid')
    if not re.fullmatch(r'\d{6}', fields[9]):
        raise ValueError('invalid RMC date')
    day, month, year = [int(fields[9][i:i + 2]) for i in (0, 2, 4)]
    year += 1900 if year >= 80 else 2000
    midnight = datetime(year, month, day, tzinfo=timezone.utc)
    tod = time_of_day(fields[1])
    utc = int(midnight.timestamp()) * 10**9 + tod
    if not 0 < utc // 10**9 < 2**31:
        raise ValueError('UTC is outside the ROS signed-second range')
    return {'utc_ns': utc, 'tod_ns': tod,
            'latitude': coordinate(fields[3], fields[4], True),
            'longitude': coordinate(fields[5], fields[6], False)}


def gga(fields):
    """Require a real fix and convert MSL plus geoid separation to ellipsoid altitude."""
    if len(fields) < 13:
        raise ValueError('truncated GGA')
    if not re.fullmatch('[0-8]', fields[6] or '0'):
        raise ValueError('invalid fix quality')
    quality = int(fields[6] or '0')
    if quality not in (1, 2, 3, 4, 5):
        raise InvalidFix('receiver_gga_no_observed_fix')
    if not re.fullmatch(r'\d{1,2}', fields[7]):
        raise ValueError('invalid satellite count')
    satellites = int(fields[7])
    if not 4 <= satellites <= 99:
        raise InvalidFix('insufficient_satellites')
    if fields[10] != 'M' or fields[12] != 'M':
        raise ValueError('unsupported altitude units or missing geoid separation')
    return {'tod_ns': time_of_day(fields[1]),
            'latitude': coordinate(fields[2], fields[3], True),
            'longitude': coordinate(fields[4], fields[5], False),
            'altitude': number(fields[9], -1500, 100000) + number(fields[11], -1000, 1000),
            'satellites': satellites, 'hdop': number(fields[8], .01, 100), 'quality': quality,
            'service': {'GP': 1, 'GL': 2, 'GB': 4, 'BD': 4, 'GA': 8}.get(fields[0][:2], 0)}


class Pairer:
    """Join independent sentences only when source epochs, positions and receipts agree."""

    def __init__(self):
        self.pending = {}
        self.last_utc = 0

    def feed(self, data, receipt):
        """Return one validated observation; never renew an old fix with a duplicate sentence."""
        fields = sentence(data)
        talker, kind = fields[0][:2], fields[0][2:]
        if talker not in ('GP', 'GN', 'GL', 'GA', 'GB', 'BD') or kind not in ('GGA', 'RMC'):
            return None
        try:
            value = gga(fields) if kind == 'GGA' else rmc(fields)
        except InvalidFix:
            self.pending.clear()
            raise
        self.pending[kind] = (receipt, value)
        if set(self.pending) != {'GGA', 'RMC'}:
            return None
        ga_at, ga = self.pending['GGA']
        rm_at, rm = self.pending['RMC']
        if (abs(ga_at - rm_at) > 1. or receipt - min(ga_at, rm_at) > 1. or
                ga['tod_ns'] != rm['tod_ns']):
            return None
        self.pending.clear()
        if (abs(ga['latitude'] - rm['latitude']) > .0001 or
                abs(ga['longitude'] - rm['longitude']) > .0001):
            raise InvalidFix('sentence_position_mismatch')
        if rm['utc_ns'] <= self.last_utc:
            raise InvalidFix('utc_not_advancing')
        self.last_utc = rm['utc_ns']
        return dict(ga, utc_ns=rm['utc_ns'], receipt=receipt)


class Framer:
    """Recover split/concatenated serial lines without retaining unbounded noise."""

    def __init__(self):
        self.buffer = bytearray()
        self.discarded = 0

    def feed(self, data):
        """Return complete bounded lines, resynchronizing at a new dollar delimiter."""
        rows = []
        for byte in data:
            if byte == ord('$'):
                if self.buffer:
                    self.discarded += 1
                self.buffer = bytearray(b'$')
            elif self.buffer:
                self.buffer.append(byte)
                if byte == 10:
                    rows.append(bytes(self.buffer))
                    self.buffer.clear()
                elif len(self.buffer) >= 128:
                    self.discarded += 1
                    self.buffer.clear()
        return rows
