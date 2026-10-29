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

"""Validate source epochs and units against independent navigation fixtures."""

from datetime import datetime, timezone
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from easyfly_gps_receiver.nmea import (  # noqa: E402
    coordinate, Framer, InvalidFix, Pairer, sentence, time_of_day)


def wire(body):
    """Encode a test sentence without sharing the production parser."""
    checksum = 0
    for byte in body.encode('ascii'):
        checksum ^= byte
    return ('$%s*%02X\r\n' % (body, checksum)).encode()


def pair(time='123519.125', date='281026', latitude='4807.038', hemisphere='N', quality='1'):
    """Give GGA and RMC the same independently specified geographic source epoch."""
    return (wire(f'GPGGA,{time},{latitude},{hemisphere},01131.000,E,{quality},08,0.9,'
                 '545.4,M,46.9,M,,'),
            wire(f'GPRMC,{time},A,{latitude},{hemisphere},01131.000,E,0.0,0.0,{date},,,A'))


def test_position_altitude_and_exact_utc():
    """Compare degree/minute conversion, ellipsoid altitude and fractional UTC to constants."""
    ga, rm = pair()
    parser = Pairer()
    assert parser.feed(ga, 1.) is None
    result = parser.feed(rm, 1.1)
    expected = int(datetime(2026, 10, 28, 12, 35, 19, tzinfo=timezone.utc).timestamp()) * 10**9
    assert result['utc_ns'] == expected + 125000000
    assert abs(result['latitude'] - 48.1173) < 1e-8
    assert abs(result['longitude'] - 11.5166666667) < 1e-8
    assert abs(result['altitude'] - 592.3) < 1e-8
    assert result['service'] == 1


def test_midnight_and_reverse_arrival_order():
    """Pair each RMC date with its GGA without borrowing the previous day's time."""
    parser = Pairer()
    ga, rm = pair(time='235959.9', date='281026')
    parser.feed(rm, 1.)
    before = parser.feed(ga, 1.1)
    ga, rm = pair(time='000000.1', date='291026')
    parser.feed(ga, 1.2)
    after = parser.feed(rm, 1.3)
    assert after['utc_ns'] - before['utc_ns'] == 200000000


@pytest.mark.parametrize('time,receipt', [
    ('123520.125', 1.1), ('123519.175', 1.1), ('123519.125', 2.1)])
def test_different_epochs_or_stale_halves_never_pair(time, receipt):
    """Reject an apparently valid fix assembled from different observations."""
    parser = Pairer()
    ga, _ = pair()
    _, rm = pair(time=time)
    parser.feed(ga, 1.)
    assert parser.feed(rm, receipt) is None


def test_duplicate_and_explicit_invalid_fix():
    """A duplicate epoch and receiver no-fix signal cannot renew an old fix."""
    parser = Pairer()
    ga, rm = pair()
    parser.feed(ga, 1.)
    parser.feed(rm, 1.1)
    parser.feed(ga, 1.2)
    with pytest.raises(InvalidFix):
        parser.feed(rm, 1.3)
    bad, _ = pair(quality='0')
    with pytest.raises(InvalidFix):
        parser.feed(bad, 1.4)
    assert not parser.pending


@pytest.mark.parametrize('text,hemisphere,latitude', [
    ('9060.0', 'N', True), ('18000.1', 'E', False), ('1260.0', 'N', True),
    ('nan', 'N', True), ('4807.038', 'E', True), ('-4807.0', 'S', True)])
def test_invalid_coordinates(text, hemisphere, latitude):
    """Reject NaNs, invalid hemispheres, out-of-range minutes and pole overflow."""
    with pytest.raises(ValueError):
        coordinate(text, hemisphere, latitude)


def test_checksum_corruption_and_bounded_resynchronization():
    """Split real bytes, discard overlong noise and recover without growing a buffer."""
    ga, rm = pair()
    with pytest.raises(ValueError):
        sentence(ga.replace(b'545.4', b'645.4'))
    framer = Framer()
    assert framer.feed(b'$' + b'x' * 4096) == []
    assert len(framer.buffer) == 0
    assert framer.feed(ga[:10]) == []
    assert framer.feed(ga[10:] + rm) == [ga, rm]
    assert framer.discarded == 1


def test_unsupported_leap_second_does_not_alias_next_day():
    """Fail explicitly on second 60 instead of inventing a UTC source timestamp."""
    with pytest.raises(ValueError):
        time_of_day('235960')
