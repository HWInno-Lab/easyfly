"""Native feedback setup guards without starting PX4 or issuing flight commands."""
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from easyfly_px4_sitl.telemetry import configured_heartbeat, configured_stream, configure_owned

STATUS = '\ninstance #0:\n transport protocol: UDP (18570, remote port: 14550)\n' + \
         '\ninstance #1:\n transport protocol: UDP (14580, remote port: 14540)\n'
STREAMS = '\ninstance #0:\n HEARTBEAT 1.00 (1.000) 21\n' + \
          '\ninstance #1:\n HEARTBEAT 5.00 (5.000) 21\n'


def test_rate_is_verified_on_the_actual_offboard_link():
    assert configured_heartbeat(STATUS, STREAMS, 5)
    assert configured_heartbeat(STATUS+STREAMS, STATUS+STREAMS, 5)
    assert not configured_heartbeat(STATUS, STREAMS.replace('5.00 (5.000)', '1.00 (1.000)'), 5)
    assert not configured_heartbeat(STATUS.replace('14540', '14541'), STREAMS, 5)
    assert not configured_heartbeat(STATUS, STREAMS.replace('instance #1', 'instance #2'), 5)


@pytest.fixture
def native_client(tmp_path):
    binary = tmp_path/'px4'
    binary.write_bytes(b'native fixture identity only; never executed')
    (tmp_path/'px4-mavlink').symlink_to(binary)
    return binary


@pytest.mark.parametrize('changed', ['parent', 'executable', 'start_ticks'])
def test_refuses_unowned_or_replaced_process(native_client, changed):
    owned = (str(native_client), os.getpid(), '100')
    replacement = {'parent': (str(native_client), os.getpid()+1, '100'),
                   'executable': ('/other/px4', os.getpid(), '100'),
                   'start_ticks': (str(native_client), os.getpid(), '101')}[changed]
    identity = Mock(side_effect=[owned, replacement] if changed == 'start_ticks' else None,
                    return_value=replacement)
    runner = Mock()
    with pytest.raises(RuntimeError):
        configure_owned(42, native_client, 5, identity=identity, runner=runner)
    runner.assert_not_called()


def test_requires_native_status_after_successful_configuration(native_client):
    identity = lambda pid: (str(native_client), os.getpid(), '100')
    runner = Mock(side_effect=[SimpleNamespace(returncode=0, stdout=s, stderr='')
                              for s in ['', STATUS, STREAMS]])
    result = configure_owned(42, native_client, 5, identity=identity, runner=runner)
    assert result['rate_hz'] == 5 and result['command_returncodes'] == [0, 0, 0]
    assert all(call.kwargs['timeout'] == 5 for call in runner.call_args_list)
    runner = Mock(side_effect=[SimpleNamespace(returncode=0, stdout=s, stderr='')
                              for s in ['', STATUS, STREAMS.replace('5.00', '1.00')]])
    with pytest.raises(RuntimeError, match='not confirmed'):
        configure_owned(42, native_client, 5, identity=identity, runner=runner)


def test_rejects_client_alias_to_another_binary(native_client, tmp_path):
    client = tmp_path/'px4-mavlink'
    client.unlink()
    client.symlink_to(Path('/bin/true'))
    runner = Mock()
    with pytest.raises(RuntimeError, match='client differs'):
        configure_owned(42, native_client, 5, runner=runner)
    runner.assert_not_called()


def test_daemon_stdout_requires_deferred_verification(native_client):
    identity = lambda pid: (str(native_client), os.getpid(), '100')
    runner = Mock(return_value=SimpleNamespace(returncode=0, stdout='', stderr=''))
    result = configure_owned(42, native_client, 5, identity=identity, runner=runner)
    assert result['native_output_deferred']
    assert not configured_heartbeat(result['status'], result['streams'], 5)


@pytest.mark.parametrize('rate', [0.5, 5])
def test_native_battery_rate_requires_actual_offboard_stream(native_client, rate):
    identity = lambda pid: (str(native_client), os.getpid(), '100')
    streams = STREAMS + f' BATTERY_STATUS {rate:.2f} ({rate:.3f}) 54\n'
    runner = Mock(side_effect=[SimpleNamespace(returncode=0, stdout=s, stderr='')
                              for s in ['', '', STATUS, streams]])
    if rate == 5:
        result = configure_owned(42, native_client, 5, battery_rate_hz=5, identity=identity, runner=runner)
        assert result['battery_rate_hz']==5 and result['command_returncodes']==[0,0,0,0]
        assert runner.call_args_list[1].args[0][-4:]==['-s','BATTERY_STATUS','-r','5']
    else:
        with pytest.raises(RuntimeError, match='not confirmed'):
            configure_owned(42, native_client, 5, battery_rate_hz=5, identity=identity, runner=runner)
    wrong_link=STREAMS.replace('instance #0:\n HEARTBEAT', 'instance #0:\n BATTERY_STATUS 5.00 (5.000) 54\n HEARTBEAT')
    assert not configured_stream(STATUS,wrong_link,5,'BATTERY_STATUS')


def test_unsupported_battery_rate_cannot_issue_native_command(native_client):
    runner=Mock()
    with pytest.raises(ValueError,match='battery rate'):
        configure_owned(42,native_client,5,battery_rate_hz=1,runner=runner)
    runner.assert_not_called()
