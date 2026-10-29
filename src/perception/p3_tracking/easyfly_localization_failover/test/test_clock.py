# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
from easyfly_localization_failover.clock_witness import Qualification,parse
import pytest


def observation(i,offset=5000,rtt=1000):
 return dict(timestamp=1_000_000+i*100_000,remote_timestamp=995_000+i*100_000,observed_offset=offset,estimated_offset=offset,round_trip_time=rtt,source_protocol=0)


def test_native_convergence_requires_all_510_distinct_good_observations():
 q=Qualification()
 for i in range(509):
  row=observation(i);q.update(row,i*.1,row['timestamp']*1000)
 assert not q.valid(50.8)
 row=observation(509);q.update(row,50.9,row['timestamp']*1000)
 assert q.valid(50.9)
 assert not q.update(row,51.,row['timestamp']*1000)
 assert q.count==510
 assert not q.valid(51.71)


@pytest.mark.parametrize('kind',['missing','offset','stale','rollback'])
def test_convergence_resets_on_unobserved_or_bad_native_sample(kind):
 q=Qualification()
 for i in range(510):
  row=observation(i);q.update(row,i*.1,row['timestamp']*1000)
 row=observation(511 if kind=='missing' else 510)
 if kind=='rtt':row['round_trip_time']=10001
 if kind=='offset':row['observed_offset']=123000
 if kind=='rollback':row['timestamp']-=300000;row['remote_timestamp']-=300000
 clock=row['timestamp']*1000+(600000000 if kind=='stale' else 0)
 q.update(row,51.1,clock)
 assert not q.valid(51.1) and q.count<510


def test_parse_native_listener_and_reject_ambiguous_output():
 text='\n'.join('  '+k+': '+str(v)+(' (0.01 seconds ago)' if k=='timestamp' else '') for k,v in observation(0).items())
 assert parse(text)==observation(0)
 with pytest.raises(ValueError):parse(text+'\n timestamp: 1000000')
 with pytest.raises(ValueError):parse('never published')


def test_native_ipc_ahead_of_queued_ros_clock_defers_without_count_or_refresh():
 q=Qualification()
 for i in range(510):
  row=observation(i);q.update(row,i*.1,row['timestamp']*1000)
 row=observation(510)
 assert not q.update(row,51.,row['timestamp']*1000-70000000)
 assert q.count==510 and q.last['timestamp']==observation(509)['timestamp']
 assert q.last_receipt==pytest.approx(50.9) and q.valid(51.)
 assert q.update(row,51.04,row['timestamp']*1000)
 assert q.count==511
 # Repeated future samples cannot renew a proof during a frozen ROS clock.
 for i in range(9):
  q.update(observation(511),51.08+i*.1,row['timestamp']*1000)
 assert not q.valid(51.89) and q.last_receipt==51.04


def converged_native_capture():
 # Finish with the exact last accepted observation in the 2026-10-29 native
 # flight capture. Only the preceding warm-up is a deterministic unit fixture.
 q=Qualification()
 for i in range(510):
  row=dict(timestamp=105887000-(509-i)*100000,
           remote_timestamp=105880000-(509-i)*100000,
           observed_offset=7000,estimated_offset=6986,round_trip_time=0,source_protocol=0)
  q.update(row,55722.205457611-(509-i)*.1,row['timestamp']*1000)
 return q


def captured_rejection():
 return dict(timestamp=106103000,remote_timestamp=105920000,
             observed_offset=175000,estimated_offset=6986,round_trip_time=16000,source_protocol=0)


def test_captured_native_rejected_rtt_keeps_filter_without_counting_bad_sample():
 q=converged_native_capture(); good_receipt=q.last_receipt
 row=captured_rejection()
 assert not q.update(row,55722.565046610,106100000000)
 assert q.count==510 and q.last_receipt==good_receipt and q.valid(55722.565046610)
 # A duplicate cannot renew either receipt or the fixed good-sample deadline.
 assert not q.update(row,55722.646902669,106170000000)
 good=dict(timestamp=106187000,remote_timestamp=106180000,
           observed_offset=7000,estimated_offset=6987,round_trip_time=0,source_protocol=0)
 assert q.update(good,55722.687643488,106200000000)
 assert q.count==511 and q.valid(55722.687643488)


@pytest.mark.parametrize('kind',['changed_estimate','protocol','second_rejection','rollback',
                                  'stale','excessive_rtt','no_convergence'])
def test_rejected_rtt_exception_requires_bounded_unchanged_native_filter(kind):
 q=converged_native_capture(); row=captured_rejection(); now=55722.565046610
 if kind=='changed_estimate':row['estimated_offset']+=1
 if kind=='protocol':row['source_protocol']=1
 if kind=='rollback':row['remote_timestamp']=105870000
 if kind=='stale':now+=.5
 if kind=='excessive_rtt':row['round_trip_time']=350001
 if kind=='no_convergence':q.count=509
 q.update(row,now,106100000000)
 if kind=='second_rejection':
  row=dict(row,timestamp=106203000,remote_timestamp=106020000)
  q.update(row,now+.1,106200000000)
 assert not q.valid(now+.1)


def test_ignored_native_rtt_cannot_extend_frozen_clock_or_missing_measurements():
 q=converged_native_capture(); row=captured_rejection()
 q.update(row,55722.565046610,106100000000)
 for now in [55722.65,55722.85,55723.05]:
  q.update(row,now,106100000000)
 assert not q.valid(55723.05)
 q=converged_native_capture();q.update(row,55722.565046610,106100000000)
 good=dict(row,timestamp=106487000,remote_timestamp=106480000,
           observed_offset=7000,estimated_offset=6987,round_trip_time=0)
 q.update(good,55722.75,106480000000)
 assert not q.valid(55722.75)


def test_captured_low_rtt_raw_jitter_does_not_reset_converged_native_filter():
 q=Qualification()
 for i in range(510):
  row=dict(timestamp=82067000-(509-i)*100000,
           remote_timestamp=82060000-(509-i)*100000,
           observed_offset=7000,estimated_offset=6971,round_trip_time=0,source_protocol=0)
  q.update(row,57551.618223928-(509-i)*.1,row['timestamp']*1000)
 row=dict(timestamp=82167000,remote_timestamp=82120000,
          observed_offset=47000,estimated_offset=7091,round_trip_time=0,source_protocol=0)
 assert q.update(row,57551.778457282,82160000000)
 assert q.count==511 and q.valid(57551.778457282)
 row=dict(timestamp=82267000,remote_timestamp=82260000,
          observed_offset=7000,estimated_offset=7091,round_trip_time=0,source_protocol=0)
 assert q.update(row,57551.937424740,82270000000)
 assert q.count==512


@pytest.mark.parametrize('kind',['startup','estimate_jump','estimate_bound'])
def test_native_filtered_clock_stays_bounded_and_startup_stays_strict(kind):
 q=Qualification()
 for i in range(510):
  row=observation(i);q.update(row,i*.1,row['timestamp']*1000)
 if kind=='startup':q.count=509
 row=observation(510);row['observed_offset']=47000
 if kind=='estimate_jump':row['estimated_offset']=6100
 if kind=='estimate_bound':row['estimated_offset']=21000
 q.update(row,51.,row['timestamp']*1000)
 assert not q.valid(51.)


def test_captured_stalled_native_exchange_keeps_bounded_unchanged_filter():
 q=Qualification()
 for i in range(510):
  row=dict(timestamp=86298000-(509-i)*100000,
           remote_timestamp=86290000-(509-i)*100000,
           observed_offset=8000,estimated_offset=7965,round_trip_time=0,source_protocol=0)
  q.update(row,58576.421293773-(509-i)*.1,row['timestamp']*1000)
 for now,clock in [(58576.780643980,86500000000),(58576.860115499,86560000000),
                   (58576.903731350,86600000000)]:
  assert not q.update(row,now,clock)
  assert q.valid(now) and q.count==510
 rejected=dict(timestamp=86610000,remote_timestamp=86550000,observed_offset=54000,
               estimated_offset=7965,round_trip_time=12000,source_protocol=0)
 q.update(rejected,58576.941113976,86610000000)
 assert q.valid(58576.941113976) and q.count==510
 good=dict(timestamp=86698000,remote_timestamp=86690000,observed_offset=8000,
           estimated_offset=7965,round_trip_time=0,source_protocol=0)
 assert q.update(good,58577.061077887,86690000000)
 assert q.valid(58577.061077887) and q.count==511


def test_unchanged_native_snapshot_has_absolute_source_and_wall_deadlines():
 for source_age,wall_age in [(.501,.6),(.2,.801)]:
  q=Qualification()
  for i in range(510):
   row=observation(i);q.update(row,i*.1,row['timestamp']*1000)
  q.update(row,50.9+wall_age,row['timestamp']*1000+int(source_age*1e9))
  assert not q.valid(50.9+wall_age)
