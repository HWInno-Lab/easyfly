# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Native action protocol fixture, exact cancellation, concurrent owner and no replay."""
import json,signal,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from easyfly_msgs.action import GoToWaypoint
from easyfly_msgs.msg import UAVState
from easyfly_msgs.srv import GetTrajectoryState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def test_owned_task_switch_cancel_concurrent_rejection_and_explicit_new_request(tmp_path):
 rclpy.init();node=rclpy.create_node('waypoint_contract_peer');executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
 stopping=threading.Event();thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
 clock=node.create_publisher(Clock,'/clock',10);health=node.create_publisher(String,'/localization/failover/status',10);state=node.create_publisher(UAVState,'/uav_state',10)
 group=ReentrantCallbackGroup();accepted=[];canceled=[];busy=False;complete=False;healthy=True;selected='fast_lio';now_ns=20_000_000_000
 def goal(request):return GoalResponse.ACCEPT if not busy else GoalResponse.REJECT
 def execute(handle):
  nonlocal busy
  busy=True;accepted.append(bytes(handle.goal_id.uuid).hex())
  while not handle.is_cancel_requested and not complete and not stopping.is_set():time.sleep(.01)
  result=GoToWaypoint.Result()
  if handle.is_cancel_requested:
   canceled.append(bytes(handle.goal_id.uuid).hex());handle.canceled();result.message='exact owned cancellation'
  elif complete:
   handle.succeed();result.success=True
  else:handle.abort()
  busy=False;return result
 server=ActionServer(node,GoToWaypoint,'/goto_waypoint',execute_callback=execute,goal_callback=goal,cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=group)
 def control(req,res):
  res.header.stamp=Time(sec=now_ns//10**9,nanosec=now_ns%10**9);res.controller_session_id='d'*32;res.idle=not busy;return res
 node.create_service(GetTrajectoryState,'/flight_control/trajectory_state',control,callback_group=group)
 def tick():
  nonlocal now_ns
  now_ns+=50_000_000;t=Time(sec=now_ns//10**9,nanosec=now_ns%10**9);clock.publish(Clock(clock=t))
  h=dict(session='e'*32,healthy=healthy,failure='',parameters_verified=True,output_stamp_ns=now_ns,selected=selected)
  health.publish(String(data=json.dumps(h)))
  m=UAVState();m.header.stamp=t;m.header.frame_id='odom';m.connected=m.pose_valid=m.attitude_valid=m.velocity_valid=m.armed=True;m.flight_mode=3;m.position_z=1.5;state.publish(m)
 node.create_timer(.05,tick,callback_group=group)
 exe=Path(get_package_prefix('easyfly_localization_failover'))/'lib/easyfly_localization_failover/redundant_waypoint';processes=[];logs=[]
 def start(name):
  out=tmp_path/name;log=(tmp_path/(name+'.log')).open('x');logs.append(log)
  process=subprocess.Popen([str(exe),'--execute','--target','0','1','1.5','--output',str(out),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT);processes.append(process);return process,out
 def until(condition,timeout=15):
  end=time.monotonic()+timeout
  while not condition():
   assert time.monotonic()<end,[(p.poll()) for p in processes]
   time.sleep(.01)
 try:
  first,out=start('first');until(lambda:len(accepted)==1)
  second,other=start('concurrent');until(lambda:second.poll() is not None)
  assert second.returncode!=0 and 'BlockingIOError' in (tmp_path/'concurrent.log').read_text()
  selected='openvins';until(lambda:len(json.loads((out/'result.json').read_text())['events'])==2)
  assert not canceled and first.poll() is None
  healthy=False;until(lambda:first.poll() is not None)
  result=json.loads((out/'result.json').read_text())
  assert first.returncode==2 and result['phase']=='terminal' and not result['busy'] and result['native_status']==5
  assert result['cancel_sent'] and canceled==accepted==[result['goal_id']]
  healthy=True;selected='fast_lio';time.sleep(.4);assert len(accepted)==1
  complete=True;third,final=start('explicit-new');until(lambda:third.poll() is not None)
  r=json.loads((final/'result.json').read_text());assert third.returncode==0 and r['success'] and not r['busy']
  assert len(accepted)==2 and accepted[-1]!=accepted[0]
 finally:
  for p in processes:
   if p.poll() is None:p.send_signal(signal.SIGINT);p.wait(timeout=20)
  stopping.set();server.destroy();executor.shutdown();thread.join(timeout=5);node.destroy_node();rclpy.shutdown()
  for log in logs:log.close()
