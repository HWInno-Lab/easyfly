# Copyright 2026 EasyFly Project. SPDX-License-Identifier: Apache-2.0
"""Real ROS wiring fixture; synthetic messages are never flight quality evidence."""
from pathlib import Path
import json,math,signal,subprocess,time
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from easyfly_msgs.msg import LocalizationStatus,UAVState
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg,CustomPoint
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile,DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu,Image,CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from easyfly_localization_failover.node import EV_PARAMETERS,PREFIX,LEVER


def test_real_ros_activation_loss_recovery_and_clock_freeze(tmp_path):
 rclpy.init();peer=rclpy.create_node('failover_contract_peer')
 log=(tmp_path/'manager.log').open('x');exe=Path(get_package_prefix('easyfly_localization_failover'))/'lib/easyfly_localization_failover/localization_failover'
 process=subprocess.Popen([str(exe),'--ros-args','-p','use_sim_time:=true','-p','enable_external_vision:=true'],stdout=log,stderr=subprocess.STDOUT)
 publishers={}
 def pub(kind,topic,qos=32):
  p=peer.create_publisher(kind,topic,qos);publishers[topic]=p;return p
 clock_proof=pub(String,PREFIX+'/clock_status');clock=pub(Clock,'/clock');lio=pub(Odometry,'/Odometry');vio=pub(PoseWithCovarianceStamped,'/localization/openvins/pose')
 status=pub(LocalizationStatus,'/localization/openvins/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 tracks=pub(DiagnosticArray,'/openvins/raw/visual_tracking');imu=pub(Imu,'/livox/imu');lidar=pub(CustomMsg,'/livox/lidar')
 pub(Image,'/camera/color/image_raw');pub(CameraInfo,'/camera/color/camera_info')
 fcu=pub(UAVState,'/uav_state');position=pub(PoseStamped,'/mavros/local_position/pose');attitude=pub(Imu,'/mavros/imu/data')
 messages=[];outputs=[];states=[]
 peer.create_subscription(String,PREFIX+'/status',lambda m:states.append(json.loads(m.data)),100)
 peer.create_subscription(PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov',outputs.append,100)
 client=peer.create_client(Trigger,PREFIX+'/activate')
 parameters=dict(EV_PARAMETERS);bad_parameters=False
 def params(req,res):
  res.values=[ParameterValue(type=2,integer_value=int(parameters[n])) if isinstance(parameters[n],int) else ParameterValue(type=3,double_value=float(parameters[n])) for n in req.names]
  return res
 def timing(req,res):
  assert req.names==['timesync_mode'];res.values=[ParameterValue(type=4,string_value='PASSTHROUGH')];return res
 def vision(req,res):
  assert req.names==['tf/listen'];res.values=[ParameterValue(type=1,bool_value=False)];return res
 peer.create_service(GetParameters,'/mavros/param/get_parameters',params)
 peer.create_service(GetParameters,'/mavros/time/get_parameters',timing)
 peer.create_service(GetParameters,'/mavros/vision_pose/get_parameters',vision)
 ns=10_000_000_000;last=0.;absent=set();armed=False
 def stamped(m,frame):
  m.header.stamp=Time(sec=(ns-50_000_000)//10**9,nanosec=(ns-50_000_000)%10**9);m.header.frame_id=frame;return m
 def emit():
  nonlocal ns
  ns+=50_000_000;clock_ns=ns//100_000_000*100_000_000;clock.publish(Clock(clock=Time(sec=clock_ns//10**9,nanosec=clock_ns%10**9)))
  clock_proof.publish(String(data=json.dumps(dict(identity={'session':'synthetic-contract-clock'},verified=True,failure='',count=510,clock_ns=clock_ns))))
  m=stamped(UAVState(),'odom');m.connected=m.pose_valid=m.velocity_valid=m.attitude_valid=True;m.armed=armed;m.position_z=.1;fcu.publish(m)
  m=stamped(PoseStamped(),'map');m.pose.position.z=.1;m.pose.orientation.w=1.;position.publish(m)
  m=stamped(Imu(),'base_link');m.orientation.w=1.;attitude.publish(m)
  m=stamped(Imu(),'livox_imu_frame');m.header.stamp=Time(sec=ns//10**9,nanosec=ns%10**9);m.linear_acceleration.z=9.8;imu.publish(m)
  m=stamped(CustomMsg(),'livox_frame');m.point_num=100;m.points=[CustomPoint() for _ in range(100)];lidar.publish(m)
  if 'fast_lio' not in absent:
   m=stamped(Odometry(),'camera_init');m.child_frame_id='body';m.pose.pose.position.x=-float(LEVER[0]);m.pose.pose.position.y=-float(LEVER[1]);m.pose.pose.position.z=.1-float(LEVER[2]);m.pose.pose.orientation.w=1.;lio.publish(m)
  if 'openvins' not in absent:
   m=stamped(PoseWithCovarianceStamped(),'openvins_odom');m.pose.pose.position.x=1.-float(LEVER[0]);m.pose.pose.position.y=-float(LEVER[1]);m.pose.pose.position.z=.1-float(LEVER[2]);m.pose.pose.orientation.w=1.;vio.publish(m)
   d=stamped(DiagnosticArray(),'global');d.status=[DiagnosticStatus(name='openvins/visual_tracking',values=[KeyValue(key='tracked_features',value='35'),KeyValue(key='initialized',value='1')])];tracks.publish(d)
   s=stamped(LocalizationStatus(),'openvins_odom');s.backend_id='openvins';s.session_id='contract-session';s.tracked_frame='livox_imu_frame';s.state=2;s.pose_valid=True;s.pose_stamp=m.header.stamp;s.covariance_available=True;status.publish(s)
 def until(predicate,seconds=15,publish=True):
  nonlocal last
  deadline=time.monotonic()+seconds
  while not predicate():
   assert time.monotonic()<deadline,states[-1:] or (tmp_path/'manager.log').read_text()
   assert process.poll() is None,(tmp_path/'manager.log').read_text()
   if publish and time.monotonic()-last>=.05:emit();last=time.monotonic()
   rclpy.spin_once(peer,timeout_sec=.005)
 def duration(seconds,publish=True):
  start=time.monotonic();until(lambda:time.monotonic()-start>=seconds,seconds+2,publish)
 def activate():
  future=client.call_async(Trigger.Request());until(future.done);return future.result()
 try:
  until(lambda:client.service_is_ready() and states and states[-1]['parameters_verified'])
  assert not outputs
  parameters['EKF2_EV_NOISE_MD']=0
  until(lambda:not states[-1]['parameters_verified'],3)
  assert not activate().success and not outputs
  parameters['EKF2_EV_NOISE_MD']=1
  until(lambda:states[-1]['parameters_verified'],3)
  armed=True;duration(.25);assert not activate().success
  armed=False;duration(3.);reply=activate();assert reply.success,(reply.message,states[-1])
  until(lambda:states[-1]['healthy'] and len(outputs)>=5)
  assert states[-1]['selected']=='fast_lio'
  activation=states[-1]['alignment']
  absent={'fast_lio'};until(lambda:states[-1]['selected']=='openvins',3)
  absent=set();until(lambda:states[-1]['selected']=='fast_lio',8)
  absent={'fast_lio','openvins'};until(lambda:not states[-1]['healthy'],3);duration(.2);count=len(outputs)
  duration(.2);assert len(outputs)==count
  absent=set();until(lambda:states[-1]['healthy'],8)
  # Stop ALL original messages and /clock. The steady timer must revoke output.
  until(lambda:bool(states[-1]['failure']),3,publish=False);duration(.2,publish=False);count=len(outputs)
  duration(.2,publish=False);assert len(outputs)==count
  assert states[-1]['alignment']==activation
  assert 'expired' in states[-1]['failure']
  assert not activate().success
  stamps=[m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in outputs]
  assert all(a<b for a,b in zip(stamps,stamps[1:]))
  assert all(all(math.isnan(v) for v in m.pose.covariance) for m in outputs)
  assert all(math.dist([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],[0.,0.,.1])<1e-9 for m in outputs)
  assert '/tf' not in dict(peer.get_publisher_names_and_types_by_node('localization_failover','/'))
 finally:
  if process.poll() is None:process.send_signal(signal.SIGINT)
  process.wait(timeout=10);log.close();peer.destroy_node();rclpy.shutdown()
 assert process.returncode==0,(tmp_path/'manager.log').read_text()


def test_native_discovery_metadata_is_not_endpoint_identity():
 from types import SimpleNamespace
 from easyfly_localization_failover.node import endpoint_key
 # Exact retained native before/after discovery from identity-capture case.
 gid=bytes.fromhex('010f87e71c726eab00000000000019030000000000000000')
 early=SimpleNamespace(node_name='_NODE_NAME_UNKNOWN_',node_namespace='_NODE_NAMESPACE_UNKNOWN_',endpoint_gid=gid)
 discovered=SimpleNamespace(node_name='laser_mapping',node_namespace='/',endpoint_gid=gid)
 assert endpoint_key('/Odometry',early)==endpoint_key('/Odometry',discovered)
 restarted=SimpleNamespace(node_name='laser_mapping',node_namespace='/',endpoint_gid=bytes.fromhex('010f87e71c726eab00000000000020030000000000000000'))
 assert endpoint_key('/Odometry',restarted)!=endpoint_key('/Odometry',discovered)
