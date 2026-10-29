"""ROS component contracts only; synthetic clocks/maps are not flight evidence."""
import copy
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from easyfly_msgs.msg import VoxelGrid
from easyfly_msgs.srv import ReserveNavigation, GetActionOwnership
from unique_identifier_msgs.msg import UUID
from std_srvs.srv import Trigger
from easyfly_fuel_exploration.node import Exploration
from easyfly_fuel_exploration.policy import Policy, geometry


class LifecycleClockTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p',
                         'evidence_directory:='+self.tmp.name])
        self.node=Exploration();self.executor=SingleThreadedExecutor();self.executor.add_node(self.node)

    def tearDown(self):
        self.executor.remove_node(self.node);self.node.destroy_node()
        self.executor.shutdown();rclpy.shutdown();self.tmp.cleanup()

    def spin_until(self, condition, timeout=2.):
        end=time.monotonic()+timeout
        while not condition() and time.monotonic()<end:self.executor.spin_once(timeout_sec=.01)
        self.assertTrue(condition())

    def test_management_idle_requires_all_owned_work_to_be_released(self):
        client = self.node.create_client(Trigger, '/exploration/management_idle')
        self.spin_until(client.service_is_ready)
        def query():
            future = client.call_async(Trigger.Request())
            self.spin_until(future.done)
            return future.result().success
        self.assertTrue(query())
        for name, value in [('admitted', True), ('active', object()), ('fault_lock', True),
                            ('worker', object()), ('lease_pending', object()),
                            ('child_pending', object())]:
            original = getattr(self.node, name, None)
            setattr(self.node, name, value)
            self.assertFalse(query(), name)
            self.assertIs(getattr(self.node, name), value)
            setattr(self.node, name, original)
        self.assertTrue(query())
        self.node.destroy_client(client)

    def test_frozen_simulation_clock_cannot_disable_wall_deadlines(self):
        # No /clock publisher: source time remains exactly zero throughout.
        self.assertEqual(self.node.get_clock().now().nanoseconds,0)
        begin=time.monotonic();future=self.executor.create_task(self.node.pause(.05))
        self.spin_until(future.done,.5)
        self.assertLess(time.monotonic()-begin,.5)
        self.assertEqual(self.node.get_clock().now().nanoseconds,0)

    def test_real_ros_clock_rollback_latches_before_any_new_navigation(self):
        pub=self.node.create_publisher(Clock,'/clock',10)
        self.spin_until(lambda:pub.get_subscription_count()>0)
        pub.publish(Clock(clock=Time(sec=100)));self.spin_until(lambda:self.node.get_clock().now().nanoseconds==100000000000)
        self.node.active=SimpleNamespace(is_cancel_requested=False,request=SimpleNamespace(timeout_sec=900.))
        self.node.ready=lambda:True
        self.node.start_source=100.;self.node.start_wall=time.monotonic();self.node.clock_last=100.
        pub.publish(Clock(clock=Time(sec=99)));self.spin_until(lambda:self.node.get_clock().now().nanoseconds==99000000000)
        with self.assertRaisesRegex(RuntimeError,'CLOCK_RESET'):self.node.check()
        self.assertEqual(self.node.error,'CLOCK_RESET')

    def test_map_generation_change_latches_even_if_new_pair_is_fresh_and_valid(self):
        raw=VoxelGrid();raw.header.frame_id='odom';raw.header.stamp.sec=10
        raw.session_id='synthetic';raw.generation=1;raw.revision=1;raw.valid=True;raw.resolution=.2
        raw.size_x=raw.size_y=20;raw.size_z=10;raw.origin.x=raw.origin.y=-2.;raw.origin.z=.4
        raw.data=[0]*4000;inflated=copy.deepcopy(raw);inflated.inflation_radius=.65
        self.node.policy=Policy(raw,[-1.8,-1.8,.6],[1.8,1.8,2.4],[])
        self.node.active=SimpleNamespace(is_cancel_requested=False)
        self.node.map_epoch=(raw.session_id,raw.generation,geometry(raw))
        raw.generation=inflated.generation=2
        self.node.on_map(raw,0);self.node.on_map(inflated,1)
        self.assertEqual(self.node.error,'SOURCE_MAP_EPOCH_CHANGED')

    def delayed_lease(self, unresolved):
        # Real ROS RPC, accelerated component deadlines; no flight or algorithm input.
        owned={'active':False};original=self.node.bounded
        async def service(req,response):
            if req.active:await self.node.pause(.15)
            owned['active']=req.active;response.success=True;return response
        server=self.node.create_service(ReserveNavigation,'/navigation/reserve',service,
                                        callback_group=self.node.group)
        self.spin_until(self.node.lease.service_is_ready)
        self.node.bounded=lambda future,seconds=3.:original(future,
            .03 if seconds==3. else .04 if unresolved else .5)
        goal=SimpleNamespace(goal_id=UUID(uuid=list(uuid.uuid4().bytes)),
            request=SimpleNamespace(timeout_sec=900.),is_cancel_requested=False,
            abort=lambda:None,publish_feedback=lambda _:None)
        future=self.executor.create_task(self.node.execute(goal))
        self.spin_until(future.done,1.)
        result=future.result();self.assertIn('RPC_TIMEOUT',result.reason)
        identity=bytes(goal.goal_id.uuid).hex()
        if unresolved:
            self.assertTrue(self.node.fault_lock)
            self.assertEqual(self.node.owned_missions[identity],GetActionOwnership.Response.OWNED)
            self.spin_until(lambda:owned['active'])
            self.assertTrue(self.node.fault_lock,'late grant cannot release the failed parent')
        else:
            self.assertFalse(owned['active'],'late granted lease must be explicitly released')
            self.assertFalse(self.node.fault_lock)
            self.assertEqual(self.node.owned_missions[identity],GetActionOwnership.Response.RELEASED)
        self.node.destroy_service(server)

    def test_late_lease_grant_is_resolved_and_released_before_parent_terminal(self):
        self.delayed_lease(False)

    def test_unconfirmed_lease_grant_retains_fault_ownership_after_late_response(self):
        self.delayed_lease(True)


if __name__=='__main__':unittest.main()


class ViewBindingLifecycleTest(unittest.TestCase):
    setUp=LifecycleClockTest.setUp
    tearDown=LifecycleClockTest.tearDown
    spin_until=LifecycleClockTest.spin_until
    def delayed_view(self, unresolved):
        from easyfly_msgs.srv import SetExplorationView
        owned={'view':False,'lease':True};events=[]
        original=self.node.bounded
        async def service(request,response):
            if request.active:await self.node.pause(.15)
            owned['view']=request.active;response.success=True;return response
        server=self.node.create_service(SetExplorationView,'/planning/fuel_viewpoint',service,
                                        callback_group=self.node.group)
        self.node.extended=True
        self.node.view_service=self.node.create_client(SetExplorationView,'/planning/fuel_viewpoint',callback_group=self.node.group)
        self.spin_until(self.node.view_service.service_is_ready)
        self.node.mission_id='a'*32
        self.node.state=SimpleNamespace(header=VoxelGrid().header)
        self.node.child_pending=None;self.node.child_result=None
        self.node.event=lambda phase,**kwargs:events.append(phase)
        async def reserve(child='',active=True):owned['lease']=active
        self.node.reserve=reserve
        self.node.bounded=lambda future,seconds=3.:original(future,
            .03 if seconds==3. else .04 if unresolved else .5)
        future=self.executor.create_task(self.node.bind_view('b'*32,[2.,0.,1.3],.5))
        self.spin_until(future.done)
        with self.assertRaisesRegex(RuntimeError,'RPC_TIMEOUT'):future.result()
        cleanup=self.executor.create_task(self.node.cleanup(True))
        self.spin_until(cleanup.done)
        if unresolved:
            with self.assertRaisesRegex(RuntimeError,'RPC_TIMEOUT'):cleanup.result()
            self.assertTrue(owned['lease']);self.assertEqual(self.node.view_id,'b'*32)
            self.spin_until(lambda:owned['view'])
            self.assertTrue(owned['lease'],'late binding cannot release the parent')
        else:
            cleanup.result()
            self.assertFalse(owned['view']);self.assertFalse(owned['lease'])
            self.assertEqual(self.node.view_id,'');self.assertIn('VIEW_BINDING_RELEASED',events)
        self.node.destroy_client(self.node.view_service);self.node.destroy_service(server)

    def test_late_view_binding_is_cleared_before_navigation_lease_release(self):
        self.delayed_view(False)

    def test_unresolved_view_binding_keeps_exact_ownership(self):
        self.delayed_view(True)


class NavigationAdmissionLifecycleTest(unittest.TestCase):
    setUp=LifecycleClockTest.setUp
    tearDown=LifecycleClockTest.tearDown
    spin_until=LifecycleClockTest.spin_until

    def test_late_second_child_cannot_reuse_previous_terminal(self):
        from easyfly_msgs.action import NavigateTo
        from rclpy.action import ActionServer,GoalResponse,CancelResponse
        from rclpy.task import Future
        observed={'canceled':False,'terminal':False,'lease':True}
        async def admit(request):
            await self.node.pause(.15)
            return GoalResponse.ACCEPT
        def cancel(handle):
            observed['canceled']=True
            return CancelResponse.ACCEPT
        async def execute(handle):
            while not handle.is_cancel_requested:await self.node.pause(.01)
            handle.canceled();observed['terminal']=True
            return NavigateTo.Result(success=False,message='canceled owned delayed child')
        server=ActionServer(self.node,NavigateTo,'/navigate',execute_callback=execute,
            goal_callback=admit,cancel_callback=cancel,callback_group=self.node.group)
        self.spin_until(self.node.nav.server_is_ready)
        self.node.check=lambda:None
        self.node.policy=SimpleNamespace(contains=lambda _:True)
        self.node.event=lambda *args,**kwargs:None
        self.node.mission_id='a'*32
        self.node.child_pending=None
        previous=Future();previous.set_result(object());self.node.child_result=previous
        async def reserve(child='',active=True):observed['lease']=active
        async def released(child):
            self.assertTrue(observed['terminal'],'must confirm the new child terminal')
            self.node.child=None
        self.node.reserve=reserve;self.node.release_child=released
        original=self.node.bounded
        self.node.bounded=lambda future,seconds=3.:original(future,.04 if seconds==3. else .5)
        future=self.executor.create_task(self.node.navigate([2.,0.,1.3],'VIEWPOINT'))
        self.spin_until(future.done)
        with self.assertRaisesRegex(RuntimeError,'RPC_TIMEOUT'):future.result()
        cleanup=self.executor.create_task(self.node.cleanup(True));self.spin_until(cleanup.done)
        cleanup.result()
        self.assertTrue(observed['canceled']);self.assertTrue(observed['terminal'])
        self.assertFalse(observed['lease']);self.assertIsNone(self.node.child)
        server.destroy()
