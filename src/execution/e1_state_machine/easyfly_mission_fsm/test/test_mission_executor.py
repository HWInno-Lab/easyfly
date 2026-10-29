"""Real MissionFSM + the real C++ SDK, with ROS action backends on domain 101."""
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from easyfly_msgs.action import ExecuteMission, Takeoff, Land, GoToWaypoint, NavigateTo, TrackTarget, SearchArea, PrecisionLand, Explore
from easyfly_msgs.msg import UAVState, MissionStepFeedback
from easyfly_msgs.srv import GetActionOwnership, SetFlightMode


class MissionExecutorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('d3_executor_mock')
        cls.group = ReentrantCallbackGroup()
        cls.calls = []
        cls.explore_goals = []
        cls.child_feedback = []
        cls.feedback_subscription = cls.node.create_subscription(
            MissionStepFeedback, "/mission/step_feedback", cls.child_feedback.append, 10)
        cls.behavior = 'success'
        cls.cancel_delay = .02
        cls.source_old = False
        cls.flight_mode = 3
        cls.position_z = 0.
        cls.battery = 1.
        cls.battery_valid = True
        cls.armed = False
        cls.mode_calls = []
        cls.ownership = {}
        cls.cancel_calls = 0
        cls.servers = {}
        cls.ownership_services=[]
        for endpoint in ('/navigation/ownership','/search/ownership','/precision_landing/ownership','/exploration/ownership'):
            def query(request,response):
                response.goal_id=request.goal_id
                release_at=cls.ownership.get(request.goal_id)
                response.state=0 if release_at is None else 2 if time.monotonic()>=release_at else 1
                return response
            cls.ownership_services.append(cls.node.create_service(GetActionOwnership,endpoint,query))
        for action, name in ((Takeoff, 'takeoff'), (Land, 'land'), (GoToWaypoint, 'goto_waypoint'),
                             (NavigateTo, 'navigate'), (TrackTarget, 'track_target'), (SearchArea, 'search_area'),
                             (PrecisionLand, 'precision_land'), (Explore, 'explore')):
            cls.add_server(action, name)
        cls.publisher = cls.node.create_publisher(UAVState, '/uav_state', 10)

        def mode(request, response):
            cls.mode_calls.append(request.flight_mode)
            response.success = True
            response.message = 'component backend acknowledged'
            return response

        cls.mode_service = cls.node.create_service(SetFlightMode, '/set_flight_mode', mode)

        def publish():
            state = UAVState(connected=True, pose_valid=True, battery_valid=cls.battery_valid,
                             armed=cls.armed,
                             battery_remaining=cls.battery, flight_mode=cls.flight_mode,
                             position_z=cls.position_z)
            state.header.frame_id = 'odom'
            state.header.stamp = cls.node.get_clock().now().to_msg()
            if cls.source_old:
                state.header.stamp.sec -= 10
            cls.publisher.publish(state)

        cls.timer = cls.node.create_timer(.02, publish)
        cls.client = ActionClient(cls.node, ExecuteMission, '/execute_mission')
        cls.executor = MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin)
        cls.thread.start()
        cls.temp = tempfile.TemporaryDirectory(prefix='easyfly-d3-executor-')
        cls.log = tempfile.TemporaryFile(mode='w+')
        cls.process = subprocess.Popen(['ros2', 'run', 'easyfly_mission_fsm', 'mission_fsm_node',
                                       '--ros-args', '-p', 'cancel_budget_s:=0.15',
                                       '-p', 'backend_discovery_s:=0.3',
                                       '-p', 'battery_invalid_grace_s:=0.1'],
                                      stdout=cls.log, stderr=cls.log, start_new_session=True)
        if not cls.client.wait_for_server(timeout_sec=8):
            cls.log.seek(0)
            raise RuntimeError(cls.log.read())
        time.sleep(.3)

    @classmethod
    def add_server(cls, action, name):
        def goal(request):
            if cls.behavior == 'late_accept' and name == 'navigate':
                time.sleep(.3)
            return GoalResponse.ACCEPT

        def cancel(handle):
            cls.cancel_calls += 1
            return CancelResponse.REJECT if cls.behavior == 'reject_cancel' else CancelResponse.ACCEPT

        def result_value(success, message):
            value = action.Result(success=success)
            setattr(value, 'reason' if action is Explore else 'message', message)
            return value

        def execute(handle):
            identity=bytes(handle.goal_id.uuid).hex()
            cls.ownership[identity]=float('inf')
            cls.calls.append(name)
            if name == 'explore':
                cls.explore_goals.append(handle.request)
            behavior = cls.behavior
            should_hold = (name == 'track_target' and behavior != 'early_track_success') or (
                name in ('navigate', 'explore') and behavior in ('hold', 'late_accept', 'reject_cancel'))
            feedback = action.Feedback()
            if hasattr(feedback, 'progress'):
                feedback.progress = .4
            if action is PrecisionLand:
                feedback.phase = 'align'
                feedback.target_valid = True
                feedback.horizontal_error_m = .12
                feedback.height_above_target_m = 1.3
            if action is Explore:
                feedback.phase = 'exploring'
                feedback.coverage = .4
                feedback.completed_viewpoints = 2
            for _ in range(5):
                handle.publish_feedback(feedback)
                time.sleep(.01)
            if name in ('navigate', 'precision_land', 'explore') and behavior=='managed_drain':
                cls.ownership[identity]=time.monotonic()+.6
                handle.abort()
                return result_value(False, 'public_failure_still_owned')
            if name=='navigate' and behavior=='late_success':
                time.sleep(.12)
            while should_hold and not handle.is_cancel_requested:
                time.sleep(.01)
                if cls.process.poll() is not None:
                    handle.abort()
                    return result_value(False, 'test_node_stopped')
                if behavior == 'reject_cancel' and cls.behavior != 'reject_cancel':
                    break
            result = action.Result()
            if handle.is_cancel_requested:
                time.sleep(cls.cancel_delay)
                handle.canceled()
                result = result_value(False, 'real_canceled')
            else:
                handle.succeed()
                result.success = behavior != 'false_success'
                result = result_value(result.success, 'done' if result.success else 'false_success')
            cls.ownership[identity]=time.monotonic()
            return result

        cls.servers[name] = ActionServer(cls.node, action, '/' + name, execute,
                                        goal_callback=goal, cancel_callback=cancel,
                                        callback_group=cls.group)

    @classmethod
    def tearDownClass(cls):
        if cls.process.poll() is None:
            os.killpg(cls.process.pid, signal.SIGINT)
        try:
            cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(cls.process.pid, signal.SIGKILL)
            cls.process.wait(timeout=5)
        cls.executor.shutdown(timeout_sec=5)
        cls.thread.join(timeout=5)
        for server in cls.servers.values():
            server.destroy()
        for service in cls.ownership_services:
            cls.node.destroy_service(service)
        cls.client.destroy()
        cls.node.destroy_service(cls.mode_service)
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.temp.cleanup()
        cls.log.close()

    def setUp(self):
        type(self).behavior = 'success'
        type(self).cancel_delay = .02
        type(self).source_old = False
        type(self).flight_mode = 3
        type(self).position_z = 0.
        type(self).battery = 1.
        type(self).battery_valid = True
        type(self).armed = False
        self.mode_calls.clear()
        time.sleep(.25)
        self.calls.clear()
        self.feedback = []

    def wait(self, future, timeout=5):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(.005)
        self.assertTrue(future.done())
        return future.result()

    def goal(self, steps):
        path = Path(self.temp.name) / 'mission.yaml'
        path.write_text('schema_version: 1\nframe_id: odom\nunits: {position: m, angle: rad, time: s}\nsteps:\n' + steps)
        return self.wait(self.client.send_goal_async(ExecuteMission.Goal(mission_file=str(path)),
                         feedback_callback=lambda feedback: self.feedback.append(feedback.feedback.current_status)))

    def result(self, handle):
        self.assertTrue(handle.accepted)
        return self.wait(handle.get_result_async())

    def test_all_seven_steps_are_strictly_ordered_and_track_requires_real_cancel(self):
        before = self.cancel_calls
        result = self.result(self.goal('  - type: takeoff\n    altitude: 1\n'
            '  - type: hover\n    duration: 0.03\n'
            '  - type: waypoint\n    x: 1\n    y: 0\n    z: 1\n'
            '  - type: navigate\n    x: 2\n    y: 0\n    z: 1\n'
            '  - type: track\n    target_id: -1\n    duration: 0.06\n'
            '  - type: search\n    center_x: 0\n    center_y: 0\n    area_width: 2\n    area_height: 2\n    altitude: 1\n    pattern: boustrophedon\n'
            '  - type: land\n'))
        self.assertEqual(result.status, 4)
        self.assertTrue(result.result.success)
        self.assertEqual(self.calls, ['takeoff', 'goto_waypoint', 'navigate', 'track_target', 'search_area', 'land'])
        self.assertGreater(self.cancel_calls, before)
        self.assertTrue(any(f.total_waypoints == 7 and f.progress > .5 for f in self.feedback))

    def test_false_success_aborts_and_does_not_auto_land(self):
        type(self).behavior = 'false_success'
        result = self.result(self.goal('  - type: takeoff\n    altitude: 1\n  - type: land\n'))
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertEqual(self.calls, ['takeoff'])

    def explore_step(self):
        return ('  - type: explore\n    min_bound: [-2, -2, 0.5]\n'
                '    max_bound: [2, 2, 2.5]\n    return_home: false\n    timeout: 5\n'
                '    forbidden_boxes: [{min: [1, 1, 0.5], max: [1.5, 1.5, 2]}]\n')

    def test_explore_transmits_bounds_and_finishes_before_land(self):
        result = self.result(self.goal(self.explore_step() + '  - type: land\n'))
        self.assertEqual(result.status, 4)
        self.assertTrue(result.result.success)
        self.assertEqual(self.calls, ['explore', 'land'])
        sent = self.explore_goals[-1]
        self.assertEqual((sent.min_bound.x, sent.max_bound.z), (-2., 2.5))
        self.assertFalse(sent.return_home)
        self.assertEqual(sent.timeout_sec, 5.)
        self.assertEqual(len(sent.forbidden_boxes), 2)
        self.assertEqual(sent.forbidden_boxes[1].x, 1.5)

    def test_explore_false_success_never_advances_to_land(self):
        type(self).behavior = 'false_success'
        result = self.result(self.goal(self.explore_step() + '  - type: land\n'))
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertIn('false_success', result.result.message)
        self.assertEqual(self.calls, ['explore'])

    def test_explore_public_failure_retains_child_ownership(self):
        type(self).behavior = 'managed_drain'
        result = self.result(self.goal(self.explore_step() + '  - type: land\n'))
        self.assertEqual(result.status, 6)
        self.assertEqual(self.calls, ['explore'])
        self.assertFalse(self.goal('  - type: hover\n    duration: 0.02\n').accepted)
        time.sleep(.8)
        self.assertTrue(self.result(self.goal('  - type: hover\n    duration: 0.02\n')).result.success)

    def test_explore_cancel_waits_for_real_terminal(self):
        type(self).behavior = 'hold'
        type(self).cancel_delay = .5
        handle = self.goal(self.explore_step() + '  - type: land\n')
        self.assertTrue(handle.accepted)
        end = time.monotonic() + 3
        while not self.calls and time.monotonic() < end:
            time.sleep(.01)
        self.assertEqual(self.calls, ['explore'])
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        self.assertEqual(self.result(handle).status, 5)
        self.assertFalse(self.goal('  - type: hover\n    duration: 0.02\n').accepted)
        time.sleep(.8)
        self.assertTrue(self.result(self.goal('  - type: hover\n    duration: 0.02\n')).result.success)
        self.assertNotIn('land', self.calls)

    def test_precision_step_reports_true_terminal_without_extra_land(self):
        self.child_feedback.clear()
        handle = self.goal('  - type: precision_land\n    tag_id: 7\n    timeout: 3\n')
        result = self.result(handle)
        identity = bytes(handle.goal_id.uuid).hex()
        received = [f for f in self.child_feedback if f.mission_goal_id == identity and f.active]
        self.assertTrue(received)
        self.assertTrue(all(f.child_goal_id != identity and len(f.child_goal_id) == 32 for f in received))
        self.assertTrue(all(f.step_type == 'precision_land' and f.phase == 'align' for f in received))
        self.assertTrue(all(f.target_valid and f.horizontal_error_m == .12 for f in received))
        self.assertTrue(all(0 <= f.feedback_age_sec <= .5 for f in received))
        self.assertEqual(result.status, 4)
        self.assertTrue(result.result.success)
        self.assertEqual(self.calls, ['precision_land'])

    def test_precision_public_failure_retains_recursive_owned_child(self):
        type(self).behavior = 'managed_drain'
        result = self.result(self.goal('  - type: precision_land\n    tag_id: 7\n    timeout: 3\n'))
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertEqual(self.calls, ['precision_land'])
        self.assertFalse(self.goal('  - type: hover\n    duration: 0.02\n').accepted)
        time.sleep(.8)
        self.assertTrue(self.result(self.goal('  - type: hover\n    duration: 0.02\n')).result.success)

    def test_timeout_keeps_fault_lock_until_real_late_cancel_terminal(self):
        type(self).behavior = 'late_accept'
        type(self).cancel_delay = .5
        result = self.result(self.goal('  - type: navigate\n    x: 1\n    y: 0\n    z: 1\n    timeout: 0.1\n'))
        self.assertEqual(result.status, 6)
        self.assertIn('timeout', result.result.message)
        self.assertFalse(self.goal('  - type: hover\n    duration: 0.02\n').accepted)
        time.sleep(.8)
        self.assertTrue(self.result(self.goal('  - type: hover\n    duration: 0.02\n')).result.success)

    def test_user_cancel_is_public_canceled_but_retains_child_ownership(self):
        type(self).behavior = 'hold'
        type(self).cancel_delay = .5
        handle = self.goal('  - type: navigate\n    x: 1\n    y: 0\n    z: 1\n')
        self.assertTrue(handle.accepted)
        end = time.monotonic() + 3
        while not self.calls and time.monotonic() < end:
            time.sleep(.01)
        self.assertTrue(self.wait(handle.cancel_goal_async()).goals_canceling)
        self.assertEqual(self.result(handle).status, 5)
        self.assertFalse(self.goal('  - type: hover\n    duration: 0.02\n').accepted)
        time.sleep(.7)
        self.assertTrue(self.result(self.goal('  - type: hover\n    duration: 0.02\n')).result.success)

    def test_track_early_success_is_not_duration_completion(self):
        type(self).behavior = 'early_track_success'
        result = self.result(self.goal('  - type: track\n    target_id: -1\n    duration: 1\n  - type: land\n'))
        self.assertEqual(result.status, 6)
        self.assertEqual(self.calls, ['track_target'])

    def test_rejected_cancel_keeps_fault_lock_until_real_result(self):
        type(self).behavior = 'reject_cancel'
        result = self.result(self.goal('  - type: navigate\n    x: 1\n    y: 0\n    z: 1\n    timeout: 0.1\n'))
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertFalse(self.goal('  - type: hover\n    duration: 0.02\n').accepted)
        # Deliberately allow the mock to finish without a cancel ACK. Only this
        # real result (not the rejected cancellation) can release ownership.
        type(self).behavior = 'success'
        time.sleep(.3)
        self.assertTrue(self.result(self.goal('  - type: hover\n    duration: 0.02\n')).result.success)

    def test_recursive_public_terminal_does_not_release_parent_until_owned_uuid_released(self):
        type(self).behavior='managed_drain'
        handle=self.goal('  - type: navigate\n    x: 1\n    y: 0\n    z: 1\n')
        result=self.result(handle)
        self.assertEqual(result.status,6)
        self.assertFalse(self.goal('  - type: hover\n    duration: 0.02\n').accepted)
        service=self.node.create_client(GetActionOwnership,'/mission/ownership')
        try:
            self.assertTrue(service.wait_for_service(timeout_sec=2))
            identity=bytes(handle.goal_id.uuid).hex()
            query=lambda:self.wait(service.call_async(GetActionOwnership.Request(goal_id=identity)))
            self.assertEqual(query().state,1)
            time.sleep(.8)
            self.assertEqual(query().state,2)
            self.assertTrue(self.result(self.goal('  - type: hover\n    duration: 0.02\n')).result.success)
        finally:
            self.node.destroy_client(service)

    def test_malformed_pose_while_hovering_aborts(self):
        handle=self.goal('  - type: hover\n    duration: 2\n')
        self.assertTrue(handle.accepted)
        type(self).position_z=float('nan')
        result=self.result(handle)
        self.assertEqual(result.status,6)
        self.assertIn('state_invalid',result.result.message)

    def test_malformed_valid_battery_uses_invalid_grace(self):
        for value in (float('nan'),1.1,-.1):
            type(self).battery=1.
            time.sleep(.25)
            handle=self.goal('  - type: hover\n    duration: 2\n')
            self.assertTrue(handle.accepted)
            type(self).battery=value
            result=self.result(handle)
            self.assertEqual(result.status,6)
            self.assertIn('battery_invalid_timeout',result.result.message)

    def test_success_after_monotonic_deadline_is_failure_not_step_completion(self):
        type(self).behavior='late_success'
        result=self.result(self.goal('  - type: navigate\n    x: 1\n    y: 0\n    z: 1\n    timeout: 0.1\n  - type: land\n'))
        self.assertEqual(result.status,6)
        self.assertIn('timeout',result.result.message)
        self.assertEqual(self.calls,['navigate'])

    def test_invalid_battery_airborne_aborts_and_requests_rtl_once(self):
        type(self).armed = True
        type(self).position_z = 1.
        time.sleep(.25)
        handle = self.goal('  - type: hover\n    duration: 2\n  - type: land\n')
        self.assertTrue(handle.accepted)
        type(self).battery_valid = False
        result = self.result(handle)
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertEqual(result.result.message, 'battery_invalid_timeout')
        time.sleep(.25)
        self.assertEqual(self.mode_calls, [6])
        self.assertFalse(self.calls)

    def test_low_battery_airborne_aborts_and_requests_rtl_once(self):
        type(self).armed = True
        type(self).position_z = 1.
        time.sleep(.25)
        handle = self.goal('  - type: hover\n    duration: 2\n  - type: land\n')
        self.assertTrue(handle.accepted)
        type(self).battery = .1
        result = self.result(handle)
        self.assertEqual(result.status, 6)
        self.assertFalse(result.result.success)
        self.assertEqual(result.result.message, 'low_battery')
        time.sleep(.25)
        self.assertEqual(self.mode_calls, [6])
        self.assertFalse(self.calls)

    def test_external_land_takeover_interrupts_hover(self):
        handle = self.goal('  - type: hover\n    duration: 2\n')
        self.assertTrue(handle.accepted)
        type(self).flight_mode = 7
        result = self.result(handle)
        self.assertEqual(result.status, 6)
        self.assertIn('takeover', result.result.message)
        self.assertFalse(self.calls)

    def test_old_source_state_rejected_despite_new_receipts(self):
        type(self).source_old = True
        time.sleep(.2)
        handle = self.goal('  - type: hover\n    duration: 0.02\n')
        self.assertFalse(handle.accepted)
        self.assertFalse(self.calls)
        self.log.seek(0)
        admission = [line for line in self.log.read().splitlines()
                     if 'mission_admission reject=state_not_ready' in line]
        self.assertTrue(admission)
        self.assertIn('goal_uuid=' + bytes(handle.goal_id.uuid).hex(), admission[-1])
        self.assertIn('fresh=0', admission[-1])
        self.assertIn('receipt_fresh=1', admission[-1])
        self.assertIn('source_time_valid=1', admission[-1])

    def test_missing_later_backend_is_detected_before_takeoff(self):
        self.servers.pop('search_area').destroy()
        try:
            time.sleep(.3)
            result = self.result(self.goal('  - type: takeoff\n    altitude: 1\n'
                '  - type: search\n    center_x: 0\n    center_y: 0\n'
                '    area_width: 2\n    area_height: 2\n    altitude: 1\n    pattern: boustrophedon\n'))
            self.assertEqual(result.status, 6)
            self.assertIn('backend', result.result.message)
            self.assertFalse(self.calls)
        finally:
            self.add_server(SearchArea, 'search_area')
            time.sleep(.3)


if __name__ == '__main__':
    unittest.main()
