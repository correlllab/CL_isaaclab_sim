"""Native ROS process for Magpie services/actions; physics lives in Isaac."""
import json
import math
import socket
import sys
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from magpie_msgs.action import DeliGrasp
from magpie_msgs.msg import GripperState
from magpie_msgs.srv import SetGripperForce, SetGripperPosition


class PhysicsClient:
    def __init__(self, endpoint):
        self.connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.connection.connect(endpoint)
        self.lock = threading.Condition()
        self.send_lock = threading.Lock()
        self.states = {}
        self.sim_time = 0.
        self.received_at = -math.inf
        self.responses = {}
        self.closed = False
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def send(self, value):
        with self.send_lock:
            self.connection.sendall((json.dumps(value, allow_nan=False) + '\n').encode())

    def _receive(self):
        try:
            with self.connection.makefile('r') as stream:
                for line in stream:
                    value = json.loads(line)
                    with self.lock:
                        if value['type'] == 'state':
                            self.states = value['states']
                            self.sim_time = value['sim_time']
                            self.received_at = time.monotonic()
                        else:
                            self.responses[value['id']] = value
                        self.lock.notify_all()
        finally:
            with self.lock:
                self.closed = True
                self.lock.notify_all()

    def snapshot(self, side):
        with self.lock:
            if self.closed or time.monotonic() - self.received_at > 5 or side not in self.states:
                raise RuntimeError('Fresh measured gripper state unavailable')
            return dict(self.states[side])

    def command(self, side, values):
        self.snapshot(side)
        request_id = uuid.uuid4().hex
        self.send(dict(id=request_id, side=side, values=values))
        deadline = time.monotonic() + 10
        with self.lock:
            while request_id not in self.responses:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self.closed:
                    raise RuntimeError('Physics did not acknowledge actuator command')
                self.lock.wait(remaining)
            response = self.responses.pop(request_id)
        if not response['success']:
            raise RuntimeError(response['message'])
        return response['state']

    def close(self):
        self.connection.shutdown(socket.SHUT_RDWR)
        self.connection.close()
        self.thread.join(timeout=2)


class GripperNode(Node):
    def __init__(self, side, physics):
        super().__init__(f'isaac_magpie_{side}')
        self.side, self.physics = side, physics
        self.operations = threading.Lock()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.group = ReentrantCallbackGroup()
        root = f'/{side}/gripper'
        self.publisher = self.create_publisher(GripperState, root + '/state', 10)
        self.create_timer(.1, self.publish_state, callback_group=self.group)
        self._service_handles = []
        for name, message_type, callback in (
            ('open', Trigger, self.open), ('close', Trigger, self.close_gripper),
            ('set_position', SetGripperPosition, self.set_position),
            ('set_force', SetGripperForce, self.set_force),
            ('calibrate', Trigger, self.calibrate), ('reset_parameters', Trigger, self.reset_parameters),
        ):
            self._service_handles.append(self.create_service(message_type, root + '/' + name, callback, callback_group=self.group))
        self.action = ActionServer(self, DeliGrasp, root + '/deligrasp', execute_callback=self.execute,
                                   goal_callback=self.accept_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)

    def publish_state(self):
        try:
            state = self.physics.snapshot(self.side)
        except RuntimeError:
            return
        msg = GripperState()
        msg.position, msg.force = float(state['position']), float(state['force'])
        msg.temperature = 25.  # No thermal model in the simulator.
        msg.is_moving, msg.contact_detected = bool(state['is_moving']), bool(state['contact_detected'])
        msg.finger_positions = list(map(float, state['finger_positions']))
        self.publisher.publish(msg)

    def apply(self, response, values, message='Actuator target applied'):
        try:
            with self.operations:
                with self.goal_lock:
                    if self.goal_active:
                        raise RuntimeError('DeliGrasp is active; cancel it before a service command')
                state = self.physics.command(self.side, values)
            response.success, response.message = True, message
            if hasattr(response, 'actual_position'):
                response.actual_position = float(state['position'])
        except (RuntimeError, ValueError, OSError) as error:
            response.success, response.message = False, str(error)
        return response

    def open(self, request, response):
        return self.apply(response, {'aperture': 110.})

    def close_gripper(self, request, response):
        return self.apply(response, {'aperture': 0.})

    def set_position(self, request, response):
        return self.apply(response, {'aperture': request.position, 'speed': request.speed})

    def set_force(self, request, response):
        return self.apply(response, {'force': request.max_force})

    def calibrate(self, request, response):
        return self.apply(response, {'aperture': 110.}, 'Open target applied; aperture calibration comes from asset geometry')

    def reset_parameters(self, request, response):
        return self.apply(response, {'force': 5., 'speed': 1.}, 'Default force and speed applied')

    def accept_goal(self, goal):
        p = goal.params
        valid = all(math.isfinite(v) for v in (p.goal_aperture, p.initial_force, p.additional_closure, p.additional_force))
        valid &= 0 <= p.goal_aperture <= 110 and 0 <= p.initial_force <= 100
        valid &= 0 <= p.additional_closure <= 110 and 0 <= p.additional_force <= 100
        try:
            self.physics.snapshot(self.side)
        except RuntimeError:
            return GoalResponse.REJECT
        with self.goal_lock:
            if not valid or self.goal_active:
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def execute(self, goal):
        result = DeliGrasp.Result()
        force_log = []
        state = None
        p = goal.request.params
        try:
            with self.operations:
                target, force = float(p.goal_aperture), float(p.initial_force)
                self.physics.command(self.side, {'aperture': target, 'force': force})
                deadline = time.monotonic() + 30
                while True:
                    state = self.physics.snapshot(self.side)
                    if goal.is_cancel_requested:
                        self.physics.command(self.side, {'aperture': state['position']})
                        goal.canceled()
                        result.message = 'Cancelled; holding measured aperture'
                        return self.finish(result, state, force_log)
                    self.feedback(goal, state, 'approach')
                    if abs(state['position'] - target) <= 2.:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError('Measured aperture did not reach the approach target')
                    time.sleep(.05)
                if not p.complete_grasp:
                    result.success, result.message = True, 'Approach aperture reached; force closure not requested'
                    goal.succeed()
                    return self.finish(result, state, force_log)
                for iteration in range(8):
                    state = self.physics.snapshot(self.side)
                    if goal.is_cancel_requested:
                        self.physics.command(self.side, {'aperture': state['position']})
                        goal.canceled()
                        result.message = 'Cancelled; holding measured aperture'
                        return self.finish(result, state, force_log)
                    force_log.append(float(state['force']))
                    self.feedback(goal, state, 'initial_close' if iteration == 0 else 'additional_close')
                    if state['contact_detected'] and state['force'] >= force:
                        result.success, result.message = True, 'Measured contact force reached'
                        goal.succeed()
                        return self.finish(result, state, force_log)
                    target = max(0., target - float(p.additional_closure))
                    force = min(100., force + float(p.additional_force))
                    self.physics.command(self.side, {'aperture': target, 'force': force})
                    time.sleep(.2)
                raise RuntimeError('Contact force not reached within eight closure steps')
        except (RuntimeError, ValueError, OSError) as error:
            result.success, result.message = False, str(error)
            goal.abort()
            return self.finish(result, state, force_log)
        finally:
            with self.goal_lock:
                self.goal_active = False

    @staticmethod
    def finish(result, state, force_log):
        if state is not None:
            result.final_aperture, result.final_force = float(state['position']), float(state['force'])
        result.force_log = force_log
        return result

    @staticmethod
    def feedback(goal, state, phase):
        msg = DeliGrasp.Feedback()
        msg.current_aperture, msg.current_force = float(state['position']), float(state['force'])
        msg.phase = phase
        goal.publish_feedback(msg)


def main():
    rclpy.init()
    physics = PhysicsClient(sys.argv[1])
    nodes = [GripperNode(side, physics) for side in ('left', 'right')]
    executor = MultiThreadedExecutor(num_threads=8)
    for node in nodes:
        executor.add_node(node)
    physics.send(dict(type='ready'))
    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in nodes:
            node.action.destroy()
            node.destroy_node()
        physics.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
