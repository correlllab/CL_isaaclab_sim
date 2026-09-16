"""Private IPC between the physics thread and the native ROS gripper process."""
import copy
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import threading
import time


class MagpieCommandBuffer:
    """Validated targets; only the physics thread acknowledges application."""
    def __init__(self):
        self.lock = threading.RLock()
        self.commands = {side: dict(aperture=110., force=5., speed=1., sequence=0) for side in ('left', 'right')}
        self.applied = dict(left=0, right=0)

    def submit(self, side, values):
        if side not in self.commands:
            raise ValueError('side must be left or right')
        if not values or set(values) - {'aperture', 'force', 'speed'}:
            raise ValueError('expected aperture, force, or speed')
        limits = {'aperture': (0., 110.), 'force': (0., 100.), 'speed': (0., 1.)}
        normalized = {}
        for key, value in values.items():
            value = float(value)
            low, high = limits[key]
            if not math.isfinite(value) or not low <= value <= high or (key == 'speed' and value == 0):
                raise ValueError(f'{key} outside supported range {limits[key]}')
            normalized[key] = value
        with self.lock:
            self.commands[side].update(normalized)
            self.commands[side]['sequence'] += 1
            return self.commands[side]['sequence']

    def get_commands(self):
        with self.lock:
            return copy.deepcopy(self.commands)

    def acknowledge(self, commands):
        with self.lock:
            for side, values in commands.items():
                sequence = int(values['sequence'])
                if sequence <= self.commands[side]['sequence']:
                    self.applied[side] = max(self.applied[side], sequence)

    def is_applied(self, side, sequence):
        with self.lock:
            return self.applied[side] == sequence


class MagpieBridge:
    """ROS callbacks cannot touch tensors; all physics access stays in main.

    get_commands/acknowledge are called around applying physical actuator
    targets. publish takes measured state dictionaries after a physics step.
    """
    def __init__(self):
        domain = int(os.environ.get('ROS_DOMAIN_ID', '1'))
        if not 1 <= domain <= 232:
            raise ValueError('Magpie simulator requires ROS_DOMAIN_ID 1..232')
        self.buffer = MagpieCommandBuffer()
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._pending = {}
        self._states = {}
        self._last_publish = -math.inf
        self._closed = threading.Event()
        self._ready = threading.Event()
        self._connection = None
        self._process = None
        self._directory = tempfile.TemporaryDirectory(prefix='golem-magpie-')
        self._listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        endpoint = str(Path(self._directory.name) / 'physics.sock')
        self._listener.bind(endpoint)
        self._listener.listen(1)
        self._listener.settimeout(.2)
        self._thread = threading.Thread(target=self._receive, name='magpie-physics-ipc', daemon=True)
        self._thread.start()
        try:
            launcher = Path(__file__).with_name('magpie_launch.sh')
            self._process = subprocess.Popen(['bash', str(launcher), endpoint], start_new_session=True)
            deadline = time.monotonic() + 180.
            while not self._ready.wait(.1):
                if self._process.poll() is not None:
                    raise RuntimeError(f'Magpie ROS process exited with {self._process.returncode}')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Magpie ROS process did not become ready in 180 seconds')
        except BaseException:
            self.close()
            raise

    def _send(self, value):
        payload = (json.dumps(value, allow_nan=False) + '\n').encode()
        with self._send_lock:
            if self._connection is not None:
                self._connection.sendall(payload)

    def _receive(self):
        try:
            while not self._closed.is_set():
                try:
                    connection, _ = self._listener.accept()
                    break
                except socket.timeout:
                    continue
            else:
                return
            self._connection = connection
            with connection.makefile('r') as stream:
                for line in stream:
                    value = json.loads(line)
                    if value.get('type') == 'ready':
                        self._ready.set()
                        continue
                    request_id = value['id']
                    try:
                        sequence = self.buffer.submit(value['side'], value['values'])
                        with self._lock:
                            self._pending[request_id] = (value['side'], sequence)
                    except (ValueError, KeyError, TypeError) as error:
                        self._send(dict(type='ack', id=request_id, success=False, message=str(error)))
        except (OSError, ValueError):
            if not self._closed.is_set():
                self._closed.set()

    def get_commands(self):
        if self._process is not None and self._process.poll() is not None:
            raise RuntimeError('Magpie ROS process stopped unexpectedly')
        return self.buffer.get_commands()

    def acknowledge(self, commands):
        self.buffer.acknowledge(commands)
        with self._lock:
            ready = [(key, side, seq) for key, (side, seq) in self._pending.items() if self.buffer.applied[side] >= seq]
            for key, side, sequence in ready:
                self._pending.pop(key)
                state = self._states.get(side)
                applied = self.buffer.is_applied(side, sequence)
                self._send(dict(type='ack', id=key, success=state is not None and applied,
                                message=('Actuator target applied' if state is not None else 'Measured state not available') if applied else 'Target superseded before application',
                                state=state))

    def publish(self, sim_time, states):
        # Only finite JSON-compatible measured values cross the process boundary.
        snapshot = json.loads(json.dumps(states, allow_nan=False))
        with self._lock:
            self._states = snapshot
        if sim_time < self._last_publish or sim_time - self._last_publish >= .1 - 1e-9:
            self._send(dict(type='state', sim_time=float(sim_time), states=snapshot))
            self._last_publish = sim_time

    def close(self):
        self._closed.set()
        if self._connection is not None:
            try:
                self._connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._connection.close()
        self._listener.close()
        if self._process is not None and self._process.poll() is None:
            import signal
            os.killpg(self._process.pid, signal.SIGTERM)
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self._process.pid, signal.SIGKILL)
                self._process.wait(timeout=5)
        self._thread.join(timeout=2)
        self._directory.cleanup()
