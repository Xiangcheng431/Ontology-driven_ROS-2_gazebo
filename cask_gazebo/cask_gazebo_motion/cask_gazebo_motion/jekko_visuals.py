"""Python-only Jekko cable display. No drive, joint or cargo commands."""
import math
import re
import shutil
import subprocess
import threading
import time


class JekkoCableVisual:
    def __init__(self, logger):
        self.logger = logger
        self._condition = threading.Condition()
        self._latest = None
        self._closed = False
        self._process = None
        self._thread = None
        self._last_warning = -math.inf
        self._ign = shutil.which('ign')
        if self._ign is None:
            logger.warning('Jekko cable display: ign is missing; display disabled')
            return
        self._thread = threading.Thread(target=self._run,
                                        name='jekko_python_cable', daemon=True)
        self._thread.start()
        logger.info('Jekko cable: Python renderer, link-local marker, nonblocking feedback')

    def update(self, base, orientation, joints):
        # Use the existing ROS joint callback; no additional subscriber/node.
        q = joints.get('jekko_hoist_joint')
        if q is None:
            return
        try:
            q = float(q)
        except (TypeError, ValueError):
            return
        if not math.isfinite(q) or not -.01 <= q <= 3.02:
            return
        with self._condition:
            if not self._closed:
                self._latest = q
                self._condition.notify()

    @staticmethod
    def _marker_request(q):
        length = q + .46
        # The parent link makes the cable follow the chassis/boom directly in
        # Gazebo. Only length changes require a request, never world-pose updates.
        return (
            'marker { ns: "cask_jekko_cable" id: 1 action: ADD_MODIFY '
            'type: CYLINDER parent: "jekko::jekko_cable_head_link" '
            f'pose {{ position {{ z: {-length / 2.:.12g} }} '
            'orientation { w: 1 } } '
            f'scale {{ x: .014 y: .014 z: {length:.12g} }} '
            'material { ambient { r: .10 g: .10 b: .12 a: 1 } '
            'diffuse { r: .16 g: .16 b: .18 a: 1 } } }'
        )

    def _send(self, q):
        # One bounded CLI request at a time, outside the robot callback thread.
        command = [self._ign, 'service', '-s', '/marker_array',
                   '--reqtype', 'ignition.msgs.Marker_V',
                   '--reptype', 'ignition.msgs.Boolean',
                   '--timeout', '1000', '--req', self._marker_request(q)]
        process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        with self._condition:
            self._process = process
            closed = self._closed
        until = time.monotonic() + 1.5
        try:
            while not closed:
                try:
                    output, _ = process.communicate(timeout=.05)
                    return process.returncode == 0 and bool(
                        re.search(r'\bdata:\s*true\b', output))
                except subprocess.TimeoutExpired:
                    with self._condition:
                        closed = self._closed
                    if time.monotonic() >= until:
                        break
            return False
        finally:
            if process.poll() is None:
                process.kill()
            process.communicate()
            with self._condition:
                if self._process is process:
                    self._process = None

    def _run(self):
        last = None
        sent = -math.inf
        retry_after = 0.
        while True:
            with self._condition:
                while True:
                    if self._closed:
                        return
                    q = self._latest
                    now = time.monotonic()
                    changed = q is not None and (last is None or abs(q-last) > .0001)
                    if q is not None and now >= retry_after and (changed or now-sent >= 1.):
                        break
                    self._condition.wait(timeout=.05)
            sent = time.monotonic()
            try:
                success = self._send(q)
            except (OSError, subprocess.SubprocessError):
                success = False
            if success:
                last = q
                retry_after = sent + .05  # bound process starts to at most 20 Hz
            else:
                retry_after = time.monotonic() + .25
                now = time.monotonic()
                with self._condition:
                    closed = self._closed
                if not closed and now-self._last_warning >= 5.:
                    self.logger.warning('Jekko cable display request failed; retrying display only')
                    self._last_warning = now

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.)
