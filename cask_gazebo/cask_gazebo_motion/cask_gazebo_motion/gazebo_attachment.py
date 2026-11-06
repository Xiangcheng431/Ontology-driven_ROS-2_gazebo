"""Load Fortress's native DetachableJoint only after the hook is aligned.

Boolean service success acknowledges queuing, not physical attachment. The
caller must wait for the native plugin's StringMsg state before lifting.
"""
import json
import re
import subprocess
import time
import threading


class NativeAttachment:
    TOPIC = '/cask/crane/cargo'

    def __init__(self, world='empty', child_model='task001_rebar', topic=None,
                 parent_model='crane', parent_link='crane_hook_link'):
        self.world = world
        for name in (world,parent_model,parent_link):
            if not re.fullmatch(r'[A-Za-z0-9_]+',name):
                raise ValueError('Invalid attachment entity')
        self.parent_model,self.parent_link = parent_model,parent_link
        if not re.fullmatch(r"[A-Za-z0-9_]+", child_model):
            raise ValueError("Invalid cargo model")
        self.child_model = child_model
        self.TOPIC = topic or type(self).TOPIC
        self.loaded = False
        self._observer = None
        self._observer_thread = None
        self._state_lock = threading.Lock()
        self._observed_revision = 0
        self._observed_state = None
        self._observer_error = None

    def state_snapshot(self):
        """Read a cache populated only by this native plugin's output topic."""
        with self._state_lock:
            return self._observed_revision, self._observed_state

    def _record_state(self, line):
        match = re.fullmatch(r'\s*data:\s*"(attached|detached)"\s*', line)
        if match:
            with self._state_lock:
                self._observed_revision += 1
                self._observed_state = match[1]

    def _start_observer(self):
        # Task001 keeps its existing attachment path. Observe Task002/005
        # native states before loading the plugin, not after releasing.
        if self.child_model == 'task001_rebar':  # CASK_GENERATED_IOC_V1
            return
        if self._observer is not None and self._observer.poll() is None:
            return
        self.close()
        with self._state_lock:
            self._observed_state = None
        try:
            process = subprocess.Popen(
                ['ign', 'topic', '-e', '-t', self.TOPIC+'/state'],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, bufsize=1)
        except OSError as error:
            self._observer_error = str(error)
            return  # ROS confirmation remains mandatory if observation fails.
        self._observer = process
        self._observer_error = None

        def consume():
            try:
                for line in process.stdout:
                    self._record_state(line)
            finally:
                process.stdout.close()

        self._observer_thread = threading.Thread(target=consume, daemon=True)
        self._observer_thread.start()

    def close(self):
        process, self._observer = self._observer, None
        if process is not None:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.)
        thread, self._observer_thread = self._observer_thread, None
        if thread is not None:
            thread.join(timeout=1.)

    @staticmethod
    def run(arguments, tick=None, confirmed=None):
        if tick is not None:
            return NativeAttachment.run_with_feedback(arguments, tick, confirmed)
        try:
            result = subprocess.run(arguments, capture_output=True, text=True, timeout=8)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError('Gazebo transport command timed out; check simulator playback') from error
        if result.returncode:
            raise RuntimeError('Gazebo transport command failed: '+result.stderr.strip())
        return result.stdout

    @staticmethod
    def run_with_feedback(arguments, tick, confirmed=None, deadline=None):
        """Keep the protocol alive while transport discovery/publishing blocks."""
        tick()  # Check authorization before launching the command.
        process = subprocess.Popen(arguments, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic()+8. if deadline is None else deadline

        def stop():
            if process.poll() is None:
                process.terminate()
            try:
                return process.communicate(timeout=1.)
            except subprocess.TimeoutExpired:
                process.kill()
                return process.communicate()

        try:
            while True:
                tick()
                # A fresh native state confirms physical completion even if
                # the command-line publisher is still waiting before exiting.
                if confirmed is not None and confirmed():
                    return stop()[0]
                remaining = deadline-time.monotonic()
                if remaining <= 0.:
                    raise RuntimeError('Gazebo transport command timed out; no native detach confirmation')
                try:
                    output, error = process.communicate(timeout=min(.1,remaining))
                except subprocess.TimeoutExpired:
                    continue
                if process.returncode:
                    raise RuntimeError('Gazebo transport command failed: '+error.strip())
                return output
        finally:
            if process.poll() is None:
                stop()

    def attach(self):
        self._start_observer()
        if self.loaded:
            self.run(['ign', 'topic', '-t', self.TOPIC+'/attach',
                      '-m', 'ignition.msgs.Empty', '-p', ''])
            return
        output = self.run(['ign', 'model', '-m', self.parent_model, '--pose'])
        match = re.search(r'Model:\s*\[(\d+)\]', output)
        if not match:
            raise RuntimeError('Cannot find crane entity ID; Gazebo world must contain model crane')
        xml = (f'<parent_link>{self.parent_link}</parent_link>'
               f'<child_model>{self.child_model}</child_model>'
               '<child_link>cargo_link</child_link>'
               f'<attach_topic>{self.TOPIC}/attach</attach_topic>'
               f'<detach_topic>{self.TOPIC}/detach</detach_topic>'
               f'<output_topic>{self.TOPIC}/state</output_topic>')
        request = (f'entity {{ id: {int(match[1])} }} plugins {{ '
                   'filename: "ignition-gazebo-detachable-joint-system" '
                   'name: "ignition::gazebo::systems::DetachableJoint" '
                   f'innerxml: {json.dumps(xml)} }}')
        response = self.run(['ign', 'service', '-s',
                             f'/world/{self.world}/entity/system/add',
                             '--reqtype', 'ignition.msgs.EntityPlugin_V',
                             '--reptype', 'ignition.msgs.Boolean',
                             '--timeout', '5000', '--req', request])
        if not re.search(r'\bdata:\s*true\b', response):
            raise RuntimeError('Gazebo rejected DetachableJoint load: '+response.strip())
        # Fortress initially attaches at current relative pose when loaded.
        self.loaded = True

    def detach(self, tick=None, confirmed=None):
        if not self.loaded:
            raise RuntimeError('DetachableJoint has not been loaded')
        command = ['ign', 'topic', '-t', self.TOPIC+'/detach',
                   '-m', 'ignition.msgs.Empty', '-p', '']
        if tick is None:
            self.run(command)
        elif confirmed is None:
            self.run(command, tick=tick)
        else:
            # A CLI exit is only publication, never physical confirmation.
            # Retry the same detach request within one bounded deadline while
            # refreshing the original support/authorization lease each tick.
            deadline = time.monotonic()+8.
            attempts = 0
            while time.monotonic() < deadline:
                tick()
                if confirmed():
                    return
                attempts += 1
                self.run_with_feedback(command, tick, confirmed, deadline=deadline)
                if confirmed():
                    return
                until = min(deadline, time.monotonic()+.5)
                while time.monotonic() < until:
                    tick()
                    if confirmed():
                        return
                    time.sleep(max(0., min(.05, until-time.monotonic())))
            revision, state = self.state_snapshot()
            raise RuntimeError('ATTACHMENT_NOT_CONFIRMED: expected native detached'
                               f'; topic={self.TOPIC}/state; attempts={attempts}'
                               f'; observed_state={state}; observed_revision={revision}')

# CASK_TASK005_EXECUTION_V1

# CASK_ALL_ROBOT_CONTEXT_V1
