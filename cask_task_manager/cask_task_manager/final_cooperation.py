"""Overlap preparation with skill execution; derive roles from matched data flow."""
import time
import math
import threading
from rclpy.action import ActionClient
from ros_interfaces.action import ExecuteSkill
from cask_interfaces.msg import SkillExecutionState
from cask_handoff.behavior import pause


class CooperativeSequence:
    def __init__(self, sequence, plan, states):
        self.sequence, self.plan, self.states = sequence, plan, states
        self.children = sequence.children
        self.status = states.IDLE

    async def tick(self, manager, task_goal_handle, execution_id, task_id):
        # A single Crane movement uses the unchanged ordinary BT sequence.
        if len(self.plan) != 3:
            return await self.sequence.tick(manager,task_goal_handle,execution_id,task_id)
        required = [step['required_uri'] for step in self.plan]
        plan = await manager.query_execution_plan(task_id,required)
        plan = manager.order_plan_by_data_flow(plan)
        if [step['candidates'][0]['process_kind'] for step in plan] != ['Lifting','Transporting','Lifting']:
            return await self.sequence.tick(manager,task_goal_handle,execution_id,task_id)
        selected = [step['candidates'][0] for step in plan]
        if selected[1]['resource_name'] != 'Innok223_01':
            return await self.sequence.tick(manager,task_goal_handle,execution_id,task_id)
        self.status = self.states.RUNNING
        pending = []
        failures = []
        # This is a physical handoff milestone, not successful completion of
        # the loader Skill. Keep that goal running and join its cleanup later.
        handoff = dict(candidate=selected[0],ready=False,failure=None)
        handoff_lock = threading.Lock()
        subscription = None
        loader_node = self.children[0]
        old_before_candidate = getattr(loader_node,'before_candidate',None)

        def bind_loader(candidate):
            with handoff_lock:
                if handoff['ready']:
                    raise RuntimeError('LOADER_FAILED_AFTER_TRANSPORT_HANDOFF')
                handoff['candidate'] = candidate
            if old_before_candidate is not None:
                old_before_candidate(candidate)

        def loading_state(message):
            with handoff_lock:
                candidate = handoff['candidate']
                if (message.execution_id!=execution_id or message.task_id!=task_id
                        or candidate['resource_name']!='Jekko_01'
                        or message.resource_id!=candidate['resource_name']
                        or message.required_capability_id!=candidate['required_name']
                        or message.provided_capability_id!=candidate['provided_name']
                        or message.skill_id!=candidate['skill_name']):
                    return
                stage = message.status.split(' | ',1)[0]
                if stage=='CARGO_READY_FOR_TRANSPORT':
                    handoff['ready'] = True
                elif handoff['ready'] and (stage.startswith('FAIL')
                        or stage in ('CANCELED','CANCELLED','ABORTED')):
                    handoff['failure'] = message.status

        def check_pending():
            if task_goal_handle.is_cancel_requested:
                raise RuntimeError('CANCELED')
            with handoff_lock:
                failure = handoff['failure']
            if failure:
                raise RuntimeError('LOADER_FAILED_AFTER_TRANSPORT_HANDOFF: '+failure)
            for future in pending:
                if future.done():
                    if future.exception():
                        raise future.exception()
                    if future.result()!=self.states.SUCCESS:
                        raise RuntimeError('Concurrent operation failed')

        async def prepare(candidate):
            if not hasattr(manager,'preparation_client'):
                manager.preparation_client = ActionClient(manager,ExecuteSkill,
                    '/cask_skill_manager/navigate',callback_group=manager.callback_group)
            client = manager.preparation_client
            if not client.wait_for_server(timeout_sec=5.):
                raise RuntimeError('Preparation Action server unavailable')
            goal = ExecuteSkill.Goal()
            goal.execution_id,goal.task_id = execution_id,task_id
            goal.required_capability_id = candidate['required_name']
            goal.provided_capability_id = candidate['provided_name']
            goal.skill_id = candidate['skill_name']
            manager.get_logger().info('PREPARATION_STARTED | '+candidate['resource_name']+' | '+candidate['required_name'])
            last_report = [-math.inf]
            def forward(message):
                now = time.monotonic()
                if now-last_report[0]<.5 or not task_goal_handle.is_active:
                    return
                last_report[0] = now
                manager.publish_task_feedback(
                    goal_handle=task_goal_handle,
                    progress=100./3 if candidate is selected[2] else 0.,
                    current_skill=candidate['skill_name'],
                    status='PREPARING | '+candidate['resource_name']+' | '
                        +candidate['required_name']+' | '+message.feedback.status)
            handle = await client.send_goal_async(goal,feedback_callback=forward)
            if not handle.accepted:
                raise RuntimeError('Preparation rejected: '+candidate['resource_name'])
            manager.set_active_skill_goal(handle)
            if task_goal_handle.is_cancel_requested:handle.cancel_goal_async()
            try:
                wrapped = await handle.get_result_async()
                if wrapped.status != 4 or not wrapped.result.success:
                    raise RuntimeError('Preparation failed: '+wrapped.result.message)
                manager.get_logger().info('PREPARATION_READY | '+candidate['resource_name']+' | '+candidate['required_name'])
                return self.states.SUCCESS
            finally:
                manager.clear_active_skill_goal(handle)

        def start(coroutine):
            async def guarded():
                # rclpy's executor rethrows an uncaught Task exception before
                # the join loop can inspect it. Convert it to a BT failure here.
                try:
                    return await coroutine
                except Exception as error:
                    failures.append(str(error))
                    return self.states.FAILURE
            future = manager.executor.create_task(guarded())
            pending.append(future)
            return future

        async def join(*futures):
            until = time.monotonic()+600.
            while not all(f.done() for f in futures):
                check_pending()
                if time.monotonic()>until:raise RuntimeError('COOPERATIVE_STAGE_TIMEOUT')
                await pause(manager)
            check_pending()
            for f in futures:
                if f.result()!=self.states.SUCCESS:raise RuntimeError('Concurrent operation failed')

        def execute(index):
            return self.children[index].tick(manager,task_goal_handle,execution_id,task_id,index,3)

        try:
            if selected[0]['resource_name']=='Jekko_01':
                loader_node.before_candidate = bind_loader
                subscription = manager.create_subscription(SkillExecutionState,
                    '/cask/skill_execution_state',loading_state,20,
                    callback_group=manager.callback_group)
            carrier = start(prepare(selected[1]))
            if selected[0]['resource_name']=='Jekko_01':
                await join(start(prepare(selected[0])))
            loader = start(execute(0))
            if subscription is not None:
                await join(carrier)
                until = time.monotonic()+600.
                while not loader.done():
                    check_pending()
                    with handoff_lock:
                        ready = handoff['ready']
                    if ready:
                        manager.get_logger().info('TRANSPORT_OVERLAPS_LOADER_STOW | '+task_id)
                        break
                    if time.monotonic()>until:
                        raise RuntimeError('LOADING_HANDOFF_TIMEOUT')
                    await pause(manager)
                check_pending()
            else:
                await join(loader,carrier)
            preparations = []
            transport = None
            if selected[2]['resource_name']==selected[0]['resource_name'] and not loader.done():
                # Task_02 reuses Jekko for unloading. Let it finish its own
                # stow before sending its next navigation goal; transport is
                # already active during that stow.
                transport = start(execute(1))
                await join(loader)
            if selected[2]['resource_name'] in ('Jekko_01', 'Crane_01'):
                preparations.append(start(prepare(selected[2])))
            if transport is None:
                transport = start(execute(1))
            await join(loader,transport,*preparations)
            await join(start(execute(2)))
            self.status = self.states.SUCCESS
            return self.status
        except Exception as error:
            message = '; '.join(failures) if failures else str(error)
            manager.get_logger().error('COOPERATIVE_EXECUTION: '+message)
            if task_goal_handle.is_active:
                manager.publish_task_feedback(goal_handle=task_goal_handle,progress=0.,
                    current_skill='',status='FAILURE | '+message)
            manager.cancel_active_skill()
            until = time.monotonic()+15.
            while any(not f.done() for f in pending) and time.monotonic()<until:
                manager.cancel_active_skill()
                await pause(manager)
            if any(not f.done() for f in pending):
                manager.handoff_drain_pending = [f for f in pending if not f.done()]
            self.status = self.states.FAILURE
            return self.status
        finally:
            if subscription is not None:
                manager.destroy_subscription(subscription)
            loader_node.before_candidate = old_before_candidate
