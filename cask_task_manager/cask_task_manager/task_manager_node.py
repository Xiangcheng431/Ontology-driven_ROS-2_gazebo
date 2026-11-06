import json
import math
import re
import threading
import uuid
from collections import defaultdict
from enum import Enum

import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from cask_interfaces.action import ExecuteTask
from cask_interfaces.srv import ModifyWorldModel, QueryOntology
from ros_interfaces.action import ExecuteSkill
from cask_handoff.channel import Channel
from cask_handoff.behavior import HandoffPair
from cask_handoff.ontology import annotate as annotate_handoffs
from cask_handoff.ontology import validate_resume as validate_handoff_resume


ABOX = 'http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#'
DIN = 'http://www.w3id.org/hsu-aut/DINEN61360#'


class BehaviorStatus(Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'


class SkillActionNode:
    """Behavior Tree leaf for one concrete Skill candidate."""

    def __init__(self, candidate):
        self.candidate = candidate
        self.status = BehaviorStatus.IDLE
        self.safe_to_retry = True

    async def tick(
        self,
        manager,
        task_goal_handle,
        execution_id,
        task_id,
        index,
        total,
    ):
        self.status = BehaviorStatus.RUNNING

        succeeded, self.safe_to_retry = await manager.execute_skill_leaf(
            task_goal_handle=task_goal_handle,
            execution_id=execution_id,
            task_id=task_id,
            candidate=self.candidate,
            index=index,
            total=total,
        )

        self.status = (
            BehaviorStatus.SUCCESS
            if succeeded
            else BehaviorStatus.FAILURE
        )
        return self.status


class FallbackNode:
    """Query current candidates after each failure, without a fixed backup."""

    def __init__(self, name, required_uri):
        self.name = name
        self.required_uri = required_uri
        self.children = []
        self.before_candidate = None
        self.status = BehaviorStatus.IDLE

    async def tick(
        self,
        manager,
        task_goal_handle,
        execution_id,
        task_id,
        index,
        total,
    ):
        self.status = BehaviorStatus.RUNNING

        attempted = set()
        while not task_goal_handle.is_cancel_requested:
            try:
                plan = await manager.query_execution_plan(
                    task_id, [self.required_uri]
                )
            except ValueError as error:
                manager.get_logger().warning(f'{self.name}: {error}')
                break

            candidates = (
                plan[0]['candidates'] if plan else []
            )
            candidate = next(
                (item for item in candidates
                 if item['skill_uri'] not in attempted),
                None,
            )
            if candidate is None:
                manager.get_logger().warning(
                    f'{self.name}: no untried compatible Skill remains'
                )
                break

            if self.before_candidate is not None:
                self.before_candidate(candidate)
            attempted.add(candidate['skill_uri'])
            child = SkillActionNode(candidate)
            self.children.append(child)
            manager.get_logger().info(
                f'{self.name}: selected '
                f'{candidate["resource_name"]}/'
                f'{candidate["skill_name"]} from current World Model'
            )
            child_status = await child.tick(
                manager=manager,
                task_goal_handle=task_goal_handle,
                execution_id=execution_id,
                task_id=task_id,
                index=index,
                total=total,
            )

            if child_status == BehaviorStatus.SUCCESS:
                self.status = BehaviorStatus.SUCCESS
                return self.status

            if task_goal_handle.is_cancel_requested:
                self.status = BehaviorStatus.FAILURE
                return self.status

            if not child.safe_to_retry:
                manager.get_logger().warning(
                    f'{self.name}: cargo may already be loaded; '
                    'stopping automatic fallback'
                )
                self.status = BehaviorStatus.FAILURE
                return self.status

            manager.get_logger().warning(
                f'{self.name}: candidate '
                f'{child.candidate["skill_name"]} failed; '
                'querying current candidates again'
            )

        self.status = BehaviorStatus.FAILURE
        return self.status


class SequenceNode:
    """Execute RequiredCapabilities in data-flow order."""

    def __init__(
        self,
        name,
        children,
        start_index=0,
        total_steps=None,
    ):
        self.name = name
        self.children = children
        self.start_index = start_index
        self.total_steps = total_steps or sum(getattr(child, 'span', 1) for child in children)
        self.status = BehaviorStatus.IDLE

    async def tick(
        self,
        manager,
        task_goal_handle,
        execution_id,
        task_id,
    ):
        self.status = BehaviorStatus.RUNNING
        total = self.total_steps

        offset = 0
        for child in self.children:
            if task_goal_handle.is_cancel_requested:
                self.status = BehaviorStatus.FAILURE
                return self.status

            index = self.start_index + offset
            child_status = await child.tick(
                manager=manager,
                task_goal_handle=task_goal_handle,
                execution_id=execution_id,
                task_id=task_id,
                index=index,
                total=total,
            )

            offset += getattr(child, 'span', 1)
            if child_status != BehaviorStatus.SUCCESS:
                self.status = BehaviorStatus.FAILURE
                return self.status

        self.status = BehaviorStatus.SUCCESS
        return self.status


class CaskTaskManager(Node):

    def __init__(self):
        super().__init__('cask_task_manager')

        self.callback_group = ReentrantCallbackGroup()
        self.active_task_lock = threading.Lock()
        self.active_skill_goals = {}
        self.handoff_channel = Channel(self)
        self.handoff_drain_pending = []
        self.active_task_id = None
        self.declare_parameter('final_scenario_enabled', False)

        self.query_client = self.create_client(
            QueryOntology,
            '/cask_world_model/query',
            callback_group=self.callback_group,
        )
        self.modify_client = self.create_client(
            ModifyWorldModel,
            '/cask_world_model/modify',
            callback_group=self.callback_group,
        )

        self.skill_client = ActionClient(
            self,
            ExecuteSkill,
            '/cask_skill_manager/execute_skill',
            callback_group=self.callback_group,
        )

        self.execute_task_server = ActionServer(
            self,
            ExecuteTask,
            '/cask_task_manager/execute_task',
            execute_callback=self.execute_task,
            goal_callback=self.validate_task_goal,
            cancel_callback=self.accept_cancel_request,
            callback_group=self.callback_group,
        )

        self.get_logger().info('CaSk Task Manager is ready')
        self.get_logger().info(
            'Execute action: /cask_task_manager/execute_task'
        )

    def validate_task_goal(self, goal_request):
        if any(not f.done() for f in self.handoff_drain_pending):
            return GoalResponse.REJECT
        if not re.fullmatch(
            r'[A-Za-z_][A-Za-z0-9_]*',
            goal_request.task_id,
        ):
            self.get_logger().error(
                f'Invalid task identifier: {goal_request.task_id}'
            )
            return GoalResponse.REJECT

        with self.active_task_lock:
            if self.active_task_id is not None:
                self.get_logger().warning(
                    f'Rejected {goal_request.task_id}: task '
                    f'{self.active_task_id} is still active'
                )
                return GoalResponse.REJECT

            self.active_task_id = goal_request.task_id

        self.get_logger().info(
            f'Accepted Task request: {goal_request.task_id}'
        )
        return GoalResponse.ACCEPT

    def accept_cancel_request(self, goal_handle):
        self.get_logger().info(
            'Accepted Task cancellation request: '
            f'{goal_handle.request.task_id}'
        )
        self.cancel_active_skill()
        return CancelResponse.ACCEPT

    def set_active_skill_goal(self, skill_goal_handle):
        with self.active_task_lock:
            self.active_skill_goals[id(skill_goal_handle)] = [skill_goal_handle, False]

    def clear_active_skill_goal(self, skill_goal_handle):
        with self.active_task_lock:
            self.active_skill_goals.pop(id(skill_goal_handle), None)

    def cancel_active_skill(self):
        pending = []
        with self.active_task_lock:
            for entry in self.active_skill_goals.values():
                if not entry[1]:
                    entry[1] = True
                    pending.append(entry[0])
        for handle in pending:
            handle.cancel_goal_async()

    def release_active_task(self, task_id):
        with self.active_task_lock:
            if self.active_task_id == task_id:
                self.active_task_id = None

    async def execute_task(self, goal_handle):
        task_id = goal_handle.request.task_id
        execution_id = uuid.uuid4().hex
        result = ExecuteTask.Result()

        try:
            self.get_logger().info(
                f'Execution {execution_id}'
            )

            required_plan = await self.query_required_plan(task_id)
            ordered_required_plan = self.order_plan_by_data_flow(
                required_plan
            )
            await annotate_handoffs(self.query_rows, ordered_required_plan)
            full_step_count = len(ordered_required_plan)

            current_state_uri = await self.query_runtime_current_state(
                task_id
            )
            remaining_required_plan, completed_count = (
                self.select_remaining_plan(
                    ordered_required_plan,
                    current_state_uri,
                )
            )
            validate_handoff_resume(ordered_required_plan, completed_count)

            if current_state_uri is not None:
                current_state_name = self.local_name(
                    current_state_uri
                )
                self.get_logger().info(
                    f'Runtime current state for {task_id}: '
                    f'{current_state_name}'
                )

                if completed_count:
                    skipped_text = ' -> '.join(
                        step['required_name']
                        for step in ordered_required_plan[
                            :completed_count
                        ]
                    )
                    self.get_logger().info(
                        'Skipping completed RequiredCapabilities: '
                        f'{skipped_text}'
                    )

                if not remaining_required_plan:
                    self.publish_task_feedback(
                        goal_handle=goal_handle,
                        progress=100.0,
                        current_skill='',
                        status='ALREADY_COMPLETED',
                    )
                    goal_handle.succeed()
                    result.success = True
                    result.message = (
                        f'{task_id} already completed at '
                        f'{current_state_name}'
                    )
                    self.get_logger().info(result.message)
                    return result

            plan_text = ' -> '.join(
                step['required_name']
                for step in remaining_required_plan
            )
            self.get_logger().info(
                f'Required capability sequence for {task_id}: '
                f'{plan_text}'
            )

            if completed_count:
                self.get_logger().info(
                    f'Resuming {task_id} with remaining plan: '
                    f'{plan_text}'
                )

            tree = self.build_behavior_tree(
                task_id,
                remaining_required_plan,
                start_index=completed_count,
                total_steps=full_step_count,
            )
            if self.get_parameter('final_scenario_enabled').value and not completed_count:
                from .final_cooperation import CooperativeSequence
                tree = CooperativeSequence(tree, remaining_required_plan, BehaviorStatus)
            tree_status = await tree.tick(
                manager=self,
                task_goal_handle=goal_handle,
                execution_id=execution_id,
                task_id=task_id,
            )

            successful_skills = [
                child.candidate['skill_name']
                for fallback_node in tree.children
                for child in fallback_node.children
                if child.status == BehaviorStatus.SUCCESS
            ]
            attempted_skills = [
                child.candidate['skill_name']
                for fallback_node in tree.children
                for child in fallback_node.children
                if child.status != BehaviorStatus.IDLE
            ]

            actual_plan_text = (
                ' -> '.join(successful_skills)
                if successful_skills
                else plan_text
            )
            attempted_plan_text = (
                ' -> '.join(attempted_skills)
                if attempted_skills
                else plan_text
            )

            if goal_handle.is_cancel_requested:
                if goal_handle.is_active:
                    goal_handle.canceled()
                result.success = False
                result.message = (
                    f'{task_id} canceled; runtime state remains at '
                    'the last successfully completed capability'
                )
            elif tree_status == BehaviorStatus.SUCCESS:
                goal_handle.succeed()
                result.success = True
                if completed_count:
                    result.message = (
                        f'{task_id} resumed and completed '
                        f'successfully: {actual_plan_text}'
                    )
                else:
                    result.message = (
                        f'{task_id} completed successfully: '
                        f'{actual_plan_text}'
                    )
            else:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f'{task_id} failed after attempting: '
                    f'{attempted_plan_text}'
                )
                failures = [
                    child.candidate.get('failure_message', '')
                    for fallback_node in tree.children
                    for child in fallback_node.children
                    if child.status == BehaviorStatus.FAILURE
                ]
                if any(failures):
                    result.message += ' | ' + '; '.join(f for f in failures if f)

            self.get_logger().info(result.message)
            return result

        except Exception as error:
            if goal_handle.is_cancel_requested:
                if goal_handle.is_active:
                    goal_handle.canceled()
            elif goal_handle.is_active:
                goal_handle.abort()
            result.success = False
            result.message = f'Task Manager execution error: {error}'
            self.get_logger().error(result.message)
            return result
        finally:
            self.release_active_task(task_id)

    async def query_rows(self, query):
        if not self.query_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                'CaSk World Model query service is unavailable'
            )

        request = QueryOntology.Request()
        request.query = query
        response = await self.query_client.call_async(request)

        if not response.success:
            raise RuntimeError(
                f'World Model query failed: {response.message}'
            )

        return json.loads(response.result_json)

    async def query_required_plan(self, task_id):
        query = f"""
PREFIX abox: <{ABOX}>
PREFIX cask: <http://www.w3id.org/hsu-aut/cask#>
PREFIX css: <http://www.w3id.org/hsu-aut/css#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>

SELECT DISTINCT ?required ?required_input ?required_output
WHERE {{
    abox:{task_id} css:requiresCapability ?required .

    ?required
        a cask:RequiredCapability ;
        vdi:hasInput ?required_input ;
        vdi:hasOutput ?required_output .
}}
ORDER BY ?required
"""

        rows = await self.query_rows(query)
        if not rows:
            raise ValueError(
                f'No RequiredCapabilities found for {task_id}'
            )

        return [
            {
                'required_uri': row['required'],
                'required_name': self.local_name(row['required']),
                'input_uri': row['required_input'],
                'output_uri': row['required_output'],
            }
            for row in rows
        ]

    async def query_execution_plan(
        self,
        task_id,
        required_uris,
    ):
        if not required_uris:
            return []

        required_values = ' '.join(
            f'<{required_uri}>'
            for required_uri in required_uris
        )
        query = f"""
PREFIX abox: <{ABOX}>
PREFIX cask: <http://www.w3id.org/hsu-aut/cask#>
PREFIX css: <http://www.w3id.org/hsu-aut/css#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>
PREFIX din: <{DIN}>

SELECT DISTINCT
    ?required ?required_input ?required_output
    ?provided ?provided_input ?provided_output
    ?resource ?skill ?endpoint ?interface_type ?message_type
    ?process_kind
WHERE {{
    VALUES ?required {{ {required_values} }}

    abox:{task_id} css:requiresCapability ?required .

    ?required
        a cask:RequiredCapability ;
        vdi:hasInput ?required_input ;
        vdi:hasOutput ?required_output ;
        css:isSpecifiedBy ?requirement_property .

    ?requirement_property
        din:has_Type_Description ?process_type ;
        din:has_Instance_Description ?requirement_description .

    ?requirement_description din:Value ?process_kind .

    ?provided
        a cask:ProvidedCapability ;
        vdi:hasInput ?provided_input ;
        vdi:hasOutput ?provided_output ;
        css:isSpecifiedBy ?assurance_property ;
        css:isRealizedBySkill ?skill .

    ?assurance_property
        din:has_Type_Description ?process_type ;
        din:has_Instance_Description ?assurance_description .

    ?assurance_description din:Value ?process_kind .

    ?resource
        css:providesCapability ?provided ;
        css:providesSkill ?skill .

    VALUES ?resource {{ abox:Crane_01 abox:Jekko_01 abox:Innok223_01 }}
    FILTER NOT EXISTS {{ ?resource abox:hasAvailabilityState "UNAVAILABLE" . }}

    OPTIONAL {{ ?required vdi:isAssignedTo ?assigned_resource . }}
    OPTIONAL {{ ?required abox:hasAssignedResource ?legacy_assigned_resource . }}
    FILTER(!BOUND(?assigned_resource) || ?resource = ?assigned_resource)
    FILTER(BOUND(?assigned_resource) || !BOUND(?legacy_assigned_resource) || ?resource = ?legacy_assigned_resource)
    ?skill css:accessibleThrough ?interface .

    ?interface
        abox:hasROSEndpoint ?endpoint ;
        abox:hasROSInterfaceType ?interface_type ;
        abox:hasROSMessageType ?message_type .
}}
ORDER BY ?required ?resource ?skill
"""

        rows = await self.query_rows(query)
        if not rows:
            raise ValueError(
                f'No executable Skill candidates found for {task_id}'
            )

        product_cache = {}
        candidate_groups = defaultdict(list)
        requirement_data = {}

        for row in rows:
            required_uri = row['required']

            candidate = {
                'required_uri': required_uri,
                'required_name': self.local_name(required_uri),
                'required_input_uri': row['required_input'],
                'required_output_uri': row['required_output'],
                'provided_uri': row['provided'],
                'provided_name': self.local_name(row['provided']),
                'provided_input_uri': row['provided_input'],
                'provided_output_uri': row['provided_output'],
                'resource_uri': row['resource'],
                'resource_name': self.local_name(row['resource']),
                'skill_uri': row['skill'],
                'skill_name': self.local_name(row['skill']),
                'endpoint': row['endpoint'],
                'interface_type': row['interface_type'],
                'message_type': row['message_type'],
                'process_kind': row['process_kind'],
            }

            requirement_data[required_uri] = {
                'required_uri': required_uri,
                'required_name': candidate['required_name'],
                'input_uri': candidate['required_input_uri'],
                'output_uri': candidate['required_output_uri'],
            }

            compatible, score, reason = await self.evaluate_candidate(
                candidate,
                product_cache,
            )

            if compatible:
                candidate['score'] = score
                candidate_groups[required_uri].append(candidate)
                self.get_logger().info(
                    f'Candidate accepted: '
                    f'{candidate["required_name"]} -> '
                    f'{candidate["resource_name"]}/'
                    f'{candidate["skill_name"]}; score={score:.6f}'
                )
            else:
                self.get_logger().info(
                    f'Candidate rejected: '
                    f'{candidate["required_name"]} -> '
                    f'{candidate["resource_name"]}/'
                    f'{candidate["provided_name"]}; {reason}'
                )

        plan = []
        for required_uri, step in requirement_data.items():
            candidates = candidate_groups.get(required_uri, [])
            if not candidates:
                raise ValueError(
                    'No compatible executable candidate for '
                    f'{step["required_name"]}'
                )

            candidates.sort(
                key=lambda candidate: (
                    candidate['score'],
                    candidate['resource_name'],
                    candidate['provided_name'],
                    candidate['skill_name'],
                )
            )
            step['candidates'] = candidates
            plan.append(step)

        return plan

    async def query_runtime_current_state(self, task_id):
        query = f"""
PREFIX abox: <{ABOX}>
PREFIX cask: <http://www.w3id.org/hsu-aut/cask#>

SELECT DISTINCT ?current_state
WHERE {{
    abox:RuntimeComponent_{task_id}
        cask:hasCurrentState ?current_state .
}}
"""

        rows = await self.query_rows(query)
        if not rows:
            return None

        states = {
            row['current_state']
            for row in rows
            if row.get('current_state')
        }

        if len(states) != 1:
            raise ValueError(
                f'Expected one runtime state for {task_id}, '
                f'found {len(states)}'
            )

        return next(iter(states))

    @staticmethod
    def select_remaining_plan(ordered_plan, current_state_uri):
        if current_state_uri is None:
            return ordered_plan, 0

        if not ordered_plan:
            return ordered_plan, 0

        if current_state_uri == ordered_plan[0]['input_uri']:
            return ordered_plan, 0

        for index, step in enumerate(ordered_plan):
            if current_state_uri == step['output_uri']:
                completed_count = index + 1
                return (
                    ordered_plan[completed_count:],
                    completed_count,
                )

        for index, step in enumerate(ordered_plan):
            if current_state_uri == step['input_uri']:
                return ordered_plan[index:], index

        raise ValueError(
            'Runtime current state is not part of the ontology '
            f'task data flow: {current_state_uri}'
        )

    async def evaluate_candidate(self, candidate, product_cache):
        required_input = await self.get_product_constraints(
            candidate['required_input_uri'], product_cache
        )
        provided_input = await self.get_product_constraints(
            candidate['provided_input_uri'], product_cache
        )
        required_output = await self.get_product_constraints(
            candidate['required_output_uri'], product_cache
        )
        provided_output = await self.get_product_constraints(
            candidate['provided_output_uri'], product_cache
        )

        if candidate['process_kind'] == 'Transporting':
            # The route determines the actual transport requirement. A
            # stored TransportDistance_TD value can become stale when the
            # task's output position changes.
            distance_type = ABOX + 'TransportDistance_TD'
            distance = await self.get_transport_distance(candidate)
            required_output = [
                constraint for constraint in required_output
                if constraint['type_uri'] != distance_type
            ] + [{
                'type_uri': distance_type,
                'type_name': 'TransportDistance_TD',
                'value': str(distance),
                'logic': '=',
                'unit': 'm',
            }]
            if not any(
                assurance['type_uri'] == distance_type
                and assurance['unit'] == 'm'
                for assurance in provided_output
            ):
                return (
                    False, math.inf,
                    'output mismatch: missing TransportDistance_TD '
                    'assurance in m',
                )

        input_ok, input_score, input_reason = self.match_constraints(
            required_input,
            provided_input,
            role='input',
        )
        if not input_ok:
            return False, math.inf, f'input mismatch: {input_reason}'

        output_ok, output_score, output_reason = self.match_constraints(
            required_output,
            provided_output,
            role='output',
        )
        if not output_ok:
            return False, math.inf, f'output mismatch: {output_reason}'

        return True, input_score + output_score, 'compatible'

    async def get_transport_distance(self, candidate):
        query = f"""
PREFIX abox: <{ABOX}>

SELECT ?input_frame ?input_x ?input_y
       ?output_frame ?output_x ?output_y
WHERE {{
    <{candidate['required_input_uri']}>
        abox:hasReferenceFrame ?input_frame ;
        abox:hasPositionX ?input_x ;
        abox:hasPositionY ?input_y .
    <{candidate['required_output_uri']}>
        abox:hasReferenceFrame ?output_frame ;
        abox:hasPositionX ?output_x ;
        abox:hasPositionY ?output_y .
}}
"""
        rows = await self.query_rows(query)
        if len(rows) != 1:
            raise ValueError(
                'Expected one pickup and delivery pose for '
                f'{candidate["required_name"]}, received {len(rows)}'
            )
        row = rows[0]
        if row['input_frame'] != 'world' or row['output_frame'] != 'world':
            raise ValueError('Transport requires world-frame poses')
        distance = math.hypot(
            float(row['output_x']) - float(row['input_x']),
            float(row['output_y']) - float(row['input_y']),
        )
        if not math.isfinite(distance) or distance <= 0:
            raise ValueError('Transport pickup and delivery must be distinct')
        return distance

    async def get_product_constraints(self, product_uri, cache):
        if product_uri in cache:
            return cache[product_uri]

        query = f"""
PREFIX din: <{DIN}>

SELECT DISTINCT
    ?property ?type ?type_code ?value ?logic ?expression_goal ?unit
WHERE {{
    <{product_uri}> din:has_Data_Element ?property .

    ?property
        din:has_Type_Description ?type ;
        din:has_Instance_Description ?description .

    ?description din:Value ?value .

    OPTIONAL {{ ?description din:Logic_Interpretation ?logic . }}
    OPTIONAL {{ ?description din:Expression_Goal ?expression_goal . }}
    OPTIONAL {{ ?type din:Code ?type_code . }}
    OPTIONAL {{ ?type din:Unit_of_Measure ?unit . }}
}}
ORDER BY ?type ?property
"""
        rows = await self.query_rows(query)

        constraints = []
        for row in rows:
            if row.get("type_code") == "handoffIntermediateState":
                continue  # Execution pose reference, not a capability assurance.
            constraints.append({
                'property_uri': row['property'],
                'type_uri': row['type'],
                'type_name': self.local_name(row['type']),
                'type_code': row.get('type_code'),
                'value': row['value'],
                'logic': row.get('logic', '='),
                'expression_goal': row.get('expression_goal'),
                'unit': row.get('unit'),
            })

        cache[product_uri] = constraints
        return constraints

    def match_constraints(self, required, provided, role):
        provided_by_type = defaultdict(list)
        for constraint in provided:
            provided_by_type[constraint['type_uri']].append(constraint)

        score = 0.0

        for requirement in required:
            assurances = provided_by_type.get(
                requirement['type_uri'],
                [],
            )

            if not assurances:
                # Categorical identity and bounded quality requirements must
                # be described by the provider. Exact numeric values may be
                # execution targets rather than capability assurances.
                if (
                    requirement['type_name'] == 'DeliveryHeightHoldDuration_TD'
                    or not self.is_number(requirement['value'])
                    or requirement['logic'] != '='
                ):
                    return (
                        False,
                        math.inf,
                        f'missing {requirement["type_name"]} assurance',
                    )
                continue

            compatible_assurances = []
            for assurance in assurances:
                compatible, margin = self.compare_constraint(
                    requirement,
                    assurance,
                    role,
                )
                if compatible:
                    compatible_assurances.append((margin, assurance))

            if not compatible_assurances:
                expected = (
                    f'{requirement["logic"]} '
                    f'{requirement["value"]}'
                )
                offered = ', '.join(
                    f'{item["logic"]} {item["value"]}'
                    for item in assurances
                )
                return (
                    False,
                    math.inf,
                    f'{requirement["type_name"]}: '
                    f'required {expected}, provided {offered}',
                )

            score += min(item[0] for item in compatible_assurances)

        return True, score, 'compatible'

    def compare_constraint(self, requirement, assurance, role):
        required_value = requirement['value']
        provided_value = assurance['value']
        required_logic = requirement['logic'] or '='
        provided_logic = assurance['logic'] or '='

        required_number = self.to_number(required_value)
        provided_number = self.to_number(provided_value)

        if required_number is None or provided_number is None:
            compatible = str(required_value) == str(provided_value)
            return compatible, 0.0 if compatible else math.inf

        tolerance = 1e-9

        if required_logic == '=':
            if provided_logic == '=':
                compatible = (
                    abs(required_number - provided_number)
                    <= tolerance
                )
            elif provided_logic == '<=':
                compatible = required_number <= provided_number + tolerance
            elif provided_logic == '>=':
                compatible = required_number >= provided_number - tolerance
            else:
                compatible = False

            margin = abs(provided_number - required_number)
            return compatible, margin if compatible else math.inf

        if required_logic == '<=' and provided_logic in ('<=', '='):
            compatible = provided_number <= required_number + tolerance
            margin = required_number - provided_number
            return compatible, max(0.0, margin) if compatible else math.inf

        if required_logic == '>=' and provided_logic in ('>=', '='):
            compatible = provided_number >= required_number - tolerance
            margin = provided_number - required_number
            return compatible, max(0.0, margin) if compatible else math.inf

        compatible = abs(required_number - provided_number) <= tolerance
        return compatible, 0.0 if compatible else math.inf

    @staticmethod
    def to_number(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @classmethod
    def is_number(cls, value):
        return cls.to_number(value) is not None

    @staticmethod
    def order_plan_by_data_flow(plan):
        step_by_required = {
            step['required_uri']: step for step in plan
        }
        successors = {
            required_uri: set()
            for required_uri in step_by_required
        }
        indegree = {
            required_uri: 0
            for required_uri in step_by_required
        }

        for first in plan:
            for second in plan:
                if first['required_uri'] == second['required_uri']:
                    continue

                if first['output_uri'] == second['input_uri']:
                    first_id = first['required_uri']
                    second_id = second['required_uri']

                    if second_id not in successors[first_id]:
                        successors[first_id].add(second_id)
                        indegree[second_id] += 1

        ready = sorted(
            required_uri
            for required_uri, degree in indegree.items()
            if degree == 0
        )
        ordered = []

        while ready:
            current = ready.pop(0)
            ordered.append(step_by_required[current])

            for successor in sorted(successors[current]):
                indegree[successor] -= 1
                if indegree[successor] == 0:
                    ready.append(successor)
                    ready.sort()

        if len(ordered) != len(plan):
            raise ValueError(
                'The ontology task data flow contains a cycle'
            )

        return ordered

    @staticmethod
    def build_behavior_tree(
        task_id,
        ordered_plan,
        start_index=0,
        total_steps=None,
    ):
        required_nodes = []

        cursor = 0
        while cursor < len(ordered_plan):
            step = ordered_plan[cursor]
            node = FallbackNode(step['required_name'], step['required_uri'])
            if step.get('handoff'):
                following = ordered_plan[cursor+1]
                receiver = FallbackNode(following['required_name'], following['required_uri'])
                required_nodes.append(HandoffPair(node, receiver, BehaviorStatus))
                cursor += 2
            else:
                required_nodes.append(node)
                cursor += 1

        return SequenceNode(
            name=f'{task_id}_Sequence',
            children=required_nodes,
            start_index=start_index,
            total_steps=total_steps,
        )

    async def execute_skill_leaf(
        self,
        task_goal_handle,
        execution_id,
        task_id,
        candidate,
        index,
        total,
    ):
        skill_id = candidate['skill_name']
        candidate.pop('failure_message', None)

        if not self.skill_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                'CaSk Skill Manager action server is unavailable'
            )

        self.publish_task_feedback(
            goal_handle=task_goal_handle,
            progress=100.0 * index / total,
            current_skill=skill_id,
            status='RUNNING',
        )

        skill_goal = ExecuteSkill.Goal()
        skill_goal.execution_id = execution_id
        skill_goal.task_id = task_id
        skill_goal.required_capability_id = candidate['required_name']
        skill_goal.provided_capability_id = candidate['provided_name']
        skill_goal.skill_id = skill_id
        skill_goal.parameter_names = []
        skill_goal.parameter_values = []
        skill_goal.parameter_units = []

        self.get_logger().info(
            f'Behavior Tree executing: '
            f'{candidate["required_name"]} -> '
            f'{candidate["resource_name"]}/'
            f'{candidate["provided_name"]} -> {skill_id}'
        )

        send_future = self.skill_client.send_goal_async(
            skill_goal,
            feedback_callback=lambda message:
                self.forward_skill_feedback(
                    task_goal_handle=task_goal_handle,
                    skill_id=skill_id,
                    index=index,
                    total=total,
                    feedback_message=message,
                ),
        )

        skill_goal_handle = await send_future

        if not skill_goal_handle.accepted:
            candidate['failure_message'] = f'{skill_id}: Skill goal rejected'
            self.publish_task_feedback(
                goal_handle=task_goal_handle,
                progress=100.0 * index / total,
                current_skill=skill_id,
                status='REJECTED',
            )
            return False, True

        self.set_active_skill_goal(skill_goal_handle)

        if task_goal_handle.is_cancel_requested:
            self.cancel_active_skill()
            self.publish_task_feedback(
                goal_handle=task_goal_handle,
                progress=100.0 * index / total,
                current_skill=skill_id,
                status='CANCELING',
            )

        result_future = skill_goal_handle.get_result_async()
        try:
            wrapped_result = await result_future
        finally:
            self.clear_active_skill_goal(skill_goal_handle)

        skill_result = wrapped_result.result

        if task_goal_handle.is_cancel_requested:
            return False, False

        if skill_result.success:
            self.publish_task_feedback(
                goal_handle=task_goal_handle,
                progress=100.0 * (index + 1) / total,
                current_skill=skill_id,
                status='SUCCESS',
            )
            return True, True

        self.get_logger().error(
            f'{skill_id} failed: {skill_result.message}'
        )
        candidate['failure_message'] = f'{skill_id}: {skill_result.message}'[:1000]
        await self.update_resource_property(
            candidate['resource_uri'],
            'hasLastFailure',
            f'{skill_id}: {skill_result.message}'[:500],
        )
        if ('Action server unavailable:' in skill_result.message
                or 'ROBOT_UNAVAILABLE' in skill_result.message):
            await self.mark_resource_unavailable(candidate['resource_uri'])
        loaded_failure = 'carrying_phase=true' in skill_result.message.lower()
        recovery_ready = False
        if loaded_failure:
            if not recovery_ready:
                self.get_logger().error(
                    'Loaded component snapshot unavailable; '
                    'stopping unsafe Skill fallback')
        self.publish_task_feedback(
            goal_handle=task_goal_handle,
            progress=100.0 * index / total,
            current_skill=skill_id,
            status=('FAILURE | ' + skill_result.message)[:1000],
        )
        safe_to_retry = (
            recovery_ready or (
                not loaded_failure and (
                    candidate['process_kind'] != 'Transporting'
                    or 'carrying_phase=false' in skill_result.message
                    or 'Action server unavailable:' in skill_result.message
                    or 'Robot rejected Skill:' in skill_result.message
                    or 'ROBOT_UNAVAILABLE' in skill_result.message
                )
            )
        )
        return False, safe_to_retry


    async def mark_resource_unavailable(self, resource_uri):
        await self.update_resource_property(
            resource_uri, 'hasAvailabilityState', 'UNAVAILABLE'
        )
        self.get_logger().warning(
            f'Marked {self.local_name(resource_uri)} UNAVAILABLE in World Model'
        )

    async def update_resource_property(self, resource_uri, property_name,
                                       value):
        if not self.modify_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('World Model modify service unavailable')
        request = ModifyWorldModel.Request()
        request.operation = 'set'
        request.subject = resource_uri
        request.predicate = ABOX + property_name
        request.object = value
        request.object_is_literal = True
        request.save_to_file = False
        response = await self.modify_client.call_async(request)
        if not response.success:
            raise RuntimeError(response.message)

    def forward_skill_feedback(
        self,
        task_goal_handle,
        skill_id,
        index,
        total,
        feedback_message,
    ):
        if not task_goal_handle.is_active:
            return

        skill_feedback = feedback_message.feedback
        overall_progress = 100.0 * (
            index + skill_feedback.progress / 100.0
        ) / total

        self.publish_task_feedback(
            goal_handle=task_goal_handle,
            progress=overall_progress,
            current_skill=skill_id,
            status=skill_feedback.status,
        )

    @staticmethod
    def publish_task_feedback(
        goal_handle,
        progress,
        current_skill,
        status,
    ):
        feedback = ExecuteTask.Feedback()
        feedback.progress = float(progress)
        feedback.current_skill = current_skill
        feedback.status = status
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def local_name(uri):
        if '#' in uri:
            return uri.rsplit('#', 1)[1]
        return uri.rsplit('/', 1)[-1]


def main(args=None):
    rclpy.init(args=args)
    node = CaskTaskManager()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
