import json
import math
import re
import threading
from pathlib import Path
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

from cask_interfaces.msg import SkillExecutionState
from cask_interfaces.srv import QueryOntology
from ros_interfaces.action import ExecuteSkill
from cask_handoff.ontology import parameterize as parameterize_handoff
from .execution_outcome import result_status


class SkillState(Enum):
    IDLE = 'IDLE'
    RUNNING = 'RUNNING'
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    CANCELED = 'CANCELED'


class CaskSkillManager(Node):

    def __init__(self):
        super().__init__('cask_skill_manager')

        self.declare_parameter('final_scene_config', '')
        scene_path = self.get_parameter('final_scene_config').value
        self.final_scene = json.loads(Path(scene_path).read_text(encoding='utf-8')) if scene_path else None
        self.skills = {}
        self.action_clients = {}
        self.robot_goal_lock = threading.Lock()
        self.robot_goal_handles = {}
        self.robot_cancel_sent = set()

        self.discovery_in_progress = False
        self.waiting_message_printed = False

        self.callback_group = ReentrantCallbackGroup()

        # Fixed Crane world pose used to convert world-frame Task targets
        # into the local coordinates expected by the Crane Skill Server.
        # Task_003 already uses crane_base_link and remains unchanged.
        self.declare_parameter('physical_cargo_mode', False)
        # Geometry-based pre-operation stand-off, not task/world coordinates.
        self.declare_parameter('crane_world_x', 2.8)
        self.declare_parameter('crane_world_y', 0.0)
        self.declare_parameter('crane_world_yaw', 0.0)

        self.crane_world_x = float(
            self.get_parameter('crane_world_x').value
        )
        self.crane_world_y = float(
            self.get_parameter('crane_world_y').value
        )
        self.crane_world_yaw = float(
            self.get_parameter('crane_world_yaw').value
        )

        # A parameterizer converts semantic task data into the numerical
        # parameters expected by one robot implementation.  Selection of the
        # robot is performed by the Task Manager; this registry only performs
        # the robot-specific conversion after that selection.
        self.parameterizers = {('Lifting', 'Crane_01'): self.load_crane_lifting_parameters}


        # Access the CaSk World Model.
        self.query_client = self.create_client(
            QueryOntology,
            '/cask_world_model/query',
            callback_group=self.callback_group
        )

        # Generic execution-state stream consumed by Component Manager and
        # other observers. It supplements the Action feedback and does not
        # change the Task Manager <-> Skill Manager Action protocol.
        self.execution_state_publisher = self.create_publisher(
            SkillExecutionState,
            '/cask/skill_execution_state',
            10,
        )

        # Discover executable Skills from the ontology.
        self.discovery_timer = self.create_timer(
            1.0,
            self.discover_skills,
            callback_group=self.callback_group
        )

        # Unified execution interface for the future Task Manager.
        self.execute_action_server = ActionServer(
            self,
            ExecuteSkill,
            '/cask_skill_manager/execute_skill',
            execute_callback=self.execute_skill,
            goal_callback=self.validate_skill_goal,
            cancel_callback=self.accept_cancel_request,
            callback_group=self.callback_group
        )

        self.navigation_goal_handles = {}
        self.navigation_action_server = ActionServer(
            self, ExecuteSkill, '/cask_skill_manager/navigate',
            execute_callback=self.execute_navigation,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=self.cancel_navigation,
            callback_group=self.callback_group)

        self.get_logger().info('CaSk Skill Manager is starting')
        self.get_logger().info(
            'Execute action: '
            '/cask_skill_manager/execute_skill'
        )

    async def navigation_parameters(self, request):
        """Use the same semantic binding and parameterizers as complete skills.

        This substep reads fresh World Model coordinates on every goal. It
        intentionally accepts no caller-supplied coordinates at this entry.
        """
        if request.parameter_names or request.parameter_values or request.parameter_units:
            raise ValueError('Ontology navigation accepts identifiers only, no explicit coordinates')
        binding = await self.resolve_execution_binding(request)
        if self.final_scene is not None:
            names, values, units = await self.resolve_goal_parameters(request, binding)
            p = dict(zip(names, values))
            if binding['resource_name'] == 'Jekko_01':
                return 'jekko', (p['jekko_work_x'],p['jekko_work_y'],p['jekko_work_yaw'])
            if binding['resource_name'] == 'Innok223_01':
                return 'innok223', (p['pickup_x'],p['pickup_y'],p['pickup_yaw'])
            if binding['resource_name'] == 'Crane_01':
                return 'crane', (p['pickup_x'],p['pickup_y'],p['pickup_yaw'])
            raise ValueError('No preparation profile for '+binding['resource_name'])
        profiles = {}
        key = binding['process_kind'], binding['resource_name']
        if key not in profiles:
            raise ValueError('No physical navigation profile for ' + str(key))
        robot, offset_name = profiles[key]
        names, values, units = await self.resolve_goal_parameters(request, binding)
        parameters = dict(zip(names, values))
        parameter_units = dict(zip(names, units))
        for name, unit in (('target_x', 'm'), ('target_y', 'm'), ('target_yaw', 'rad')):
            if parameter_units.get(name) != unit or not math.isfinite(parameters[name]):
                raise ValueError('Invalid World Model navigation coordinate: ' + name)
        offset = float(self.get_parameter(offset_name).value)
        if not math.isfinite(offset) or offset < 0:
            raise ValueError('Invalid robot approach geometry')
        x, y, yaw = (parameters[k] for k in ('target_x', 'target_y', 'target_yaw'))
        target = (x - offset * math.cos(yaw), y - offset * math.sin(yaw), yaw)
        self.get_logger().info(
            f'WORLD_MODEL_TARGET | {request.required_capability_id} | '
            f'site=({x:.3f},{y:.3f},{yaw:.3f}) | base_goal={target}')
        return robot, target

    def cancel_navigation(self, goal_handle):
        key = bytes(goal_handle.goal_id.uuid)
        with self.robot_goal_lock:
            child = self.navigation_goal_handles.get(key)
        if child is not None:
            child.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute_navigation(self, goal_handle):
        """Navigation-only check; does not emit capability completion events."""
        request = goal_handle.request
        key = bytes(goal_handle.goal_id.uuid)
        child = None
        finished = False
        result = ExecuteSkill.Result()
        try:
            robot, target = await self.navigation_parameters(request)
            if goal_handle.is_cancel_requested:
                raise RuntimeError('CANCELED')
            endpoint = '/' + robot + '/navigate'
            if endpoint not in self.action_clients:
                self.action_clients[endpoint] = ActionClient(
                    self, ExecuteSkill, endpoint, callback_group=self.callback_group)
            client = self.action_clients[endpoint]
            if not client.wait_for_server(timeout_sec=5.0):
                raise RuntimeError('Navigation Action server unavailable: ' + endpoint)
            goal = ExecuteSkill.Goal()
            for field in ('execution_id', 'task_id', 'required_capability_id',
                          'provided_capability_id', 'skill_id'):
                setattr(goal, field, getattr(request, field))
            goal.parameter_names = ['target_x', 'target_y', 'target_yaw']
            goal.parameter_values = list(target)
            goal.parameter_units = ['m', 'm', 'rad']
            if self.final_scene is not None:
                binding = await self.resolve_execution_binding(request)
                goal.parameter_names, goal.parameter_values, goal.parameter_units = await self.resolve_goal_parameters(request,binding)
            def forward(message):
                if goal_handle.is_active:
                    goal_handle.publish_feedback(message.feedback)
            child = await client.send_goal_async(goal, feedback_callback=forward)
            if not child.accepted:
                raise RuntimeError('Navigation rejected: robot busy or invalid target')
            with self.robot_goal_lock:
                self.navigation_goal_handles[key] = child
            if goal_handle.is_cancel_requested:
                child.cancel_goal_async()
            wrapped = await child.get_result_async()
            finished = True
            result = wrapped.result
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'Navigation canceled'
                goal_handle.canceled()
            elif result.success and wrapped.status == 4:
                goal_handle.succeed()
            else:
                result.success = False
                goal_handle.abort()
        except Exception as error:
            result.success, result.message = False, str(error)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        finally:
            if child is not None and child.accepted and not finished:
                child.cancel_goal_async()
            with self.robot_goal_lock:
                self.navigation_goal_handles.pop(key, None)
        return result

    def discover_skills(self):
        if self.discovery_in_progress:
            return

        if not self.query_client.wait_for_service(
            timeout_sec=0.0
        ):
            if not self.waiting_message_printed:
                self.get_logger().warning(
                    'Waiting for CaSk World Model'
                )
                self.waiting_message_printed = True
            return

        self.waiting_message_printed = False
        self.discovery_in_progress = True

        request = QueryOntology.Request()

        request.query = """
PREFIX css: <http://www.w3id.org/hsu-aut/css#>
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>

SELECT ?skill ?interface ?endpoint ?interface_type ?message_type
WHERE {
    VALUES ?resource { abox:Crane_01 abox:Jekko_01 abox:Innok223_01 }
    ?resource css:providesSkill ?skill .
    ?skill a css:Skill ;
           css:accessibleThrough ?interface .

    ?interface abox:hasROSEndpoint ?endpoint ;
               abox:hasROSInterfaceType ?interface_type ;
               abox:hasROSMessageType ?message_type .
}
ORDER BY ?skill
"""

        future = self.query_client.call_async(request)

        future.add_done_callback(
            self.handle_skill_discovery
        )

    def handle_skill_discovery(self, future):
        self.discovery_in_progress = False

        try:
            response = future.result()

            if not response.success:
                self.get_logger().error(
                    f'Skill discovery failed: '
                    f'{response.message}'
                )
                return

            rows = json.loads(response.result_json)

            self.skills.clear()

            for row in rows:
                skill_uri = row['skill']
                skill_name = self.local_name(skill_uri)

                self.skills[skill_name] = {
                    'uri': skill_uri,
                    'interface': row['interface'],
                    'endpoint': row['endpoint'],
                    'interface_type': row['interface_type'],
                    'message_type': row['message_type'],
                    'state': SkillState.IDLE
                }

            self.get_logger().info(
                f'Discovered {len(self.skills)} Skill(s)'
            )

            for skill_name, skill_data in self.skills.items():
                self.get_logger().info(
                    f'{skill_name}: '
                    f'{skill_data["endpoint"]} | '
                    f'{skill_data["message_type"]} | '
                    f'{skill_data["state"].value}'
                )

            self.discovery_timer.cancel()

            self.get_logger().info(
                'CaSk Skill Manager is ready'
            )

        except Exception as error:
            self.get_logger().error(
                f'Failed to process Skill data: {error}'
            )

    def validate_skill_goal(self, goal_request):
        if goal_request.skill_id not in self.skills:
            self.get_logger().error(
                f'Unknown Skill: {goal_request.skill_id}'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted Skill request: '
            f'{goal_request.skill_id}'
        )

        return GoalResponse.ACCEPT

    def accept_cancel_request(self, goal_handle):
        self.get_logger().info(
            f'Accepted Skill cancellation request for '
            f'{goal_handle.request.skill_id}'
        )

        self.cancel_robot_goal((goal_handle.request.execution_id, goal_handle.request.skill_id))

        return CancelResponse.ACCEPT

    def set_robot_goal(self, execution_id, robot_goal_handle):
        with self.robot_goal_lock:
            self.robot_goal_handles[execution_id] = robot_goal_handle
            self.robot_cancel_sent.discard(execution_id)

    def clear_robot_goal(self, execution_id, robot_goal_handle):
        with self.robot_goal_lock:
            current = self.robot_goal_handles.get(execution_id)
            if current is robot_goal_handle:
                self.robot_goal_handles.pop(execution_id, None)
                self.robot_cancel_sent.discard(execution_id)

    def cancel_robot_goal(self, execution_id):
        with self.robot_goal_lock:
            robot_goal_handle = self.robot_goal_handles.get(
                execution_id
            )
            if (
                robot_goal_handle is None
                or execution_id in self.robot_cancel_sent
            ):
                return

            self.robot_cancel_sent.add(execution_id)

        self.get_logger().info(
            'Forwarding cancellation to the robot Action server'
        )
        robot_goal_handle.cancel_goal_async()

    async def resolve_goal_parameters(self, request, binding=None):
        """Use explicit Goal parameters or derive them from the World Model."""
        names = list(request.parameter_names)
        values = list(request.parameter_values)
        units = list(request.parameter_units)

        if names or values or units:
            if not (
                len(names) == len(values) == len(units)
                and len(names) > 0
            ):
                raise ValueError(
                    'parameter_names, parameter_values and '
                    'parameter_units must have the same non-zero length'
                )

            self.get_logger().info(
                'Using parameters supplied in the Action Goal'
            )
            return names, values, units

        if binding is None:
            raise ValueError(
                'Automatic ontology parameter inference requires '
                'required_capability_id and provided_capability_id'
            )

        if self.final_scene is not None:
            from .final_parameters import parameters
            return await parameters(self, request, binding)

        if request.task_id == 'Task_005' and binding['resource_name'] == 'Jekko_01':
            return await self.load_task005_parameters(request, binding)

        parameterizer_key = (
            binding['process_kind'],
            binding['resource_name'],
        )
        parameterizer = self.parameterizers.get(parameterizer_key)

        if parameterizer is None:
            raise ValueError(
                'No parameterizer registered for '
                f'process={binding["process_kind"]}, '
                f'resource={binding["resource_name"]}'
            )

        self.get_logger().info(
            'Using semantic parameterizer: '
            f'{binding["process_kind"]} + '
            f'{binding["resource_name"]}'
        )

        return await parameterizer(request.required_capability_id)

    async def resolve_execution_binding(self, request):
        """Validate the Task Manager's semantic execution binding."""
        identifiers = {
            'task_id': request.task_id,
            'required_capability_id': request.required_capability_id,
            'provided_capability_id': request.provided_capability_id,
            'skill_id': request.skill_id,
        }

        for label, identifier in identifiers.items():
            if not re.fullmatch(
                r'[A-Za-z_][A-Za-z0-9_]*',
                identifier,
            ):
                raise ValueError(
                    f'Invalid or missing {label}: {identifier}'
                )

        ontology_request = QueryOntology.Request()
        ontology_request.query = f"""
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX cask: <http://www.w3id.org/hsu-aut/cask#>
PREFIX css: <http://www.w3id.org/hsu-aut/css#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>

SELECT DISTINCT ?resource ?process_kind
WHERE {{
    abox:{request.task_id}
        css:requiresCapability
        abox:{request.required_capability_id} .

    abox:{request.required_capability_id}
        a cask:RequiredCapability ;
        css:isSpecifiedBy ?requirement_property .

    ?requirement_property
        din:has_Type_Description ?process_type ;
        din:has_Instance_Description ?requirement_description .

    ?requirement_description din:Value ?process_kind .

    abox:{request.provided_capability_id}
        a cask:ProvidedCapability ;
        css:isSpecifiedBy ?assurance_property ;
        css:isRealizedBySkill abox:{request.skill_id} .

    ?assurance_property
        din:has_Type_Description ?process_type ;
        din:has_Instance_Description ?assurance_description .

    ?assurance_description din:Value ?process_kind .

    ?resource
        css:providesCapability
            abox:{request.provided_capability_id} ;
        css:providesSkill abox:{request.skill_id} .
}}
"""

        response = await self.query_client.call_async(ontology_request)

        if not response.success:
            raise RuntimeError(
                f'World Model binding query failed: {response.message}'
            )

        rows = json.loads(response.result_json)
        if len(rows) != 1:
            raise ValueError(
                'Invalid or ambiguous execution binding: '
                f'{request.task_id} / '
                f'{request.required_capability_id} / '
                f'{request.provided_capability_id} / '
                f'{request.skill_id}; received {len(rows)} result(s)'
            )

        binding = rows[0]
        binding['resource_name'] = self.local_name(
            binding['resource']
        )

        self.get_logger().info(
            'Validated execution binding: '
            f'{request.required_capability_id} -> '
            f'{request.provided_capability_id} -> '
            f'{binding["resource_name"]}/'
            f'{request.skill_id}'
        )
        return binding

    async def load_delivery_hold_duration(self, required_capability_id):
        request = QueryOntology.Request()
        request.query = f"""
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>
SELECT ?output ?duration ?unit ?logic WHERE {{
  abox:{required_capability_id} vdi:hasOutput ?output .
  ?output din:has_Data_Element ?property .
  ?property din:has_Type_Description abox:DeliveryHeightHoldDuration_TD ;
            din:has_Instance_Description ?description .
  ?description din:Value ?duration ; din:Logic_Interpretation ?logic .
  abox:DeliveryHeightHoldDuration_TD din:Unit_of_Measure ?unit .
}}"""
        response = await self.query_client.call_async(request)
        if not response.success:
            raise RuntimeError('Hold requirement query failed: '+response.message)
        rows = json.loads(response.result_json)
        if not rows:
            return 0.0
        if len(rows) != 1:
            raise ValueError('Ambiguous delivery hold requirement')
        row = rows[0]
        duration = float(row['duration'])
        if (row['unit'] != 's' or row['logic'] != '='
                or not math.isfinite(duration) or duration < 0):
            raise ValueError('Invalid delivery hold requirement or scope')
        # The same VDI output product owns both DIN data elements.
        # Fail on missing/ambiguous height instead of interpreting a missing join
        # as absence of the duration requirement.
        request.query = f"""
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>
SELECT ?height ?value ?unit ?logic WHERE {{
  <{row['output']}> din:has_Data_Element ?height .
  ?height din:has_Type_Description abox:LiftingHeight_TD ;
          din:has_Instance_Description ?description .
  ?description din:Value ?value ; din:Logic_Interpretation ?logic .
  abox:LiftingHeight_TD din:Unit_of_Measure ?unit .
}}"""
        checked = await self.query_client.call_async(request)
        if not checked.success:
            raise RuntimeError('Hold height query failed: '+checked.message)
        heights = json.loads(checked.result_json)
        if len(heights) != 1:
            raise ValueError('Hold output requires exactly one lifting height')
        height = heights[0]
        if (height['unit'] != 'm' or height['logic'] != '='
                or not math.isfinite(float(height['value']))):
            raise ValueError('Invalid output lifting height for hold')
        if duration > 0 and not self.get_parameter('physical_cargo_mode').value:
            raise ValueError('Measured delivery hold requires physical_cargo_mode')
        self.get_logger().info(f'Ontology delivery-height hold: {duration:.3f} s')
        return duration

    async def load_crane_lifting_parameters(
        self,
        required_capability_id,
    ):
        """Read lifting input/output Products and height from the ontology."""
        if not re.fullmatch(
            r'[A-Za-z_][A-Za-z0-9_]*',
            required_capability_id,
        ):
            raise ValueError(
                'Invalid RequiredCapability identifier: '
                f'{required_capability_id}'
            )

        request = QueryOntology.Request()
        request.query = f"""
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX css: <http://www.w3id.org/hsu-aut/css#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>

SELECT DISTINCT
    ?pickup_frame ?pickup_x ?pickup_y ?pickup_z ?pickup_yaw
    ?delivery_frame ?delivery_x ?delivery_y
    ?delivery_z ?delivery_yaw
    ?target_height ?height_unit
WHERE {{
    abox:{required_capability_id}
        vdi:hasInput ?pickup_product ;
        vdi:hasOutput ?delivery_product .

    ?pickup_product
        abox:hasReferenceFrame ?pickup_frame ;
        abox:hasPositionX ?pickup_x ;
        abox:hasPositionY ?pickup_y .

    ?delivery_product
        abox:hasReferenceFrame ?delivery_frame ;
        abox:hasPositionX ?delivery_x ;
        abox:hasPositionY ?delivery_y ;
        din:has_Data_Element ?height_property .

    OPTIONAL {{ ?pickup_product abox:hasPositionZ ?pickup_z . }}
    OPTIONAL {{ ?pickup_product abox:hasOrientationYaw ?pickup_yaw . }}
    OPTIONAL {{ ?delivery_product abox:hasPositionZ ?delivery_z . }}
    OPTIONAL {{ ?delivery_product abox:hasOrientationYaw ?delivery_yaw . }}

    ?height_property
        din:has_Type_Description abox:LiftingHeight_TD ;
        din:has_Instance_Description ?height_description .

    ?height_description din:Value ?target_height .
    abox:LiftingHeight_TD din:Unit_of_Measure ?height_unit .
}}
"""

        response = await self.query_client.call_async(request)

        if not response.success:
            raise RuntimeError(
                f'World Model query failed: {response.message}'
            )

        rows = json.loads(response.result_json)

        if len(rows) != 1:
            raise ValueError(
                f'Expected exactly one lifting parameter set for '
                f'{required_capability_id}, received {len(rows)}'
            )

        row = rows[0]

        recovery_pickup_height = None
        pickup_frame = row['pickup_frame']
        delivery_frame = row['delivery_frame']

        if pickup_frame != delivery_frame:
            raise ValueError(
                'Pickup and delivery coordinates use different '
                f'reference frames: {pickup_frame}, {delivery_frame}'
            )

        pickup_x = float(row['pickup_x'])
        pickup_y = float(row['pickup_y'])
        delivery_x = float(row['delivery_x'])
        delivery_y = float(row['delivery_y'])
        target_height = float(row['target_height'])
        height_unit = row['height_unit']
        physical = bool(self.get_parameter('physical_cargo_mode').value)
        if physical and pickup_frame != 'world':
            raise ValueError('Physical Task_001 cargo coordinates must use world frame')
        if physical and row.get('pickup_z') in (None, ''):
            raise ValueError('Physical Task_001 input product requires hasPositionZ')
        if physical and row.get('delivery_z') in (None, ''):
            raise ValueError('Physical Task_001 output product requires hasPositionZ')
        cargo_pickup = [pickup_x,pickup_y,float(row.get('pickup_z') or 0)]
        cargo_delivery = [delivery_x,delivery_y,float(row.get('delivery_z') or 0),float(row.get('delivery_yaw') or 0)]

        if pickup_frame == 'world':
            pickup_x, pickup_y = self.world_to_crane_coordinates(
                pickup_x,
                pickup_y,
            )
            delivery_x, delivery_y = self.world_to_crane_coordinates(
                delivery_x,
                delivery_y,
            )
        elif pickup_frame != 'crane_base_link':
            raise ValueError(
                'The Crane parameterizer supports frame "world" or '
                f'"crane_base_link", received "{pickup_frame}"'
            )

        pickup_tower_yaw = round(
            math.atan2(pickup_y, pickup_x),
            5
        )
        pickup_trolley_position = math.hypot(
            pickup_x,
            pickup_y
        )
        delivery_tower_yaw = round(
            math.atan2(delivery_y, delivery_x),
            5
        )
        delivery_trolley_position = math.hypot(
            delivery_x,
            delivery_y
        )

        if physical:
            pickup_trolley_position -= 0.40
            delivery_trolley_position -= 0.40

        names = [
            'pickup_tower_yaw',
            'pickup_trolley_position',
            'target_height',
            'delivery_tower_yaw',
            'delivery_trolley_position',
        ]
        values = [
            pickup_tower_yaw,
            pickup_trolley_position,
            target_height,
            delivery_tower_yaw,
            delivery_trolley_position,
        ]
        units = [
            'rad',
            'm',
            height_unit,
            'rad',
            'm',
        ]
        if physical:
            names += ['cargo_pickup_x','cargo_pickup_y','cargo_pickup_z',
                      'cargo_delivery_x','cargo_delivery_y','cargo_delivery_z','cargo_delivery_yaw','rigging_gap']
            values += cargo_pickup + cargo_delivery + [0.75 if required_capability_id == 'RequiredLifting_002' else 0.60]
            units += ['m','m','m','m','m','m','rad','m']
        if recovery_pickup_height is not None and not physical:
            names.append('recovery_pickup_height')
            values.append(recovery_pickup_height)
            units.append('m')
        if required_capability_id == 'RequiredLift_001' and not physical:
            if row.get('delivery_z') in (None, ''):
                raise ValueError('Task_001 delivery product requires hasPositionZ')
            names.append('delivery_hook_height')
            # Must match the Rebar attachment offset in Component Manager.
            values.append(float(row['delivery_z']) + 0.21)
            units.append('m')

        self.get_logger().info(
            f'Generated lifting parameters from World Model: '
            f'frame={pickup_frame}, '
            f'pickup_yaw={pickup_tower_yaw:.4f}, '
            f'pickup_trolley={pickup_trolley_position:.4f}, '
            f'target_height={target_height:.4f}, '
            f'delivery_yaw={delivery_tower_yaw:.4f}, '
            f'delivery_trolley={delivery_trolley_position:.4f}'
        )

        duration = await self.load_delivery_hold_duration(required_capability_id)
        names.append('delivery_hold_seconds')
        values.append(duration)
        units.append('s')
        return names, values, units

    def world_to_crane_coordinates(self, world_x, world_y):
        """Transform a world point into the fixed Crane base frame."""
        delta_x = world_x - self.crane_world_x
        delta_y = world_y - self.crane_world_y

        cosine = math.cos(self.crane_world_yaw)
        sine = math.sin(self.crane_world_yaw)

        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        return local_x, local_y

    async def load_required_product_value(
        self, required_capability_id, type_description
    ):
        """Read one numerical output requirement and its unit."""
        for identifier in (required_capability_id, type_description):
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', identifier):
                raise ValueError(f'Invalid ontology identifier: {identifier}')

        request = QueryOntology.Request()
        request.query = f"""
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>

SELECT DISTINCT ?value ?unit
WHERE {{
    abox:{required_capability_id}
        vdi:hasOutput ?product .
    ?product din:has_Data_Element ?property .
    ?property
        din:has_Type_Description abox:{type_description} ;
        din:has_Instance_Description ?description .
    ?description din:Value ?value .
    abox:{type_description} din:Unit_of_Measure ?unit .
}}
"""
        response = await self.query_client.call_async(request)
        if not response.success:
            raise RuntimeError(
                f'World Model query failed: {response.message}'
            )
        rows = json.loads(response.result_json)
        if len(rows) != 1:
            raise ValueError(
                f'Expected exactly one {type_description} value for '
                f'{required_capability_id}, received {len(rows)}'
            )
        value = float(rows[0]['value'])
        unit = rows[0]['unit']
        if not math.isfinite(value) or value <= 0 or unit != 'm':
            raise ValueError(
                f'Invalid {type_description} value or unit: '
                f'{value} {unit}'
            )
        return value, unit







    async def handoff_query(self, query):
        request = QueryOntology.Request()
        request.query = query
        response = await self.query_client.call_async(request)
        if not response.success:
            raise RuntimeError(response.message)
        return json.loads(response.result_json)

    async def execute_skill(self, goal_handle):
        request = goal_handle.request
        skill_id = request.skill_id
        resource_id = ''

        result = ExecuteSkill.Result()

        try:
            skill_data = self.skills[skill_id]
            endpoint = skill_data['endpoint']

            skill_data['state'] = SkillState.RUNNING

            self.get_logger().info(
                f'Executing {skill_id} through {endpoint}'
            )

            if endpoint not in self.action_clients:
                self.action_clients[endpoint] = ActionClient(
                    self,
                    ExecuteSkill,
                    endpoint,
                    callback_group=self.callback_group
                )

            action_client = self.action_clients[endpoint]

            if not action_client.wait_for_server(
                timeout_sec=5.0
            ):
                skill_data['state'] = SkillState.FAILURE
                goal_handle.abort()

                result.success = False
                result.message = (
                    f'Action server unavailable: {endpoint}'
                )

                self.publish_execution_state(
                    request,
                    resource_id,
                    0.0,
                    SkillState.FAILURE.value,
                )

                self.get_logger().error(result.message)
                return result

            has_explicit_parameters = bool(
                request.parameter_names
                or request.parameter_values
                or request.parameter_units
            )

            binding = None
            if not has_explicit_parameters:
                binding = await self.resolve_execution_binding(request)
                resource_id = binding['resource_name']

            robot_goal = ExecuteSkill.Goal()
            robot_goal.execution_id = request.execution_id
            robot_goal.task_id = request.task_id
            robot_goal.required_capability_id = (
                request.required_capability_id
            )
            robot_goal.provided_capability_id = (
                request.provided_capability_id
            )
            robot_goal.skill_id = request.skill_id

            (
                robot_goal.parameter_names,
                robot_goal.parameter_values,
                robot_goal.parameter_units,
            ) = await self.resolve_goal_parameters(
                request,
                binding,
            )

            if not has_explicit_parameters and self.get_parameter('physical_cargo_mode').value:
                (robot_goal.parameter_names, robot_goal.parameter_values,
                 robot_goal.parameter_units) = await parameterize_handoff(
                    self.handoff_query, request, robot_goal.parameter_names,
                    robot_goal.parameter_values, robot_goal.parameter_units)

            self.publish_execution_state(
                request,
                resource_id,
                0.0,
                SkillState.RUNNING.value,
            )

            self.get_logger().info(
                f'Forwarding Goal to {endpoint}'
            )

            send_goal_future = action_client.send_goal_async(
                robot_goal,
                feedback_callback=lambda feedback:
                    self.forward_feedback(
                        goal_handle,
                        feedback,
                        request,
                        resource_id,
                    )
            )

            robot_goal_handle = await send_goal_future

            if not robot_goal_handle.accepted:
                skill_data['state'] = SkillState.FAILURE
                goal_handle.abort()

                result.success = False
                result.message = (
                    f'Robot rejected Skill: {skill_id}'
                )

                self.publish_execution_state(
                    request,
                    resource_id,
                    0.0,
                    SkillState.FAILURE.value,
                )

                self.get_logger().error(result.message)
                return result

            self.get_logger().info(
                f'Robot accepted Skill: {skill_id}'
            )

            execution_id = (request.execution_id, request.skill_id)
            self.set_robot_goal(execution_id, robot_goal_handle)

            if goal_handle.is_cancel_requested:
                self.cancel_robot_goal(execution_id)

            result_future = robot_goal_handle.get_result_async()
            try:
                wrapped_result = await result_future
            finally:
                self.clear_robot_goal(
                    execution_id,
                    robot_goal_handle,
                )

            robot_result = wrapped_result.result

            if goal_handle.is_cancel_requested:
                skill_data['state'] = SkillState.CANCELED
                if goal_handle.is_active:
                    goal_handle.canceled()

                result.success = False
                result.message = f'{skill_id} canceled'

                self.publish_execution_state(
                    request,
                    resource_id,
                    0.0,
                    SkillState.CANCELED.value,
                )
                self.get_logger().info(result.message)
                return result

            result.success = robot_result.success
            result.message = robot_result.message

            if robot_result.success:
                skill_data['state'] = SkillState.SUCCESS
                goal_handle.succeed()
            else:
                skill_data['state'] = SkillState.FAILURE
                goal_handle.abort()

            final_progress = 100.0 if robot_result.success else 0.0
            self.publish_execution_state(
                request,
                resource_id,
                final_progress,
                result_status(robot_result.success, robot_result.message),
            )

            self.get_logger().info(
                f'{skill_id}: '
                f'{skill_data["state"].value} - '
                f'{result.message}'
            )

            return result

        except Exception as error:
            if skill_id in self.skills:
                self.skills[skill_id][
                    'state'
                ] = (
                    SkillState.CANCELED
                    if goal_handle.is_cancel_requested
                    else SkillState.FAILURE
                )

            if goal_handle.is_cancel_requested:
                if goal_handle.is_active:
                    goal_handle.canceled()
            elif goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = (
                f'Skill Manager execution error: {error}'
            )

            final_state = (
                SkillState.CANCELED
                if goal_handle.is_cancel_requested
                else SkillState.FAILURE
            )
            self.publish_execution_state(
                request,
                resource_id,
                0.0,
                final_state.value,
            )

            self.get_logger().error(
                result.message
            )

            return result

    def forward_feedback(
        self,
        manager_goal_handle,
        feedback_message,
        request,
        resource_id,
    ):
        if not manager_goal_handle.is_active:
            return

        feedback = ExecuteSkill.Feedback()

        feedback.progress = (
            feedback_message.feedback.progress
        )
        feedback.status = (
            feedback_message.feedback.status
        )

        manager_goal_handle.publish_feedback(
            feedback
        )

        self.publish_execution_state(
            request,
            resource_id,
            feedback.progress,
            feedback.status,
        )

    def publish_execution_state(
        self,
        request,
        resource_id,
        progress,
        status,
    ):
        message = SkillExecutionState()
        message.execution_id = request.execution_id
        message.task_id = request.task_id
        message.required_capability_id = (
            request.required_capability_id
        )
        message.provided_capability_id = (
            request.provided_capability_id
        )
        message.skill_id = request.skill_id
        message.resource_id = resource_id
        message.progress = float(progress)
        message.status = status

        self.execution_state_publisher.publish(message)

    @staticmethod
    def local_name(uri):
        if '#' in uri:
            return uri.rsplit('#', 1)[1]

        return uri.rsplit('/', 1)[-1]

    async def load_task005_parameters(self, request, binding):
        """Numerical targets for the new chair models, read from ABox coordinates."""
        prefixes = '''
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>
'''
        rows = await self.handoff_query(prefixes + f'''
SELECT DISTINCT ?input ?output WHERE {{
    abox:{request.required_capability_id} vdi:hasInput ?input ; vdi:hasOutput ?output .
}}''')
        if len(rows) != 1:
            raise ValueError('Task005 requires one unambiguous product flow')
        references = {'pickup': rows[0]['input'], 'delivery': rows[0]['output']}
        if binding.get('resource_name') == 'Jekko_01':
            references['jekko_work'] = 'http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#Task005JekkoWorkPose'
        names, values, units = [], [], []
        for key, uri in references.items():
            poses = await self.handoff_query(prefixes + f'''
SELECT DISTINCT ?frame ?x ?y ?z ?yaw WHERE {{
    <{uri}> abox:hasReferenceFrame ?frame ; abox:hasPositionX ?x ;
            abox:hasPositionY ?y ; abox:hasPositionZ ?z ; abox:hasOrientationYaw ?yaw .
}}''')
            if len(poses) != 1 or poses[0]['frame'] != 'world':
                raise ValueError('Missing/ambiguous world pose in Task005 ABox: '+uri)
            for axis in ('x','y','z','yaw'):
                value = float(poses[0][axis])
                if not math.isfinite(value):
                    raise ValueError('Non-finite Task005 pose: '+uri)
                names.append(key+'_'+axis)
                values.append(value)
                units.append('rad' if axis == 'yaw' else 'm')
        # Requirements remain semantic DIN properties, not constants in motion code.
        for key, td, product, unit in (
                ('target_height','LiftingHeight_TD','Task005LiftingAfter','m'),):
            requirements = await self.handoff_query(prefixes + f'''
SELECT DISTINCT ?value ?unit WHERE {{
    abox:{product} din:has_Data_Element ?property .
    ?property din:has_Type_Description abox:{td} ; din:has_Instance_Description ?description .
    ?description din:Expression_Goal "Requirement" ; din:Value ?value .
    OPTIONAL {{ abox:{td} din:Unit_of_Measure ?unit . }}
}}''')
            if len(requirements) != 1:
                raise ValueError('Missing/ambiguous Task005 requirement: '+td)
            row = requirements[0]
            value = float(row['value'])
            actual_unit = str(row.get('unit') or unit)
            if key == 'position_error':
                if actual_unit == 'mm':
                    value /= 1000.
                elif actual_unit != 'm':
                    raise ValueError('Position error must use m or mm')
                actual_unit = 'm'
            elif actual_unit != unit:
                raise ValueError('Unsupported Task005 unit: '+actual_unit)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Task005 requirements must be finite and positive')
            names.append(key)
            values.append(value)
            units.append(actual_unit)
        return names, values, units




def main(args=None):
    rclpy.init(args=args)

    node = CaskSkillManager()

    executor = MultiThreadedExecutor(
        num_threads=4
    )
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

# CASK_TASK005_EXECUTION_V1

# CASK_JEKKO_APPROACH_V1
