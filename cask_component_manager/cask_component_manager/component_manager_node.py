import json
import math
import re
import zlib
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from cask_interfaces.msg import SkillExecutionState
from cask_interfaces.srv import ModifyWorldModel, QueryOntology


class CaskComponentManager(Node):

    # Shared by the SDF generator and the clearance description: no duplicate dimensions.
    CARGO_BOXES = (
        ('deck', (0, 0, 0.125), (1.2, 1.0, 0.05), '0.45 0.30 0.16 1'),
        ('left_runner', (0, 0.48, -0.15), (1.2, 0.10, 0.5), '0.4 0.26 0.13 1'),
        ('right_runner', (0, -0.48, -0.15), (1.2, 0.10, 0.5), '0.4 0.26 0.13 1'),
        ('steel_bundle', (0, 0, 0.26), (0.22, 1.6, 0.22), '0.5 0.27 0.12 1'),
    )

    IDENTIFIER_PATTERN = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')

    ABOX_NAMESPACE = (
        'http://www.semanticweb.org/lixiangcheng/'
        'CASK+CRO-abox#'
    )
    RDF_TYPE = (
        'http://www.w3.org/1999/02/22-rdf-syntax-ns#type'
    )
    OWL_NAMED_INDIVIDUAL = (
        'http://www.w3.org/2002/07/owl#NamedIndividual'
    )
    VDI_PRODUCT = (
        'http://www.w3id.org/hsu-aut/VDI3682#Product'
    )
    CASK_HAS_CURRENT_STATE = (
        'http://www.w3id.org/hsu-aut/cask#hasCurrentState'
    )

    ATTACHED_STAGES = {
        'ATTACHING',
        'LIFTING',
        'ROTATING_TO_DELIVERY',
        'HOLDING_AT_DELIVERY',
        'DRIVING_TO_DELIVERY',
        'LOWERING',
        'LOADING',
        'TRANSPORTING',
        'SCOOPING',
        'RETRACTING',
    }

    INPUT_STAGES = {
        'RUNNING',
        'MOVING_TO_PICKUP',
        'DRIVING_TO_PICKUP',
    }

    def __init__(self):
        super().__init__('cask_component_manager')

        marker_qos = QoSProfile(depth=100)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_publisher = self.create_publisher(
            Marker,
            '/cask/components',
            marker_qos,
        )

        self.state_subscription = self.create_subscription(
            SkillExecutionState,
            '/cask/skill_execution_state',
            self.execution_state_callback,
            10,
        )

        self.query_client = self.create_client(
            QueryOntology,
            '/cask_world_model/query',
        )

        self.modify_client = self.create_client(
            ModifyWorldModel,
            '/cask_world_model/modify',
        )

        self.transfer_ack_publisher = self.create_publisher(
            String, '/cask/transfer_ack', 10)
        self.declare_parameter('gazebo_physics', False)
        self.gazebo_physics = bool(self.get_parameter('gazebo_physics').value)
        self.declare_parameter('spawn_legacy_components', False)
        self.spawn_legacy_components = bool(self.get_parameter('spawn_legacy_components').value)
        self.physical_pose = None
        self.physical_pose_time = None
        self.physics_context = None
        self.physics_initializing = False
        self.physics_spawned = False
        self.physics_spawn_error = None
        self.physics_write_busy = False
        self.physics_last_write = 0.0
        if self.gazebo_physics and self.spawn_legacy_components:
            self.clearance_publisher = self.create_publisher(
                String, '/cask/task001/cargo_clearance', 10)
            self.physics_publisher = self.create_publisher(
                PoseStamped, '/cask/task001/cargo_pose', 10)
            self.physics_subscription = self.create_subscription(
                TFMessage, '/model/task001_rebar/pose', self.gazebo_cargo_pose, 10)
            self.physics_timer = self.create_timer(0.5, self.initialize_physical_cargo)
        self.contexts = {}
        self.pending_queries = set()
        self.latest_states = {}
        self.last_logged_locations = {}
        self.persisted_successes = set()
        self.pending_persistence = set()
        self.runtime_restore_requested = False
        self.marker_cache = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.runtime_restore_timer = self.create_timer(
            0.5,
            self.restore_runtime_components,
        )
        self.marker_republish_timer = self.create_timer(
            1.0,
            self.republish_cached_markers,
        )

        # Task005 is an optional measured physical component in the existing manager.
        self.task005_spawned = False
        self.task005_initializing = False
        self.task005_pose = None
        self.task005_pose_time = 0.
        self.task005_support_top = 0.
        self.task005_support_xy = (0.,0.)
        if self.gazebo_physics and self.spawn_legacy_components:
            self.task005_pose_publisher = self.create_publisher(PoseStamped,'/cask/task005/cargo_pose',10)
            self.task005_clearance_publisher = self.create_publisher(String,'/cask/task005/cargo_clearance',10)
            self.task005_subscription = self.create_subscription(TFMessage,'/model/task005_steel/pose',self.task005_cargo_pose,10)
            self.task005_timer = self.create_timer(.5,self.initialize_task005_scene)

        from cask_gazebo_motion.generated_component_motion import GeneratedComponents
        self.generated_components = None  # IOC scene import is disabled in this workspace.

        self.declare_parameter('final_scene_enabled', False)
        self.declare_parameter('final_scene_config', '')
        self.final_scene = None
        if self.get_parameter('final_scene_enabled').value:
            from .final_scene import FinalScene
            self.final_scene = FinalScene(self, self.get_parameter('final_scene_config').value)

        self.get_logger().info('CaSk Component Manager is ready')
        self.get_logger().info(
            'Execution state topic: /cask/skill_execution_state'
        )
        self.get_logger().info('RViz Marker topic: /cask/components')
        self.get_logger().info(
            'Runtime state service: /cask_world_model/modify'
        )

    def destroy_node(self):
        if self.final_scene is not None:
            self.final_scene.close()
        return super().destroy_node()

    def initialize_physical_cargo(self):
        if self.physics_spawn_error:
            self.get_logger().error(self.physics_spawn_error)
            self.physics_timer.cancel()
            return
        if self.physics_spawned or self.physics_initializing or not self.query_client.service_is_ready():
            return
        self.physics_initializing = True
        request = QueryOntology.Request()
        request.query = self.component_query('RequiredLift_001').replace(
            '?recovery_x ?recovery_y ?recovery_z ?recovery_yaw\nWHERE',
            '?recovery_x ?recovery_y ?recovery_z ?recovery_yaw ?payload\nWHERE')
        request.query = request.query.replace(
            '    ?input_product din:has_Data_Element ?kind_property .',
            """    ?input_product din:has_Data_Element ?kind_property .
    ?input_product din:has_Data_Element ?weight_property .
    ?weight_property din:has_Type_Description abox:Payload_TD ;
                     din:has_Instance_Description ?weight_id .
    ?weight_id din:Value ?payload .""")
        future = self.query_client.call_async(request)
        future.add_done_callback(self.physical_cargo_query_completed)

    def physical_cargo_query_completed(self, future):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            rows = json.loads(response.result_json)
            if len(rows) != 1:
                raise ValueError('Expected one Task_001 cargo with payload, got ' + str(len(rows)))
            row = rows[0]
            context = self.make_context(row)
            if context['input_pose']['frame'] != 'world':
                raise ValueError('Task_001 physical cargo must be in world coordinates')
            mass = float(row['payload'])
            if not math.isfinite(mass) or mass <= 0:
                raise ValueError('Cargo payload must be positive')
            self.physics_context = context
            sdf = self.physical_cargo_sdf(context['input_pose'], mass)
            threading.Thread(target=self.spawn_physical_cargo,
                             args=(sdf, mass, dict(context['input_pose'])), daemon=True).start()
        except Exception as error:
            self.physics_spawn_error = 'Physical cargo initialization failed: ' + str(error)

    def spawn_physical_cargo(self, sdf, mass, pose):
        try:
            result = subprocess.run([
                'ros2', 'run', 'ros_gz_sim', 'create', '-world', 'empty',
                '-name', 'task001_rebar', '-allow_renaming', 'false', '-string', sdf,
                '-x', str(pose['x']), '-y', str(pose['y']),
                '-z', str(pose['z']), '-Y', str(pose['yaw'])],
                capture_output=True, text=True, timeout=30)
            output = result.stdout + result.stderr
            if result.returncode or 'OK creation of entity' not in output:
                raise RuntimeError(output)
            self.physics_spawned = True
            self.get_logger().info(f'PHYSICAL_CARGO_READY: Task_001, payload={mass} kg; '
                                   'collision and gravity enabled; no pose following.')
        except Exception as error:
            self.physics_spawn_error = ('Cargo creation failed; reopen Gazebo to remove any '
                                       'old cargo model: ' + str(error))

    @staticmethod
    def physical_cargo_sdf(pose, mass):
        # The product origin is a handling reference at z=0.4, not its COM.
        # Side runners reach the ground; the underside of the deck is 0.1 m
        # above the reference, leaving a pocket for the 0.45 m-high fork tops.
        if abs(pose['z'] - 0.4) > 0.01:
            raise ValueError('This Task_001 ground pallet expects reference z=0.4 m')
        root = ET.Element('sdf', version='1.7')
        model = ET.SubElement(root, 'model', name='task001_rebar')
        ET.SubElement(model, 'pose').text = ' '.join(str(v) for v in
            (pose['x'], pose['y'], pose['z'], 0, 0, pose['yaw']))
        link = ET.SubElement(model, 'link', name='cargo_link')
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'pose').text = '0 0 0.22 0 0 0'
        ET.SubElement(inertial, 'mass').text = str(mass)
        inertia = ET.SubElement(inertial, 'inertia')
        for key, value in {'ixx': mass*(1.6**2+0.8**2)/12,
                           'iyy': mass*(1.2**2+0.8**2)/12,
                           'izz': mass*(1.2**2+1.6**2)/12,
                           'ixy': 0, 'ixz': 0, 'iyz': 0}.items():
            ET.SubElement(inertia, key).text = str(value)
        for name, position, size, color in CaskComponentManager.CARGO_BOXES:
            for tag in ('visual', 'collision'):
                element = ET.SubElement(link, tag, name=name)
                ET.SubElement(element, 'pose').text = ' '.join(str(v) for v in (*position, 0, 0, 0))
                box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
                ET.SubElement(box, 'size').text = ' '.join(str(v) for v in size)
                if tag == 'visual':
                    material = ET.SubElement(element, 'material')
                    ET.SubElement(material, 'ambient').text = color
                    ET.SubElement(material, 'diffuse').text = color
                else:
                    ode = ET.SubElement(ET.SubElement(ET.SubElement(element, 'surface'), 'friction'), 'ode')
                    ET.SubElement(ode, 'mu').text = '0.9'
                    ET.SubElement(ode, 'mu2').text = '0.9'
        plugin = ET.SubElement(model, 'plugin',
            filename='ignition-gazebo-pose-publisher-system',
            name='ignition::gazebo::systems::PosePublisher')
        for key, value in {'publish_model_pose': 'true', 'publish_nested_model_pose': 'true',
                           'publish_link_pose': 'false',
                           'use_pose_vector_msg': 'true', 'update_frequency': '30'}.items():
            ET.SubElement(plugin, key).text = value
        return ET.tostring(root, encoding='unicode')

    def gazebo_cargo_pose(self, message):
        for transform in message.transforms:
            name = transform.child_frame_id.replace('::', '/').rstrip('/').split('/')[-1]
            if name != 'task001_rebar':
                continue
            p, q = transform.transform.translation, transform.transform.rotation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                return
            norm = sum(v*v for v in values[3:])
            if abs(norm-1.0) > 0.05:
                return
            self.physical_pose = {'frame': 'world', 'x': p.x, 'y': p.y, 'z': p.z,
                'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
            self.physical_pose_time = time.monotonic()
            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = 'world'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x, p.y, p.z
            pose.pose.orientation = q
            self.physics_publisher.publish(pose)
            # Current adapter: level pallet, resting on ground or forks inside
            # its open pocket. The deck underside is a conservative upper bound
            # for those supports, NOT for arbitrary surrounding obstacles/masts.
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
            deck = next(box for box in self.CARGO_BOXES if box[0] == 'deck')
            bottom = min(pos[2]-size[2]/2 for _,pos,size,_ in self.CARGO_BOXES)
            description = String()
            description.data = json.dumps({
                'frame': 'world', 'model': 'task001_rebar',
                'pose': [p.x,p.y,p.z], 'roll': roll, 'pitch': pitch,
                'bottom_offset': bottom,
                'support_top_z': p.z+deck[1][2]-deck[2][2]/2,
                'support_assumption': 'stationary_support_inside_pallet_pocket',
            })
            self.clearance_publisher.publish(description)
            if self.physics_context:
                state = SkillExecutionState()
                state.task_id = 'Task_001'
                self.publish_component_marker(state, self.physics_context, self.physical_pose)
            return

    def apply_physical_execution_state(self, state, context):
        if self.physical_pose is None or time.monotonic()-self.physical_pose_time > 2:
            self.get_logger().warning('Waiting for measured Gazebo cargo pose; no synthetic position update')
            return
        pose = dict(self.physical_pose)
        self.publish_component_marker(state, context, pose)
        if self.execution_stage(state.status) == 'SUCCESS':
            self.persist_success_state(state, context, pose)
        elif (time.monotonic()-self.physics_last_write > 1.0
              and not self.physics_write_busy and self.modify_client.service_is_ready()):
            self.physics_last_write = time.monotonic()
            self.physics_write_busy = True
            updates = [('hasPositionX',pose['x']),('hasPositionY',pose['y']),
                       ('hasPositionZ',pose['z']),('hasOrientationYaw',pose['yaw']),
                       ('hasReferenceFrame','world')]
            self.write_measured_coordinates(self.runtime_component_uri(state.task_id), updates)


    def write_measured_coordinates(self, subject, updates):
        if not updates:
            self.physics_write_busy = False
            return
        key, value = updates[0]
        request = self.modify_request(ModifyWorldModel.Request.SET, subject,
            self.ABOX_NAMESPACE + key, str(value), True)
        future = self.modify_client.call_async(request)
        def completed(done):
            try:
                response = done.result()
                if not response.success:
                    raise RuntimeError(response.message)
                self.write_measured_coordinates(subject, updates[1:])
            except Exception as error:
                self.physics_write_busy = False
                self.get_logger().error('Measured cargo coordinate update failed: ' + str(error))
        future.add_done_callback(completed)

    @staticmethod
    def local_name(uri):
        if uri is None:
            return ''
        return str(uri).rsplit('#', 1)[-1].rsplit('/', 1)[-1]

    @staticmethod
    def optional_float(row, name, default):
        value = row.get(name)
        if value in (None, ''):
            return float(default)
        return float(value)

    def restore_runtime_components(self):
        if self.runtime_restore_requested:
            return

        if not self.query_client.service_is_ready():
            return

        self.runtime_restore_requested = True

        request = QueryOntology.Request()
        request.query = self.runtime_component_query()

        future = self.query_client.call_async(request)
        future.add_done_callback(
            self.runtime_components_query_completed
        )

    @staticmethod
    def runtime_component_query():
        return """
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX cask: <http://www.w3id.org/hsu-aut/cask#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>

SELECT DISTINCT
    ?runtime_component ?current_state ?product_kind
    ?frame ?x ?y ?z ?yaw
WHERE {
    ?runtime_component cask:hasCurrentState ?current_state .

    FILTER(
        STRSTARTS(
            STR(?runtime_component),
            STR(abox:RuntimeComponent_)
        )
    )

    ?current_state din:has_Data_Element ?kind_property .

    ?kind_property
        din:has_Type_Description abox:ProductKind_TD ;
        din:has_Instance_Description ?kind_description .

    ?kind_description din:Value ?product_kind .

    OPTIONAL { ?runtime_component abox:hasReferenceFrame ?frame . }
    OPTIONAL { ?runtime_component abox:hasPositionX ?x . }
    OPTIONAL { ?runtime_component abox:hasPositionY ?y . }
    OPTIONAL { ?runtime_component abox:hasPositionZ ?z . }
    OPTIONAL { ?runtime_component abox:hasOrientationYaw ?yaw . }
}
ORDER BY ?runtime_component
"""

    def runtime_components_query_completed(self, future):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)

            rows = json.loads(response.result_json)
            for row in rows:
                self.publish_restored_runtime_component(row)

            if rows:
                self.get_logger().info(
                    'Restored '
                    f'{len(rows)} runtime component marker(s)'
                )
            else:
                self.get_logger().info(
                    'No persisted runtime components to restore'
                )

            self.runtime_restore_timer.cancel()

        except Exception as error:
            self.runtime_restore_requested = False
            self.get_logger().warning(
                f'Runtime component restoration failed: {error}; '
                'retrying'
            )

    def publish_restored_runtime_component(self, row):
        runtime_name = self.local_name(
            row['runtime_component']
        )
        task_id = runtime_name.removeprefix(
            'RuntimeComponent_'
        )
        current_state = self.local_name(row['current_state'])

        pose = {
            'frame': row.get('frame') or 'world',
            'x': self.optional_float(row, 'x', 0.0),
            'y': self.optional_float(row, 'y', 0.0),
            'z': self.optional_float(row, 'z', 0.0),
            'yaw': self.optional_float(row, 'yaw', 0.0),
        }

        context = {
            'input_product': current_state,
            'output_product': current_state,
            'product_kind': str(row['product_kind']),
            'input_pose': pose,
            'output_pose': pose,
        }

        state = SkillExecutionState()
        state.task_id = task_id

        self.publish_component_marker(state, context, pose)
        self.get_logger().info(
            'Restored runtime component: '
            f'{task_id} -> {current_state} '
            f'in frame {pose["frame"]}'
        )

    def execution_state_callback(self, message):
        key = (
            message.execution_id,
            message.required_capability_id,
        )
        self.latest_states[key] = message

        if key in self.contexts:
            self.apply_execution_state(message, self.contexts[key])
            return

        if key in self.pending_queries:
            return

        if not self.IDENTIFIER_PATTERN.fullmatch(
            message.required_capability_id
        ):
            self.get_logger().error(
                'Invalid RequiredCapability identifier: '
                f'{message.required_capability_id}'
            )
            return

        if not self.query_client.service_is_ready():
            self.get_logger().warning(
                'World Model query service is not ready; waiting for '
                'the next execution-state update'
            )
            return

        self.pending_queries.add(key)

        request = QueryOntology.Request()
        request.query = self.component_query(
            message.required_capability_id, message.execution_id
        )

        future = self.query_client.call_async(request)
        future.add_done_callback(
            lambda completed, query_key=key:
            self.component_query_completed(completed, query_key)
        )

    @staticmethod
    def component_query(required_capability_id, execution_id=None):
        recovery_clause = ''
        if required_capability_id in ('RequiredLift_001', 'RequiredTransporting_002'):
            recovery_clause = '''
    OPTIONAL {
        abox:RuntimeComponent_Task_001
            abox:hasRecoveryStatus ?recovery_status ;
            abox:hasReferenceFrame ?recovery_frame ;
            abox:hasPositionX ?recovery_x ;
            abox:hasPositionY ?recovery_y ;
            abox:hasPositionZ ?recovery_z ;
            abox:hasOrientationYaw ?recovery_yaw .
        FILTER(STR(?recovery_status) = "READY" &&
               STR(?recovery_frame) = "world")
    }
'''
            if required_capability_id == 'RequiredTransporting_002':
                recovery_clause = recovery_clause.replace('Task_001', 'Task_002')
            if execution_id is not None:
                recovery_clause = recovery_clause.replace(
                    'abox:hasRecoveryStatus ?recovery_status ;',
                    'abox:hasRecoveryExecutionId ?recovery_execution ; '
                    'abox:hasRecoveryStatus ?recovery_status ;')
                recovery_clause = recovery_clause.replace(
                    'FILTER(', 'FILTER(STR(?recovery_execution) = '
                    + json.dumps(execution_id) + ' && ')
        return f"""
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>

SELECT DISTINCT
    ?input_product ?output_product ?product_kind
    ?input_frame ?input_x ?input_y ?input_z ?input_yaw
    ?output_frame ?output_x ?output_y ?output_z ?output_yaw
    ?input_height ?output_height
    ?recovery_x ?recovery_y ?recovery_z ?recovery_yaw
WHERE {{
    abox:{required_capability_id}
        vdi:hasInput ?input_product ;
        vdi:hasOutput ?output_product .

    ?input_product din:has_Data_Element ?kind_property .

    ?kind_property
        din:has_Type_Description abox:ProductKind_TD ;
        din:has_Instance_Description ?kind_description .

    ?kind_description din:Value ?product_kind .

    OPTIONAL {{ ?input_product abox:hasReferenceFrame ?input_frame . }}
    OPTIONAL {{ ?input_product abox:hasPositionX ?input_x . }}
    OPTIONAL {{ ?input_product abox:hasPositionY ?input_y . }}
    OPTIONAL {{ ?input_product abox:hasPositionZ ?input_z . }}
    OPTIONAL {{ ?input_product abox:hasOrientationYaw ?input_yaw . }}

    OPTIONAL {{ ?output_product abox:hasReferenceFrame ?output_frame . }}
    OPTIONAL {{ ?output_product abox:hasPositionX ?output_x . }}
    OPTIONAL {{ ?output_product abox:hasPositionY ?output_y . }}
    OPTIONAL {{ ?output_product abox:hasPositionZ ?output_z . }}
    OPTIONAL {{ ?output_product abox:hasOrientationYaw ?output_yaw . }}

    OPTIONAL {{
        ?input_product din:has_Data_Element ?input_height_property .
        ?input_height_property
            din:has_Type_Description abox:LiftingHeight_TD ;
            din:has_Instance_Description ?input_height_description .
        ?input_height_description din:Value ?input_height .
    }}

    OPTIONAL {{
        ?output_product din:has_Data_Element ?output_height_property .
        ?output_height_property
            din:has_Type_Description abox:LiftingHeight_TD ;
            din:has_Instance_Description ?output_height_description .
        ?output_height_description din:Value ?output_height .
    }}
    {recovery_clause}
}}
"""

    def component_query_completed(self, future, key):
        self.pending_queries.discard(key)

        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)

            rows = json.loads(response.result_json)
            if len(rows) != 1:
                raise ValueError(
                    'Expected exactly one product flow for '
                    f'{key[1]}, received {len(rows)}'
                )

            context = self.make_context(rows[0])
            self.contexts[key] = context

            self.get_logger().info(
                f'Loaded component flow: {key[1]} -> '
                f'{context["input_product"]} -> '
                f'{context["output_product"]}; '
                f'kind={context["product_kind"]}'
            )

            latest_state = self.latest_states.get(key)
            if latest_state is not None:
                self.apply_execution_state(latest_state, context)

        except Exception as error:
            self.get_logger().error(
                f'Component query failed for {key[1]}: {error}'
            )

    def make_context(self, row):
        input_height = self.optional_float(
            row,
            'input_height',
            0.0,
        )
        input_z = self.optional_float(
            row,
            'input_z',
            input_height,
        )

        output_height = self.optional_float(
            row,
            'output_height',
            input_z,
        )
        output_z = self.optional_float(
            row,
            'output_z',
            output_height,
        )

        input_frame = row.get('input_frame') or 'world'
        output_frame = row.get('output_frame') or input_frame

        input_pose = {
            'frame': input_frame,
            'x': self.optional_float(row, 'input_x', 0.0),
            'y': self.optional_float(row, 'input_y', 0.0),
            'z': input_z,
            'yaw': self.optional_float(row, 'input_yaw', 0.0),
        }
        if row.get('recovery_x') not in (None, ''):
            input_pose.update({
                'frame': 'world',
                'x': float(row['recovery_x']),
                'y': float(row['recovery_y']),
                'z': float(row['recovery_z']),
                'yaw': float(row['recovery_yaw']),
            })

        output_pose = {
            'frame': output_frame,
            'x': self.optional_float(
                row,
                'output_x',
                input_pose['x'],
            ),
            'y': self.optional_float(
                row,
                'output_y',
                input_pose['y'],
            ),
            'z': output_z,
            'yaw': self.optional_float(
                row,
                'output_yaw',
                input_pose['yaw'],
            ),
        }

        return {
            'input_product': self.local_name(row['input_product']),
            'output_product': self.local_name(row['output_product']),
            'product_kind': str(row['product_kind']),
            'input_pose': input_pose,
            'output_pose': output_pose,
        }

    @staticmethod
    def execution_stage(status):
        return status.split('|', 1)[0].strip().upper()


    def apply_execution_state(self, state, context):
        final = getattr(self, 'final_scene', None)
        if final is not None and context['input_product'].rsplit('#',1)[-1].startswith('FinalScenarioSteel'):
            if final.latest is None or time.monotonic()-final.last_seen > 2.:
                self.get_logger().warning('Waiting for measured final cargo pose')
                return
            x,y,z,yaw = final.latest
            pose = dict(frame='world',x=x,y=y,z=z,yaw=yaw)
            self.publish_component_marker(state,context,pose)
            if self.execution_stage(state.status)=='SUCCESS':
                self.persist_success_state(state,context,pose)
            return
        if self.generated_components is not None and self.generated_components.apply(state, context):
            return
        if self.gazebo_physics and state.task_id == 'Task_005':
            if self.task005_pose is None or time.monotonic() - self.task005_pose_time > 2.0:
                self.get_logger().warning('Waiting for measured Task005 physical component pose')
                return
            pose = dict(self.task005_pose)
            self.publish_component_marker(state, context, pose)
            if self.execution_stage(state.status) == 'SUCCESS':
                self.persist_success_state(state, context, pose)
            return
        stage = self.execution_stage(state.status)
        if self.gazebo_physics and state.task_id == 'Task_001':
            self.apply_physical_execution_state(state, context)
            return
        if stage == 'SUCCESS':
            pose = context['output_pose']
            location = 'OUTPUT'
        elif stage in self.ATTACHED_STAGES:
            pose = self.attachment_pose(state.resource_id, context['product_kind'])
            if pose is None:
                pose = context['input_pose']
                location = 'INPUT'
            else:
                location = 'ATTACHED'
                context['attached_resource'] = state.resource_id
        elif stage == 'FAILURE':
            return
        elif stage in ('PATH_READY', 'REPLANNING', 'PATH_BLOCKED') and context.get('attached_resource') == state.resource_id:
            pose = self.attachment_pose(state.resource_id, context['product_kind'])
            location = 'ATTACHED'
        else:
            pose = context['input_pose']
            location = 'INPUT'
        self.publish_component_marker(state, context, pose)
        if stage == 'SUCCESS':
            self.persist_success_state(state, context, pose)
        log_key = (state.execution_id, context['product_kind'])
        log_value = (location, pose['frame'], context['output_product'] if location == 'OUTPUT' else context['input_product'])
        if self.last_logged_locations.get(log_key) != log_value:
            self.last_logged_locations[log_key] = log_value
            self.get_logger().info(f"Component {context['product_kind']}: {location} in frame {pose['frame']}")






    def persist_success_state(self, state, context, pose):
        persistence_key = (
            state.execution_id,
            state.required_capability_id,
        )

        if persistence_key in self.persisted_successes:
            return

        if persistence_key in self.pending_persistence:
            return

        if not self.modify_client.service_is_ready():
            self.get_logger().error(
                'Cannot persist component state: World Model '
                'modify service is not ready'
            )
            return

        runtime_component = self.runtime_component_uri(
            state.task_id
        )
        output_state = (
            self.ABOX_NAMESPACE + context['output_product']
        )
        current_state_property = self.CASK_HAS_CURRENT_STATE

        operations = [
            self.modify_request(
                operation=ModifyWorldModel.Request.ADD,
                subject=runtime_component,
                predicate=self.RDF_TYPE,
                obj=self.OWL_NAMED_INDIVIDUAL,
                object_is_literal=False,
            ),
            self.modify_request(
                operation=ModifyWorldModel.Request.ADD,
                subject=runtime_component,
                predicate=self.RDF_TYPE,
                obj=self.VDI_PRODUCT,
                object_is_literal=False,
            ),
            self.modify_request(
                operation=ModifyWorldModel.Request.SET,
                subject=runtime_component,
                predicate=current_state_property,
                obj=output_state,
                object_is_literal=False,
            ),
            self.modify_request(
                operation=ModifyWorldModel.Request.SET,
                subject=runtime_component,
                predicate=self.ABOX_NAMESPACE + 'hasPositionX',
                obj=self.number_text(pose['x']),
                object_is_literal=True,
            ),
            self.modify_request(
                operation=ModifyWorldModel.Request.SET,
                subject=runtime_component,
                predicate=self.ABOX_NAMESPACE + 'hasPositionY',
                obj=self.number_text(pose['y']),
                object_is_literal=True,
            ),
            self.modify_request(
                operation=ModifyWorldModel.Request.SET,
                subject=runtime_component,
                predicate=self.ABOX_NAMESPACE + 'hasPositionZ',
                obj=self.number_text(pose['z']),
                object_is_literal=True,
            ),
            self.modify_request(
                operation=ModifyWorldModel.Request.SET,
                subject=runtime_component,
                predicate=(
                    self.ABOX_NAMESPACE + 'hasOrientationYaw'
                ),
                obj=self.number_text(pose['yaw']),
                object_is_literal=True,
            ),
            self.modify_request(
                operation=ModifyWorldModel.Request.SET,
                subject=runtime_component,
                predicate=(
                    self.ABOX_NAMESPACE + 'hasReferenceFrame'
                ),
                obj=pose['frame'],
                object_is_literal=True,
                save_to_file=not (state.task_id == 'Task_001'
                                  and state.resource_id == 'Crane_01' and not self.gazebo_physics),
            ),
        ]

        self.pending_persistence.add(persistence_key)
        self.execute_modify_sequence(
            operations,
            persistence_key,
            state,
            context,
        )

    @classmethod
    def runtime_component_uri(cls, task_id):
        safe_task_id = re.sub(
            r'[^A-Za-z0-9_]',
            '_',
            task_id,
        )
        return (
            cls.ABOX_NAMESPACE
            + 'RuntimeComponent_'
            + safe_task_id
        )

    @staticmethod
    def number_text(value):
        return format(float(value), '.15g')

    @staticmethod
    def modify_request(
        operation,
        subject,
        predicate,
        obj,
        object_is_literal,
        save_to_file=False,
    ):
        request = ModifyWorldModel.Request()
        request.operation = operation
        request.subject = subject
        request.predicate = predicate
        request.object = obj
        request.object_is_literal = object_is_literal
        request.save_to_file = save_to_file
        return request

    def execute_modify_sequence(
        self,
        operations,
        persistence_key,
        state,
        context,
        index=0,
    ):
        if index >= len(operations):
            self.pending_persistence.discard(persistence_key)
            self.persisted_successes.add(persistence_key)
            self.get_logger().info(
                'Persisted runtime component state: '
                f'{state.task_id} -> {context["output_product"]}'
            )
            return

        future = self.modify_client.call_async(operations[index])
        future.add_done_callback(
            lambda completed: self.modify_step_completed(
                completed,
                operations,
                persistence_key,
                state,
                context,
                index,
            )
        )

    def modify_step_completed(
        self,
        future,
        operations,
        persistence_key,
        state,
        context,
        index,
    ):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)

            self.execute_modify_sequence(
                operations,
                persistence_key,
                state,
                context,
                index + 1,
            )

        except Exception as error:
            self.pending_persistence.discard(persistence_key)
            self.get_logger().error(
                'Failed to persist runtime component state for '
                f'{state.required_capability_id}: {error}'
            )

    @staticmethod
    def attachment_pose(resource_id, product_kind):
        kind = product_kind.lower()
        if resource_id == 'Crane_01':
            half_height = 0.11 if 'rebar' in kind else 0.06
            return {'frame': 'crane_hook_tip_link', 'x': 0.0, 'y': 0.0, 'z': -0.1 - half_height, 'yaw': 0.0}
        return None

    def publish_component_marker(self, state, context, pose):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = pose['frame']
        marker.ns = 'cask_components_' + state.task_id
        marker.id = zlib.crc32(
            context['product_kind'].encode('utf-8')
        ) & 0x7fffffff
        marker.action = Marker.ADD
        marker.frame_locked = pose['frame'] != 'world'

        self.apply_marker_style(marker, context['product_kind'])

        marker.pose.position.x = pose['x']
        marker.pose.position.y = pose['y']
        marker.pose.position.z = pose['z']

        half_yaw = pose['yaw'] / 2.0
        marker.pose.orientation.z = math.sin(half_yaw)
        marker.pose.orientation.w = math.cos(half_yaw)

        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_publisher.publish(marker)
        self.marker_cache[(marker.ns, marker.id)] = marker

    def republish_cached_markers(self):
        if not self.marker_cache:
            return

        current_stamp = self.get_clock().now().to_msg()

        for marker in self.marker_cache.values():
            marker.header.stamp = current_stamp
            self.marker_publisher.publish(marker)

    @staticmethod
    def apply_marker_style(marker, product_kind):
        kind = product_kind.lower()

        if 'roofcomponent' in kind:
            marker.type = Marker.CUBE
            marker.scale.x = 1.20
            marker.scale.y = 0.60
            marker.scale.z = 0.12
            marker.color.r = 0.15
            marker.color.g = 0.45
            marker.color.b = 0.80
            return

        if 'rebar' in kind:
            marker.type = Marker.CUBE
            marker.scale.x = 1.60
            marker.scale.y = 0.22
            marker.scale.z = 0.22
            marker.color.r = 0.72
            marker.color.g = 0.25
            marker.color.b = 0.08
            return

        if 'soil' in kind:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.80
            marker.scale.y = 0.80
            marker.scale.z = 0.45
            marker.color.r = 0.38
            marker.color.g = 0.22
            marker.color.b = 0.08
            return

        marker.type = Marker.CUBE
        marker.scale.x = 0.80
        marker.scale.y = 0.50
        marker.scale.z = 0.30
        marker.color.r = 0.65
        marker.color.g = 0.65
        marker.color.b = 0.65

    def initialize_task005_scene(self):
        if self.task005_spawned or self.task005_initializing or not self.query_client.service_is_ready():
            return
        self.task005_initializing = True
        request = QueryOntology.Request()
        request.query = '''
PREFIX abox: <http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#>
PREFIX vdi: <http://www.w3id.org/hsu-aut/VDI3682#>
PREFIX din: <http://www.w3id.org/hsu-aut/DINEN61360#>
SELECT DISTINCT ?ix ?iy ?iz ?iyaw ?ox ?oy ?oz ?oyaw ?mass WHERE {
    abox:RequiredLifting_005 vdi:hasInput ?input ; vdi:hasOutput ?output .
    ?input abox:hasReferenceFrame "world" ; abox:hasPositionX ?ix ; abox:hasPositionY ?iy ;
           abox:hasPositionZ ?iz ; abox:hasOrientationYaw ?iyaw ; din:has_Data_Element ?payload .
    ?payload din:has_Type_Description abox:Payload_TD ; din:has_Instance_Description ?description .
    ?description din:Value ?mass .
    ?output abox:hasReferenceFrame "world" ; abox:hasPositionX ?ox ; abox:hasPositionY ?oy ;
            abox:hasPositionZ ?oz ; abox:hasOrientationYaw ?oyaw .
}'''
        future = self.query_client.call_async(request)
        def completed(done):
            try:
                response = done.result()
                if not response.success:
                    raise RuntimeError(response.message)
                rows = json.loads(response.result_json)
                if not rows:
                    # Optional task: an old ABox never blocks the existing scene.
                    self.task005_initializing = False
                    return
                if len(rows) != 1:
                    raise ValueError('Task005 scene pose/payload is ambiguous')
                row = {k:float(v) for k,v in rows[0].items()}
                if not all(math.isfinite(v) for v in row.values()) or row['mass'] <= 0 or row['oz'] <= .15:
                    raise ValueError('Invalid Task005 scene parameters')
                self.task005_support_top = row['oz']-.15
                self.task005_support_xy = (row['ox'],row['oy'])
                threading.Thread(target=self.spawn_task005_scene,args=(row,),daemon=True).start()
            except Exception as error:
                self.task005_timer.cancel()
                self.get_logger().error('Task005 scene initialization failed: '+str(error))
        future.add_done_callback(completed)

    @staticmethod
    def task005_box_sdf(name, size, mass=None):
        root = ET.Element('sdf',version='1.7')
        model = ET.SubElement(root,'model',name=name)
        ET.SubElement(model,'static').text = 'true' if mass is None else 'false'
        link = ET.SubElement(model,'link',name='cargo_link' if mass is not None else 'support_link')
        if mass is not None:
            inertia = ET.SubElement(link,'inertial')
            ET.SubElement(inertia,'mass').text = str(mass)
            matrix = ET.SubElement(inertia,'inertia')
            x,y,z = size
            for key,val in (('ixx',mass*(y*y+z*z)/12),('iyy',mass*(x*x+z*z)/12),
                            ('izz',mass*(x*x+y*y)/12),('ixy',0),('ixz',0),('iyz',0)):
                ET.SubElement(matrix,key).text = str(val)
        for kind in ('collision','visual'):
            part = ET.SubElement(link,kind,name=kind)
            box = ET.SubElement(ET.SubElement(part,'geometry'),'box')
            ET.SubElement(box,'size').text = ' '.join(map(str,size))
            if kind == 'visual':
                material = ET.SubElement(part,'material')
                color = '.55 .58 .62 1' if mass is not None else '.15 .18 .21 1'
                ET.SubElement(material,'ambient').text = color
                ET.SubElement(material,'diffuse').text = color
        if mass is not None:
            # Visible lifting sling, part of the rigid simulated component assembly.
            for index,(sx,sy) in enumerate(((-.25,-.18),(-.25,.18),(.25,-.18),(.25,.18))):
                length = math.sqrt(sx*sx+sy*sy+.45**2)
                visual = ET.SubElement(link,'visual',name='sling_'+str(index))
                ET.SubElement(visual,'pose').text = f'{sx/2} {sy/2} .375 0 {math.acos(.45/length)} {math.atan2(-sy,-sx)}'
                cylinder = ET.SubElement(ET.SubElement(visual,'geometry'),'cylinder')
                ET.SubElement(cylinder,'radius').text = '.008'
                ET.SubElement(cylinder,'length').text = str(length)
                material = ET.SubElement(visual,'material')
                ET.SubElement(material,'ambient').text = '.9 .55 .06 1'
                ET.SubElement(material,'diffuse').text = '.9 .55 .06 1'
            plugin = ET.SubElement(model,'plugin',filename='ignition-gazebo-pose-publisher-system',
                                   name='ignition::gazebo::systems::PosePublisher')
            for key,value in {'publish_model_pose':'true','publish_link_pose':'false',
                    'publish_nested_model_pose':'true','use_pose_vector_msg':'true','update_frequency':'30'}.items():
                ET.SubElement(plugin,key).text = value
        return ET.tostring(root,encoding='unicode')

    def spawn_task005_scene(self,row):
        try:
            height = row['oz']-.15
            models = [('task005_work_support',(.8,.5,height),None,
                       (row['ox'],row['oy'],height/2,row['oyaw'])),
                      ('task005_steel',(.8,.5,.3),row['mass'],
                       (row['ix'],row['iy'],row['iz'],row['iyaw']))]
            for name,size,mass,pose in models:
                result = subprocess.run(['ros2','run','ros_gz_sim','create','-world','empty',
                    '-name',name,'-allow_renaming','false','-string',self.task005_box_sdf(name,size,mass),
                    '-x',str(pose[0]),'-y',str(pose[1]),'-z',str(pose[2]),'-Y',str(pose[3])],
                    capture_output=True,text=True,timeout=30)
                if result.returncode or 'OK creation of entity' not in result.stdout+result.stderr:
                    raise RuntimeError(name+': '+result.stdout+result.stderr)
            self.task005_spawned = True
            self.task005_timer.cancel()
            self.get_logger().info('Task005 physical SteelComponent and work support are ready')
        except Exception as error:
            self.task005_timer.cancel()
            self.get_logger().error('Task005 scene spawn failed: '+str(error))

    def task005_cargo_pose(self,message):
        for transform in message.transforms:
            if transform.child_frame_id.replace('::','/').split('/')[-1] != 'task005_steel':
                continue
            p,q = transform.transform.translation,transform.transform.rotation
            if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
                return
            if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .05:
                return
            self.task005_pose = dict(frame='world',x=p.x,y=p.y,z=p.z,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            self.task005_pose_time = time.monotonic()
            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = 'world'
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = p.x,p.y,p.z
            pose.pose.orientation = q
            self.task005_pose_publisher.publish(pose)
            support = self.task005_support_top if math.dist((p.x,p.y),self.task005_support_xy) < .6 else 0.
            clearance = String()
            clearance.data = json.dumps(dict(frame='world',model='task005_steel',pose=[p.x,p.y,p.z],
                bottom_offset=-.15,support_top_z=support,
                roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
                pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
                support_assumption='ground_or_fixed_work_support'))
            self.task005_clearance_publisher.publish(clearance)
            return




def main(args=None):
    rclpy.init(args=args)
    node = CaskComponentManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# CASK_TASK005_EXECUTION_V1
