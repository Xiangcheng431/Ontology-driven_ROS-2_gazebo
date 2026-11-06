"""Crane, Jekko and Innok223 bringup. Existing motion implementations are reused."""
from pathlib import Path
import importlib.util
import math
import json
import os
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from cask_handoff.world import contact_world
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node


SCENE_PATH = Path(get_package_share_directory('cask_component_manager')) / 'config/final_scene.json'
SCENE = json.loads(SCENE_PATH.read_text(encoding='utf-8'))
ROBOTS = tuple((name, *pose) for name, pose in SCENE['robots'].items())
CRANE_X, CRANE_Y, CRANE_Z, CRANE_YAW = SCENE['robots']['crane']
CRANE_BOOM_LENGTH = float(SCENE['crane_geometry']['boom_length'])
CRANE_TROLLEY_UPPER = CRANE_BOOM_LENGTH - 0.80


def configure_final_jekko_hoist(text):
    """Select exactly one native velocity controller in the final model only."""
    document = ET.fromstring(text)
    controllers = [plugin for plugin in document.findall('gazebo/plugin')
                   if plugin.findtext('joint_name')=='jekko_hoist_joint']
    if len(controllers)!=1:
        raise RuntimeError('Expected exactly one Jekko hoist controller')
    plugin = controllers[0]
    plugin.set('filename','ignition-gazebo-joint-controller-system')
    plugin.set('name','ignition::gazebo::systems::JointController')
    for child in list(plugin):
        plugin.remove(child)
    for key,value in {'joint_name':'jekko_hoist_joint','topic':'/jekko/hoist_velocity_cmd',
                      'use_force_commands':'false','initial_velocity':'0'}.items():
        ET.SubElement(plugin,key).text = value
    return ET.tostring(document,encoding='unicode')
WAIT_WORLD = """
import subprocess, time, sys
until = time.monotonic() + 90
while time.monotonic() < until:
    try:
        result = subprocess.run(['ign', 'service', '-l'], capture_output=True, text=True, timeout=3)
        if '/world/empty/create' in result.stdout.splitlines():
            sys.exit(0)
    except subprocess.TimeoutExpired:
        pass
    time.sleep(.5)
sys.exit('Gazebo world creation service did not become ready')
"""


def continue_after_success(label, following):
    def callback(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=label + ' failed; stopping startup'),
                    EmitEvent(event=Shutdown(reason=label + ' failed'))]
        return following
    return callback


def generate_launch_description():
    # Resolving __file__ also works with colcon's symlink installation.
    source_workspace = Path(__file__).resolve().parents[3]
    configured = os.environ.get('CASK_WORKSPACE')
    workspace = (Path(configured).expanduser() if configured else source_workspace
                 if source_workspace.name == 'cask_gazebo_final_ws'
                 else Path.home() / 'cask_gazebo_final_ws').resolve()
    if workspace.name != 'cask_gazebo_final_ws':
        raise RuntimeError('CASK_WORKSPACE must point to cask_gazebo_final_ws')
    ontology = workspace / 'src/ontology/CASK+CRO-abox.ttl'
    if not ontology.is_file():
        raise RuntimeError('Ontology file does not exist: ' + str(ontology))
    share = Path(get_package_share_directory('cask_robot_description'))
    temporary = Path(tempfile.mkdtemp(prefix='cask_gazebo_final_'))

    try:
        # Reuse the already corrected model conversion, including explicit colours.
        helper_path = Path(__file__).with_name('innok223_model_test.launch.py')
        spec = importlib.util.spec_from_file_location('innok223_model_support', helper_path)
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        descriptions, models = {}, {}
        for robot, *_ in ROBOTS:
            if robot == 'innok223':
                model_path = share / 'urdf/innok223_01.urdf'
                document = ET.parse(model_path).getroot()
                models[robot] = str(helper.gazebo_model(model_path, temporary / 'innok223.sdf'))
            else:
                mappings = {'boom_length': str(CRANE_BOOM_LENGTH)} if robot == 'crane' else {}
                text = xacro.process_file(str(share / 'urdf' / (robot + '_01.urdf.xacro')),
                                          mappings=mappings).toxml()
                text = text.replace('package://cask_robot_description/', str(share) + os.sep)
                if robot == 'jekko':
                    text = configure_final_jekko_hoist(text)
                models[robot] = str(temporary / (robot + '.urdf'))
                Path(models[robot]).write_text(text, encoding='utf-8')
                document = ET.fromstring(text)
            for element in list(document):
                if element.tag == 'gazebo':
                    document.remove(element)
                elif robot == 'crane' and (
                        element.tag == 'link' and element.get('name') == 'world'
                        or element.tag == 'joint' and element.get('name') == 'crane_world_fixed_joint'):
                    document.remove(element)
            descriptions[robot] = ET.tostring(document, encoding='unicode')
        world_file = contact_world(temporary / 'empty_final.sdf')
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    def node(package, executable, name, parameters=None):
        return Node(package=package, executable=executable, name=name, output='screen',
                    parameters=[{'use_sim_time': True, **(parameters or {})}])

    bridge_arguments = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/model/'+SCENE['cargo']['model']+'/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/innok223/steering_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
        '/crane/trolley_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
        '/crane/hook_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
        '/crane/hook_yaw_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
        '/crane/tower_velocity_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
        '/crane/hook_yaw_velocity_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
        '/crane/trolley_velocity_cmd@std_msgs/msg/Float64]ignition.msgs.Double',
        '/crane/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ]
    for robot in ('jekko', 'innok223'):
        bridge_arguments.extend([
            f'/{robot}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            f'/{robot}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/{robot}/odom_tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            f'/{robot}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            f'/model/{robot}/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ])
    for side in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        for operation in ('extension', 'foot'):
            bridge_arguments.append(f'/jekko/{side}_{operation}_cmd@std_msgs/msg/Float64]ignition.msgs.Double')
    for operation in ('slew', 'boom_pitch', 'telescope_1', 'telescope_2', 'hook_level', 'hoist', 'hook_yaw'):
        bridge_arguments.append(f'/jekko/{operation}_cmd@std_msgs/msg/Float64]ignition.msgs.Double')
    bridge_arguments.append('/jekko/hoist_velocity_cmd@std_msgs/msg/Float64]ignition.msgs.Double')
    # Reuse this transport publisher when the placed load is released.
    bridge_arguments.append('/cask/jekko/final_steel/detach@std_msgs/msg/Empty]ignition.msgs.Empty')
    bridge_arguments.append('/cask/innok223/final_steel/detach@std_msgs/msg/Empty]ignition.msgs.Empty')
    # Preserve the existing Crane/Jekko attachment state interfaces.
    for topic in ('/cask/crane/cargo', '/cask/crane/soil', '/cask/crane/task005', '/cask/jekko/task005',
                  '/cask/crane/final_steel', '/cask/jekko/final_steel', '/cask/innok223/final_steel'):
        bridge_arguments.append(topic + '/state@std_msgs/msg/String[ignition.msgs.StringMsg')
    remappings = [('/clock', '/cask_gazebo_motion/raw/clock')]
    remappings += [(f'/{robot}/joint_states', f'/cask_gazebo_motion/raw/{robot}/joint_states') for robot, *_ in ROBOTS]
    remappings += [(f'/{robot}/odom_tf', '/tf') for robot in ('jekko', 'innok223')]
    system_nodes = [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='final_robot_bridge',
             arguments=bridge_arguments, remappings=remappings, output='screen'),
        node('cask_gazebo_motion', 'gazebo_feedback_relay', 'gazebo_feedback_relay', {'use_sim_time': False}),
        node('cask_world_model', 'world_model_node', 'cask_world_model', {
            'ontology_template_path': str(ontology),
            'runtime_ontology_path': str(workspace / 'runtime/CASK+CRO-runtime.ttl'),
            'world_state_mode': 'reset', 'enable_ioc_import': False}),
        node('cask_component_manager', 'component_manager_node', 'cask_component_manager',
             {'gazebo_physics': True, 'spawn_legacy_components': False,
              'final_scene_enabled': True, 'final_scene_config': str(SCENE_PATH)}),
        node('cask_gazebo_motion', 'obstacle_manager_node', 'cask_obstacle_manager',
             {'obstacle_radius': 0.45, 'gazebo_physics': True}),
        node('cask_robot_skill_servers', 'final_crane_server', 'crane_lifting_server', {
            'final_scene_config': str(SCENE_PATH),
            'physical_cargo_mode': True, 'angular_velocity_control': True,
            'trolley_velocity_control': True, 'crane_world_x': CRANE_X,
            'crane_world_y': CRANE_Y, 'crane_world_yaw': CRANE_YAW,
            'trolley_joint_upper': CRANE_TROLLEY_UPPER}),
        node('cask_robot_skill_servers', 'final_jekko_server', 'jekko_lifting_server', {
            'final_scene_config': str(SCENE_PATH),
            'final_hoist_velocity_control': True,
            'crane_world_x': CRANE_X, 'crane_world_y': CRANE_Y, 'crane_world_yaw': CRANE_YAW}),
        node('cask_robot_skill_servers', 'innok223_transporting_server', 'innok223_transporting_server',
             {'final_scene_config': str(SCENE_PATH)}),
        node('cask_skill_manager', 'skill_manager_node', 'cask_skill_manager', {
            'final_scene_config': str(SCENE_PATH),
            'physical_cargo_mode': True, 'crane_world_x': CRANE_X,
            'crane_world_y': CRANE_Y, 'crane_world_yaw': CRANE_YAW}),
        node('cask_task_manager', 'task_manager_node', 'cask_task_manager', {'final_scenario_enabled': True}),
        Node(package='tf2_ros', executable='static_transform_publisher', name='crane_world_transform',
             arguments=['--x', str(CRANE_X), '--y', str(CRANE_Y), '--z', str(CRANE_Z), '--yaw', str(CRANE_YAW), '--pitch', '0',
                        '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'crane_base_link'],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]
    for robot, x, y, z, yaw in ROBOTS:
        system_nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
            name=robot + '_gazebo_state_publisher', output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': descriptions[robot]}],
            remappings=[('joint_states', '/' + robot + '/joint_states'),
                        ('robot_description', '/' + robot + '/robot_description')]))
        if robot != 'crane':
            system_nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                name=robot + '_world_odom', output='screen', parameters=[{'use_sim_time': True}],
                arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--yaw', str(yaw),
                           '--pitch', '0', '--roll', '0', '--frame-id', 'world',
                           '--child-frame-id', robot + '_odom']))
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', str(world_file)], output='screen')
    wait_world = ExecuteProcess(cmd=[sys.executable, '-c', WAIT_WORLD], output='screen')
    # Bridge/clock relay, world model and Component Manager can initialize while
    # robots are being created. Keep robot creation and skill-server order intact.
    initialization_nodes, runtime_nodes = system_nodes[:4], system_nodes[4:]
    spawns = [Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
        '-world', 'empty', '-file', models[robot], '-name', robot,
        '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)]) for robot, x, y, z, yaw in ROBOTS]

    def cleanup(context):
        shutil.rmtree(temporary, ignore_errors=True)
        return []

    handlers = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=wait_world,
            on_exit=continue_after_success('Gazebo readiness', initialization_nodes + [spawns[0]]))),
    ]
    for index, spawn in enumerate(spawns):
        following = [spawns[index+1]] if index+1 < len(spawns) else runtime_nodes + [LogInfo(
            msg='Final scene started. Task_01-Task_05 use ontology matching and concurrent preparation; wait for scene READY before sending a task.')]
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=continue_after_success(ROBOTS[index][0] + ' creation', following))))
    return LaunchDescription(handlers + [gazebo, wait_world])
