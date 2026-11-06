"""Standalone Innok223/trailer model check in a fresh Fortress world."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from cask_handoff.world import contact_world
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WAIT_WORLD = """
import subprocess, sys, time
deadline = time.monotonic() + 60
while time.monotonic() < deadline:
    try:
        result = subprocess.run(['ign', 'service', '-l'], capture_output=True,
                                text=True, timeout=3)
        if '/world/innok223_test/create' in result.stdout.splitlines():
            sys.exit(0)
    except subprocess.TimeoutExpired:
        pass
    time.sleep(0.25)
sys.exit('Innok223 test world did not become ready')
"""


def gazebo_model(urdf_path, destination):
    """Use Fortress's URDF conversion, then set native SDF visual colours."""
    result = subprocess.run(
        ['ign', 'sdf', '-p', str(urdf_path)], capture_output=True,
        text=True, timeout=30, check=True,
    )
    start = result.stdout.find('<sdf')
    if start < 0:
        raise RuntimeError('Fortress did not return a converted Innok223 SDF: ' + result.stderr)
    sdf = ET.fromstring(result.stdout[start:])
    model = sdf.find('model')
    if model is None:
        raise RuntimeError('Converted Innok223 SDF has no model')
    hitch = model.find("joint[@name='innok223_trailer_hitch_joint']")
    if (hitch is None or hitch.get('type') != 'revolute'
            or hitch.findtext('parent') != 'innok223_base_link'
            or hitch.findtext('child') != 'innok223_trailer_link'):
        raise RuntimeError('URDF conversion lost the Innok223 trailer hitch')
    source = ET.parse(urdf_path).getroot()
    for link in source.findall('link'):
        visuals = link.findall('visual')
        if not visuals:
            continue
        target = model.find("link[@name='" + link.get('name') + "']")
        if target is None:
            raise RuntimeError('Converted SDF lost visible link: ' + link.get('name'))
        for index, visual in enumerate(visuals):
            name = visual.get('name')
            converted = target.find("visual[@name='" + name + "']")
            if converted is None:
                # libsdformat12 appends _visual and a per-link index.
                sdf_name = name + '_visual' + ('_' + str(index) if index else '')
                converted = next((v for v in target.findall('visual') if v.get('name', '').endswith(sdf_name)), None)
            if converted is None:
                raise RuntimeError('Converted SDF lost visual: ' + name)
            rgba = visual.find('material/color').get('rgba')
            for previous in list(converted.findall('material')):
                converted.remove(previous)
            material = ET.SubElement(converted, 'material')
            ET.SubElement(material, 'ambient').text = rgba
            ET.SubElement(material, 'diffuse').text = rgba
            ET.SubElement(material, 'specular').text = '0.12 0.12 0.12 1'
    ET.ElementTree(sdf).write(destination, encoding='unicode')
    return destination


def start_model(context):
    model_path = Path(get_package_share_directory('cask_robot_description')) / 'urdf' / 'innok223_01.urdf'
    model = model_path.read_text(encoding='utf-8')
    description = ET.fromstring(model)
    for element in list(description.findall('gazebo')):
        description.remove(element)

    temporary = Path(tempfile.mkdtemp(prefix='cask_innok223_'))
    simulation_model = gazebo_model(model_path, temporary / 'innok223_coloured.sdf')
    world_file = temporary / 'innok223_test.sdf'
    contact_world(world_file)
    document = ET.parse(world_file)
    world = document.getroot().find('world')
    world.set('name', 'innok223_test')
    if LaunchConfiguration('test_load').perform(context).lower() == 'true':
        # 100 kg test steel block. It rests on the deck through contact only.
        load = ET.fromstring('''<model name="innok223_test_steel">
          <pose>0 0 0 0 0 0</pose>
          <link name="steel_link">
            <inertial><mass>100</mass><inertia>
              <ixx>1.8075</ixx><iyy>12.12</iyy><izz>13.6875</izz>
              <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
            </inertia></inertial>
            <collision name="collision"><geometry>
              <box><size>1.2 0.45 0.12</size></box>
            </geometry></collision>
            <visual name="visual"><geometry>
              <box><size>1.2 0.45 0.12</size></box>
            </geometry><material><ambient>0.35 0.38 0.42 1</ambient>
              <diffuse>0.35 0.38 0.42 1</diffuse></material></visual>
          </link></model>''')
        # Spawn the load after the trailer so it cannot fall before the deck exists.
        load_path = temporary / 'test_steel.sdf'
        sdf = ET.Element('sdf', version='1.7')
        sdf.append(load)
        ET.ElementTree(sdf).write(load_path, encoding='unicode')
    else:
        load_path = None
    document.write(world_file, encoding='unicode')

    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', str(world_file)], output='screen')
    ready = ExecuteProcess(cmd=[sys.executable, '-c', WAIT_WORLD], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
        '-world', 'innok223_test', '-file', str(simulation_model), '-name', 'innok223', '-z', '0.01'])
    load_spawn = [] if load_path is None else [Node(
        package='ros_gz_sim', executable='create', output='screen', arguments=[
            '-world', 'innok223_test', '-file', str(load_path), '-name', 'innok223_test_steel',
            '-x', '-1.52', '-y', '0', '-z', '0.555'])]

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='innok223_test_bridge', output='screen', arguments=[
        '/innok223/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/innok223/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/innok223/odom_tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/innok223/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    ], remappings=[('/innok223/odom_tf', '/tf')])
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     name='innok223_state_publisher', output='screen', parameters=[{
        'use_sim_time': True, 'robot_description': ET.tostring(description, encoding='unicode'),
    }], remappings=[('joint_states', '/innok223/joint_states')])

    def after_ready(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Innok223 world readiness failed'))]
        return [spawn]

    def after_spawn(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Innok223 model creation failed'))]
        return [bridge, publisher, *load_spawn,
                LogInfo(msg='Innok223 model ready. Motion input: /innok223/cmd_vel')]

    def cleanup(_context):
        shutil.rmtree(temporary, ignore_errors=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[
            EmitEvent(event=Shutdown(reason='Innok223 test Gazebo exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        gazebo, ready,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('test_load', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start_model),
    ])
