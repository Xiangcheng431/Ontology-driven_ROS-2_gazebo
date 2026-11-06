"""Display the retained robot models in RViz; no robot commands are published."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('cask_robot_description'))
    nodes = []
    for robot in ('crane', 'jekko', 'innok223'):
        path = share / 'urdf' / (robot + '_01.urdf' if robot == 'innok223' else robot + '_01.urdf.xacro')
        description = path.read_text() if robot == 'innok223' else xacro.process_file(str(path)).toxml()
        nodes.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
            namespace=robot, name='display_state_publisher', output='screen',
            parameters=[{'robot_description': description}],
            remappings=[('joint_states', '/' + robot + '/joint_states')]))
    nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d',str(share / 'rviz/cask_multi_robot.rviz')]))
    return LaunchDescription(nodes)
