"""Alias for the retained Gazebo-backed skill server launch."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(__file__).with_name('cask_skill_servers.launch.py'))))])
