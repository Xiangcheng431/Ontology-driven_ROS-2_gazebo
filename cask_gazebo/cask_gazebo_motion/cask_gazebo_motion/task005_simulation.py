"""Gazebo-only solver configuration for the complete Task005 scene.

Keep the DART engine, physics time step, collision detector, model geometry
and robot controllers. Only select the existing DART constraint solver.
"""

import os
from pathlib import Path
import xml.etree.ElementTree as ET


def configure_task005_physics(world_file):
    choice = os.environ.get('CASK_GAZEBO_DART_SOLVER', 'pgs').strip().lower()
    if choice == 'original':
        print('[CASK Gazebo] Keeping the original world physics configuration.', flush=True)
        return world_file
    if choice not in ('pgs', 'dantzig'):
        raise ValueError('CASK_GAZEBO_DART_SOLVER must be pgs, dantzig or original')
    path = Path(world_file)
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    if world is None:
        raise RuntimeError('Generated Gazebo world has no world element')
    profiles = world.findall('physics')
    if not profiles:
        # Do not invent a new time step or other physical settings.
        raise RuntimeError('Generated Gazebo world has no physics profile')
    defaults = [profile for profile in profiles
                if profile.get('default', '').lower() in ('true', '1')]
    if len(defaults) > 1:
        raise RuntimeError('Generated Gazebo world has multiple default physics profiles')
    profile = defaults[0] if defaults else profiles[0]
    dart = profile.find('dart')
    if dart is None:
        dart = ET.SubElement(profile, 'dart')
    solver = dart.find('solver')
    if solver is None:
        solver = ET.SubElement(dart, 'solver')
    solver_type = solver.find('solver_type')
    if solver_type is None:
        solver_type = ET.SubElement(solver, 'solver_type')
    solver_type.text = choice
    tree.write(path, encoding='unicode')
    step = profile.findtext('max_step_size', 'SDF default')
    print(f'[CASK Gazebo] DART solver={choice}; unchanged physics step={step}.', flush=True)
    return world_file
