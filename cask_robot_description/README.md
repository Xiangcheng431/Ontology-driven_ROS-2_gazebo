# cask_robot_description

Robot models and visualization assets. This package defines geometry, kinematic relationships, simulator controller configuration, and RViz resources; task logic lives in the manager/server packages.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [urdf/crane_01.urdf.xacro](urdf/crane_01.urdf.xacro) | Tower crane links/joints, geometry, and simulator systems; boom_length is supplied by the final launch. |
| [urdf/jekko_01.urdf.xacro](urdf/jekko_01.urdf.xacro) | Jekko base, supports, boom/telescopes, hoist/hook, and simulator configuration. |
| [urdf/innok223_01.urdf](urdf/innok223_01.urdf) | Research tractor–trailer model including steering and hitch geometry. |
| [urdf/jekko_01_track.obj](urdf/jekko_01_track.obj) | Jekko visual mesh asset. |
| [urdf/jekko_task005_wheel_treads_28ebab203028.obj](urdf/jekko_task005_wheel_treads_28ebab203028.obj) | Retained Jekko tread/visual mesh asset. |
| [urdf/roof_component_03.urdf.xacro](urdf/roof_component_03.urdf.xacro) | Retained component description from earlier scene work. |
| [urdf/timber_component.sdf](urdf/timber_component.sdf) | Retained timber component simulation resource. |
| [launch/display_cask_robots.launch.py](launch/display_cask_robots.launch.py) | Publish robot descriptions/transforms and open RViz; it does not publish robot commands. |
| [rviz/cask_multi_robot.rviz](rviz/cask_multi_robot.rviz) | RViz display configuration. |
| [CMakeLists.txt](CMakeLists.txt) | Install the urdf, rviz, and launch directories as package share resources. |
| [package.xml](package.xml) | Package dependencies and ament metadata. |

## How the final scene uses these files

The final cask_oneopen launch reads the robot descriptions, expands Xacro, resolves resource paths, and prepares the Gazebo model variants. For Innok223 it reuses the model-conversion helper in innok223_model_test.launch.py.

The launch also constructs descriptions for robot_state_publisher and configures world/odometry transforms. The final Jekko model selects a native hoist velocity controller during launch-time preparation.

Robot skill servers send commands through topics corresponding to the model's joints/controllers and receive measured joint/base feedback. Joint names and limits must agree across descriptions, launch configuration, and controller code.

## Display launch

After building and sourcing the workspace:

~~~bash
ros2 launch cask_robot_description display_cask_robots.launch.py
~~~

This display entry point starts descriptions and RViz. It is separate from the integrated Gazebo final-scene launch and does not execute tasks.

## Editing models

Review collision geometry and inertia as well as visual geometry. Changes to wheelbase, hitch/trailer dimensions, crane base/boom geometry, or Jekko reach can require matching changes in planning and parameterization code.

The Innok223 model was created for the research simulation. These assets do not establish manufacturer certification or compatibility with physical robot drivers.

---

[Project overview](../../README.md)
