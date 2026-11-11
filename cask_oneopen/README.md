# cask_oneopen

Integrated launch package for the final crane/Jekko/Innok223 scenario. It connects scene configuration, models, simulator startup, bridges, managers, and robot executors.


## Construction site initialization

The launch obtains the installed [scene JSON](../cask_component_manager/config/final_scene.json) from `cask_component_manager` and passes it to the component and execution processes. `component_manager_node` constructs `FinalScene`, whose enabled reference-site branch calls `roof_site.py` to create the roof, receiving building, and site models. Their meshes are installed under `share/cask_component_manager/assets/roof_site/`.

The launch entry point is `cask_gazebo_final_oneopen.launch.py`. No separate site launch or OBJ import command is required. `setup.py` in the Component Manager installs the assets, so source files alone are insufficient if the installed overlay is stale. Rebuild and source the workspace before launching.

The roof retains the model name `final_steel`, which explains the bridge and attachment topic names. The active physical destination is the Neuss lower building even though its model is named `final_workbench`.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [launch/cask_gazebo_final_oneopen.launch.py](launch/cask_gazebo_final_oneopen.launch.py) | Main final-scene launch, model preparation, bridges, process ordering, and runtime node parameters. |
| [launch/innok223_model_test.launch.py](launch/innok223_model_test.launch.py) | Innok223 model-test launch and reusable URDF-to-Gazebo model support. |

## Launch sequence

The main entry point is [cask_gazebo_final_oneopen.launch.py](launch/cask_gazebo_final_oneopen.launch.py).

It:

1. Resolves the workspace and scene configuration.
2. Converts/prepares robot models and selects the final Jekko hoist controller.
3. Creates a temporary Fortress world with required systems.
4. Starts Gazebo and waits for its creation service.
5. Starts initial bridge/feedback/World Model/component processes.
6. Spawns the three robots in order.
7. Starts remaining servers, managers, state publishers, and world transforms.

The helper [innok223_model_test.launch.py](launch/innok223_model_test.launch.py) contains model-conversion support reused by the final launch.

### Bridge and feedback relay

ros_gz_bridge connects ROS 2 to Ignition transport for commands, odometry, joint measurements, model poses, attachment states, and clock messages.

[gazebo_feedback_relay.py](../cask_gazebo/cask_gazebo_motion/cask_gazebo_motion/gazebo_feedback_relay.py) relays raw bridge clock/joint messages to their public ROS topics. It gates forwarding by measured simulation timestamps at a default 200 Hz, preserving source messages and timestamps. It does not synthesize samples or change the physics time step.

CASK_GAZEBO_FEEDBACK_HZ accepts values from 200 to 1000. Actual forwarded data still depend on incoming simulator measurements.

## Workspace and startup assumptions

CASK_WORKSPACE may specify the workspace path, but the final directory name must be cask_gazebo_final_ws. The launch validates the template at src/ontology/CASK+CRO-abox.ttl.

Scene configuration is loaded from the installed cask_component_manager share directory. The launch sets world_state_mode=reset, enables the final scene and final task cooperation, and disables legacy component spawning.

Gazebo startup waits for /world/empty/create before model creation. Robots are created in the configured order. Initialization failures stop the startup sequence. Temporary generated model/world files are removed on shutdown.

## Start and check readiness

~~~bash
cd ~/cask_gazebo_final_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export CASK_WORKSPACE="$HOME/cask_gazebo_final_ws"

ros2 launch cask_oneopen cask_gazebo_final_oneopen.launch.py
~~~

Wait for FINAL_SCENE_READY. From another sourced terminal:

~~~bash
ros2 topic echo /cask/final/scene_status --once
ros2 topic echo /cask/final/cargo_pose --once
~~~

The scene must report READY and provide fresh measured payload feedback before task submission.

## Run and time a task

~~~bash
source /opt/ros/humble/setup.bash
source ~/cask_gazebo_final_ws/install/local_setup.bash

/usr/bin/time -f 'Elapsed wall time: %E' \
  ros2 action send_goal --feedback \
  /cask_task_manager/execute_task \
  cask_interfaces/action/ExecuteTask \
  '{task_id: "Task_01"}'
~~~

Replace Task_01 with the desired final task ID. The GNU time wrapper measures wall-clock command duration, separate from Gazebo simulation time. It does not imply successful execution; inspect the action result.

Relaunch between independent trials to restore the initial physical scene and reset-mode semantic state.

## Diagnostics

| Symptom | Check |
| --- | --- |
| Workspace/path validation failure | Workspace basename, CASK_WORKSPACE, and template ontology path. |
| Installed Fortress world missing | /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf. |
| Scene creation fails | Gazebo service readiness and robot/model creation output. |
| Scene stays unready | Scene-status output, World Model service availability, and measured final_steel pose. |
| Missing actions or feedback | Current overlay, launched processes, bridge topics, and ros2 action list -t. |

Changes to controller selection, command topics, bridges, and URDF joints must agree. In particular, the final Jekko hoist controller is selected during model preparation rather than by changing every upstream target into a velocity.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_oneopen/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_oneopen/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| setup.cfg | Installation layout for ROS executable scripts. |
| test/ | Existing copyright/style/docstring checks; these are separate from robot simulation validation. |
| cask_oneopen/__init__.py | Python package initializer. |

---

[Project overview](../README.md)

