# Ontology-Driven ROS 2 Capability Matching Multi-Robot Framework for Construction Tasks

**Master's thesis — Xiangcheng Li, M.Sc. Construction Robotics, RWTH Aachen University**

This project connects construction task requirements to executable robot skills through a CaSk-based semantic World Model, ROS 2, and Gazebo. It demonstrates how a task identifier can be resolved into compatible robot capabilities, ordered through product input/output dependencies, and executed by cooperating machines with measured simulation feedback.

The construction scenario uses a tower crane, a Jekko mini crane, and an Innok223 tractor–trailer vehicle to pick up, transport, and install a Neuss roof assembly on a lower building. The simulated site contains terrain, fences, containers, and surrounding buildings.

## Research focus

The framework combines semantic task/resource representation, capability matching, skill-interface binding, coordinated execution, and runtime state updates. Its ontology uses CaSk/CSS capability–skill concepts, VDI 3682 process/product relationships, and DIN EN 61360 requirements and assurances.

The World Model exposes RDF data through SPARQL. The Task Manager matches and orders capabilities, the Skill Manager resolves action interfaces and parameters, and robot skill servers execute motion using measured Gazebo state. The implementation uses its own ROS 2 executor; it does not run a native SkiROS2 executor or PDDL planner.

## System overview

<img src="system-overview.svg" alt="System overview: semantic task matching, skill execution, world state updates and replanning" width="907">

The diagram summarizes the control flow. A task is submitted to the ExecuteTask action after the integrated launch is ready. Failure recovery depends on candidate eligibility, resource assignments, and physical payload ownership; a loaded failure is not an unrestricted automatic retry.

## Construction scene

| Physical object | Implementation |
| --- | --- |
| Roof assembly | Dynamic Neuss roof mesh, collision proxies, and simulated payload mass. |
| Lower building | Static Neuss building mesh and support collision box for placement. |
| Site | Static terrain, fence, container, and surrounding-building meshes with collision proxies. |
| Machines | Crane, Jekko, and Innok223 descriptions prepared by the integrated launch. |

The [Component Manager](cask_component_manager/README.md) loads [final_scene.json](cask_component_manager/config/final_scene.json), calls [roof_site.py](cask_component_manager/cask_component_manager/roof_site.py), and creates the Gazebo scene from the packaged [STL assets and geometry metadata](cask_component_manager/assets/roof_site/README.md).

The executable identifiers `final_steel`, `FinalSteel_01`, `cargo_link`, `final_workbench`, and `platform_link` refer to the roof and its receiving building in this scene. Names containing `Steel` or `Workbench` are also used by semantic product states, topics, and attachment interfaces. They are identifiers, not a separate steel-payload demonstration.

| Configuration | Value |
| --- | --- |
| Roof envelope | 1.58424 × 0.82110 × 0.81024 m |
| Simulated roof mass | 100 kg |
| Building support envelope | 1.61000 × 2.03665 × 1.59390 m |
| Roof start product position | (-2.2, 0.0, 0.40512) m |
| Roof final product position | (5.5, -6.1, 1.99902) m |
| Trailer deck height | 0.48 m |
| Crane boom length | 10 m |

The initial roof spawn height is 0.41012 m, providing a 0.005 m offset above the modeled start product height. Product pose heights describe the roof reference position, not the ground or support surface alone. Numerical dimensions and mass are simulation settings.

## Module documentation

| Module | Responsibility |
| --- | --- |
| [ontology](ontology/README.md) | Task/resource instances, constraints, product states, and skill interfaces. |
| [cask_world_model](cask_world_model/README.md) | RDF graph, SPARQL queries, semantic modifications, and persistence. |
| [cask_task_manager](cask_task_manager/README.md) | Capability matching, data-flow ordering, fallback, and cooperation. |
| [cask_skill_manager](cask_skill_manager/README.md) | Semantic binding, numerical parameterization, and action dispatch. |
| [cask_component_manager](cask_component_manager/README.md) | Site/roof/building creation and measured component state. |
| [cask_robot_skill_servers](cask_robot_skill_servers/README.md) | Robot execution, preparation, attachment, and feedback handling. |
| [cask_robot_description](cask_robot_description/README.md) | Robot geometry, joints, simulator configuration, and RViz resources. |
| [cask_interfaces](cask_interfaces/README.md) | Task action, ontology services, and execution-state message. |
| [ros_interfaces](ros_interfaces/README.md) | Robot-skill ExecuteSkill action. |
| [cask_oneopen](cask_oneopen/README.md) | Integrated launch, model preparation, bridges, and startup sequence. |
| [cask_gazebo](cask_gazebo/README.md) | Overview of simulation support packages. |
| [cask_gazebo_motion](cask_gazebo/cask_gazebo_motion/README.md) | Shared motion, planning, attachment, obstacles, and measured feedback. |
| [cask_handoff](cask_gazebo/cask_handoff/README.md) | Optional semantic handoff protocol and Fortress world preparation. |

Each module guide explains its internal files and its connections to the framework. The asset directory also has a guide to meshes, collision geometry, and coordinate conventions.

## Five task configurations

All tasks move the same roof between the common start and receiving building. Assignments are defined in the ontology.

| Task | Loading / first operation | Transport | Final placement |
| --- | --- | --- | --- |
| Task_01 | Tower crane picks up the roof | Tower crane moves the suspended roof | Tower crane installs it on the building |
| Task_02 | Jekko loads the trailer | Innok223 | Jekko installs the roof |
| Task_03 | Tower crane loads the trailer | Innok223 | Jekko installs the roof |
| Task_04 | Jekko loads the trailer | Innok223 | Tower crane installs the roof |
| Task_05 | Tower crane loads the trailer | Innok223 | Tower crane installs the roof |

Task_01 uses one required capability. Task_02–Task_05 use loading, transport, and unloading capabilities connected by product states. Carrier preparation can overlap loading; receiving-machine preparation can overlap transport. Jekko can signal that the roof is ready for transport while its unloaded stow operation continues.

## Build and run

The target environment is Ubuntu 22.04, Python 3.10, ROS 2 Humble, and Ignition Gazebo Fortress / Gazebo 6. Dependencies include colcon, rosdep, ros_gz_sim, ros_gz_bridge, ros_gz_interfaces, RDFLib, Xacro, robot_state_publisher, and TF2.

### Source layout

This repository stores its packages directly at its root. On the execution machine, place that root at `~/cask_gazebo_final_ws/src` so the launch can find `src/ontology/CASK+CRO-abox.ttl`. The workspace directory must be named `cask_gazebo_final_ws`.

For a fresh checkout into an empty workspace:

```bash
mkdir -p ~/cask_gazebo_final_ws
git clone https://github.com/Xiangcheng431/Ontology-driven_ROS-2_gazebo.git ~/cask_gazebo_final_ws/src
```

For a downloaded archive, extract it and copy its contents into `~/cask_gazebo_final_ws/src/`. Do not place another repository-named directory between `src` and the packages.

### Build

With ROS 2, Fortress, colcon, and initialized rosdep available:

```bash
cd ~/cask_gazebo_final_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
source install/local_setup.bash
```

The Component Manager installs both `config/` and `assets/roof_site/`. Rebuild the package after changing these resources so the installed scene matches the source.

### Terminal 1: launch the scene

```bash
cd ~/cask_gazebo_final_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export CASK_WORKSPACE="$HOME/cask_gazebo_final_ws"
ros2 launch cask_oneopen cask_gazebo_final_oneopen.launch.py
```

Wait for `FINAL_SCENE_READY` before submitting a task. The launch selects reset mode for the World Model and initializes the physical scene.

### Terminal 2: execute a task

```bash
source /opt/ros/humble/setup.bash
source ~/cask_gazebo_final_ws/install/local_setup.bash
ros2 action send_goal --feedback \
  /cask_task_manager/execute_task \
  cask_interfaces/action/ExecuteTask \
  '{task_id: "Task_01"}'
```

Use `Task_02`, `Task_03`, `Task_04`, or `Task_05` for another configuration. Submit one task at a time. Relaunch between independent trials to restore the starting physical and semantic state.

To measure wall-clock command duration, prefix the action command with `/usr/bin/time -f 'Elapsed wall time: %E'`. This includes command startup and waiting until it exits; check the action result separately. See the [launch guide](cask_oneopen/README.md) for the complete timed command and diagnostics.

## Configuration and execution boundaries

Keep [ontology product poses](ontology/CASK+CRO-abox.ttl), [scene geometry](cask_component_manager/config/final_scene.json), and [mesh/collision metadata](cask_component_manager/assets/roof_site/geometry.json) consistent. The Skill Manager reads semantic poses and robot-specific rigging gaps; the Jekko execution adapter applies the configured roof clearance during unloaded stowing.

The semantic matcher currently considers Crane_01, Jekko_01, and Innok223_01. Additional robot types need corresponding ontology, matching, and execution support. Runtime state persistence does not restore an interrupted physical simulation by itself. Static Gazebo collision geometry and the shared planner's obstacle state are separate representations; adding a mesh does not automatically update every planner obstacle.

Source-side tests and package lint scaffolding are included where provided. Documentation and archive checks are not a ROS 2 build or a Gazebo execution test. Hardware deployment and arbitrary construction-task coverage are outside the demonstrated scope.

## Repository contents

Commit source, configuration, model assets, interfaces, and documentation. Build products, installed overlays, logs, runtime snapshots, Python caches, and machine-specific generated interface files do not belong in the source repository. Package-level license declarations are preserved; this document does not assign a repository-wide license.

**Author:** Xiangcheng Li  
**Institution:** RWTH Aachen University
