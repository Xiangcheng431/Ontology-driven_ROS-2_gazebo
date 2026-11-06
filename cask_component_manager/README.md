# cask_component_manager

Owns component-state integration and final-scene initialization. It links measured Gazebo payload state to ROS topics and the semantic World Model.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [cask_component_manager/component_manager_node.py](cask_component_manager/component_manager_node.py) | CaskComponentManager node: execution-state observation, component context/state handling, semantic updates, marker publication, and final-scene integration. |
| [cask_component_manager/final_scene.py](cask_component_manager/final_scene.py) | Validate configuration, generate/spawn final scene objects, relay measured payload pose, write initial metadata, and report readiness. |
| [config/final_scene.json](config/final_scene.json) | Robot spawn poses, site/payload/workbench geometry, task locations, trailer geometry, and crane boom length. |

## Scene lifecycle

[component_manager_node.py](cask_component_manager/component_manager_node.py) retains component-state integration. The active final-scene implementation is [final_scene.py](cask_component_manager/final_scene.py), configured by [final_scene.json](config/final_scene.json).

The final-scene module:

1. Validates robot poses, dimensions, and payload mass.
2. Generates and spawns the steel payload, solid workbench, and visual site markings.
3. Writes scene metadata into the World Model.
4. Receives the measured payload model pose from Gazebo.
5. Publishes payload pose and clearance information.
6. Updates measured payload state in the semantic graph.
7. Reports READY only after initialization and a measured-pose write complete.

It does not implement robot driving or lifting trajectories.

### Current scene configuration

| Item | Value in final_scene.json |
| --- | --- |
| Gazebo world | empty |
| Payload model / individual | final_steel / FinalSteel_01 |
| Payload dimensions | 1.2 × 0.45 × 0.12 m |
| Payload mass | 100 kg |
| Site dimensions | 20 × 14 m |
| Workbench dimensions | 4 × 2.4 × 1 m |
| Crane boom length | 10 m |
| Trailer deck height | 0.48 m |

Scene poses use [x, y, z, yaw], in metres and radians. The JSON includes robot spawn poses, loading/unloading bays, work positions, and trailer geometry.

Changes to scene geometry must remain consistent with the ontology's product poses, reachability limits, and robot model geometry.

## Measurement and initialization flow

Gazebo model pose → /cask/final/cargo_pose and clearance data → ordered World Model updates.

The scene spawner runs in a worker. ROS measurements and World Model service callbacks remain asynchronous. Initial metadata writes are serialized with one request in flight. READY requires completed initialization and a completed measured-payload-pose write.

The Component Manager does not drive robot bases or generate lifting trajectories.

## Interfaces

| Interface | Purpose |
| --- | --- |
| /model/final_steel/pose | Bridged model-pose measurement used by the final scene. |
| /cask/final/cargo_pose | Measured payload PoseStamped in world. |
| /cask/final/cargo_clearance | Payload pose, bottom offset, support height, and orientation information. |
| /cask/final/scene_status | Initialization/error/readiness state. |
| /cask/skill_execution_state | Execution events used by component-state integration. |
| /cask_world_model/query and /modify | Component lookup and semantic state updates. |

FINAL_SCENE_READY is the launch-readiness milestone. A visible payload alone does not establish that semantic initialization and feedback are ready.

## Final-launch configuration

The final launch sets gazebo_physics=true, spawn_legacy_components=false, final_scene_enabled=true, and passes final_scene_config. Earlier scene/component helpers remain in the implementation but are distinct from the active final_steel scene.

When editing the JSON, keep scene locations, semantic product targets, robot geometry, and reachability consistent.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_component_manager/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_component_manager/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| setup.cfg | Installation layout for ROS executable scripts. |
| test/ | Existing copyright/style/docstring checks; these are separate from robot simulation validation. |
| cask_component_manager/__init__.py | Python package initializer. |

---

[Project overview](../../README.md)
