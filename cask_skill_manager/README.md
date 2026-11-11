# cask_skill_manager

Bridge between semantic skill descriptions and executable robot actions. Robot selection happens upstream in the Task Manager; this package binds and dispatches the selected skill.


## Roof-task parameterization

The payload is the Neuss roof assembly. `final_parameters.py` reads complete world-frame product input/output poses from the ontology and combines them with the configured roof dimensions and robot work positions.

`cargo.rigging_gap.jekko` and `cargo.rigging_gap.crane` in [final_scene.json](../cask_component_manager/config/final_scene.json) supply the hook-to-payload spacing. Their values are 0.44512 m and 0.52512 m. If an override is absent, the parameterizer uses its built-in geometry-based/default value. This parameter does not change which robot is selected.

The roof product reference height is 0.40512 m at the start, 0.88512 m on the trailer, and 1.99902 m at the building. `target_height` comes from the output product's z coordinate. Jekko work positions are selected by spatial proximity to the configured start/end locations. The same skill action carries these numerical parameters to the final robot adapters.

## Responsibilities and parameter binding

The main implementation is [skill_manager_node.py](cask_skill_manager/skill_manager_node.py).

Its responsibilities are to:

- Discover skill/interface metadata through the World Model.
- Validate the required-capability/provided-capability/resource/skill binding.
- Resolve the selected skill to its ROS 2 action endpoint.
- Derive numerical parameters when the request does not supply them explicitly.
- Forward feedback, results, and cancellation.
- Publish execution-state events for observers and the Component Manager.

**Robot selection occurs in the Task Manager.** Parameter derivation in the Skill Manager adapts the chosen resource's semantic input to its controller interface.

| File | Role |
| --- | --- |
| skill_manager_node.py | Discovery, binding validation, action forwarding, parameter dispatch, and execution-state publication. |
| [final_parameters.py](cask_skill_manager/final_parameters.py) | Read unique world-frame input/output product poses and form pickup, delivery, cargo, height, and robot work-position parameters. |
| [execution_outcome.py](cask_skill_manager/execution_outcome.py) | Preserve loaded-failure and cargo-on-carrier information in execution-state events. |

Final-scene parameterization reads fresh product coordinates from the ontology. It also uses configured scene geometry, such as cargo dimensions and Jekko work positions. The current implementation therefore depends on both semantic task data and scene configuration.

The navigation/preparation action reuses semantic binding and parameterization without reporting that a complete required capability has finished.

## Internal workflow

1. Discover skill/interface metadata from the World Model.
2. Receive an ExecuteSkill request with execution, task, capability, and skill IDs.
3. Validate the semantic binding when parameters must be inferred.
4. Obtain numerical parameters from the relevant parameterizer.
5. Resolve the robot action endpoint and forward the goal.
6. Forward progress and cancellation while tracking the corresponding robot goal.
7. Return the robot result and publish execution-state events.

Explicit parameter arrays must have equal nonzero lengths. In automatic final-scene binding, a unique input and output product must each have a complete finite world-frame pose.

## Interfaces

| Endpoint / topic | Role |
| --- | --- |
| /cask_skill_manager/execute_skill | Full skill action server using ros_interfaces/action/ExecuteSkill. |
| /cask_skill_manager/navigate | Preparation/navigation action using the same action type. |
| /cask_world_model/query | Semantic query client. |
| /cask/skill_execution_state | cask_interfaces/msg/SkillExecutionState publisher. |

Preparation uses the same binding logic but does not emit completion of the full required capability.

## Parameters and geometry

| Parameter | Meaning |
| --- | --- |
| final_scene_config | Final scene JSON used for geometry-dependent parameterization. |
| physical_cargo_mode | Enables physical payload-related parameter handling. |
| crane_world_x / crane_world_y / crane_world_yaw | Fixed crane pose used for world/local coordinate conversion. |

The final parameterizer generates pickup/delivery and cargo poses, target height, rigging gap, and Jekko work-position parameters. It chooses the configured Jekko work side by spatial proximity rather than by a task-number switch.

execution_outcome.py preserves cargo_on_carrier=true on success and identifies carrying_phase=true failures as FAILED_WITH_LOAD. Consumers use this information to avoid treating physical cargo state as a simple retryable failure.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_skill_manager/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_skill_manager/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| setup.cfg | Installation layout for ROS executable scripts. |
| test/ | Existing copyright/style/docstring checks; these are separate from robot simulation validation. |
| cask_skill_manager/__init__.py | Python package initializer. |

---

[Project overview](../README.md)

