# cask_robot_skill_servers

Robot-facing execution layer for crane lifting/transport, Jekko lifting/preparation, and Innok223 transport.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [cask_robot_skill_servers/crane_lifting_server.py](cask_robot_skill_servers/crane_lifting_server.py) | Base crane action server, parameter handling, availability/failure hooks, and dispatch into shared crane motion. |
| [cask_robot_skill_servers/jekko_skill_server.py](cask_robot_skill_servers/jekko_skill_server.py) | Base Jekko execution orchestration, goal checks, feedback, availability/failure hooks, and operation dispatch. |
| [cask_robot_skill_servers/final_lifting_servers.py](cask_robot_skill_servers/final_lifting_servers.py) | CarrierGate, FinalCraneServer, and FinalJekkoServer adapters used by the final launch. |
| [cask_robot_skill_servers/innok223_transporting_server.py](cask_robot_skill_servers/innok223_transporting_server.py) | Measured tractor–trailer preparation, route following, payload attachment, transport, and release. |
| [launch/cask_skill_servers.launch.py](launch/cask_skill_servers.launch.py) | Retained crane/Jekko server-only launch; it does not bring up the complete final scene. |
| [launch/gazebo_motion_servers.launch.py](launch/gazebo_motion_servers.launch.py) | Alias that includes the retained server-only launch. |

## Robot behavior

### Tower crane

[crane_lifting_server.py](cask_robot_skill_servers/crane_lifting_server.py) provides the base crane action server and failure-injection hooks. It uses the shared [crane_motion.py](../cask_gazebo/cask_gazebo_motion/cask_gazebo_motion/crane_motion.py) implementation.

FinalCraneServer in [final_lifting_servers.py](cask_robot_skill_servers/final_lifting_servers.py) adds the final-scene carrier gate, transport endpoint, and preparation action.

The crane coordinates tower rotation, trolley movement, hoisting, and hook yaw. It checks target range, measured joint state, payload feedback, pickup clearance, attachment state, and release conditions.

The final adapter chooses an equivalent hook-yaw angle within joint limits and waits for a measured, stopped trailer at the handover boundary before lowering onto it.

### Jekko mini crane

[jekko_skill_server.py](cask_robot_skill_servers/jekko_skill_server.py) provides base action execution, availability checks, failure injection, and operation orchestration.

FinalJekkoServer in final_lifting_servers.py adapts this behavior to the final scene. Its responsibilities include navigation/preparation, support deployment, hook positioning, controlled placement, native attachment acknowledgement, release, and stowing.

The final launch enables a native **hoist velocity controller**. A position target is converted into bounded actuator velocity using measured joint feedback.

### Innok223 tractor–trailer vehicle

[innok223_transporting_server.py](cask_robot_skill_servers/innok223_transporting_server.py) performs preparation and loaded transport.

It checks measured tractor/trailer geometry and steering feedback, plans a route to the bay, verifies payload position, attaches the payload to the trailer for transport, drives to the target, and verifies release and stability.

| Parameter | Default |
| --- | --- |
| drive_speed | 0.30 m/s |
| motion_timeout | 600 s |
| steering_limit_deg | 85° |

The controller turns while moving and does not use a stationary in-place yaw primitive.

### Executables started by the final launch

| Executable | Implementation | Node name |
| --- | --- | --- |
| final_crane_server | final_lifting_servers.py:crane_main | crane_lifting_server |
| final_jekko_server | final_lifting_servers.py:jekko_main | jekko_lifting_server |
| innok223_transporting_server | innok223_transporting_server.py:main | innok223_transporting_server |

## Action endpoints

All endpoints in this table use ros_interfaces/action/ExecuteSkill.

| Robot | Complete skill endpoints | Preparation endpoint |
| --- | --- | --- |
| Crane | /crane_01/lifting and /crane_01/transporting | /crane/navigate |
| Jekko | /jekko_01/lifting | /jekko/navigate |
| Innok223 | /innok223_01/transporting | /innok223/navigate |

The full final scene should be started through cask_oneopen. Starting only the retained base servers does not select all final adapters, models, and bridge configuration.

## Availability and failure experiments

The crane and Jekko base implementations include the following experiment controls:

| Parameter | Meaning |
| --- | --- |
| fail_at_progress | Inject a failure at a skill-progress threshold. |
| fail_before_pickup_at_progress | Apply the threshold only before pickup and before any load has been held. |
| fail_loaded_at_progress | Apply the threshold only while carrying the payload. |
| failure_task_id / failure_skill_id | Optional filters for the affected task and skill. |

A negative threshold disables injection. Thresholds use skill feedback progress, not elapsed seconds or distance percentage.

Crane availability uses lifting_available; Jekko uses available. These server parameters and semantic resource availability are separate. The Task Manager can mark a failed resource unavailable in the World Model.

Failure results report carrying_phase where applicable. Restoring a parameter alone does not reset a partially moved payload.

## Connections and boundaries

- The Skill Manager supplies semantically bound parameters and receives feedback/results.
- cask_gazebo_motion provides shared planning, actuator, attachment, and feedback utilities.
- The Component Manager publishes measured payload/clearance information.
- final_scene.json supplies scene geometry to the final adapters.

Controllers operate from measured robot and payload state. A completed service request, elapsed delay, or target command alone is not sufficient proof of completed manipulation.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_robot_skill_servers/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_robot_skill_servers/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| setup.cfg | Installation layout for ROS executable scripts. |
| test/ | Existing copyright/style/docstring checks; these are separate from robot simulation validation. |
| cask_robot_skill_servers/__init__.py | Python package initializer. |

---

[Project overview](../../README.md)
