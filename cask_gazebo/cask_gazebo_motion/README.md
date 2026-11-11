# cask_gazebo_motion

Shared Gazebo execution utilities used by robot skill servers. This package contains the motion/planning implementations and the obstacle/feedback processes; it does not select semantic task candidates.


## Motion around the construction scene

Robot servers use these utilities to execute roof pickup, transport, and placement using measured robot and payload state. `final_steel` is the roof payload model. The Component Manager supplies the physical site, roof, and building; the final lifting adapters supply scene-specific carrier gates and unloaded roof clearance.

[geometry.json](../../cask_component_manager/assets/roof_site/geometry.json) defines static Gazebo collision proxies. Planner obstacles are distributed separately through `/cask/obstacles`. Physical collision geometry does not by itself imply that every mesh is represented in every navigation planner; keep both representations consistent when extending the scene.

## Motion modules

Shared modules are under [cask_gazebo_motion](cask_gazebo_motion).

### Ground navigation

The planners calculate reference routes; controllers then follow those routes using measured poses. Planning states do not overwrite Gazebo's measured robot state.

| File | Responsibility |
| --- | --- |
| [ground_navigation.py](cask_gazebo_motion/ground_navigation.py) | Vehicle footprint planning, approach-pose conversion, and measured-pose navigation control. |
| [physical_path_planner.py](cask_gazebo_motion/physical_path_planner.py) | Orientation-aware planar A* search, sampled collision checks, path smoothing, and short-horizon command checks. |
| [jekko_navigation.py](cask_gazebo_motion/jekko_navigation.py) | Jekko-specific measured-pose approach control. |
| [trailer_navigation.py](cask_gazebo_motion/trailer_navigation.py) | Articulated tractor/trailer state, bed-pose calculation, forward/reverse lattice planning, and collision checks. |
| [trailer_maneuver.py](cask_gazebo_motion/trailer_maneuver.py) | Fit curved approaches outside the bay, followed by straight terminal entry; use planned reverse segments when needed. |
| [innok223_steering.py](cask_gazebo_motion/innok223_steering.py) | Convert requested movement to front steering and velocity consistent with measured steering; limit steering and yaw rate. |
| [task005_route_planner.py](cask_gazebo_motion/task005_route_planner.py) | Shared route/clearance checks for the retained chair-model motion implementation. |
| [local_path_planner.py](cask_gazebo_motion/local_path_planner.py) | Simpler grid planner retained for kinematic visualization; it does not model full steering dynamics. |
| [task005_geometry.py](cask_gazebo_motion/task005_geometry.py) | Angle normalization, quaternion rotation, and oriented-rectangle overlap tests. |

The physical planar planner searches position and heading with eight discrete directions. It checks robot footprints and turns instead of treating each vehicle as a point.

The Innok223 planner represents tractor position, tractor heading, and trailer heading. Its route search includes forward and reverse motion, curvature choices, articulation limits, and obstacle clearance. During execution, the server checks measured deviation and predicted collisions and replans when necessary.

### Lifting and actuator control

| File | Responsibility |
| --- | --- |
| [crane_motion.py](cask_gazebo_motion/crane_motion.py) | Crane target geometry, phased movement, feedback checks, pickup clearance, attachment, and placement. |
| [crane_servo.py](cask_gazebo_motion/crane_servo.py) | Condition crane angular/trolley commands while leaving task targets and completion rules upstream. |
| [jekko_motion.py](cask_gazebo_motion/jekko_motion.py) | Jekko navigation, supports, arm/hoist sequencing, payload manipulation, and measured completion. |
| [jekko_geometry.py](cask_gazebo_motion/jekko_geometry.py) | Geometric inverse/forward calculations for slew, boom pitch, telescoping, hoist, and hook pose; reject unreachable targets. |
| [jekko_hoist_servo.py](cask_gazebo_motion/jekko_hoist_servo.py) | Convert hoist position error into bounded measured-feedback velocity control. |
| [jekko_vertical_motion.py](cask_gazebo_motion/jekko_vertical_motion.py) | Bounded-lead vertical hoist motion. |
| [jekko_loaded_reference.py](cask_gazebo_motion/jekko_loaded_reference.py) | Shape loaded-motion references with braking-distance-aware arrival. |
| [gazebo_motion.py](cask_gazebo_motion/gazebo_motion.py) | Shared motion locks, finite-target validation, cancellation, fresh-feedback checks, timeouts, and measured-position completion. |
| [delivery_hold.py](cask_gazebo_motion/delivery_hold.py) | Hold the measured payload near the requested delivery pose for a continuous duration in simulation time. |
| [motion_speed.py](cask_gazebo_motion/motion_speed.py) | Shared ground/crane speed and timing factors. |
| [gazebo_joint_commands.py](cask_gazebo_motion/gazebo_joint_commands.py) | Avoid resending identical scalar commands to controllers that retain their last target. |
| [jekko_visuals.py](cask_gazebo_motion/jekko_visuals.py) | Cable visualization only; it does not command joints, driving, or payload motion. |

The shared speed factors are 2.0 for the existing ground profiles and 1.5 for crane profiles. The final Jekko adapter additionally applies 0.875 to its inherited motion scale. These factors belong to particular motion implementations; they should not be interpreted as one global simulator-speed setting.

### Obstacles and additional support

| File | Responsibility |
| --- | --- |
| [obstacle_manager_node.py](cask_gazebo_motion/obstacle_manager_node.py) | Receive obstacle updates, publish versioned shared snapshots and RViz markers, and manage physical obstacle models when enabled. |
| [generated_cargo.py](cask_gazebo_motion/generated_cargo.py) | Bind existing controllers to physical payload contexts, topics, and attachments. |
| [generated_component_motion.py](cask_gazebo_motion/generated_component_motion.py) | Optional generated physical component spawning and measured-pose relay. |
| [task005_simulation.py](cask_gazebo_motion/task005_simulation.py) | Retained helper for DART solver configuration; it is not the final launch entry point. |

## Attachment and feedback

| File | Responsibility |
| --- | --- |
| [cask_gazebo_motion/gazebo_attachment.py](cask_gazebo_motion/gazebo_attachment.py) | Configure/use native DetachableJoint attachments and observe attached/detached acknowledgements. |
| [cask_gazebo_motion/crane_handoff.py](cask_gazebo_motion/crane_handoff.py) | Crane adapter for the optional suspended-handoff protocol. |
| [cask_gazebo_motion/gazebo_feedback_relay.py](cask_gazebo_motion/gazebo_feedback_relay.py) | Forward raw bridge clock/joint measurements to public ROS topics while preserving source data/timestamps. |

The feedback relay gates forwarding using measured simulation timestamps. Its default is 200 Hz; CASK_GAZEBO_FEEDBACK_HZ accepts 200–1000. It cannot create samples missing from the simulator and does not change the physics time step.

Native attachment requests require measured native-state acknowledgement. Queueing a service request is not proof that a joint has attached or detached.

## Runtime executables

| Executable | Role |
| --- | --- |
| obstacle_manager_node | Shared obstacle snapshots, visualization, and optional physical obstacle updates. |
| gazebo_feedback_relay | Clock/joint-state relay used by the final launch. |

Most other Python modules are imported by skill servers rather than launched as separate nodes.

## Obstacle interfaces

| Topic | Purpose |
| --- | --- |
| /cask/obstacle_updates | Incoming PoseArray snapshot in world frame. |
| /cask/obstacles | Shared versioned obstacle state. |
| /cask/obstacle_markers | MarkerArray for RViz. |

The obstacle manager validates finite coordinates and expected frame. Physical obstacle mode also creates/removes corresponding Gazebo models.

## Motion completion and failure

Reference paths and predicted poses are planning/control data. Actual task completion uses measured robot/payload state, tolerances, attachment state, and any required stable-hold duration.

The modules include checks for cancellation, non-finite targets, lost feedback, unreachable geometry, collision risk, and timeouts. These checks do not establish hardware safety certification or validate every physical contact condition.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_gazebo/cask_gazebo_motion/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_gazebo/cask_gazebo_motion/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| setup.cfg | Installation layout for ROS executable scripts. |
| cask_gazebo_motion/__init__.py | Python package initializer. |

---

[Project overview](../../README.md)

