# ros_interfaces

Defines the common ROS 2 action contract used to request an executable robot skill or preparation substep.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [action/ExecuteSkill.action](action/ExecuteSkill.action) | Skill request, execution result, and progress feedback. |
| [CMakeLists.txt](CMakeLists.txt) | Generate ExecuteSkill with rosidl_generate_interfaces. |
| [package.xml](package.xml) | ament/rosidl metadata and interface-package declaration. |

## Goal fields

| Field | Purpose |
| --- | --- |
| execution_id | Identify an execution session across managers, robot goals, and events. |
| task_id | Identify the semantic task. |
| required_capability_id | Identify the capability requested by the task. |
| provided_capability_id | Identify the selected resource's capability. |
| skill_id | Identify the executable skill. |
| parameter_names | Ordered numerical parameter names. |
| parameter_values | float64 values corresponding to the names. |
| parameter_units | Units corresponding to the names/values. |

Parameter arrays are parallel arrays and must be consistent. The Skill Manager can receive empty arrays and infer parameters from a valid semantic binding. Explicit arrays must have matching nonzero lengths.

The numerical representation is suitable for the implemented targets, poses, and control parameters. It is not a generic container for arbitrary structured robot programs.

## Result and feedback

| Part | Fields |
| --- | --- |
| Result | bool success; string message |
| Feedback | float32 progress; string status |

Consumers should distinguish the ROS action's terminal status from the custom success flag/message. Result messages can also carry information such as whether a failure occurred while carrying a payload.

## Where the action is used

- Task Manager → Skill Manager at /cask_skill_manager/execute_skill.
- Cooperative preparation → Skill Manager at /cask_skill_manager/navigate.
- Skill Manager → selected crane, Jekko, or Innok223 skill endpoint.
- Skill Manager → robot-specific preparation endpoints.

The same message type is reused across endpoints, but endpoint behavior and required parameters differ. Preparation does not mean a complete construction capability has finished.

## Inspect the installed definition

~~~bash
ros2 interface show ros_interfaces/action/ExecuteSkill
ros2 action list -t
~~~

For the task-level action and semantic services, see [cask_interfaces](../cask_interfaces/README.md).

---

[Project overview](../../README.md)
