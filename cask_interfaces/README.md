# cask_interfaces

Framework-level ROS 2 interface definitions for task execution, semantic queries/modification, and observable skill state. This package generates message/service/action types; it contains no task or robot controller node.


## Role in roof installation

A client submits Task_01–Task_05 through ExecuteTask. QueryOntology and ModifyWorldModel connect the managers to semantic roof product poses and execution state. SkillExecutionState carries the identities and milestones used by component observers and cooperative loading/transport.

All four definitions listed below are source inputs to CMake. In particular, both `.srv` files must be present when building this package; generated interfaces in an installed overlay do not replace those source definitions. Mesh geometry and scene dimensions are configured outside the message definitions.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [action/ExecuteTask.action](action/ExecuteTask.action) | Task-level goal, result, and progress feedback. |
| [srv/QueryOntology.srv](srv/QueryOntology.srv) | SPARQL query service request and JSON result response. |
| [srv/ModifyWorldModel.srv](srv/ModifyWorldModel.srv) | Semantic add/remove/set requests with object/literal and save flags. |
| [msg/SkillExecutionState.msg](msg/SkillExecutionState.msg) | Execution/task/capability/skill/resource identity and observable progress/status. |
| [CMakeLists.txt](CMakeLists.txt) | Register the four interface definitions with rosidl_generate_interfaces. |
| [package.xml](package.xml) | ament/rosidl build metadata and interface-package declaration. |

## ExecuteTask action

| Part | Fields |
| --- | --- |
| Goal | string task_id |
| Result | bool success; string message |
| Feedback | float32 progress; string current_skill; string status |

The Task Manager exposes this action at /cask_task_manager/execute_task. A client submits a semantic task identifier rather than a complete trajectory or robot command list.

## QueryOntology service

| Request | Response |
| --- | --- |
| string query | bool success; string result_json; string message |

The World Model exposes it at /cask_world_model/query. Successful row values are serialized as JSON for the requesting manager.

## ModifyWorldModel service

| Request field | Meaning |
| --- | --- |
| operation | add, remove, or set; constants ADD/REMOVE/SET are defined in the service. |
| subject / predicate / object | RDF statement terms supplied as strings. |
| object_is_literal | Distinguish a literal value from a resource object. |
| save_to_file | Request runtime graph serialization after the update. |

The response has success and message. The World Model exposes this service at /cask_world_model/modify.

## SkillExecutionState message

The message contains execution_id, task_id, required_capability_id, provided_capability_id, skill_id, resource_id, progress, and status.

The Skill Manager publishes it on /cask/skill_execution_state. It supplements action feedback so component observers and cooperative execution can associate a milestone with the correct execution and semantic binding.

Status is a string and can contain additional stage/ownership information. It is distinct from the transport-level ROS action result status.

## Build and inspect

Build interfaces with the workspace before importing them from dependent Python packages. After sourcing the overlay:

~~~bash
ros2 interface show cask_interfaces/action/ExecuteTask
ros2 interface show cask_interfaces/srv/QueryOntology
ros2 interface show cask_interfaces/srv/ModifyWorldModel
ros2 interface show cask_interfaces/msg/SkillExecutionState
~~~

Robot-skill execution uses the separate ros_interfaces/action/ExecuteSkill definition. See [ros_interfaces](../ros_interfaces/README.md).

---

[Project overview](../README.md)

