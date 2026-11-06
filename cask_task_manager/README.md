# cask_task_manager

Task-level coordinator. It turns a task ID into a semantically matched and ordered set of robot skill executions.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [cask_task_manager/task_manager_node.py](cask_task_manager/task_manager_node.py) | Goal validation, World Model queries, constraint matching, candidate ranking, data-flow ordering, behavior-tree nodes, cancellation, and runtime progress. |
| [cask_task_manager/final_cooperation.py](cask_task_manager/final_cooperation.py) | Concurrent preparation and coordinated loading/transport/unloading for the final scene. |

## Planning and execution

The main implementation is [task_manager_node.py](cask_task_manager/task_manager_node.py).

### Matching and ranking

The Task Manager:

1. Retrieves a task's required capabilities and their input/output products.
2. Queries provided capabilities with compatible process-kind descriptions.
3. Resolves the resource, skill, endpoint, and message/interface metadata.
4. Filters unavailable resources and respects explicit resource assignments.
5. Compares typed product requirements with capability assurances.
6. Ranks compatible candidates by the implemented constraint-margin score, with deterministic name-based tie breaking.

Categorical constraints require matching values. Numerical constraints are compared using their modeled operators. For transport, the implementation derives a distance requirement from the current task input/output coordinates.

The score is the sum of compatible constraint margins. It is not a general optimizer for cost, energy, or task duration.

The current final-scene query explicitly limits candidates to **Crane_01, Jekko_01, and Innok223_01**. Extending the framework to another resource therefore requires reviewing this filter as well as adding ontology and execution support.

### Data-flow ordering

The method order_plan_by_data_flow builds dependencies where one required capability's output product is another's input product. A topological sort produces the execution order and rejects cycles.

This is how loading precedes transport and transport precedes unloading. RDF statement order and alphabetical task names do not define the workflow.

### Behavior-tree execution

| Node | Responsibility |
| --- | --- |
| SkillActionNode | Execute one concrete skill candidate. |
| FallbackNode | Query current candidates and try an unattempted compatible skill after a retryable failure. |
| SequenceNode | Execute required capabilities in product-data-flow order. |
| HandoffPair | Coordinate an optional provider/receiver handoff group. |
| CooperativeSequence | Overlap preparation and execution in the final three-stage scene. |

The manager tracks active robot requests, forwards cancellation, records task progress, and can mark a failed resource UNAVAILABLE in the World Model.

Automatic fallback stops when the result indicates that a payload may already be loaded. A logical candidate change must not discard physical cargo ownership.

### Concurrent preparation

[final_cooperation.py](cask_task_manager/final_cooperation.py) specializes a three-stage Lifting → Transporting → Lifting plan whose middle resource is Innok223.

It prepares the carrier while loading is being prepared/executed. It can also prepare the receiving crane or Jekko during transport.

For Jekko loading, the CARGO_READY_FOR_TRANSPORT event permits transport to overlap the loader's remaining stow operation. The loader goal remains active and is joined later. When Jekko also performs unloading, its next preparation waits for its previous stow operation to finish.

A single-operation task uses the ordinary execution sequence.

## External interface

| Interface | Type / purpose |
| --- | --- |
| /cask_task_manager/execute_task | cask_interfaces/action/ExecuteTask; accepts task_id. |
| /cask_skill_manager/execute_skill | ExecuteSkill client used for complete skills. |
| /cask_skill_manager/navigate | ExecuteSkill client used for preparation. |
| /cask_world_model/query | Read requirements, candidate metadata, constraints, and runtime task state. |
| /cask_world_model/modify | Update execution-relevant semantic state. |
| /cask/skill_execution_state | Observe loading milestones for cooperative execution. |

Task feedback contains progress, current_skill, and status. The result contains success and message.

Only one task goal is accepted at a time. Invalid identifiers, an already active task, or undrained previous handoff operations can cause rejection.

## Runtime state and recovery

The manager can identify completed required capabilities from modeled current state when a task is submitted. This is conditional resume logic, not physical scene restoration. Suspended handoff state is checked to prevent reusing an earlier session's partial transfer.

A retryable skill failure triggers a fresh candidate query. Explicit resource assignments and the current three-resource filter can leave no alternative candidate. Failures with an already loaded payload stop automatic fallback.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_task_manager/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_task_manager/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| setup.cfg | Installation layout for ROS executable scripts. |
| test/ | Existing copyright/style/docstring checks; these are separate from robot simulation validation. |
| cask_task_manager/__init__.py | Python package initializer. |

---

[Project overview](../../README.md)
