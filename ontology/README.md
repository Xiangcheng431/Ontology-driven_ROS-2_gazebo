# Ontology and scenario data

Semantic input for capability matching, skill binding, and task data flow. This is a data directory rather than a ROS 2 package.

## Main files

| File | Role |
| --- | --- |
| [CASK+CRO-abox.ttl](CASK+CRO-abox.ttl) | Active final-launch template containing task/resource instances, capabilities, constraints, product states, and skill interfaces. |
| [catalog-v001.xml](catalog-v001.xml) | Ontology-editor catalog metadata. |
| [cask-scene-abox.ttl](cask-scene-abox.ttl) and [cask-scene-abox.report.json](cask-scene-abox.report.json) | Additional retained scene data/report; these are not selected as the main template by the final launch. |

Other process/import resources are retained in this folder but are outside this documentation's execution walkthrough.

## Modeling roles

| Concept / vocabulary | Role |
| --- | --- |
| CaSk RequiredCapability / ProvidedCapability | Describe requested operations and resource offerings. |
| CSS skill/resource relationships | Connect capabilities to executable skills and resources. |
| VDI 3682 hasInput / hasOutput | Link operations through product states. |
| VDI 3682 isAssignedTo | Constrain an operation to a resource. |
| DIN EN 61360 descriptions | Represent typed requirements/assurances, values, and comparison operators. |
| ROS interface metadata | Store endpoint, interface kind, and message type for executable skills. |

The runtime queries RDF using SPARQL. It does not invoke an OWL reasoner to choose robots.

## Final task assignments

| Task | Loading / first operation | Transport | Unloading / placement |
| --- | --- | --- | --- |
| Task_01 | Crane | Suspended transfer by crane | Crane |
| Task_02 | Jekko | Innok223 | Jekko |
| Task_03 | Crane | Innok223 | Jekko |
| Task_04 | Jekko | Innok223 | Crane |
| Task_05 | Crane | Innok223 | Crane |

The final tasks link these common product states:

- FinalScenarioSteelStart
- FinalScenarioSteelOnTrailerStart
- FinalScenarioSteelOnTrailerEnd
- FinalScenarioSteelOnWorkbench

Task_01 directly connects the start/workbench states. The other final tasks use loading, transport, and unloading required capabilities whose output/input products form a chain.

Ordering comes from these relationships, not RDF statement order. The Task Manager rejects cyclic dependencies.

## Who reads and writes the data?

| Consumer | Use |
| --- | --- |
| World Model | Load the Turtle graph and expose query/update services. |
| Task Manager | Read task requirements, candidate constraints/assignments, and skill metadata; update runtime eligibility/progress where implemented. |
| Skill Manager | Validate bindings and read numerical product-state targets. |
| Component Manager | Add/update scene metadata and measured component pose/state. |

The final launch reads this template in reset mode. Runtime serialization uses runtime/CASK+CRO-runtime.ttl outside this folder. Persisted graph state is separate from physical simulation state.

## Editing and extending

Keep product identities, capability input/output links, assignment resources, coordinate frames, pose values, and execution endpoint metadata consistent.

Numerical final-scene poses must be complete, finite, and world-framed for the active parameterizer. Scene geometry is also configured in final_scene.json; an ontology edit alone does not move every physical scene object.

The current Task Manager limits candidates to Crane_01, Jekko_01, and Innok223_01. New ontology resources need compatible execution support and a matching-query update.

Earlier Task_001-style individuals and other robots remain in the file. They are distinct from final Task_01–Task_05 and are not all launched by the final scenario.

## Related documentation

- [World Model](../cask_world_model/README.md)
- [Task Manager](../cask_task_manager/README.md)
- [Skill Manager](../cask_skill_manager/README.md)
- [Scene configuration](../cask_component_manager/README.md)

---

[Project overview](../../README.md)
