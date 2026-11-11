# cask_world_model

Semantic knowledge service for task execution. This package owns the live RDF graph and exposes it to the managers and scene integration through ROS 2 services.


## Scene knowledge and physical feedback

For roof installation, the template contains the four `FinalScenarioSteel...` product states and their roof reference poses. The Component Manager creates the physical site and writes initialized scene metadata and measured roof state into this graph. Model identifiers such as `final_steel` continue to identify the roof.

This package does not load STL files or spawn the site. Its role is to expose semantic data to the Task and Skill Managers and accept component-state modifications. The active template is [CASK+CRO-abox.ttl](../ontology/CASK+CRO-abox.ttl); scene geometry is configured in [final_scene.json](../cask_component_manager/config/final_scene.json). Both must agree on product positions.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [cask_world_model/world_model_node.py](cask_world_model/world_model_node.py) | CaskWorldModel node: graph loading, SPARQL, add/remove/set operations, and optional runtime serialization. |

## Services and configuration

[world_model_node.py](cask_world_model/world_model_node.py) loads a Turtle graph and exposes:

- /cask_world_model/query — execute SPARQL and return JSON rows.
- /cask_world_model/modify — add, remove, or replace graph statements.

The graph is protected by a reentrant lock. Modification requests can optionally save the graph to the runtime ontology file.

| Parameter | Purpose |
| --- | --- |
| ontology_template_path | Initial ontology template. |
| runtime_ontology_path | Destination for persisted runtime state. |
| world_state_mode | reset loads the template; resume loads the runtime file if present, otherwise the template. |

The integrated final launch explicitly selects **reset**. Persisting a graph and restoring a physically interrupted task are separate concerns.

Capability lookup uses **SPARQL through RDFLib**. The runtime does not invoke an OWL reasoner to select robots.

## Startup and state lifecycle

1. Resolve the template/runtime paths and validate reset or resume mode.
2. Load the selected Turtle file into an RDFLib Graph.
3. Advertise query and modification services.
4. Serialize graph access with a reentrant lock.
5. Persist the graph only when an accepted modification requests save_to_file.

reset loads the template. resume loads the runtime file when available and otherwise falls back to the template. The integrated final launch selects reset.

## Query and modification behavior

QueryOntology accepts a SPARQL query string and returns rows encoded as JSON. Query errors return success=false and an explanatory message.

ModifyWorldModel provides add, remove, and set operations over RDF statements. The request distinguishes resource objects from literal objects. set replaces existing values for the requested subject/predicate. The save_to_file flag controls runtime serialization.

This package stores and exposes knowledge. It does not choose a robot, execute a behavior tree, or publish actuator commands.

## Connections

- The Task Manager queries requirements/candidates and updates resource availability.
- The Skill Manager queries skill bindings and product-state parameters.
- The Component Manager writes scene and measured component state.
- The ontology folder supplies the template; runtime/ stores optional persisted state.

Other retained import utilities are outside the scope of this guide. They are not part of the final-scene execution walkthrough.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_world_model/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_world_model/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| setup.cfg | Installation layout for ROS executable scripts. |
| test/ | Existing copyright/style/docstring checks; these are separate from robot simulation validation. |
| cask_world_model/__init__.py | Python package initializer. |

---

[Project overview](../README.md)

