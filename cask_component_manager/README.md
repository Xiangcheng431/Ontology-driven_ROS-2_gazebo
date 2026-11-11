# cask_component_manager

Creates the construction scene and connects measured Gazebo component state to ROS 2 and the semantic World Model. The scene consists of a movable Neuss roof assembly, a receiving lower building, and a static reference site.

## Files and responsibilities

| File or directory | Responsibility |
| --- | --- |
| [component_manager_node.py](cask_component_manager/component_manager_node.py) | ROS node, execution-state observation, component context, semantic updates, markers, and final-scene integration. |
| [final_scene.py](cask_component_manager/final_scene.py) | Configuration validation, scene generation/spawning, measured roof feedback, initial semantic metadata, and readiness. |
| [roof_site.py](cask_component_manager/roof_site.py) | Generate SDF models for the roof, lower building, and reference site from packaged meshes and collision metadata. |
| [final_scene.json](config/final_scene.json) | Robot spawn poses, roof/building dimensions, locations, rigging gaps, stow clearance, trailer geometry, and scene selection. |
| [assets/roof_site](assets/roof_site/README.md) | Six STL meshes and geometry.json, including roof and background collision proxies. |
| [setup.py](setup.py) | Install Python code, scene JSON, assets, and component_manager_node. |
| [package.xml](package.xml) | ROS package dependencies and metadata. |
| resource/, setup.cfg | ament package registration and executable installation layout. |
| test/ | Package copyright/style/docstring checks. |

## Scene creation and launch

1. The integrated `cask_oneopen` launch passes the installed `config/final_scene.json` to `component_manager_node`, with `final_scene_enabled=true` and legacy component spawning disabled.
2. The node constructs `FinalScene`. Configuration loading validates robot poses, positive dimensions, and payload mass.
3. With `reference_site.enabled=true`, `final_scene.py` resolves the package asset root and calls `roof_site.scene_models()`.
4. `roof_site.py` reads `geometry.json`, references the installed STL files, and produces three SDF models.
5. `FinalScene` spawns the models into Gazebo and initializes semantic scene metadata.
6. Measured roof poses arrive through the bridge, are published as ROS feedback, and are written into the World Model.
7. Readiness requires initialization and a completed measured-pose write. Wait for `FINAL_SCENE_READY` before task submission.

Scene creation runs in a worker while ROS callbacks remain asynchronous. Initial World Model writes are serialized. The package does not generate robot driving or lifting trajectories.

## Physical models and identifiers

| Object | Model / link | Behavior |
| --- | --- | --- |
| Neuss roof | final_steel / cargo_link | Dynamic payload, 100 kg mass, box-based inertia, oriented collision proxies, and model-pose publication. |
| Neuss lower building | final_workbench / platform_link | Static visual mesh with a support collision box. |
| Reference site | final_reference_site / site_link | Static terrain, fences, containers, surrounding buildings, and collision proxies. |

`FinalSteel_01`, `final_steel`, and the `workbench` configuration key identify the roof and building used by the execution interfaces. Keep these identifiers consistent with topics, native attachment configuration, and ontology individuals.

The lower building uses a conservative support proxy. The visual mesh does not imply that interior spaces are traversable. Roof and site collision boxes are defined in geometry.json; visual geometry and collision geometry have different purposes.

## Scene configuration

| Setting | Value |
| --- | --- |
| Gazebo world | empty |
| Roof envelope | 1.58424 × 0.82110 × 0.81024 m |
| Roof mass | 100 kg |
| Roof initial pose | [-2.2, 0.0, 0.41012, 0.0] |
| Building center | [5.50644, -5.633905, 0.79695] m |
| Building support envelope | 1.61 × 2.03665 × 1.59390 m |
| Semantic start height | 0.40512 m |
| Semantic placement height | 1.99902 m |
| Trailer deck height | 0.48 m |
| Jekko / crane rigging gap | 0.44512 / 0.52512 m |
| Jekko empty stow roof margin | 0.04 m |
| Crane boom length | 10 m |

Poses use metres and yaw in radians. The initial spawn is 0.005 m above the semantic start position. `site.center` and `site.size` are configuration fields; the enabled reference site's detailed layout comes from the packaged meshes and collision metadata.

`reference_site.source`, scale, and translation fields describe the source preparation. The runtime loader reads the prepared STL files and geometry.json; it does not import ReferenceSite.obj or recompute those mesh transforms at launch. See the [asset guide](assets/roof_site/README.md) before editing geometry.

## Interfaces and measured state

| Interface | Purpose |
| --- | --- |
| /model/final_steel/pose | Bridged measured roof model pose. |
| /cask/final/cargo_pose | Roof PoseStamped in the world frame. |
| /cask/final/cargo_clearance | Roof pose, bottom offset, support height, and orientation data. |
| /cask/final/scene_status | Initialization, error, and readiness state. |
| /cask/skill_execution_state | Execution events observed by component-state integration. |
| /cask_world_model/query and /modify | Semantic lookup and state updates. |

A visible roof alone does not establish readiness. The controller requires fresh measured component feedback. A saved ontology also does not recreate physical attachment or ownership after a restart.

## Maintaining a consistent scene

Keep JSON dimensions, mesh geometry, collision proxies, ontology product poses, work positions, and robot reach consistent. The Skill Manager uses rigging gaps from this configuration; the final Jekko server uses the unloaded stow margin. Update and install the complete asset directory together with geometry.json.

Build from the workspace root and source the resulting overlay. Run the integrated scene with `ros2 launch cask_oneopen cask_gazebo_final_oneopen.launch.py`.

[Project overview](../README.md)
