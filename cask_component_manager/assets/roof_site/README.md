# Roof and reference-site assets

Prepared geometry for the Neuss roof installation scene. [roof_site.py](../../cask_component_manager/roof_site.py) reads these files and generates the SDF models used by Gazebo.

## Files

| File | Purpose |
| --- | --- |
| [neuss_roof.stl](neuss_roof.stl) | Visual mesh of the dynamic roof assembly. |
| [neuss_building.stl](neuss_building.stl) | Visual mesh of the static receiving building. |
| [site_terrain.stl](site_terrain.stl) | Terrain visual geometry. |
| [site_fence.stl](site_fence.stl) | Site boundary/fence visual geometry. |
| [site_containers.stl](site_containers.stl) | Container visual geometry. |
| [site_surroundings.stl](site_surroundings.stl) | Surrounding building/site visual geometry. |
| [geometry.json](geometry.json) | Roof and background collision boxes, mesh mapping, dimensions, and preparation metadata. |

## Runtime interpretation

The roof and building meshes use the model poses from [final_scene.json](../../config/final_scene.json). The background is created as `final_reference_site` at the world origin; its prepared mesh coordinates and collision definitions establish the site layout.

geometry.json contains 17 roof collision proxies and 111 background collision proxies. Each proxy supplies a box center, size, and optional roll/pitch/yaw. The building instead uses the configured support-envelope box. Mesh triangles are used for visuals; the collision representation is intentionally simpler.

The `background_meshes` mapping associates terrain, fence, containers, and surroundings with their STL files. Other fields record geometry preparation, including source transforms and the placement of surrounding buildings. `roof_site.py` does not recompute these preparation transforms or import OBJ files at launch. Editing a source-scale or translation metadata field alone therefore does not reposition the prepared meshes.

## Physical model

The roof is dynamic with a configured mass of 100 kg and box-envelope inertia. The building and site are static. The roof publishes measured model poses for the bridge and Component Manager. Attachment uses the roof's `cargo_link`.

Keep mesh coordinates, collision proxies, configured dimensions, semantic pickup/placement poses, and hook clearance consistent. Changes to visuals alone do not update collisions or the planner's obstacle snapshot.

## Installation

The Component Manager's setup.py installs all files in this directory under `share/cask_component_manager/assets/roof_site/`. Include the complete directory when distributing the source and rebuild the package after changing assets.

[Component Manager](../../README.md)
