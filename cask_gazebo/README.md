# Gazebo support packages

This directory groups the two ROS 2 packages that support physical simulation, shared motion, and coordinated payload transfer.


## Construction scene integration

The simulated payload is a Neuss roof assembly and its destination is a lower building. Their scene generation, meshes, and collision proxies are owned by [cask_component_manager](../cask_component_manager/README.md). The two packages here provide motion, attachment, feedback, optional handoff coordination, and Fortress world preparation.

The integrated launch starts all required processes. There is no separate robot-selection algorithm or site mesh importer in this grouping directory. The model name `final_steel` identifies the roof throughout feedback and attachment channels.

## Packages

| Package | Responsibility | Detailed guide |
| --- | --- | --- |
| cask_gazebo_motion | Planning, navigation, lifting helpers, native attachment, obstacles, and measured-feedback relay. | [Motion README](cask_gazebo_motion/README.md) |
| cask_handoff | Optional semantic suspended-handoff protocol and Fortress world-system preparation. | [Handoff README](cask_handoff/README.md) |

This directory is a grouping directory, not a third ROS package. colcon discovers the package.xml files in the two nested packages.

## Role in the framework

Robot skill servers reuse these helpers to execute numerical targets. The Task Manager remains responsible for candidate selection and task order; the Skill Manager remains responsible for interface/parameter binding.

The final launch also starts the motion package's obstacle manager and feedback relay and uses the handoff package's world setup.

## Final scene versus retained helpers

The roof-installation demonstration uses crane, Jekko, Innok223, and the final_steel roof model. Some helper filenames contain Task005 or generated-component terminology for shared execution utilities. Read the specific module documentation before assuming a helper is active in every task.

Final-scene trailer transfers are implemented by final lifting adapters and carrier state. Optional suspended-handoff groups additionally require compatible semantic metadata and implemented participant adapters.

## Build and launch

Build from the workspace root so the nested packages and their dependencies are discovered:

~~~bash
colcon build --symlink-install --base-paths src
~~~

Start the integrated scene with:

~~~bash
ros2 launch cask_oneopen cask_gazebo_final_oneopen.launch.py
~~~

See the root README for environment setup and task submission.

---

[Project overview](../README.md)

