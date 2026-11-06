# Gazebo support packages

This directory groups the two ROS 2 packages that support physical simulation, shared motion, and coordinated payload transfer.

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

The active final demonstration uses crane, Jekko, Innok223, and final_steel. Some helper filenames contain Task005 or generated-component terminology from earlier integration stages. Read the specific module documentation before assuming a helper is active in every task.

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

[Project overview](../../README.md)
