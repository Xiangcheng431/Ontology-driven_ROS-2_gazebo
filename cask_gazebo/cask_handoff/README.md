# cask_handoff

Optional semantic handoff coordination and Fortress world setup. The protocol keeps provider and receiver execution connected through scoped events and physical support evidence.

## Transfer mechanisms

### Physical attachment

[gazebo_attachment.py](../cask_gazebo_motion/cask_gazebo_motion/gazebo_attachment.py) uses Fortress's native DetachableJoint mechanism.

An accepted Gazebo service request only confirms that a request was queued. Controllers must observe the native attached/detached state before treating a physical attachment transition as complete.

Final-scene attachment channels use the crane, Jekko, or Innok223 owner and the final_steel payload. Cargo pose and supporting-surface information are checked alongside native state.

### Final-scene transfer

The final lifting adapters and Innok223 server coordinate through measured payload state and /cask/final/carrier_state. A loader waits for a suitable carrier pose before placing cargo. The carrier verifies the payload before transport. Release and departure are distinct milestones.

For Jekko loading, transport may start at CARGO_READY_FOR_TRANSPORT while loader cleanup continues. Receiving robots can prepare during transport.

### Optional suspended-handoff protocol

The repository also contains a semantic provider/receiver protocol under [cask_handoff](cask_handoff).

| File | Responsibility |
| --- | --- |
| ontology.py | Read optional intermediate-state metadata through existing DIN relationships, validate poses/data flow, and derive handoff parameters. |
| behavior.py | Run a cooperative provider/receiver group, join goals, forward cancellation, and handle timeouts. |
| channel.py | Session-scoped event transport, freshness checks, and participant validation. |
| contact.py | Verify fresh physical support/contact evidence. |
| timing.py | Track receiver approach progress and bounded waiting. |
| world.py | Add required systems to the installed Fortress empty world while preserving existing configuration and avoiding duplicates. |

[crane_handoff.py](../cask_gazebo_motion/cask_gazebo_motion/crane_handoff.py) provides the crane-side suspended-load adapter.

This optional protocol is activated through handoff metadata. Its presence does not establish that every robot or final task supports every suspended-transfer arrangement. The root handoff/ directory documents an earlier experiment and is not the startup guide for the final five-task scene.

## Protocol concepts

A provider can remain RUNNING while holding a suspended payload. A receiver starts only after the provider reaches the required hold milestone. The cooperative group owns both goals and joins their completion.

Protocol events are checked against execution/session identity, participant identity, and freshness. Position agreement alone is distinct from fresh support/contact evidence. Receiver progress and bounded waits prevent heartbeats from authorizing indefinite waiting.

A new execution cannot reuse a previous session's partial handoff. Such cases can produce HANDOFF_RECOVERY_REQUIRED.

## World preparation

world.py reads the installed Fortress empty world and ensures one instance of each required Physics, UserCommands, SceneBroadcaster, and Contact system. It preserves existing configuration and recognizes the supported namespace aliases when avoiding duplicate systems.

The final launch uses this world preparation even when a particular task does not activate a suspended-handoff group.

The expected installed input is:

~~~text
/usr/share/ignition/ignition-gazebo6/worlds/empty.sdf
~~~

## Integration points

| Consumer | Use |
| --- | --- |
| Task Manager | Annotate optional handoff dependencies, execute HandoffPair, and validate partial-session resume. |
| Skill Manager | Derive additional handoff parameters from semantic product metadata. |
| Crane motion adapter | Hold, lower, verify support, release, and clear the hook. |
| Final launch | Prepare the temporary Fortress world. |

The package is a Python library used by these consumers. setup.py does not define a separate console node for the protocol.

## Retained experiment material

The workspace's root handoff/ directory contains ontology assertions, a checker, and instructions from an earlier suspended-transfer experiment. These are reference materials; the final three-robot startup procedure is documented in the root README and cask_oneopen README.

## Package support files

| File or directory | Role |
| --- | --- |
| [src/cask_gazebo/cask_handoff/package.xml](package.xml) | ROS dependency and package metadata. |
| [src/cask_gazebo/cask_handoff/setup.py](setup.py) | Python package installation and available console entry points. |
| resource/ | Package marker used by the ament index. |
| cask_handoff/__init__.py | Python package initializer. |

---

[Project overview](../../../README.md)
