"""Preserve the installed Fortress empty world and add its contact system."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


WORLD_SYSTEMS = {
    "Physics": "physics",
    "UserCommands": "user-commands",
    "SceneBroadcaster": "scene-broadcaster",
    "Contact": "contact",
}


def world_system(plugin):
    """Recognize both Gazebo namespaces and shared-library filename aliases."""
    name = plugin.get("name", "")
    filename = Path(plugin.get("filename", "")).name
    filename = filename.removeprefix("lib").split(".so", 1)[0]
    for system, suffix in WORLD_SYSTEMS.items():
        if name in (
            "ignition::gazebo::systems::" + system,
            "gz::sim::systems::" + system,
        ) or filename in (
            "ignition-gazebo-" + suffix + "-system",
            "gz-sim-" + suffix + "-system",
        ):
            return system
    return None


def ensure_world_systems(world):
    """Keep one configured instance of each required world system.

    Fortress accepts the gz::sim and ignition::gazebo names for the same
    systems. Comparing raw names can instantiate two physics engines. Preserve
    the first existing plugin and its parameters, and leave other systems alone.
    """
    present = set()
    for plugin in list(world.findall("plugin")):
        system = world_system(plugin)
        if system is None:
            continue
        if system in present:
            world.remove(plugin)
        else:
            present.add(system)
    # Explicit world systems suppress Fortress defaults, so add missing systems.
    for system, suffix in WORLD_SYSTEMS.items():
        if system not in present:
            ET.SubElement(
                world,
                "plugin",
                name="ignition::gazebo::systems::" + system,
                filename="ignition-gazebo-" + suffix + "-system",
            )


def read_world(source):
    """Read SDF elements without depending on installed XML comment syntax.

    Gazebo's reader can accept comments containing repeated dashes, whereas
    ElementTree rejects them. Comments have no simulation semantics; omit them
    before parsing while retaining line breaks for useful error locations.
    """
    source = Path(source)
    document = source.read_bytes()
    document = re.sub(
        rb"<!--.*?-->",
        lambda match: b"\n" * match.group().count(b"\n"),
        document,
        flags=re.DOTALL,
    )
    try:
        root = ET.fromstring(document)
    except ET.ParseError as error:
        raise RuntimeError(f"Cannot parse Gazebo world XML {source}: {error}") from error
    if root.tag != "sdf" or root.find("world") is None:
        raise RuntimeError(f"Gazebo world {source} requires <sdf><world> elements")
    return root


def contact_world(destination):
    candidates = list(
        Path("/usr/share/ignition/ignition-gazebo6/worlds").glob("empty.sdf")
    )
    if not candidates:
        raise RuntimeError(
            "Fortress empty.sdf not found under /usr/share/ignition/ignition-gazebo6/worlds"
        )
    root = read_world(candidates[0])
    world = root.find("world")
    if world is None:
        raise RuntimeError("Installed empty.sdf has no world")
    ensure_world_systems(world)
    ET.ElementTree(root).write(destination, encoding="unicode")
    return str(destination)
