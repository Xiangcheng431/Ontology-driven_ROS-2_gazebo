"""Optional handoff metadata using existing DIN EN 61360 ABox relations.

No task/robot identifiers or world coordinates are encoded in the scheduler.
"""

import math
import re

DIN = "http://www.w3id.org/hsu-aut/DINEN61360#"
ABOX = "http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#"
VDI = "http://www.w3id.org/hsu-aut/VDI3682#"
CODE = "handoffIntermediateState"


def uri(value):
    if not re.fullmatch(r'https?://[^\s<>"{}]+', value):
        raise ValueError("Invalid ontology resource URI")
    return "<" + value + ">"


async def profile(query, product):
    rows = await query(f"""PREFIX din: <{DIN}>
SELECT DISTINCT ?middle WHERE {{
 {uri(product)} din:has_Data_Element ?property .
 ?property din:has_Type_Description ?type ; din:has_Instance_Description ?instance .
 ?type din:Code "{CODE}" . ?instance din:Value ?middle .
}}""")
    if not rows:
        return None
    if len(rows) != 1:
        raise ValueError("Handoff requires exactly one intermediate state reference")
    middle = rows[0]["middle"]
    poses = await query(f"""PREFIX abox: <{ABOX}>
SELECT ?x ?y ?z ?yaw ?frame WHERE {{ {uri(middle)}
 abox:hasPositionX ?x ; abox:hasPositionY ?y ; abox:hasPositionZ ?z ;
 abox:hasOrientationYaw ?yaw ; abox:hasReferenceFrame ?frame . }}""")
    if len(poses) != 1 or poses[0]["frame"] != "world":
        raise ValueError("Handoff intermediate state requires one complete world pose")
    pose = tuple(float(poses[0][k]) for k in ("x", "y", "z", "yaw"))
    if not all(math.isfinite(v) for v in pose):
        raise ValueError("Non-finite handoff pose")
    return dict(middle=middle, pose=pose, product=product)


async def annotate(query, ordered):
    for index, step in enumerate(ordered):
        data = await profile(query, step["output_uri"])
        if data is None:
            continue
        if (
            index + 1 >= len(ordered)
            or ordered[index + 1]["input_uri"] != step["output_uri"]
        ):
            raise ValueError(
                "Handoff output must feed exactly the following capability"
            )
        if index and ordered[index - 1].get("handoff"):
            raise ValueError("Overlapping handoff groups are unsupported")
        step["handoff"] = data


def validate_resume(ordered, completed):
    """A new execution cannot reuse a prior session's partial handoff."""
    if 0 < completed < len(ordered) and ordered[completed - 1].get("handoff"):
        raise ValueError(
            "HANDOFF_RECOVERY_REQUIRED: receiver already holds the load; "
            "fresh suspended handoff cannot resume from an old session"
        )


async def parameterize(query, request, names, values, units):
    rows = await query(f"""PREFIX vdi: <{VDI}>
SELECT ?input ?output WHERE {{ {uri(ABOX+request.required_capability_id)}
 vdi:hasInput ?input ; vdi:hasOutput ?output . }}""")
    if len(rows) != 1:
        raise ValueError("Capability requires a unique input/output")
    output = await profile(query, rows[0]["output"])
    source = await profile(query, rows[0]["input"])
    if output and source:
        raise ValueError("A skill cannot provide and receive the same handoff")
    data = output or source
    if data is None:
        return names, values, units
    p = dict(zip(names, values))
    prefix = "cargo_delivery_" if output else "cargo_pickup_"
    needed = [prefix + k for k in ("x", "y", "z")] + (
        ["cargo_delivery_yaw"] if output else ["pickup_yaw", "target_height"]
    )
    if any(name not in p for name in needed):
        raise ValueError(
            "HANDOFF_ADAPTER_UNSUPPORTED: missing physical cargo parameters"
        )
    final = tuple(p[prefix + k] for k in ("x", "y", "z"))
    receive_z = p.get('handoff_receiver_z', final[2]) if source else final[2]
    if not math.isfinite(receive_z) or receive_z < 0:
        raise ValueError('Invalid receiver world height')
    yaw = p["cargo_delivery_yaw"] if output else p["pickup_yaw"]
    mx, my, mz, ma = data["pose"]
    if (
        math.hypot(mx - final[0], my - final[1]) > 0.001
        or abs(math.atan2(math.sin(ma - yaw), math.cos(ma - yaw))) > 0.001
    ):
        raise ValueError(
            "Handoff middle and receiver pose must have identical world XY/yaw"
        )
    if mz < receive_z + 0.25:
        raise ValueError("Handoff wait height needs at least 0.25 m clearance")
    names = list(names) + ["handoff_mode", "handoff_middle_z", "handoff_receive_z"]
    values = list(values) + [1.0 if output else 2.0, mz, receive_z]
    units = list(units) + ["1", "m", "m"]
    if source:
        # The observed input is suspended at Middle. The desired support height
        # comes from the selected receiver profile, or the existing After state.
        values[names.index("cargo_pickup_z")] = mz
        values[names.index("target_height")] = receive_z
    return names, values, units
