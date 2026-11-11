"""Supplied site and Neuss meshes, using the existing cargo/link interfaces.

Only scene geometry is defined here. Controllers, task matching, attachments,
feedback and cooperative execution remain in their existing implementations.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def _text(parent, name, value):
    ET.SubElement(parent, name).text = str(value)


def _pose(parent, xyz, rpy=(0., 0., 0.)):
    _text(parent, 'pose', ' '.join(str(v) for v in (*xyz, *rpy)))


def _material(visual, color):
    material = ET.SubElement(visual, 'material')
    for name in ('ambient', 'diffuse'):
        _text(material, name, color)


def _mesh(link, name, path, color):
    visual = ET.SubElement(link, 'visual', name=name)
    mesh = ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh')
    _text(mesh, 'uri', str(path))
    _material(visual, color)


def _collision(link, index, spec):
    collision = ET.SubElement(link, 'collision', name='contact_' + str(index))
    _pose(collision, spec['center'], spec.get('rpy', (0., 0., 0.)))
    geometry = ET.SubElement(ET.SubElement(collision, 'geometry'), 'box')
    _text(geometry, 'size', ' '.join(str(v) for v in spec['size']))
    ode = ET.SubElement(ET.SubElement(ET.SubElement(collision, 'surface'), 'friction'), 'ode')
    _text(ode, 'mu', .9)
    _text(ode, 'mu2', .9)


def _model(name, xyz, yaw=0., dynamic=False):
    root = ET.Element('sdf', version='1.7')
    model = ET.SubElement(root, 'model', name=name)
    _text(model, 'static', 'false' if dynamic else 'true')
    _pose(model, xyz, (0., 0., yaw))
    return root, model


def scene_models(scene):
    """Keep final_steel/cargo_link and final_workbench/platform_link identities."""
    asset_root = Path(scene['_asset_root']) / 'assets' / 'roof_site'
    geometry = json.loads((asset_root / 'geometry.json').read_text())
    result = []
    bench = scene['workbench']
    root, model = _model(bench['model'], bench['center'])
    link = ET.SubElement(model, 'link', name='platform_link')
    _mesh(link, 'neuss_lower_building', asset_root / 'neuss_building.stl', '.68 .71 .73 1')
    # A conservative static support proxy inside the building envelope keeps
    # the four roof bearings on its original top plane. Interior access is
    # outside these five roof installation tasks.
    _collision(link, 0, dict(center=(0., 0., 0.), size=bench['size']))
    result.append((bench['model'], ET.tostring(root, encoding='unicode')))

    cargo = scene['cargo']
    initial = cargo['initial_pose']
    root, model = _model(cargo['model'], initial[:3], initial[3], dynamic=True)
    link = ET.SubElement(model, 'link', name='cargo_link')
    _mesh(link, 'neuss_roof_assembly', asset_root / 'neuss_roof.stl', '.62 .72 .80 1')
    for i, collision in enumerate(geometry['roof_collisions']):
        _collision(link, i, collision)
    # Same mass and box-inertia convention as the previous final steel model.
    mass = cargo['mass']
    x, y, z = cargo['size']
    inertial = ET.SubElement(link, 'inertial')
    _text(inertial, 'mass', mass)
    inertia = ET.SubElement(inertial, 'inertia')
    for name, value in dict(ixx=mass*(y*y+z*z)/12,
                           iyy=mass*(x*x+z*z)/12,
                           izz=mass*(x*x+y*y)/12, ixy=0., ixz=0., iyz=0.).items():
        _text(inertia, name, value)
    plugin = ET.SubElement(model, 'plugin', filename='ignition-gazebo-pose-publisher-system',
                           name='ignition::gazebo::systems::PosePublisher')
    for name, value in dict(publish_model_pose='true', publish_nested_model_pose='true',
                            publish_link_pose='false', use_pose_vector_msg='true',
                            update_frequency='30').items():
        _text(plugin, name, value)
    result.append((cargo['model'], ET.tostring(root, encoding='unicode')))

    root, model = _model('final_reference_site', (0., 0., 0.))
    link = ET.SubElement(model, 'link', name='site_link')
    colors = dict(terrain='.48 .51 .40 1', fence='.67 .72 .76 1',
                  containers='.48 .60 .69 1', surroundings='.76 .77 .73 1')
    for name, filename in geometry['background_meshes'].items():
        _mesh(link, name, asset_root / filename, colors[name])
    for i, collision in enumerate(geometry['background_collisions']):
        _collision(link, i, collision)
    result.append(('final_reference_site', ET.tostring(root, encoding='unicode')))
    return result
