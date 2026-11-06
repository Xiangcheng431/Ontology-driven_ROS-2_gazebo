"""Convert live semantic product poses to existing robot interfaces."""
import math

A = 'http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#'
V = 'http://www.w3id.org/hsu-aut/VDI3682#'


async def parameters(manager, request, binding):
    rows = await manager.handoff_query(f'''
SELECT DISTINCT ?input ?output WHERE {{
 <{A+request.required_capability_id}> <{V}hasInput> ?input ; <{V}hasOutput> ?output .
}}''')
    if len(rows) != 1:
        raise ValueError('Final scene requires one product input and output')
    poses = {}
    for role, uri in rows[0].items():
        data = await manager.handoff_query(f'''
SELECT DISTINCT ?frame ?x ?y ?z ?yaw WHERE {{
 <{uri}> <{A}hasReferenceFrame> ?frame ; <{A}hasPositionX> ?x ;
 <{A}hasPositionY> ?y ; <{A}hasPositionZ> ?z ; <{A}hasOrientationYaw> ?yaw .
}}''')
        if len(data) != 1 or data[0]['frame'] != 'world':
            raise ValueError('Missing or ambiguous world product pose: '+uri)
        poses[role] = tuple(float(data[0][k]) for k in ('x','y','z','yaw'))
        if not all(math.isfinite(v) for v in poses[role]):
            raise ValueError('Non-finite product pose')
    pickup, delivery = poses['input'], poses['output']
    scene = manager.final_scene
    names, values, units = ['final_scene'], [1.], ['1']
    def add(prefix, pose):
        for axis, value in zip(('x','y','z','yaw'), pose):
            names.append(prefix+'_'+axis); values.append(value)
            units.append('rad' if axis == 'yaw' else 'm')
    add('pickup', pickup); add('delivery', delivery)
    add('cargo_pickup', pickup); add('cargo_delivery', delivery)
    gap = scene['cargo']['size'][2]/2+.04 if binding['resource_name']=='Jekko_01' else .60
    names += ['target_height','rigging_gap']
    values += [delivery[2], gap]; units += ['m','m']
    if binding['resource_name'] == 'Jekko_01':
        locations = scene['locations']
        # Select a configured approach by spatial proximity, never a task number.
        start_cost = math.dist(pickup[:2], locations['startpoint'][:2])
        end_cost = math.dist(delivery[:2], locations['endpoint'][:2])
        work = locations['jekko_start_work' if start_cost < end_cost else 'jekko_end_work']
        add('jekko_work', work)
    return names, values, units
