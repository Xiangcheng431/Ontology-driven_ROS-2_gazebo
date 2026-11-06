"""Compose material delivery/unloading from IOC semantics and scene facts.

No robot is selected here. Existing provided capabilities and ROS skills stay
the resources used by Task Manager. A handling profile expands the two source
records into the existing complete pick/carry/place skill contract.
"""
import json
import math
import os
import re
import subprocess
import tempfile
from pathlib import Path

from rdflib import Graph, Literal, Namespace, OWL, RDF, RDFS, URIRef, XSD

A = Namespace('http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#')
S = Namespace('urn:cask:scene:')
CSS = Namespace('http://www.w3id.org/hsu-aut/css#')
CASK = Namespace('http://www.w3id.org/hsu-aut/cask#')
VDI = Namespace('http://www.w3id.org/hsu-aut/VDI3682#')
DIN = Namespace('http://www.w3id.org/hsu-aut/DINEN61360#')
PROV = Namespace('http://www.w3.org/ns/prov#')


def one(g, s, p):
    values = set(g.objects(s, p))
    if len(values) != 1:
        raise ValueError(f'Expected one {p} for {s}, got {len(values)}')
    return next(iter(values))


def pose(g, s):
    if str(one(g, s, A.hasReferenceFrame)) != 'world':
        raise ValueError('Scene poses must use world frame.')
    result = [float(one(g, s, A[k])) for k in
              ('hasPositionX', 'hasPositionY', 'hasPositionZ', 'hasOrientationYaw')]
    if not all(map(math.isfinite, result)):
        raise ValueError('Non-finite scene pose.')
    return result


def data_element(g, name, td, value, goal):
    prop, desc = A[name+'_Property'], A[name+'_ID']
    g.add((prop, RDF.type, DIN.Data_Element))
    g.add((prop, DIN.has_Type_Description, td))
    g.add((prop, DIN.has_Instance_Description, desc))
    g.add((desc, RDF.type, DIN.Instance_Description))
    g.add((desc, DIN.Instance_Description_has_Type, td))
    for p, v in ((DIN.Expression_Goal, goal), (DIN.Logic_Interpretation, '='),
                 (DIN.Value, value)):
        g.add((desc, p, Literal(v)))
    return prop


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(dir=path.parent, prefix=path.name+'.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
        os.replace(temp, path)
    finally:
        Path(temp).unlink(missing_ok=True)


def _compose_pallet(api, args, base, source, identifiers):
    # The selected material-delivery scenario defines its starting condition.
    # External historical records are reported, not silently treated as simulated.
    ordered, external = api.select_processes(source, identifiers, True)
    exact = {t for t in base.subjects(PROV.wasDerivedFrom, ordered[0])
             if set(base.objects(t, PROV.wasDerivedFrom)) == set(ordered)
             and (t, RDF.type, CSS.Process) in base}
    if len(exact)>1:
        raise ValueError('Multiple existing tasks have the same source scope.')
    if exact:
        task = next(iter(exact)); api.ordered_requirements(base, task)
        config = json.loads(str(one(base, task, S.executionProfile)))
        return base, config, False, external
    rules_path = args.workspace.expanduser()/'src/ontology/ioc-semantic-rules.json'
    scene_path = args.scene or args.workspace.expanduser()/'src/ontology/cask-scene-abox.ttl'
    if not rules_path.is_file() or not scene_path.is_file():
        raise ValueError('Install IOC semantic rules and the scene supplement first.')
    rules = json.loads(rules_path.read_text())
    operations = []
    detected_materials = set()
    for process in ordered:
        descriptions = {str(v).strip() for p in (RDFS.label,RDFS.comment) for v in source.objects(process,p)}
        matches = set()
        for rule in rules['operations']:
            for description in descriptions:
                if any(re.search(pattern,description,re.I) for pattern in rule['patterns']):
                    matches.add(rule['operation'])
        for material,patterns in rules['materials'].items():
            if any(re.search(pattern,description,re.I) for pattern in patterns for description in descriptions):
                detected_materials.add(material)
        if len(matches) != 1:
            raise ValueError('No unambiguous semantic operation rule for '+str(process))
        operations.append(next(iter(matches)))
    patterns = [p for p in rules['patterns'] if p['operations'] == operations]
    if len(patterns) != 1:
        raise ValueError('No execution profile covers this ordered IOC operation sequence.')
    pattern = patterns[0]
    if len(detected_materials)>1:
        raise ValueError('Selected IOC records refer to different materials; separate scene bindings are required.')
    if not detected_materials:
        raise ValueError('Material meaning is not covered by the semantic rules; no scene object is guessed from labels.')
    scene = Graph().parse(scene_path, format='turtle')
    scenarios = {s for s in scene.subjects(RDF.type, S.MaterialTransferScenario)
                 if not detected_materials or str(one(scene,s,S.material)) in detected_materials}
    if len(scenarios) != 1:
        raise ValueError('Scene must identify one material transfer scenario.')
    scenario = next(iter(scenarios))
    material = str(one(scene,scenario,S.material))
    pickup_node, delivery_node = one(scene, scenario, S.pickup), one(scene, scenario, S.delivery)
    pickup, delivery = pose(scene, pickup_node), pose(scene, delivery_node)
    work_nodes=set(scene.objects(scenario,S.mobileCraneWorkPose))
    if len(work_nodes)>1:raise ValueError('Ambiguous mobile-crane work hint.')
    work = (pose(scene,next(iter(work_nodes))) if work_nodes else
            [(pickup[0]+delivery[0])/2,(pickup[1]+delivery[1])/2,.045,0.])
    mass = float(one(scene, scenario, S.massKg))
    carry = float(one(scene, scenario, S.carryHeight))
    approach = float(one(scene,scenario,S.pickupApproachDistance))
    if not 0 < mass or not math.isfinite(mass) or not .65 <= carry <= 3.0:
        raise ValueError('Invalid mass or carrying height.')
    if not math.isfinite(approach) or not 1.8<=approach<=4.:
        raise ValueError('Ground pallet approach distance must be between 1.8 and 4 m.')
    if abs(pickup[2]-.4)>.01 or abs(delivery[2]-.4)>.01:
        raise ValueError('This profile requires a ground pallet reference at z=0.4 m.')
    if math.dist(pickup[:2], delivery[:2]) < .5:
        raise ValueError('Pickup and intermediate storage must be distinct.')
    task_id = args.task_id or api.next_task_id(base)
    task = api.task_iri(task_id)
    if any(base.triples((task, None, None))):
        raise ValueError('Target ID already exists; it will not be overwritten.')
    result = api.clone(base)
    for triple in scene:
        result.add(triple)
    required = A[task_id+'_Required_001']
    before, after = A[task_id+'_TimberBefore'], A[task_id+'_TimberAfter']
    for state, coords in ((before, pickup), (after, delivery)):
        result.add((state, RDF.type, OWL.NamedIndividual))
        result.add((state, RDF.type, VDI.Product))
        for key, value in zip(('hasPositionX','hasPositionY','hasPositionZ','hasOrientationYaw'),coords):
            result.add((state, A[key], Literal(value)))
        result.add((state, A.hasReferenceFrame, Literal('world')))
        kind = data_element(result, task_id+'_MaterialKind', A.ProductKind_TD, material, 'Requirement')
        result.add((state, DIN.has_Data_Element, kind))
    result.add((before, DIN.has_Data_Element,
                data_element(result, task_id+'_Payload', A.Payload_TD, mass, 'Requirement')))
    result.add((after, DIN.has_Data_Element,
                data_element(result, task_id+'_LiftHeight', A.LiftingHeight_TD, carry, 'Requirement')))
    catalog = api.schema_catalog(base)
    properties = catalog.get(pattern['required_kind'],set())
    if len(properties)!=1:
        raise ValueError('Existing capability requirement schema is missing or ambiguous.')
    for typ in (OWL.NamedIndividual, VDI.ProcessOperator, CASK.RequiredCapability):
        result.add((required,RDF.type,typ))
    for p,v in ((VDI.hasInput,before),(VDI.hasOutput,after),(CSS.isSpecifiedBy,next(iter(properties)))):
        result.add((required,p,v))
    for typ in (OWL.NamedIndividual,CSS.Process): result.add((task,RDF.type,typ))
    result.add((task,CSS.requiresCapability,required)); result.add((task,VDI.consistsOf,required))
    for process in ordered:
        result.add((task,PROV.wasDerivedFrom,process)); result.add((required,PROV.wasDerivedFrom,process))
    # Extend material compatibility of EXISTING physical pallet/eye interfaces;
    # no new ProvidedCapability, Skill, payload limit, or robot is invented.
    assurance = data_element(result, 'PalletizedMaterial_'+str(scenario).rsplit(':',1)[-1]+'_Assurance', A.ProductKind_TD,material,'Assurance')
    allowed=[]
    for entry in rules['handling_interfaces']:
        robot, cap = A[entry['resource']], A[entry['provided_capability']]
        if (robot,CSS.providesCapability,cap) not in base:
            continue
        for p in (VDI.hasInput,VDI.hasOutput):
            product = one(base,cap,p)
            result.add((product,DIN.has_Data_Element,assurance))
        allowed.append(entry)
    if not allowed:
        raise ValueError('No installed physical handling interface supports this profile.')
    config = dict(version=1,task_id=task_id,profile=pattern['profile'],material=material,
                  model=str(one(scene,scenario,S.gazeboModelName)),mass=mass,
                  pickup=pickup,delivery=delivery,jekko_work=work,carry_height=carry,
                  pickup_approach_distance=approach,
                  required_capability=required.split('#')[-1],input_state=str(before),output_state=str(after),
                  handling_interfaces=allowed,source_processes=list(map(str,ordered)))
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',config['model']):
        raise ValueError('Invalid Gazebo model name.')
    # A model cannot silently serve two tasks with different physical states.
    for existing in base.objects(None,S.executionProfile):
        previous=json.loads(str(existing))
        if previous.get('model')==config['model']:
            raise ValueError('Scene model already belongs to another task; add another scene instance.')
    result.add((task,S.executionProfile,Literal(json.dumps(config,sort_keys=True))))
    api.ordered_requirements(result,task)
    return result,config,True,external


def compose(api, args, base, source, identifiers):
    """Keep the verified physical transfer adapter; otherwise compose any kind."""
    ordered, external = api.select_processes(source, identifiers, True)
    existing = {t for t in base.subjects(PROV.wasDerivedFrom,ordered[0])
                if (t,RDF.type,CSS.Process) in base
                and set(base.objects(t,PROV.wasDerivedFrom))==set(ordered)}
    if len(existing)>1: raise ValueError('Multiple tasks have the same complete IOC scope.')
    if existing:
        task=next(iter(existing));api.ordered_requirements(base,task)
        return base,json.loads(str(one(base,task,S.executionProfile))),False,external
    path=args.workspace.expanduser()/'src/ontology/ioc-semantic-rules.json'
    rules=json.loads(path.read_text()) if path.is_file() else {}
    operations=[]
    for process in ordered:
        # Explicit annotations take priority over a legacy lexical profile.
        if set(source.objects(process,S.processKind)):operations=[];break
        descriptions=[str(v) for p in (RDFS.label,RDFS.comment) for v in source.objects(process,p)]
        matches={r['operation'] for r in rules.get('operations',[])
                 if any(re.search(p,d,re.I) for p in r['patterns'] for d in descriptions)}
        if len(matches)!=1:operations=[];break
        operations.append(next(iter(matches)))
    legacy=[p for p in rules.get('patterns',[]) if p['operations']==operations
            and p.get('profile')=='pallet_transfer']
    scene_path=args.scene or args.workspace.expanduser()/'src/ontology/cask-scene-abox.ttl'
    annotated=Graph().parse(scene_path,format='turtle') if scene_path.is_file() else Graph()
    if any(set(annotated.objects(p,S.processKind)) for p in ordered):legacy=[]
    if len(legacy)==1 and operations:
        return _compose_pallet(api,args,base,source,identifiers)
    try:
        from .ioc_capability_conversion import compose_generic
    except ImportError:
        from ioc_capability_conversion import compose_generic
    return compose_generic(api,args,base,source,identifiers,data_element)


def synchronize(workspace, delta_path, task_id):
    """Source ROS in a child shell; merge itself needs no terminal ROS setup."""
    helper=Path(__file__).with_name('ioc_world_model_import.py')
    setup=workspace/'install/setup.bash'
    if not helper.is_file() or not setup.is_file():
        raise ValueError('World Model import helper or built workspace is missing.')
    command=['bash','-c','source /opt/ros/humble/setup.bash && source "$1" && exec python3 "$2" --import-file "$3" --task-id "$4"',
             'ioc-import',str(setup),str(helper),str(delta_path),task_id]
    done=subprocess.run(command,text=True,capture_output=True,timeout=55)
    if done.returncode:
        raise ValueError('World Model import failed; no task execution command issued. '+done.stdout+done.stderr)
    return done.stdout.strip()


def merge_new(api,args,base,source,identifiers,cask_path):
    original=cask_path.read_bytes()
    if getattr(args,'preview',False):
        result,config,created,external=compose(api,args,base,source,identifiers)
        return dict(status='generated' if created else 'existing',task_id=config['task_id'],preview=True,
                    source_process=' + '.join(config['source_processes']),scene_config=config,
                    capabilities=[r['kind'] for r in api.ordered_requirements(result,api.task_iri(config['task_id']))],
                    world_model_updated=False,source_cask_changed=False,external_predecessors=external)
    runtime=args.workspace.expanduser()/'runtime/ioc_task_scenes'
    runtime.mkdir(parents=True,exist_ok=True)
    # Current-session tasks and numbering come from the RUNNING graph, not a
    # cache from a previous launch. The source CASK ABox is never amended here.
    fd,snapshot_file=tempfile.mkstemp(dir=runtime,suffix='.ttl'); os.close(fd)
    try:
        snapshot_world_model(args.workspace.expanduser(),Path(snapshot_file))
        active=Graph().parse(snapshot_file,format='turtle')
    finally:
        Path(snapshot_file).unlink(missing_ok=True)
    result,config,created,external=compose(api,args,active,source,identifiers)
    task_id=config['task_id']
    # Send a complete task closure with existing schema, enabling idempotent
    # reimport after a reset launch. The server only adds, never replaces state.
    fd,tmp=tempfile.mkstemp(dir=runtime,suffix='.ttl'); os.close(fd)
    try:
        result.serialize(tmp,format='turtle')
        sync_message=synchronize(args.workspace.expanduser(),Path(tmp),task_id)
    finally:
        Path(tmp).unlink(missing_ok=True)
    if cask_path.read_bytes()!=original:
        raise ValueError('Source CASK file changed concurrently; this converter did not modify it.')
    return dict(status='generated' if created else 'existing',task_id=task_id,
                source_process=' + '.join(config['source_processes']),
                capabilities=[r['kind'] for r in api.ordered_requirements(result,api.task_iri(task_id))],world_model_updated=True,association_added=False,
                backup=None,sync_message=sync_message,source_cask_changed=False,
                external_predecessors=external,scene_config=config)


def snapshot_world_model(workspace,target):
    helper=Path(__file__).with_name('ioc_world_model_import.py')
    command=['bash','-c','source /opt/ros/humble/setup.bash && source "$1" && exec python3 "$2" --snapshot-file "$3"',
             'ioc-snapshot',str(workspace/'install/setup.bash'),str(helper),str(target)]
    done=subprocess.run(command,text=True,capture_output=True,timeout=20)
    if done.returncode:
        raise ValueError('Cannot read running World Model: '+done.stdout+done.stderr)
