#!/usr/bin/env python3
"""IOC -> CASK requirement converter (Python 3.10+/rdflib).

Normal `merge` reuses confirmed scenarios or composes a capability sequence
from IOC order, structured semantics/configurable aliases and Scene facts.
The running World Model is imported and saved separately from the source CASK.
`merge ... --preview` validates without ROS or file mutation.

The verified physical pallet-transfer adapter remains available. Generic
conversion does not invent provided abilities, skill implementations or
Gazebo components. Unsupported execution is distinct from conversion success.
The historical direct --rules interface is retained for compatibility.
"""
import argparse
import hashlib
import json
import math
import os
import re
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

from rdflib import Graph, Literal, Namespace, OWL, RDF, URIRef

A = Namespace('http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#')
CSS = Namespace('http://www.w3id.org/hsu-aut/css#')
CASK = Namespace('http://www.w3id.org/hsu-aut/cask#')
VDI = Namespace('http://www.w3id.org/hsu-aut/VDI3682#')
DIN = Namespace('http://www.w3id.org/hsu-aut/DINEN61360#')
IOC = Namespace('http://w3id.org/ioc#')
PROV = Namespace('http://www.w3.org/ns/prov#')
SCENE = Namespace('urn:cask:scene:')
# User-confirmed source-to-execution correspondence. This is a mapping rule,
# not a capability-label inference and not a new task or robot assignment.
CONFIRMED_TASK_BINDINGS = {
    frozenset(('https://ip.org/targetx/ioc/task_006',
               'https://ip.org/targetx/ioc/task_007')): 'Task_001',
    frozenset(('https://ip.org/targetx/ioc/task_018',
               'https://ip.org/targetx/ioc/task_019')): 'Task_002',
    frozenset(('https://ip.org/targetx/ioc/task_035',
               'https://ip.org/targetx/ioc/task_036',
               'https://ip.org/targetx/ioc/task_038',
               'https://ip.org/targetx/ioc/task_042')): 'Task_005',
}
POSE_NAMES = ('hasPositionX', 'hasPositionY', 'hasPositionZ',
              'hasOrientationYaw', 'hasReferenceFrame')


def single(graph, subject, predicate):
    values = set(graph.objects(subject, predicate))
    if len(values) != 1:
        raise ValueError(f'{subject} must have exactly one {predicate}; got {len(values)}')
    return next(iter(values))


def iri(value):
    if not isinstance(value, str) or not value:
        raise ValueError('Expected an individual name or full URI.')
    return URIRef(value) if ':' in value else A[value]


def task_iri(name):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
        raise ValueError('Task ID must contain only letters, digits and underscores.')
    return A[name]


def clone(graph):
    result = Graph()
    for prefix, namespace in graph.namespaces():
        result.bind(prefix, namespace)
    result += graph
    return result


def process_property(graph, required):
    """Read existing DIN ProcessKind Requirement descriptors, not labels."""
    candidates = set()
    for prop in graph.objects(required, CSS.isSpecifiedBy):
        if A.ProcessKind_TD not in set(graph.objects(prop, DIN.has_Type_Description)):
            continue
        for desc in graph.objects(prop, DIN.has_Instance_Description):
            if any(str(goal) == 'Requirement' for goal in graph.objects(desc, DIN.Expression_Goal)):
                candidates.add((str(single(graph, desc, DIN.Value)), prop))
    if len(candidates) != 1:
        raise ValueError(f'{required} has no unambiguous ProcessKind Requirement.')
    return next(iter(candidates))


def schema_catalog(graph):
    """Learn how each existing process kind is described for matching."""
    result = {}
    for required in graph.subjects(RDF.type, CASK.RequiredCapability):
        name, prop = process_property(graph, required)
        result.setdefault(name, set()).add(prop)
    if not result:
        raise ValueError('No existing RequiredCapabilities to learn the CASK schema from.')
    return result


def validate_state(graph, state):
    if not any(graph.triples((state, None, None))):
        raise ValueError('Missing state data: '+str(state))
    if (state, RDF.type, VDI.Product) not in graph:
        raise ValueError('State must use the existing VDI Product representation: '+str(state))
    # Logical requirements need not invent a pose. If a pose is present,
    # retain the complete world-pose validation used by existing motion.
    spatial = any(set(graph.objects(state,A[n])) for n in POSE_NAMES)
    for name in POSE_NAMES if spatial else ():
        value = single(graph, state, A[name])
        if name == 'hasReferenceFrame':
            if str(value) != 'world':
                raise ValueError('Current motion parameterization requires frame world: '+str(state))
        elif not math.isfinite(float(value)):
            raise ValueError('Non-finite coordinate: '+str(state))
    properties = set(graph.objects(state, DIN.has_Data_Element))
    for prop in properties:
        td = single(graph, prop, DIN.has_Type_Description)
        descriptions = set(graph.objects(prop, DIN.has_Instance_Description))
        if not descriptions:
            raise ValueError('Missing parameter description: '+str(prop))
        if not any(graph.triples((td, None, None))):
            raise ValueError('Missing DIN Type_Description: '+str(td))
        for desc in descriptions:
            value = single(graph, desc, DIN.Value)
            single(graph, desc, DIN.Expression_Goal)
            if isinstance(value, Literal) and isinstance(value.toPython(), (int, float)):
                if not math.isfinite(float(value)):
                    raise ValueError('Non-finite requirement parameter: '+str(desc))


def ordered_requirements(graph, task):
    required = set(graph.objects(task, CSS.requiresCapability))
    if not required:
        raise ValueError('No RequiredCapabilities for '+str(task))
    rows = {}
    for req in required:
        if (req, RDF.type, CASK.RequiredCapability) not in graph:
            raise ValueError('Missing RequiredCapability type: '+str(req))
        name, _ = process_property(graph, req)
        before, after = single(graph, req, VDI.hasInput), single(graph, req, VDI.hasOutput)
        validate_state(graph, before)
        validate_state(graph, after)
        rows[req] = {'required': req, 'kind': name, 'input': before, 'output': after}
    pending = {req: {other for other in required if req != other and
                    rows[other]['output'] == rows[req]['input']} for req in required}
    ordered = []
    while pending:
        ready = sorted(req for req, parents in pending.items() if not parents)
        if len(ready) != 1:
            raise ValueError('RequiredCapability data flow is cyclic, disconnected or parallel.')
        current = ready[0]
        ordered.append(rows[current])
        del pending[current]
        for parents in pending.values():
            parents.discard(current)
    for first, second in zip(ordered, ordered[1:]):
        if first['output'] != second['input']:
            raise ValueError('Input/output chain is not connected.')
    return ordered


def select_processes(graph, identifiers, external_complete=False):
    """Read exact IDs, flatten hierarchy, then topologically sort source edges."""
    selected = set()
    def resolve(name):
        uri = URIRef(name)
        if (uri, RDF.type, IOC.Process) in graph:
            return uri
        matches = {s for s in graph.subjects(RDF.type, IOC.Process)
                   if str(s).rsplit('#', 1)[-1].rsplit('/', 1)[-1] == name}
        if len(matches) != 1:
            raise ValueError(f'IOC identifier {name!r} has {len(matches)} matches; use its full URI.')
        return next(iter(matches))
    def visit(subject, stack):
        if subject in stack:
            raise ValueError('Cycle in IOC process hierarchy.')
        children = set(graph.objects(subject, IOC.hasChild)) | set(graph.subjects(IOC.hasParent, subject))
        if children:
            for child in children:
                visit(child, stack | {subject})
        else:
            if (subject, RDF.type, IOC.Process) not in graph:
                raise ValueError('Selected child is not an IOC Process: '+str(subject))
            selected.add(subject)
    for identifier in identifiers:
        visit(resolve(identifier), set())
    edges = set(graph.subject_objects(IOC.hasSuccessor)) | {
        (before, after) for after, before in graph.subject_objects(IOC.hasPredecessor)}
    external = sorted({str(a) for a, b in edges if b in selected and a not in selected})
    if external and not external_complete:
        raise ValueError('External predecessors need confirmation: '+', '.join(external))
    pending = {s: {a for a, b in edges if b == s and a in selected} for s in selected}
    ordered = []
    while pending:
        ready = sorted(s for s, parents in pending.items() if not parents)
        if len(ready) != 1:
            raise ValueError('IOC source order is cyclic or ambiguous; no order is guessed from IDs.')
        current = ready[0]
        ordered.append(current)
        del pending[current]
        for parents in pending.values():
            parents.discard(current)
    return ordered, external


def merge_scene(base, scene):
    """Add new scene descriptions; do not silently override existing data."""
    result = clone(base)
    functional = set(A[name] for name in POSE_NAMES) | {
        DIN.Value, DIN.Expression_Goal, DIN.Logic_Interpretation,
        DIN.has_Type_Description, DIN.has_Instance_Description, DIN.Unit_of_Measure}
    for s, p, o in scene:
        old = set(base.objects(s, p))
        if p in functional and old and old != {o}:
            raise ValueError('Scene conflicts with existing ABox: '+str(s)+' '+str(p))
        result.add((s, p, o))
    return result


def make_task(base, source, identifiers, rules, scene, task_id, external_complete=False):
    """Generate only new task/requirement relations, retaining source state URIs."""
    task = task_iri(task_id)
    if any(base.triples((task, None, None))):
        raise ValueError('Target task already has data. Incomplete existing tasks are not overwritten.')
    ordered, external = select_processes(source, identifiers, external_complete)
    bindings = rules.get('processes', {})
    missing = [str(process) for process in ordered if str(process) not in bindings]
    if missing:
        raise ValueError('No explicit semantic mapping for: '+', '.join(missing))
    result = merge_scene(base, scene)
    catalog = schema_catalog(result)
    initial = iri(rules['initial_state'])
    validate_state(result, initial)
    before, kinds, number, states = initial, [], 0, {initial}
    result.add((task, RDF.type, OWL.NamedIndividual))
    result.add((task, RDF.type, CSS.Process))
    for process in ordered:
        steps = bindings[str(process)].get('steps', [])
        if not steps:
            raise ValueError('No execution steps mapped for '+str(process))
        result.add((task, PROV.wasDerivedFrom, process))
        for step in steps:
            name = step['kind']
            properties = catalog.get(name, set())
            if len(properties) != 1:
                raise ValueError('Process kind has no unique existing requirement schema: '+name)
            after = iri(step['after'])
            validate_state(result, after)
            if after in states:
                raise ValueError('Output reuses an earlier state and would create a data-flow cycle: '+str(after))
            states.add(after)
            number += 1
            required = A[task_id+'_Required_'+format(number, '03d')]
            if any(result.triples((required, None, None))):
                raise ValueError('Generated requirement ID already exists: '+str(required))
            for t in (OWL.NamedIndividual, VDI.ProcessOperator, CASK.RequiredCapability):
                result.add((required, RDF.type, t))
            result.add((required, CSS.isSpecifiedBy, next(iter(properties))))
            result.add((required, VDI.hasInput, before))
            result.add((required, VDI.hasOutput, after))
            result.add((required, PROV.wasDerivedFrom, process))
            result.add((task, CSS.requiresCapability, required))
            result.add((task, VDI.consistsOf, required))
            before = after
            kinds.append(name)
    verified = ordered_requirements(result, task)
    if [row['kind'] for row in verified] != kinds:
        raise ValueError('Generated data-flow order differs from IOC expansion.')
    if set(base)-set(result):
        raise ValueError('Conversion attempted to remove existing triples.')
    return result, {'task_id': task_id, 'source_processes': [str(s) for s in ordered],
                    'capabilities': kinds, 'external_preconditions_assumed_complete': external,
                    'triples_added': len(result-base)}


def save_existing_file(path, graph):
    """Backup and atomically amend the selected TTL; never create a second ontology."""
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    backup = path.with_name(path.name+'.before-ioc-'+stamp)
    fd, name = tempfile.mkstemp(prefix=path.name+'.ioc-', suffix='.tmp', dir=path.parent)
    os.close(fd)
    temporary = Path(name)
    try:
        graph.serialize(destination=str(temporary), format='turtle')
        checked = Graph().parse(str(temporary), format='turtle')
        if set(checked) != set(graph):
            raise ValueError('Serialized graph does not preserve all triples.')
        shutil.copy2(path, backup)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
    return backup


def execute_command(task_id):
    return ('source /opt/ros/humble/setup.bash\n'
            'source ~/cask_gazebo_final_ws/install/setup.bash\n\n'
            'ros2 action send_goal --feedback /cask_task_manager/execute_task '
            'cask_interfaces/action/ExecuteTask '+"'{task_id: \""+task_id+"\"}'")


def next_task_id(graph):
    """Allocate above the largest occupied numeric ID, starting at Task_006."""
    numbers = [5]
    for subject in set(graph.subjects()):
        match = re.fullmatch(re.escape(str(A))+r'Task_(\d+)', str(subject))
        if match:
            numbers.append(int(match.group(1)))
    return 'Task_'+format(max(numbers)+1, '03d')


def run(args):
    path = args.cask.expanduser().resolve()
    original_bytes = path.read_bytes()
    base = Graph().parse(data=original_bytes, format='turtle')
    task = task_iri(args.task_id) if args.task_id else None
    if task is not None and any(base.triples((task, None, None))):
        # First branch is reuse. No generated copy, reference cache or file edit.
        chain = ordered_requirements(base, task)
        return {'status': 'existing', 'task_id': args.task_id,
                'capabilities': [row['kind'] for row in chain],
                'triples_added': 0, 'file_changed': False, 'world_model_updated': False}
    if not args.ioc or not args.processes or not args.rules:
        raise ValueError('Missing task generation requires --ioc, --processes and --rules. '
                         'No data has been changed.')
    source = Graph().parse(str(args.ioc.expanduser()), format='turtle')
    rules = json.loads(args.rules.expanduser().read_text())
    scene = Graph().parse(str(args.scene.expanduser()), format='turtle') if args.scene else Graph()
    if task is None:
        ordered, _ = select_processes(source, args.processes, args.external_preconditions_satisfied)
        scope = set(ordered)
        matches = {candidate for candidate in base.subjects(PROV.wasDerivedFrom, ordered[0])
                   if (candidate, RDF.type, CSS.Process) in base
                   and set(base.objects(candidate, PROV.wasDerivedFrom)) == scope}
        if len(matches) > 1:
            raise ValueError('Multiple CASK tasks already have this exact IOC source scope.')
        if matches:
            task = next(iter(matches))
            args.task_id = str(task).removeprefix(str(A))
            task_iri(args.task_id)
            chain = ordered_requirements(base, task)
            return {'status': 'existing', 'task_id': args.task_id,
                    'capabilities': [row['kind'] for row in chain],
                    'triples_added': 0, 'file_changed': False, 'world_model_updated': False}
        args.task_id = next_task_id(base)
    candidate, report = make_task(base, source, args.processes, rules, scene,
                                   args.task_id, args.external_preconditions_satisfied)
    if path.read_bytes() != original_bytes:
        raise ValueError('Source file changed during conversion; retry from current data.')
    backup = save_existing_file(path, candidate)
    report.update(status='generated', backup=str(backup), file_changed=True,
                  world_model_updated=False)
    return report


def selection_identifiers(tokens):
    """Accept 'ioc task_a and task_b' or the plain list 'task_a task_b'."""
    tokens = list(tokens)
    if tokens and tokens[0].lower() == 'ioc':
        tokens.pop(0)
    identifiers = []
    previous_and = False
    for token in tokens:
        if token.lower() == 'and':
            if not identifiers or previous_and:
                raise ValueError("'and' must appear between IOC identifiers.")
            previous_and = True
        else:
            identifiers.append(token)
            previous_and = False
    if not identifiers or previous_and:
        raise ValueError('Provide IOC identifiers; do not end the request with and.')
    return identifiers


def merge_request(args):
    """Resolve an exact IOC process selection to one existing execution task.

    An explicit --reuse-task associates an execution SCENARIO, not an assertion
    that a manually created old task was historically derived from this IOC.
    Only this metadata is added; no existing task parameters are rewritten.
    """
    workspace = args.workspace.expanduser().resolve()
    cask_path = (args.cask or workspace/'src/ontology/CASK+CRO-abox.ttl').expanduser().resolve()
    ioc_path = (args.ioc or workspace/'src/ontology/IP_Restage_IOC_Process.ttl').expanduser().resolve()
    # Optional conventional inputs keep the short merge command usable once
    # the user's execution rules and scene supplement have been configured.
    if args.rules is None:
        default_rules = workspace/'src/ontology/ioc_to_cask_rules.json'
        if default_rules.is_file() and not (workspace/'src/ontology/ioc-semantic-rules.json').is_file():
            args.rules = default_rules
    if args.scene is None:
        default_scene = workspace/'src/ontology/cask-scene-abox.ttl'
        if default_scene.is_file():
            args.scene = default_scene
    source = Graph().parse(str(ioc_path), format='turtle')
    processes = []
    for identifier in selection_identifiers(args.processes):
        process = URIRef(identifier)
        if (process, RDF.type, IOC.Process) not in source:
            matches = {s for s in source.subjects(RDF.type, IOC.Process)
                       if str(s).rsplit('#', 1)[-1].rsplit('/', 1)[-1] == identifier}
            if len(matches) != 1:
                raise ValueError(f'IOC identifier {identifier!r} has {len(matches)} matches; use its full URI.')
            process = next(iter(matches))
        if process in processes:
            raise ValueError('Duplicate IOC process: '+str(process))
        processes.append(process)
    scope = frozenset(processes)
    # Bind the complete selection, never each member separately. Sorting makes
    # an explicit association independent of argument order.
    selection = (processes[0] if len(scope) == 1 else
                 SCENE['selection_'+hashlib.sha256(
                     json.dumps(sorted(map(str, scope))).encode()).hexdigest()])
    original_bytes = cask_path.read_bytes()
    graph = Graph().parse(data=original_bytes, format='turtle')
    targets = set(graph.objects(selection, SCENE.mapsToTask))
    # Generated task provenance is eligible only if the complete source scope
    # matches the complete selection; a subset must not run the whole task.
    for task in graph.subjects(PROV.wasDerivedFrom, processes[0]):
        if (task, RDF.type, CSS.Process) in graph and set(graph.objects(task, PROV.wasDerivedFrom)) == scope:
            targets.add(task)
    confirmed = CONFIRMED_TASK_BINDINGS.get(frozenset(map(str, scope)))
    if confirmed is not None:
        confirmed_task = task_iri(confirmed)
        if targets and targets != {confirmed_task}:
            raise ValueError('Stored association conflicts with the user-confirmed mapping to '+confirmed)
        targets.add(confirmed_task)
    if args.reuse_task:
        task = task_iri(args.reuse_task)
        if targets and targets != {task}:
            raise ValueError('IOC process already maps to another task: '+', '.join(map(str, targets)))
        chain = ordered_requirements(graph, task)
        if not targets:
            candidate = clone(graph)
            candidate.add((selection, SCENE.mapsToTask, task))
            candidate.add((selection, SCENE.mappingScope, Literal('Explicit existing execution scenario reuse')))
            if len(scope) > 1:
                candidate.add((selection, RDF.type, SCENE.ProcessSelection))
                for process in processes:
                    candidate.add((selection, SCENE.includesProcess, process))
            if cask_path.read_bytes() != original_bytes:
                raise ValueError('CASK file changed during processing; retry.')
            return {'status': 'existing', 'task_id': args.reuse_task,
                    'source_process': ' + '.join(map(str, processes)),
                    'source_processes': list(map(str, processes)),
                    'capabilities': [row['kind'] for row in chain],
                    'association_added': False, 'source_cask_changed': False,
                    'task_data_changed': False, 'world_model_updated': False}
        targets = {task}
    if len(targets) > 1:
        raise ValueError('Multiple task associations found; resolve the mapping before executing.')
    if targets and any(graph.objects(next(iter(targets)), SCENE.executionProfile)):
        from ioc_semantic_conversion import merge_new
        return merge_new(sys.modules[__name__], args, graph, source,
                         list(map(str, processes)), cask_path)
    if targets:
        task = next(iter(targets))
        chain = ordered_requirements(graph, task)
        task_id = str(task).removeprefix(str(A))
        task_iri(task_id)
        return {'status': 'existing', 'task_id': task_id,
                'source_process': ' + '.join(map(str, processes)),
                'source_processes': list(map(str, processes)),
                'capabilities': [row['kind'] for row in chain], 'association_added': False,
                'task_data_changed': False, 'world_model_updated': False}
    if args.rules:
        raise ValueError('merge uses runtime-only execution profiles. Legacy --rules export writes a TTL and is not enabled in merge; configure ioc-semantic-rules.json instead.')
    semantic_rules = workspace/'src/ontology/ioc-semantic-rules.json'
    if semantic_rules.is_file() or Path(__file__).with_name('ioc_capability_conversion.py').is_file():
        from ioc_semantic_conversion import merge_new
        return merge_new(sys.modules[__name__], args, graph, source,
                         list(map(str, processes)), cask_path)
    available = []
    for task in sorted(set(graph.subjects(CSS.requiresCapability, None))):
        try:
            chain = ordered_requirements(graph, task)
        except ValueError:
            continue
        available.append(str(task).split('#')[-1]+': '+' -> '.join(r['kind'] for r in chain))
    raise ValueError('IOC processes exist, but no confirmed association for this EXACT selection was found. '
                     'First use --reuse-task TASK_ID to associate an existing execution scenario, '
                     'or provide --rules for a new conversion. No file was changed.\n'+
                     '\n'.join(available))


def merge_main(argv):
    parser = argparse.ArgumentParser(description='Merge an exact IOC process selection into one CASK task.')
    parser.add_argument('processes', nargs='+', help='ioc task_006 and task_007, or a list of IOC IDs.')
    parser.add_argument('--workspace', type=Path, default=Path.home()/'cask_gazebo_final_ws')
    parser.add_argument('--cask', type=Path)
    parser.add_argument('--ioc', type=Path)
    parser.add_argument('--reuse-task', help='One-time explicit association with an existing execution scenario.')
    parser.add_argument('--rules', type=Path)
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--task-id', help='Optional new task ID; otherwise allocate above occupied numeric IDs.')
    parser.add_argument('--preview', action='store_true',help='Validate/compose against source CASK without ROS, file writes or import.')
    parser.add_argument('--external-preconditions-satisfied', action='store_true')
    args = parser.parse_args(argv)
    try:
        result = merge_request(args)
        print('IOC processes: '+result['source_process'])
        if args.preview:
            print('Preview successful: '+result['task_id']+' | '+' -> '.join(result['capabilities']))
            for step in result.get('scene_config',{}).get('steps',[]):
                print(step['kind']+': '+str(len(step['same_kind_candidates']))+' capability/skill entries of the same type; execution has not been verified.')
            print('The original TTL, runtime graph and Gazebo were not modified. Preview task IDs come from the source CASK; merge uses the running World Model.')
            return 0
        if result.get('world_model_updated'):
            action = 'generated' if result['status'] == 'generated' else 'reused'
            print('Conversion successful: '+action+' '+result['task_id']+'; synchronized with the running World Model.')
            if result.get('source_cask_changed') is False:
                print('The original CASK ABox is unchanged; the new task is stored only in the runtime graph and runtime state.')
                print('Restarting the launch restores the initial tasks; run merge again after restarting.')
            c = result['scene_config']
            print('Requirement sequence: '+' -> '.join(result['capabilities']))
            if c['profile']=='pallet_transfer':
                print('Component: '+c['model']+' | simulated mass='+str(c['mass'])+' kg')
                print('Pickup pose: '+str(c['pickup'])+' | Intermediate storage pose: '+str(c['delivery']))
                print('A measured pose is available for the Gazebo component.')
            else:
                for step in c['steps']:
                    n=len(step['same_kind_candidates'])
                    print(step['required_capability']+' / '+step['kind']+': '+
                          ('No ProvidedCapability/Skill of the same type is currently available; the requirement has been retained.' if n==0 else
                           str(n)+' capability/skill entries of the same type; parameters and constraints are checked by the existing matching system.'))
                    if not step['parameters']:
                        print('  No explicit ROS parameters are configured; execution requires an existing semantic parameterizer for this capability.')
                print('Conversion does not establish that a robot can execute the task. This mode does not automatically add Gazebo entities or motion implementations.')
            if result.get('external_predecessors'):
                print('Predecessor processes outside the execution scope: '+', '.join(result['external_predecessors'])+'; not included in this simulation.')
            print('Robot selection is handled by the Task/Skill Manager. Copy the following commands to request execution:')
        elif result['status'] == 'existing':
            print('Conversion successful: task association confirmed; reusing '+result['task_id']+'.')
            print('Existing capability requirements, Input/Output relations and parameters were not regenerated or modified.')
            if result.get('association_added'):
                print('The initial association was recorded in the existing CASK; only mapping metadata for the selected scope was added.')
                print('Backup: '+result['association_backup'])
            print('If the task is loaded in the current World Model, copy and run:')
        else:
            print('Conversion successful: new requirements were added to the existing CASK TTL but have not been synchronized with the running World Model.')
            print('After loading the new data and ensuring that the skill server supports this task ID, run:')
        print(execute_command(result['task_id']))
        return 0
    except Exception as exc:
        print('Merge failed: '+str(exc), file=sys.stderr)
        return 2


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == 'merge':
        return merge_main(argv[1:])
    parser = argparse.ArgumentParser(description='Use existing CASK tasks; add only missing IOC-derived tasks.')
    parser.add_argument('--cask', required=True, type=Path)
    parser.add_argument('--task-id', help='Existing task ID to reuse, or ID for a new task.')
    parser.add_argument('--ioc', type=Path)
    parser.add_argument('--processes', nargs='+')
    parser.add_argument('--rules', type=Path)
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--external-preconditions-satisfied', action='store_true')
    args = parser.parse_args(argv)
    try:
        result = run(args)
        if result['status'] == 'existing':
            print('Conversion successful: '+result['task_id']+' already has complete requirement data and will be reused.')
            print('No new CASK TTL was generated; the existing file is unchanged.')
            print('If the existing file is loaded in the World Model, run:')
        else:
            print('Conversion successful: requirement data was added to the existing CASK TTL for '+result['task_id']+'.')
            print('Triples added: '+str(result['triples_added']))
            print('Original file backup: '+result['backup'])
            print('This review version has not updated the in-memory graph of the running World Model.')
            print('After loading the new data and ensuring that the skill server supports the task ID, run:')
        print('Capability sequence: '+' -> '.join(result['capabilities']))
        print(execute_command(result['task_id']))
        return 0
    except Exception as exc:
        print('Conversion failed: '+str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
