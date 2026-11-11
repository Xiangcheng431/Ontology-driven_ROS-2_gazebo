"""Bind existing robot controllers to task-independent physical cargo contexts.

This module chooses topics and attachments, not trajectories, gains or speeds.
All legacy task calls return to their original implementation unchanged.
"""
import json
import os
import re
import time
from pathlib import Path


def config_path(task):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',task): raise ValueError('Invalid task ID.')
    root=Path(os.environ.get('CASK_WORKSPACE',str(Path.home()/'cask_gazebo_final_ws')))
    return root/'runtime/ioc_task_scenes'/(task+'.json')


def configuration(task):
    path=config_path(task)
    if not path.is_file(): return None
    c=json.loads(path.read_text())
    if c.get('task_id')!=task or c.get('profile')!='pallet_transfer':
        raise ValueError('Invalid generated task configuration.')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',c['model']): raise ValueError('Invalid cargo model.')
    return c








def accepts_jekko(node,request):
    """Validate the lifting interface, not a task number."""
    if node.busy: return False
    names=request.parameter_names; values=request.parameter_values; units=request.parameter_units
    if not names or not(len(names)==len(values)==len(units)) or len(set(names))!=len(names): return False
    import math
    if not all(math.isfinite(v) for v in values): return False
    needed={prefix+'_'+axis for prefix in ('pickup','delivery','jekko_work') for axis in ('x','y','z','yaw')}|{'target_height'}
    return needed.issubset(names)

# CASK_ALL_ROBOT_CONTEXT_V1
# This extends the existing cargo-context module; it is not a motion controller.
_A = 'http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#'
_S = 'urn:cask:scene:'
_KNOWN_CARGO = {
    'task001_rebar': dict(legacy='Task_001', geometry='pallet',
        pose_topic='/cask/task001/cargo_pose', clearance_topic='/cask/task001/cargo_clearance',
        crane_topic='/cask/crane/cargo'),
    'task002_soil': dict(legacy='Task_002', geometry='soil_bin',
        pose_topic='/cask/task002/cargo_pose', clearance_topic='/cask/task002/cargo_clearance',
        contacts_topic='/cask/soil/contacts', crane_topic='/cask/crane/soil'),
    'task005_steel': dict(legacy='Task_005', geometry='steel_fixture',
        pose_topic='/cask/task005/cargo_pose', clearance_topic='/cask/task005/cargo_clearance',
        crane_topic='/cask/crane/task005', jekko_topic='/cask/jekko/task005'),
}
_LEGACY = {'crane': {'Task_001':'task001_rebar', 'Task_002':'task002_soil', 'Task_005':'task005_steel'},
           'jekko': {'Task_005':'task005_steel'}}


def accepts_numeric(node, request):
    """The endpoint defines the operation; a task number does not define ability."""
    import math
    names, values, units = request.parameter_names, request.parameter_values, request.parameter_units
    return (not node.busy and bool(names) and len(names) == len(values) == len(units)
            and len(set(names)) == len(names) and all(math.isfinite(v) for v in values))


def _local(value):
    return str(value).rsplit('#', 1)[-1].rsplit('/', 1)[-1]


def _runtime_profile(node, goal):
    """Read the live graph, never authorize a task from a stale runtime JSON."""
    from cask_interfaces.srv import QueryOntology
    from rclpy.callback_groups import ReentrantCallbackGroup
    task = goal.request.task_id
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', task):
        raise RuntimeError('INVALID_TASK_ID')
    if not hasattr(node, '_cargo_query_client'):
        node._cargo_query_client = node.create_client(QueryOntology, '/cask_world_model/query',
            callback_group=ReentrantCallbackGroup())
    client = node._cargo_query_client
    if not client.wait_for_service(timeout_sec=2.):
        raise RuntimeError('WORLD_MODEL_NOT_READY: cargo context query unavailable')
    request = QueryOntology.Request()
    request.query = 'SELECT ?profile WHERE { <'+_A+task+'> <'+_S+'executionProfile> ?profile }'
    future = client.call_async(request)
    deadline = time.monotonic()+5.
    # The skill servers run a MultiThreadedExecutor. The query uses a
    # reentrant group; do not recursively spin the executing action's node.
    while not future.done():
        if goal.is_cancel_requested:
            future.cancel(); raise RuntimeError('CANCELED')
        if time.monotonic() > deadline:
            future.cancel(); raise RuntimeError('WORLD_MODEL_QUERY_TIMEOUT')
        time.sleep(.02)
    result = future.result()
    if not result.success: raise RuntimeError(result.message)
    rows = json.loads(result.result_json)
    if len(rows) != 1:
        raise RuntimeError('EXECUTION_CONTEXT_MISSING: task needs one runtime execution profile')
    profile = json.loads(rows[0]['profile'])
    if profile.get('task_id') != task: raise RuntimeError('EXECUTION_PROFILE_MISMATCH')
    return profile


def resolve_context(node, goal, robot):
    request = goal.request
    if dict(zip(request.parameter_names, request.parameter_values)).get('final_scene') == 1.:
        return _validate_context(dict(model='final_steel', geometry='steel_fixture',
            pose_topic='/cask/final/cargo_pose',clearance_topic='/cask/final/cargo_clearance',
            crane_topic='/cask/crane/final_steel',jekko_topic='/cask/jekko/final_steel',
            stable_attachment_key='final_steel',native_state_bridged=True),robot)
    model = _LEGACY[robot].get(request.task_id)
    if model is not None:
        return dict(_KNOWN_CARGO[model], model=model, original=True)
    profile = _runtime_profile(node, goal)
    if profile.get('profile') == 'pallet_transfer':
        if _local(request.required_capability_id) != _local(profile['required_capability']):
            raise RuntimeError('REQUIRED_CAPABILITY_CONTEXT_MISMATCH')
        if robot != 'jekko':
            raise RuntimeError('EXECUTION_CONTEXT_MISSING: pallet profile has no '+robot+' interface')
        return _validate_context(dict(model=profile['model'], geometry='pallet',
            pose_topic='/cask/'+request.task_id.lower()+'/cargo_pose',
            jekko_topic='/cask/jekko/'+request.task_id.lower(), native_state_bridged=True), robot)
    if profile.get('profile') != 'capability_sequence':
        raise RuntimeError('UNKNOWN_EXECUTION_PROFILE')
    steps = [s for s in profile['steps'] if _local(s['required_capability']) == _local(request.required_capability_id)]
    if len(steps) != 1: raise RuntimeError('REQUIRED_CAPABILITY_CONTEXT_MISMATCH')
    resource = {'crane':'Crane_01', 'jekko':'Jekko_01'}[robot]
    contexts = [c for c in steps[0].get('execution_contexts', [])
                if ('resource' not in c or _local(c['resource']).casefold() == resource.casefold())
                and ('skill' not in c or _local(c['skill']) == _local(request.skill_id))
                and ('provided_capability' not in c or _local(c['provided_capability']) == _local(request.provided_capability_id))]
    # Specific resource/skill bindings override a shared cargo context.
    if contexts:
        rank = max(sum(k in c for k in ('resource','skill','provided_capability')) for c in contexts)
        contexts = [c for c in contexts if sum(k in c for k in ('resource','skill','provided_capability')) == rank]
    if len(contexts) != 1:
        raise RuntimeError('EXECUTION_CONTEXT_MISSING_OR_AMBIGUOUS: specify scene:executionContext for '+resource)
    return _validate_context(contexts[0], robot)


def _validate_context(context, robot):
    context = dict(context)
    model = context.get('model', '')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', model): raise RuntimeError('INVALID_CARGO_MODEL')
    known = _KNOWN_CARGO.get(model, {})
    if any(k.endswith('_topic') and k in context and context[k] != v for k,v in known.items()):
        known = {k:v for k,v in known.items() if k != 'legacy'}
    context = dict(known, **context)
    for key, value in context.items():
        if key.endswith('_topic') and (not isinstance(value, str) or not re.fullmatch(r'/[A-Za-z0-9_/]+', value)):
            raise RuntimeError('INVALID_CARGO_TOPIC: '+key)
    if not context.get('pose_topic'): raise RuntimeError('CARGO_POSE_CONTEXT_MISSING')
    geometries = {'jekko':('pallet','steel_fixture'), 'crane':('pallet','soil_bin','steel_fixture')}
    if context.get('geometry') not in geometries[robot]:
        raise RuntimeError('CARGO_GEOMETRY_INTERFACE_UNSUPPORTED: '+robot+' requires a compatible modeled handling interface')
    if robot == 'crane' and not context.get('clearance_topic'):
        raise RuntimeError('CARGO_CLEARANCE_CONTEXT_MISSING: measured clearance is required')
    return context


def _subscriptions(node):
    if not hasattr(node, '_execution_subscriptions'): node._execution_subscriptions = {}
    return node._execution_subscriptions


def _subscribe(node, key, msg_type, topic, callback):
    subscriptions = _subscriptions(node)
    if key not in subscriptions:
        subscriptions[key] = node.create_subscription(msg_type, topic, callback, 10)


def _set_contacts(node, context, required=False):
    from cask_handoff.contact import SupportContacts
    if not hasattr(node, '_execution_contacts'):
        node._execution_contacts = {'task002_soil':node.handoff_contacts}
    topic = context.get('contacts_topic')
    if not topic:
        if required: raise RuntimeError('CARGO_CONTACT_CONTEXT_MISSING')
        # Do not retain evidence from a previously selected cargo.
        node.handoff_contacts = _NoContacts()
        return
    key = (context['model'], topic)
    if context['model'] == 'task002_soil' and topic == '/cask/soil/contacts':
        node.handoff_contacts = node._execution_contacts['task002_soil']; return
    if key not in node._execution_contacts:
        node._execution_contacts[key] = SupportContacts(node, context['model'], topic)
    node.handoff_contacts = node._execution_contacts[key]


class _NoContacts:
    def supported(self, *args): return False
    def status(self, *args): return 'CARGO_CONTACT_CONTEXT_MISSING'


def _attachment_bridge(node, topic):
    import atexit
    import subprocess
    if not hasattr(node, '_execution_bridges'): node._execution_bridges = {}
    if topic in node._execution_bridges and node._execution_bridges[topic].poll() is None: return
    process = subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
        topic+'/state@std_msgs/msg/String[ignition.msgs.StringMsg'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node._execution_bridges[topic] = process
    def close():
        if process.poll() is None:
            process.terminate()
            try: process.wait(timeout=1.)
            except subprocess.TimeoutExpired: process.kill()
    atexit.register(close)


def select_for_goal(node, goal, robot):
    """Resolve identity first, then invoke the robot's existing controller."""
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    context = resolve_context(node, goal, robot)
    node.execution_context = context
    node.execution_context_task = goal.request.task_id
    legacy = context.get('legacy')
    if robot == 'crane':
        if legacy:
            node.select_cargo(legacy)
        else:
            from .gazebo_attachment import NativeAttachment
            key = context.get('stable_attachment_key') or goal.request.task_id+'__'+_local(goal.request.required_capability_id)
            topic = context.get('crane_topic', '/cask/crane/'+context['model'])
            if key not in node.native_attachments:
                node.native_attachments[key] = NativeAttachment(node.get_parameter('gazebo_world').value,
                    child_model=context['model'], topic=topic)
                for kind, typ, name in (('pose', PoseStamped, context['pose_topic']),
                        ('clearance', String, context['clearance_topic']), ('native', String, topic+'/state')):
                    _subscribe(node, ('crane',key,kind), typ, name,
                        lambda msg,k=key,t=kind:node.cargo_message(k,t,msg))
                if not context.get('native_state_bridged', False):
                    _attachment_bridge(node, topic)
            node.select_cargo(key)
        _set_contacts(node, context, dict(zip(goal.request.parameter_names, goal.request.parameter_values)).get('handoff_mode') == 1.)
        return
    if robot == 'jekko':
        return _select_jekko_context(node, goal, context)
    raise RuntimeError('Unsupported cargo-handling robot: '+robot)


def _select_jekko_context(node, goal, context):
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from .gazebo_attachment import NativeAttachment
    task = context.get('stable_attachment_key') or ('Task_005' if context.get('legacy') == 'Task_005' else goal.request.task_id+'__'+_local(goal.request.required_capability_id))
    topic = context.get('jekko_topic', '/cask/jekko/'+context['model'])
    if getattr(node,'generated_selected_task','Task_005') == task: return
    if node.loaded: raise RuntimeError('LOAD_ALREADY_ATTACHED')
    if not hasattr(node,'generated_attachment_contexts'):
        node.generated_attachment_contexts = {'Task_005':node.attachment}
    node.attachment.close()
    if task not in node.generated_attachment_contexts:
        node.generated_attachment_contexts[task] = NativeAttachment(child_model=context['model'], topic=topic,
            parent_model='jekko',parent_link='jekko_cargo_attachment_link')
    node.attachment = node.generated_attachment_contexts[task]
    node.generated_selected_task = task
    node.cargo = None; node.cargo_time = 0.; node.native_state = None; node.native_revision = 0
    # Task005's original subscriptions are restored, not duplicated.
    if task == 'Task_005': return
    _subscribe(node, ('jekko',task,'pose'), PoseStamped, context['pose_topic'],
        lambda msg:node.on_cargo(msg) if node.generated_selected_task == task else None)
    _subscribe(node, ('jekko',task,'native'), String, topic+'/state',
        lambda msg:node.on_native(msg) if node.generated_selected_task == task else None)
    if not context.get('native_state_bridged', False): _attachment_bridge(node, topic)


def select_jekko(node, goal):
    return select_for_goal(node, goal, 'jekko')


def has_selected_context(node, goal):
    return getattr(node, 'execution_context_task', None) == goal.request.task_id




def uses_soil_interface(node):
    return (node.active_cargo_task == 'Task_002' or
            getattr(node, 'execution_context', {}).get('geometry') == 'soil_bin')
