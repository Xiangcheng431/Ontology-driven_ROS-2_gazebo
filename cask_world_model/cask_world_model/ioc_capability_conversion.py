"""Type-independent IOC requirement composition; no robot or trajectory selection.

Meaning comes from structured process-kind facts, exact known capability names,
or configurable aliases. This is not an unrestricted natural-language planner.
"""
import json
import math
import re
from rdflib import Graph, Literal, Namespace, OWL, RDF, RDFS, URIRef

A=Namespace('http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#')
S=Namespace('urn:cask:scene:')
CSS=Namespace('http://www.w3id.org/hsu-aut/css#')
CASK=Namespace('http://www.w3id.org/hsu-aut/cask#')
VDI=Namespace('http://www.w3id.org/hsu-aut/VDI3682#')
DIN=Namespace('http://www.w3id.org/hsu-aut/DINEN61360#')
PROV=Namespace('http://www.w3.org/ns/prov#')
POSE=('hasPositionX','hasPositionY','hasPositionZ','hasOrientationYaw','hasReferenceFrame')


def one(g,s,p,optional=False):
    values=set(g.objects(s,p))
    if optional and not values:return None
    if len(values)!=1:raise ValueError(f'Expected one {p} for {s}; got {len(values)}')
    return next(iter(values))


def process_kinds(g):
    """Discover kind vocabulary from both requirements AND provided assurances."""
    result=set()
    for prop in g.subjects(DIN.has_Type_Description,A.ProcessKind_TD):
        for desc in g.objects(prop,DIN.has_Instance_Description):
            result.update(str(v) for v in g.objects(desc,DIN.Value))
    return result


def descriptors(g,subject):
    return {str(v) for prop in g.objects(subject,CSS.isSpecifiedBy)
            if A.ProcessKind_TD in set(g.objects(prop,DIN.has_Type_Description))
            for desc in g.objects(prop,DIN.has_Instance_Description)
            for v in g.objects(desc,DIN.Value)}


def recognize(source,process,catalog,aliases):
    explicit=set(str(v) for v in source.objects(process,S.processKind))|descriptors(source,process)
    if explicit:
        if len(explicit)!=1:raise ValueError('Ambiguous structured process kind: '+str(process))
        name=next(iter(explicit))
        matches={k for k in catalog|set(aliases) if k.casefold()==name.casefold()}
        return [next(iter(matches)) if len(matches)==1 else name]
    patterns={kind:list(values) for kind,values in aliases.items()}
    for kind in catalog:
        words=re.sub(r'(?<=[a-z])(?=[A-Z])',' ',kind)
        patterns.setdefault(kind,[]).extend([r'\b'+re.escape(kind)+r'\b',r'\b'+re.escape(words)+r'\b'])
    # Prefer labels. Comments are a fallback, rather than a source of an
    # additional unrequested operation mentioned in a narrative aside.
    for predicate in (RDFS.label,RDFS.comment):
        alternatives=[]
        for description in source.objects(process,predicate):
            text=str(description);found={}
            for kind,expressions in patterns.items():
                positions=[m.start() for pattern in expressions for m in re.finditer(pattern,text,re.I)]
                if positions:found[kind]=min(positions)
            if not found:continue
            # Do not silently drop an unrecognized conjunct such as
            # "lift, align and permanently fix".
            clauses=[p.strip() for p in re.split(r'\bthen\b|\band\b|[,;]',text,flags=re.I) if p.strip()]
            if len(clauses)>1 and any(not any(re.search(p,c,re.I) for ps in patterns.values() for p in ps) for c in clauses):
                raise ValueError('Compound IOC description has an unmapped clause; annotate scene:processKind or add a semantic alias: '+str(process))
            ordered=tuple(k for k,pos in sorted(found.items(),key=lambda item:(item[1],item[0])))
            if len(set(found.values()))!=len(found):raise ValueError('Ambiguous overlapping capability meanings: '+str(process))
            alternatives.append(ordered)
        if alternatives:
            if len(set(alternatives))!=1:raise ValueError('Descriptions disagree about capability order: '+str(process))
            return list(alternatives[0])
    raise ValueError('Unknown IOC meaning: '+str(process)+'. Add scene:processKind or capability_aliases; no Lifting fallback.')


def select_scene(scene,source,process,kind):
    exact={s for s in scene.subjects(S.sourceProcess,process)
           if not set(scene.objects(s,S.processKind)) or kind in set(map(str,scene.objects(s,S.processKind)))}
    candidates=exact or {s for s,value in scene.subject_objects(S.processKind)
                         if str(value)==kind and not set(scene.objects(s,S.sourceProcess))}
    if len(candidates)>1:raise ValueError('Ambiguous Scene binding for '+kind+' / '+str(process))
    if candidates:return scene,next(iter(candidates))
    if set(source.objects(process,VDI.hasInput)) and set(source.objects(process,VDI.hasOutput)):
        return source,process
    raise ValueError('Missing Scene input/output binding for '+kind+' / '+str(process)+'. Add a scene:CapabilityScenario with pickup/delivery or inputState/outputState; no coordinates invented.')


def parameters(scene,scenario):
    result=[];seen=set()
    for item in sorted(scene.objects(scenario,S.parameter),key=str):
        name=str(one(scene,item,S.parameterName));value=float(one(scene,item,S.value))
        unit=str(one(scene,item,S.unit,True) or '')
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',name) or not math.isfinite(value):
            raise ValueError('Invalid numeric ROS skill parameter: '+name)
        entry=dict(name=name,value=value,unit=unit)
        for pred,key in ((S.forProvidedCapability,'provided_capability'),(S.forResource,'resource'),(S.forSkill,'skill')):
            v=one(scene,item,pred,True)
            if v is not None:entry[key]=str(v)
        identity=tuple(entry.get(k) for k in ('name','provided_capability','resource','skill'))
        if identity in seen:raise ValueError('Duplicate skill parameter binding: '+name)
        seen.add(identity);result.append(entry)
    return result


def execution_contexts(scene, scenario):
    """Physical identity facts, independent of process kind and task number.

    No topic/model is guessed from an arbitrary task ID. Numeric motion/tool
    targets remain in scene:parameter; this carries their physical cargo binding.
    Missing bindings do not prevent generating an unmatched semantic task.
    """
    result=[]
    fields=((S.gazeboModelName,'model'),(S.geometryProfile,'geometry'),
            (S.cargoPoseTopic,'pose_topic'),(S.cargoClearanceTopic,'clearance_topic'),
            (S.cargoContactsTopic,'contacts_topic'),(S.craneAttachmentTopic,'crane_topic'),
            (S.jekkoAttachmentTopic,'jekko_topic'),(S.forResource,'resource'),
            (S.forSkill,'skill'),(S.forProvidedCapability,'provided_capability'))
    for item in sorted(scene.objects(scenario,S.executionContext),key=str):
        entry={}
        for predicate,key in fields:
            value=one(scene,item,predicate,True)
            if value is not None:entry[key]=str(value)
        clear=one(scene,item,S.clearProcessingArea,True)
        if clear is not None:
            if str(clear).lower() not in ('true','false','1','0'):raise ValueError('clearProcessingArea must be boolean')
            entry['clear_processing_area']=str(clear).lower() in ('true','1')
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',entry.get('model','')):
            raise ValueError('Execution context needs a valid scene:gazeboModelName')
        for key,value in entry.items():
            if key.endswith('_topic') and not re.fullmatch(r'/[A-Za-z0-9_/]+',value):
                raise ValueError('Invalid execution context topic: '+key)
        result.append(entry)
    return result


def compose_generic(api,args,base,source,identifiers,data_element):
    ordered,external=api.select_processes(source,identifiers,True)
    scene_path=args.scene or args.workspace.expanduser()/'src/ontology/cask-scene-abox.ttl'
    scene=Graph().parse(scene_path,format='turtle') if scene_path.is_file() else Graph()
    rules_path=args.workspace.expanduser()/'src/ontology/ioc-semantic-rules.json'
    rules=json.loads(rules_path.read_text()) if rules_path.is_file() else {}
    aliases=rules.get('capability_aliases',{})
    if not isinstance(aliases,dict) or any(not isinstance(v,list) or not all(isinstance(p,str) for p in v) for v in aliases.values()):
        raise ValueError('capability_aliases must map kind names to regex lists.')
    # Structured annotations may be supplied in the supplemental Scene ABox.
    semantic=api.clone(source);semantic+=scene
    catalog=process_kinds(base)
    operations=[(p,k) for p in ordered for k in recognize(semantic,p,catalog,aliases)]
    task_id=getattr(args,'task_id',None) or api.next_task_id(base);task=api.task_iri(task_id)
    if any(base.triples((task,None,None))):raise ValueError('Target task ID is occupied: '+task_id)
    if any(str(s).startswith(str(task)+'_') for s in set(base.subjects())|set(scene.subjects())):
        raise ValueError('Generated task namespace is occupied by existing state/requirement data: '+task_id)
    result=api.merge_scene(base,scene)
    for typ in (OWL.NamedIndividual,CSS.Process):result.add((task,RDF.type,typ))
    steps=[];previous=None
    def copy_state(graph,prototype,target,scenario,index,side):
        if not any(graph.triples((prototype,None,None))):raise ValueError('Missing state data: '+str(prototype))
        for predicate,value in graph.predicate_objects(prototype):
            if predicate==DIN.has_Data_Element:
                prop=A[f'{task_id}_Step{index:03d}_{side}_Property{len(list(result.objects(target,DIN.has_Data_Element))):03d}']
                td=one(graph,value,DIN.has_Type_Description)
                descriptions=list(graph.objects(value,DIN.has_Instance_Description))
                if not descriptions:raise ValueError('Missing state parameter value: '+str(value))
                result.add((prop,RDF.type,DIN.Data_Element));result.add((prop,DIN.has_Type_Description,td))
                for p,o in graph.predicate_objects(value):
                    if p not in (DIN.has_Instance_Description,DIN.has_Type_Description):result.add((prop,p,o))
                for j,description in enumerate(descriptions):
                    desc=URIRef(str(prop)+f'_ID{j}')
                    result.add((prop,DIN.has_Instance_Description,desc))
                    for p,o in graph.predicate_objects(description):
                        result.add((desc,p,Literal('Requirement') if p==DIN.Expression_Goal else o))
                    result.add((desc,DIN.Expression_Goal,Literal('Requirement')))
                result.add((target,DIN.has_Data_Element,prop))
            elif predicate in {A[n] for n in POSE}:
                existing=set(result.objects(target,predicate))
                if existing:
                    equal=all(str(v)==str(value) if predicate==A.hasReferenceFrame else float(v)==float(value) for v in existing)
                    if not equal:raise ValueError('Adjacent IOC operations have disconnected scene states: '+str(target))
                else:result.add((target,predicate,value))
            elif predicate==RDF.type:
                if value==VDI.Product:result.add((target,predicate,value))
            elif predicate==RDFS.label:result.add((target,predicate,value))
        result.add((target,RDF.type,VDI.Product))
        result.add((target,RDF.type,OWL.NamedIndividual))
        # Material is an explicit Scene fact, never a guessed robot assurance.
        material=one(graph,scenario,S.material,True)
        if material is not None and not any(A.ProductKind_TD in set(result.objects(p,DIN.has_Type_Description)) for p in result.objects(target,DIN.has_Data_Element)):
            prop=data_element(result,f'{task_id}_Step{index:03d}_{side}_Material',A.ProductKind_TD,str(material),'Requirement')
            result.add((target,DIN.has_Data_Element,prop))
        api.validate_state(result,target)
    for index,(process,kind) in enumerate(operations,1):
        graph,scenario=select_scene(scene,source,process,kind)
        at=one(graph,scenario,S.at,True)
        before=one(graph,scenario,S.inputState,True) or one(graph,scenario,S.pickup,True) or one(graph,scenario,VDI.hasInput,True) or at
        after=one(graph,scenario,S.outputState,True) or one(graph,scenario,S.delivery,True) or one(graph,scenario,VDI.hasOutput,True) or at
        if before is None or after is None:raise ValueError('Scene binding needs input/output or at: '+str(scenario))
        # Include the IOC-state closure when the source itself supplies states.
        if graph is source:result+=source
        input_state=previous or A[f'{task_id}_State_000']
        output_state=A[f'{task_id}_State_{index:03d}']
        copy_state(graph,before,input_state,scenario,index,'Input')
        copy_state(graph,after,output_state,scenario,index,'Output')
        required=A[f'{task_id}_Required_{index:03d}']
        existing={p for p in result.subjects(DIN.has_Type_Description,A.ProcessKind_TD)
                  for d in result.objects(p,DIN.has_Instance_Description)
                  if any(str(v)=='Requirement' for v in result.objects(d,DIN.Expression_Goal))
                  and kind in set(map(str,result.objects(d,DIN.Value)))}
        canonical=A[kind+'_Requirement_Property']
        if existing:
            process_prop=canonical if canonical in existing else sorted(existing,key=str)[0]
        else:
            terms={v for p in base.subjects(DIN.has_Type_Description,A.ProcessKind_TD)
                   for d in base.objects(p,DIN.has_Instance_Description)
                   for v in base.objects(d,DIN.Value) if str(v)==kind} or {Literal(kind)}
            # Retain registered RDF literal datatypes/languages so the current
            # Task Manager's shared ?process_kind variable actually joins.
            terms=sorted(terms,key=lambda v:v.n3())
            name=f'{task_id}_ProcessKind_{index:03d}'
            process_prop=data_element(result,name,A.ProcessKind_TD,terms[0],'Requirement')
            for number,value in enumerate(terms[1:],1):
                desc=A[name+f'_Variant{number}']
                result.add((process_prop,DIN.has_Instance_Description,desc))
                for p,v in ((RDF.type,DIN.Instance_Description),(DIN.Instance_Description_has_Type,A.ProcessKind_TD),
                            (DIN.Expression_Goal,Literal('Requirement')),(DIN.Logic_Interpretation,Literal('=')),(DIN.Value,value)):
                    result.add((desc,p,v))
        for typ in (OWL.NamedIndividual,CASK.RequiredCapability,VDI.ProcessOperator):result.add((required,RDF.type,typ))
        for p,v in ((CSS.isSpecifiedBy,process_prop),(VDI.hasInput,input_state),(VDI.hasOutput,output_state),(PROV.wasDerivedFrom,process)):
            result.add((required,p,v))
        result.add((task,CSS.requiresCapability,required));result.add((task,VDI.consistsOf,required));result.add((task,PROV.wasDerivedFrom,process))
        candidates=[]
        for provided in set(base.subjects(RDF.type,CASK.ProvidedCapability)):
            if kind not in descriptors(base,provided):continue
            for resource in base.subjects(CSS.providesCapability,provided):
                for skill in base.objects(provided,CSS.isRealizedBySkill):
                    candidates.append(dict(provided_capability=str(provided),resource=str(resource),skill=str(skill)))
        steps.append(dict(required_capability=str(required).split('#')[-1],kind=kind,input_state=str(input_state),output_state=str(output_state),
                          parameters=parameters(graph,scenario),execution_contexts=execution_contexts(graph,scenario),
                          same_kind_candidates=candidates,source_process=str(process)))
        previous=output_state
    config=dict(version=2,profile='capability_sequence',task_id=task_id,source_processes=list(map(str,ordered)),steps=steps)
    result.add((task,S.executionProfile,Literal(json.dumps(config,sort_keys=True))))
    chain=api.ordered_requirements(result,task)
    if [r['kind'] for r in chain]!=[s['kind'] for s in steps]:raise ValueError('Generated chain differs from IOC order.')
    if set(base)-set(result):raise ValueError('Conversion would remove existing facts.')
    return result,config,True,external
