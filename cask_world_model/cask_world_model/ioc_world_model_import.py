"""Atomic additive import through the existing World Model service type."""
import argparse
import json
import os
import re
import time
from pathlib import Path

from rdflib import BNode, Graph, Literal, Namespace, RDF, URIRef

A=Namespace('http://www.semanticweb.org/lixiangcheng/CASK+CRO-abox#')
S=Namespace('urn:cask:scene:')
DIN=Namespace('http://www.w3id.org/hsu-aut/DINEN61360#')


def graph_from_rows(rows):
    snapshot=Graph()
    for row in rows:
        subject=BNode(row['s']) if row.get('sb')=='true' else URIRef(row['s'])
        if row.get('lit')=='true':
            datatype=row.get('dt')
            if datatype=='http://www.w3.org/2001/XMLSchema#string': datatype=None
            obj=Literal(row['o'],lang=row.get('lang') or None,
                        datatype=URIRef(datatype) if datatype and not row.get('lang') else None)
        else:
            obj=BNode(row['o']) if row.get('ob')=='true' else URIRef(row['o'])
        snapshot.add((subject,URIRef(row['p']),obj))
    return snapshot


def import_task(node,request,response):
    """Only this opt-in operation enters this branch; legacy edits are untouched."""
    from .ioc_semantic_conversion import atomic_json
    try:
        incoming=Graph().parse(data=request.object,format='turtle')
        task=URIRef(request.subject)
        if not re.fullmatch(re.escape(str(A))+r'Task_[A-Za-z0-9_]+',str(task)):
            raise ValueError('Invalid task URI.')
        configs=set(incoming.objects(task,S.executionProfile))
        if len(configs)!=1: raise ValueError('Missing unique generated execution profile.')
        config=json.loads(str(next(iter(configs))))
        if str(A[config['task_id']])!=str(task) or config['profile'] not in ('pallet_transfer','capability_sequence'):
            raise ValueError('Task profile does not match import request.')
        generic=config['profile']=='capability_sequence'
        if generic:
            from .ioc_to_cask import ordered_requirements
            chain=ordered_requirements(incoming,task)
            steps=config.get('steps',[])
            expected=[(str(r['required']).split('#')[-1],r['kind'],str(r['input']),str(r['output'])) for r in chain]
            supplied=[(s['required_capability'],s['kind'],s['input_state'],s['output_state']) for s in steps]
            if expected!=supplied:raise ValueError('Execution profile differs from generated RDF requirements.')
        with node.graph_lock:
            old=node.graph
            stored=set(old.objects(task,S.executionProfile))
            if any(old.triples((task,None,None))) and not stored:
                raise ValueError('Task ID is already occupied in the running World Model; no overwrite.')
            if stored and any(json.loads(str(v))!=config for v in stored):
                raise ValueError('Existing task profile differs; no overwrite.')
            candidate=Graph()
            for prefix,namespace in old.namespaces(): candidate.bind(prefix,namespace)
            candidate+=old
            old_subjects=set(old.subjects())
            material_props=set() if generic else {p for p in incoming.subjects(DIN.has_Type_Description,A.ProductKind_TD)
                            if any(str(v)==config['material'] for d in incoming.objects(p,DIN.has_Instance_Description)
                                   for v in incoming.objects(d,DIN.Value))}
            for s,p,o in incoming:
                if s not in old_subjects or (p==DIN.has_Data_Element and o in material_props):
                    candidate.add((s,p,o))
            node.graph=candidate
            try:
                node.save_runtime_world_model()
                workspace=Path(node.template_path).resolve().parents[2]
                folder='ioc_task_plans' if generic else 'ioc_task_scenes'
                atomic_json(workspace/'runtime'/folder/(config['task_id']+'.json'),config)
            except Exception:
                node.graph=old
                node.save_runtime_world_model()
                raise
        response.success=True
        response.message='Generated task imported and runtime graph saved: '+config['task_id']
    except Exception as error:
        response.success=False; response.message=str(error)
    return response


def main():
    import rclpy
    from rclpy.node import Node
    from cask_interfaces.srv import ModifyWorldModel,QueryOntology
    parser=argparse.ArgumentParser()
    modes=parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--import-file',type=Path)
    modes.add_argument('--snapshot-file',type=Path)
    parser.add_argument('--task-id'); args=parser.parse_args()
    rclpy.init(); node=Node('ioc_task_import_client')
    def call(client,request,timeout):
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done(): raise RuntimeError('World Model service timeout.')
        response=future.result()
        if not response.success: raise RuntimeError(response.message)
        return response
    try:
        if args.snapshot_file is not None:
            query=node.create_client(QueryOntology,'/cask_world_model/query')
            if not query.wait_for_service(timeout_sec=3.):
                raise RuntimeError('Start the Gazebo framework before merging a new task.')
            request=QueryOntology.Request()
            request.query='''SELECT ?s ?p ?o (isLiteral(?o) AS ?lit)
                (isBlank(?s) AS ?sb) (isBlank(?o) AS ?ob)
                (datatype(?o) AS ?dt) (lang(?o) AS ?lang)
                WHERE { ?s ?p ?o }'''
            rows=json.loads(call(query,request,8.).result_json)
            snapshot=graph_from_rows(rows)
            snapshot.serialize(args.snapshot_file,format='turtle')
            print('Read running World Model snapshot.'); return 0
        if not args.task_id: raise ValueError('--task-id is required for import.')
        client=node.create_client(ModifyWorldModel,'/cask_world_model/modify')
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Start the Gazebo framework launch before merging a new task.')
        request=ModifyWorldModel.Request()
        request.operation='import_task'; request.subject=str(A[args.task_id])
        request.object=args.import_file.read_text(); request.object_is_literal=True
        request.save_to_file=True
        response=call(client,request,8.)
        incoming=Graph().parse(args.import_file,format='turtle')
        config=json.loads(str(next(incoming.objects(A[args.task_id],S.executionProfile))))
        if config['profile']=='capability_sequence':
            # Ontology conversion does not require a robot or a new Gazebo
            # entity. Existing matching decides whether execution is possible.
            print(response.message+'; capability sequence imported (execution not asserted).')
            return 0
        query=node.create_client(QueryOntology,'/cask_world_model/query')
        if not query.wait_for_service(timeout_sec=2.): raise RuntimeError('World Model query unavailable.')
        # Only advertise an execution command after Component Manager has a
        # measured pose from the newly spawned physical object and bridge.
        until=time.monotonic()+20.
        while time.monotonic()<until:
            check=QueryOntology.Request()
            check.query=f'SELECT ?ready WHERE {{ <{A[args.task_id]}> <{S.gazeboReady}> ?ready }}'
            rows=json.loads(call(query,check,2.).result_json)
            if any(str(r.get('ready')).lower()=='true' for r in rows):
                print(response.message+'; measured Gazebo component ready.'); return 0
            rclpy.spin_once(node,timeout_sec=.2)
        raise RuntimeError('Task imported, but generated component is not ready. Check Component Manager output; retry merge after it is ready.')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
