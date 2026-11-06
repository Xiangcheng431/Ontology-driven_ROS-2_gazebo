import json
import os
import threading

import rclpy
from rclpy.node import Node
from rdflib import Graph, Literal, OWL, RDF, RDFS, URIRef

from cask_interfaces.srv import ModifyWorldModel, QueryOntology


class CaskWorldModel(Node):

    RESET_MODE = 'reset'
    RESUME_MODE = 'resume'

    def __init__(self):
        super().__init__('cask_world_model')

        default_template_path = os.path.expanduser(
            '~/cask_gazebo_final_ws/src/ontology/CASK+CRO-abox.ttl'
        )
        default_runtime_path = os.path.expanduser(
            '~/cask_gazebo_final_ws/runtime/CASK+CRO-runtime.ttl'
        )

        self.declare_parameter(
            'ontology_template_path',
            default_template_path,
        )
        self.declare_parameter(
            'runtime_ontology_path',
            default_runtime_path,
        )
        self.declare_parameter(
            'world_state_mode',
            self.RESET_MODE,
        )

        self.template_path = self.parameter_string(
            'ontology_template_path'
        )
        self.runtime_path = self.parameter_string(
            'runtime_ontology_path'
        )
        self.world_state_mode = self.parameter_string(
            'world_state_mode'
        ).strip().lower()

        if self.world_state_mode not in {
            self.RESET_MODE,
            self.RESUME_MODE,
        }:
            raise ValueError(
                'world_state_mode must be "reset" or "resume"'
            )

        self.declare_parameter('enable_ioc_import', False)
        self.graph = Graph()
        self.graph_lock = threading.RLock()
        self.loaded_path = self.select_load_path()

        try:
            self.graph.parse(self.loaded_path, format='turtle')
        except Exception as error:
            self.get_logger().fatal(
                f'Failed to load ontology: {error}'
            )
            raise

        individuals = set(
            self.graph.subjects(RDF.type, OWL.NamedIndividual)
        )

        self.query_service = self.create_service(
            QueryOntology,
            '/cask_world_model/query',
            self.query_ontology_callback,
        )

        self.modify_service = self.create_service(
            ModifyWorldModel,
            '/cask_world_model/modify',
            self.modify_world_model_callback,
        )

        self.get_logger().info(
            f'World-state mode: {self.world_state_mode}'
        )
        self.get_logger().info(
            f'Ontology loaded: {self.loaded_path}'
        )
        self.get_logger().info(
            f'Runtime save path: {self.runtime_path}'
        )
        self.get_logger().info(
            f'Number of triples: {len(self.graph)}'
        )
        self.get_logger().info(
            f'Number of named individuals: {len(individuals)}'
        )
        self.get_logger().info(
            'Query service: /cask_world_model/query'
        )
        self.get_logger().info(
            'Modify service: /cask_world_model/modify'
        )
        self.get_logger().info('CaSk world model is running')

    def parameter_string(self, name):
        return os.path.expanduser(
            self.get_parameter(name)
            .get_parameter_value()
            .string_value
        )

    def select_load_path(self):
        if self.world_state_mode == self.RESUME_MODE:
            if os.path.isfile(self.runtime_path):
                return self.runtime_path

            self.get_logger().warning(
                'Resume mode requested, but no runtime ontology '
                f'exists at {self.runtime_path}; loading the '
                'template instead'
            )

        return self.template_path

    def query_ontology_callback(self, request, response):
        try:
            with self.graph_lock:
                query_result = self.graph.query(request.query)

                rows = []
                for result in query_result:
                    row = {
                        str(variable): (
                            str(value)
                            if value is not None
                            else None
                        )
                        for variable, value
                        in result.asdict().items()
                    }
                    rows.append(row)

            response.success = True
            response.result_json = json.dumps(
                rows,
                ensure_ascii=False,
            )
            response.message = (
                f'Query completed: {len(rows)} result(s)'
            )

        except Exception as error:
            response.success = False
            response.result_json = '[]'
            response.message = f'SPARQL query failed: {error}'
            self.get_logger().error(response.message)

        return response

    def modify_world_model_callback(self, request, response):
        # CASK_GENERATED_IOC_V1
        if request.operation == 'import_task':
            if not self.get_parameter('enable_ioc_import').value:
                response.success = False
                response.message = 'IOC import is disabled in the final workspace'
                return response
            from .ioc_world_model_import import import_task
            return import_task(self, request, response)
        try:
            subject = URIRef(request.subject)
            predicate = URIRef(request.predicate)

            with self.graph_lock:
                obj = self.make_object(
                    request,
                    subject,
                    predicate,
                )

                if request.operation == ModifyWorldModel.Request.ADD:
                    self.graph.add((subject, predicate, obj))

                elif request.operation == ModifyWorldModel.Request.REMOVE:
                    self.remove_requested_object(
                        request,
                        subject,
                        predicate,
                        obj,
                    )

                elif request.operation == ModifyWorldModel.Request.SET:
                    self.graph.remove((subject, predicate, None))
                    self.graph.add((subject, predicate, obj))

                else:
                    raise ValueError(
                        'Operation must be add, remove or set'
                    )

                if request.save_to_file:
                    self.save_runtime_world_model()

                triple_count = len(self.graph)

            response.success = True
            response.message = (
                f'Operation "{request.operation}" completed. '
                f'World Model contains {triple_count} triples.'
            )
            self.get_logger().info(response.message)

        except Exception as error:
            response.success = False
            response.message = (
                f'World Model modification failed: {error}'
            )
            self.get_logger().error(response.message)

        return response

    def make_object(self, request, subject, predicate):
        if not request.object_is_literal:
            return URIRef(request.object)

        datatype = None
        language = None

        for existing in self.graph.objects(subject, predicate):
            if isinstance(existing, Literal):
                datatype = existing.datatype
                language = existing.language
                break

        if datatype is None and language is None:
            datatype = next(
                self.graph.objects(predicate, RDFS.range),
                None,
            )

        if language:
            return Literal(request.object, lang=language)

        if datatype:
            return Literal(request.object, datatype=datatype)

        return Literal(request.object)

    def remove_requested_object(
        self,
        request,
        subject,
        predicate,
        obj,
    ):
        if not request.object_is_literal:
            self.graph.remove((subject, predicate, obj))
            return

        matching_triples = list(
            self.graph.triples((subject, predicate, None))
        )

        for triple in matching_triples:
            if (
                isinstance(triple[2], Literal)
                and str(triple[2]) == request.object
            ):
                self.graph.remove(triple)

    def save_runtime_world_model(self):
        runtime_directory = os.path.dirname(self.runtime_path)
        if runtime_directory:
            os.makedirs(runtime_directory, exist_ok=True)

        temporary_path = self.runtime_path + '.tmp'
        self.graph.serialize(
            destination=temporary_path,
            format='turtle',
        )
        os.replace(temporary_path, self.runtime_path)

        self.get_logger().info(
            f'Runtime World Model saved: {self.runtime_path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CaskWorldModel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
