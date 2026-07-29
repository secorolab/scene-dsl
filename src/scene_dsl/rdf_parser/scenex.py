# SPDX-License-Identifier: MPL-2.0
from collections.abc import Generator
from dataclasses import dataclass

from rdf_utils.models.common import AttrLoaderProtocol, ModelBase, ModelLoader, get_node_types
from rdf_utils.models.execution import load_attr_path
from rdf_utils.models.geom_rel import FrameModel
from rdf_utils.models.python import load_py_module_attr
from rdf_utils.models.vocab import (
    URI_AGN_PRED_HAS_AGN_MODEL,
    URI_AGN_PRED_OF_AGN,
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_ENV_PRED_OF_OBJ,
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_HAS_MODELLED_AGN,
    URI_EXEC_PRED_HAS_MODELLED_OBJ,
    URI_EXEC_PRED_MAPS,
    URI_EXEC_PRED_MODEL,
    URI_EXEC_PRED_MODEL_ENTITY,
    URI_EXEC_PRED_PATH,
    URI_EXEC_TYPE_SCENE_INST,
    URI_GEOM_TYPE_KTREE,
    URI_GEOM_TYPE_RIGID_BODY,
)
from rdflib import Graph, Literal, URIRef

from scene_dsl.rdf_parser.common import ensure_one_obj_literal, ensure_one_obj_uri
from scene_dsl.rdf_parser.ktree import RigidBodyModel, get_root_frame
from scene_dsl.rdf_parser.vocab import URI_ROS_PRED_PACKAGE_NAME, URI_ROS_TYPE_PACKAGE

__ALLOWED_MAPPINGS = {URI_GEOM_TYPE_RIGID_BODY, URI_GEOM_TYPE_KTREE}


@dataclass
class ModelMapping:
    target_id: URIRef
    target_type: URIRef
    entity: str | None = None


def get_model_mapping(mapping_id: URIRef, graph: Graph) -> ModelMapping:
    target_uri = ensure_one_obj_uri(graph=graph, subject=mapping_id, predicate=URI_EXEC_PRED_MAPS)
    if target_uri is None:
        raise ValueError(f"mapping '{mapping_id}' does not map to an URI target")

    target_types = get_node_types(graph=graph, node_id=target_uri) & __ALLOWED_MAPPINGS
    if len(target_types) != 1:
        raise ValueError(f"mapping '{mapping_id}' target '{target_uri}' must be one body or tree")

    entity_literal = ensure_one_obj_literal(
        graph=graph, subject=mapping_id, predicate=URI_EXEC_PRED_MODEL_ENTITY
    )

    entity = None
    if entity_literal is not None:
        entity = entity_literal.toPython()

        if not isinstance(entity, str):
            raise TypeError(f"mapping '{mapping_id}' entity must be a string")

    return ModelMapping(target_uri, target_types.pop(), entity=entity)


class ElementResourceModel(ModelBase):
    _mappings: dict[URIRef, ModelMapping]
    _mapped_targets: dict[URIRef, URIRef]  # target URI -> mapping URI
    _mapped_types: dict[URIRef, set[URIRef]]  # map type URI -> mapping URI

    def __init__(self, node_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=node_id, graph=graph)
        self._mappings = {}
        self._mapped_targets = {}
        self._mapped_types = {}

        for mapping_id in graph.objects(self.id, URI_EXEC_PRED_HAS_MAPPING):
            if not isinstance(mapping_id, URIRef):
                raise TypeError(f"ElementResource '{self}' has non-URI mapping: {mapping_id}")

            mapping = get_model_mapping(mapping_id, graph)
            self._mappings[mapping_id] = mapping

            if mapping.target_id in self._mapped_targets:
                raise ValueError(f"multiple mappings found for target {mapping.target_id}")
            self._mapped_targets[mapping.target_id] = mapping_id

            if mapping.target_type not in self._mapped_types:
                self._mapped_types[mapping.target_type] = set()
            self._mapped_types[mapping.target_type].add(mapping_id)

    def get_mapping_by_target_uri(self, target_uri: URIRef) -> ModelMapping | None:
        if target_uri not in self._mapped_targets:
            return None

        return self._mappings[self._mapped_targets[target_uri]]

    def get_mappings_by_target_type(
        self, target_type: URIRef
    ) -> Generator[ModelMapping, None, None]:
        if target_type in self._mapped_types:
            for map_id in self._mapped_types[target_type]:
                yield self._mappings[map_id]


def load_ros_path(graph: Graph, model: ModelBase, **kwargs: object) -> None:
    """Load the ROS package name for a resource stored in a ROS package."""
    if URI_ROS_TYPE_PACKAGE not in model.types:
        return

    package_name = graph.value(subject=model.id, predicate=URI_ROS_PRED_PACKAGE_NAME)
    if not isinstance(package_name, Literal):
        raise TypeError(f"ROS model '{model.id}' has no literal '{URI_ROS_PRED_PACKAGE_NAME}'")
    model.set_attr(key=URI_ROS_PRED_PACKAGE_NAME, val=str(package_name.toPython()))


DEFAULT_MODEL_LOADERS: tuple[AttrLoaderProtocol, ...] = (
    load_attr_path,
    load_py_module_attr,
    load_ros_path,
)


def get_ros_pkg_path(model: ModelBase) -> tuple[str, str] | None:
    """Return ROS package and relative path metadata without resolving it."""
    if URI_ROS_TYPE_PACKAGE not in model.types:
        return None

    package_name = model.get_attr(URI_ROS_PRED_PACKAGE_NAME)
    path = model.get_attr(URI_EXEC_PRED_PATH)
    if not isinstance(package_name, str) or not isinstance(path, str):
        raise TypeError(f"ROS model '{model.id}' has invalid package or path metadata")
    return package_name, path


class SceneInstanceModel(ModelBase):
    models: dict[URIRef, ElementResourceModel]

    _modelled_objects: dict[URIRef, dict[URIRef, ElementResourceModel]]
    _obj_modelled_maps: dict[URIRef, set[URIRef]]  # Object URI -> set of ModelledObject URIs
    _modelled_agents: dict[URIRef, dict[URIRef, ElementResourceModel]]
    _agn_modelled_maps: dict[URIRef, set[URIRef]]  # Agent URI -> set of ModelledAgent URIs
    _model_loader: ModelLoader

    def __init__(
        self,
        scn_inst_id: URIRef,
        graph: Graph,
        loaders: list[AttrLoaderProtocol] | None = None,
    ) -> None:
        super().__init__(node_id=scn_inst_id, graph=graph)

        if URI_EXEC_TYPE_SCENE_INST not in self.types:
            raise TypeError(f"node '{scn_inst_id.n3(self._ns_manager)}' is not a SceneInstance")

        self._model_loader = ModelLoader()
        for loader in DEFAULT_MODEL_LOADERS if loaders is None else loaders:
            self._model_loader.register(loader)

        self.models = self._load_models(
            graph=graph, owner_id=self.id, predicate=URI_EXEC_PRED_MODEL
        )

        self._load_modelled_objects(graph=graph)
        self._load_modelled_agents(graph=graph)

    def get_obj_body(self, obj_id: URIRef, graph: Graph) -> RigidBodyModel | None:
        if obj_id not in self._obj_modelled_maps:
            return None

        body_uris = []
        for modelled_obj_id in self._obj_modelled_maps[obj_id]:
            modelled_obj = self._modelled_objects[modelled_obj_id]
            for resource_model in modelled_obj.values():
                for mapping in resource_model.get_mappings_by_target_type(
                    target_type=URI_GEOM_TYPE_RIGID_BODY
                ):
                    body_uris.append(mapping.target_id)

        if not body_uris:
            return None

        if len(body_uris) > 1:
            raise ValueError(
                f"Object {obj_id} is not mapped to zero or one RigidBody, found: {body_uris}"
            )

        return RigidBodyModel(body_id=body_uris[0], graph=graph)

    def get_agn_tree_root_frame(self, agn_id: URIRef, graph: Graph) -> FrameModel | None:
        if agn_id not in self._agn_modelled_maps:
            return None

        tree_uris = []
        for modelled_agn_id in self._agn_modelled_maps[agn_id]:
            modelled_agn = self._modelled_agents[modelled_agn_id]
            for resource_model in modelled_agn.values():
                for mapping in resource_model.get_mappings_by_target_type(
                    target_type=URI_GEOM_TYPE_KTREE
                ):
                    tree_uris.append(mapping.target_id)

        if not tree_uris:
            return None

        if len(tree_uris) > 1:
            raise ValueError(
                f"Agent {agn_id} is not mapped to zero or one KinematicTree, found: {tree_uris}"
            )

        return get_root_frame(target_id=tree_uris[0], graph=graph)

    def get_elem_root_frame(self, elem_id: URIRef, graph: Graph) -> FrameModel | None:
        """Return the root frame when the element kind is not known."""
        obj_body = self.get_obj_body(obj_id=elem_id, graph=graph)
        if obj_body is not None:
            return obj_body.root_frame

        return self.get_agn_tree_root_frame(agn_id=elem_id, graph=graph)

    def _load_models(
        self, graph: Graph, owner_id: URIRef, predicate: URIRef
    ) -> dict[URIRef, ElementResourceModel]:
        models = {}

        for model_id in graph.objects(subject=owner_id, predicate=predicate):
            if not isinstance(model_id, URIRef):
                raise TypeError(
                    f"Model owner ({owner_id.n3(self._ns_manager)})'s model ID is not a URIRef: {model_id}"
                )

            model = ElementResourceModel(node_id=model_id, graph=graph)
            self._model_loader.load_attributes(graph=graph, model=model)
            models[model_id] = model

        return models

    def _load_modelled_objects(self, graph: Graph) -> None:
        self._modelled_objects = {}
        self._obj_modelled_maps = {}
        for modelled_obj_id in graph.objects(
            subject=self.id, predicate=URI_EXEC_PRED_HAS_MODELLED_OBJ
        ):
            if not isinstance(modelled_obj_id, URIRef):
                raise TypeError(
                    f"SceneInstance ({self.id.n3(self._ns_manager)})'s modelled obj ID is not a URIRef: {modelled_obj_id}"
                )

            modelled_obj = self._load_models(
                graph=graph, owner_id=modelled_obj_id, predicate=URI_ENV_PRED_HAS_OBJ_MODEL
            )
            self._modelled_objects[modelled_obj_id] = modelled_obj

            obj_id = ensure_one_obj_uri(
                graph=graph, subject=modelled_obj_id, predicate=URI_ENV_PRED_OF_OBJ
            )
            if obj_id is None:
                raise ValueError(f"ModelledObject {modelled_obj} does not link to an Object")

            if obj_id not in self._obj_modelled_maps:
                self._obj_modelled_maps[obj_id] = set()

            self._obj_modelled_maps[obj_id].add(modelled_obj_id)

    def _load_modelled_agents(self, graph: Graph) -> None:
        self._modelled_agents = {}
        self._agn_modelled_maps = {}
        for modelled_agn_id in graph.objects(
            subject=self.id, predicate=URI_EXEC_PRED_HAS_MODELLED_AGN
        ):
            if not isinstance(modelled_agn_id, URIRef):
                raise TypeError(
                    f"SceneInstance ({self.id.n3(self._ns_manager)})'s modelled agn ID is not a URIRef: {modelled_agn_id}"
                )

            modelled_agn = self._load_models(
                graph=graph, owner_id=modelled_agn_id, predicate=URI_AGN_PRED_HAS_AGN_MODEL
            )
            self._modelled_agents[modelled_agn_id] = modelled_agn

            agn_id = ensure_one_obj_uri(
                graph=graph, subject=modelled_agn_id, predicate=URI_AGN_PRED_OF_AGN
            )
            if agn_id is None:
                raise ValueError(f"ModelledAgent {modelled_agn} does not link to an Agent")

            if agn_id not in self._agn_modelled_maps:
                self._agn_modelled_maps[agn_id] = set()

            self._agn_modelled_maps[agn_id].add(modelled_agn_id)
