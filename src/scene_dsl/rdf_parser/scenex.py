# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass

from rdf_utils.models.common import AttrLoaderProtocol, ModelBase, ModelLoader, get_node_types
from rdf_utils.models.execution import load_attr_path
from rdf_utils.models.python import load_py_module_attr
from rdf_utils.models.vocab import (
    URI_AGN_PRED_OF_AGN,
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

from scene_dsl.rdf_parser.agent import AgentModel
from scene_dsl.rdf_parser.common import ensure_one_obj_literal, ensure_one_obj_uri
from scene_dsl.rdf_parser.environment import ObjectModel, WorkspaceModel
from scene_dsl.rdf_parser.ktree import RigidBodyModel
from scene_dsl.rdf_parser.scene import SceneElementLoader, SceneModel
from scene_dsl.rdf_parser.vocab import (
    URI_BDD_PRED_OF_SCENE,
    URI_ROS_PRED_PACKAGE_NAME,
    URI_ROS_TYPE_PACKAGE,
)

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


def load_attr_mappings(graph: Graph, model: ModelBase, **kwargs: object) -> None:
    mappings = []
    targets = set()
    for mapping_id in graph.objects(model.id, URI_EXEC_PRED_HAS_MAPPING):
        if not isinstance(mapping_id, URIRef):
            raise TypeError(f"model '{model}' has non-URI mapping: {mapping_id}")
        mapping = get_model_mapping(mapping_id, graph)
        if mapping.target_id in targets:
            raise ValueError(f"multiple mappings found for target {mapping.target_id}")
        targets.add(mapping.target_id)
        mappings.append(mapping)
    model.set_attr(URI_EXEC_PRED_HAS_MAPPING, tuple(mappings))


class MappingLoader:
    def __init__(self, cache: bool = True) -> None:
        self._cache = cache
        self._by_target: dict[URIRef, list[tuple[ModelBase, ModelMapping]]] = {}
        self._by_type: dict[URIRef, list[tuple[ModelBase, ModelMapping]]] = {}
        self._by_entity: dict[str, list[tuple[ModelBase, ModelMapping]]] = {}

    def __call__(self, graph: Graph, model: ModelBase, **kwargs: object) -> None:
        load_attr_mappings(graph, model)
        if not self._cache:
            return
        for mapping in get_model_mappings(model):
            entry = (model, mapping)
            self._by_target.setdefault(mapping.target_id, []).append(entry)
            self._by_type.setdefault(mapping.target_type, []).append(entry)
            if mapping.entity is not None:
                self._by_entity.setdefault(mapping.entity, []).append(entry)

    def by_target(self, target_id: URIRef) -> tuple[tuple[ModelBase, ModelMapping], ...]:
        return tuple(self._by_target.get(target_id, ()))

    def by_type(self, target_type: URIRef) -> tuple[tuple[ModelBase, ModelMapping], ...]:
        return tuple(self._by_type.get(target_type, ()))

    def by_entity(self, entity: str) -> tuple[tuple[ModelBase, ModelMapping], ...]:
        return tuple(self._by_entity.get(entity, ()))


def get_model_mappings(
    model: ModelBase, target_type: URIRef | None = None
) -> tuple[ModelMapping, ...]:
    mappings = model.get_attr(URI_EXEC_PRED_HAS_MAPPING)
    if not isinstance(mappings, tuple) or not all(
        isinstance(mapping, ModelMapping) for mapping in mappings
    ):
        raise TypeError(f"model '{model}' has no loaded mappings")
    if target_type is None:
        return mappings
    return tuple(mapping for mapping in mappings if mapping.target_type == target_type)


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
    models: dict[URIRef, ModelBase]
    scene_model: SceneModel
    object_models: dict[URIRef, ObjectModel]
    agent_models: dict[URIRef, AgentModel]
    workspace_models: dict[URIRef, WorkspaceModel]
    _model_loader: ModelLoader

    def __init__(
        self,
        scn_inst_id: URIRef,
        graph: Graph,
        loaders: list[AttrLoaderProtocol] | None = None,
        scene_model: SceneModel | None = None,
    ) -> None:
        super().__init__(node_id=scn_inst_id, graph=graph)

        if URI_EXEC_TYPE_SCENE_INST not in self.types:
            raise TypeError(f"node '{scn_inst_id.n3(self._ns_manager)}' is not a SceneInstance")

        scene_id = ensure_one_obj_uri(
            graph=graph,
            subject=self.id,
            predicate=URI_BDD_PRED_OF_SCENE,
        )
        if scene_id is None:
            raise ValueError(f"SceneInstance '{self.id}' does not link to a scene")
        if scene_model is not None and scene_model.id != scene_id:
            raise ValueError(
                f"SceneInstance '{self.id}' links to scene '{scene_id}', "
                f"not supplied scene '{scene_model.id}'"
            )
        self.scene_model = (
            SceneModel(graph=graph, scene_id=scene_id) if scene_model is None else scene_model
        )

        self.element_loader = SceneElementLoader()
        for loader in DEFAULT_MODEL_LOADERS if loaders is None else loaders:
            self.element_loader.register(loader)
        self.mapping_loader = MappingLoader()
        self.element_loader.register(self.mapping_loader)

        self.models = self._load_models(
            graph=graph, owner_id=self.id, predicate=URI_EXEC_PRED_MODEL
        )

        object_wrappers = self._modelled_elements(
            graph, URI_EXEC_PRED_HAS_MODELLED_OBJ, URI_ENV_PRED_OF_OBJ, "Object"
        )
        agent_wrappers = self._modelled_elements(
            graph, URI_EXEC_PRED_HAS_MODELLED_AGN, URI_AGN_PRED_OF_AGN, "Agent"
        )
        for obj_id, wrappers in object_wrappers.items():
            self.element_loader.load_object_model(
                graph,
                obj_id,
                modelled_ids=wrappers,
                model=self.scene_model.objects[obj_id],
            )
        for ws_id in self.scene_model.workspaces:
            self.element_loader.load_ws_model(
                graph, ws_id, model=self.scene_model.workspaces[ws_id]
            )
        for agn_id, wrappers in agent_wrappers.items():
            self.element_loader.load_agent_model(graph, agn_id, modelled_ids=wrappers)

        self.object_models = self.element_loader.object_models
        self.workspace_models = self.element_loader.workspace_models
        self.agent_models = self.element_loader.agent_models

    def get_scene_entity_body_by_name(self, name: str, graph: Graph) -> RigidBodyModel | None:
        matched_ids = [
            mapping.target_id
            for _, mapping in self.mapping_loader.by_entity(name)
            if mapping.target_type == URI_GEOM_TYPE_RIGID_BODY
        ]

        if not matched_ids:
            return None

        if len(matched_ids) > 1:
            raise ValueError(f"entity name '{name}' matched multiple bodies: {matched_ids}")

        return RigidBodyModel(body_id=matched_ids[0], graph=graph)

    def get_resources_by_types(self, type_ids: set[URIRef]) -> tuple[ModelBase, ...]:
        return tuple(model for model in self.models.values() if model.types & type_ids)

    def get_obj_resources_by_types(
        self, obj_id: URIRef, type_ids: set[URIRef]
    ) -> tuple[ModelBase, ...]:
        model = self.object_models.get(obj_id)
        return (
            ()
            if model is None
            else tuple(resource for resource in model.models.values() if resource.types & type_ids)
        )

    def get_agn_resources_by_types(
        self, agn_id: URIRef, type_ids: set[URIRef]
    ) -> tuple[ModelBase, ...]:
        model = self.agent_models.get(agn_id)
        return (
            ()
            if model is None
            else tuple(resource for resource in model.models.values() if resource.types & type_ids)
        )

    def _load_models(
        self, graph: Graph, owner_id: URIRef, predicate: URIRef
    ) -> dict[URIRef, ModelBase]:
        models = {}

        for model_id in graph.objects(subject=owner_id, predicate=predicate):
            if not isinstance(model_id, URIRef):
                raise TypeError(
                    f"Model owner ({owner_id.n3(self._ns_manager)})'s model ID is not a URIRef: {model_id}"
                )

            model = ModelBase(node_id=model_id, graph=graph)
            self.element_loader.load_attributes(graph=graph, model=model)
            models[model_id] = model

        return models

    def _modelled_elements(
        self,
        graph: Graph,
        has_modelled: URIRef,
        of_element: URIRef,
        label: str,
    ) -> dict[URIRef, list[URIRef]]:
        elements: dict[URIRef, list[URIRef]] = {}
        for modelled_id in graph.objects(self.id, has_modelled):
            if not isinstance(modelled_id, URIRef):
                raise TypeError(f"SceneInstance '{self.id}' has non-URI modelled {label}")
            element_id = ensure_one_obj_uri(graph, modelled_id, of_element)
            if element_id is None:
                raise ValueError(f"Modelled{label} '{modelled_id}' has no {label}")
            elements.setdefault(element_id, []).append(modelled_id)
        return elements
