# SPDX-License-Identifier: MPL-2.0
from dataclasses import dataclass

from rdf_utils.models.common import AttrLoaderProtocol, ModelBase, ModelLoader, get_node_types
from rdf_utils.models.execution import load_attr_path
from rdf_utils.models.python import load_py_module_attr
from rdf_utils.models.vocab import (
    URI_AGN_PRED_HAS_AGN_MODEL,
    URI_AGN_PRED_OF_AGN,
    URI_AGN_TYPE_MOD_AGN,
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_ENV_PRED_OF_OBJ,
    URI_ENV_TYPE_MOD_OBJ,
    URI_EXEC_PRED_HAS_CONFIG,
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
from rdflib import RDF, Graph, Literal, URIRef

from scene_dsl.rdf_parser.common import (
    ensure_one_obj_literal,
    ensure_one_obj_uri,
    load_attr_has_config,
)
from scene_dsl.rdf_parser.ktree import RigidBodyModel
from scene_dsl.rdf_parser.scene import SceneModel
from scene_dsl.rdf_parser.vocab import (
    URI_BDD_PRED_OF_SCENE,
    URI_ROS_PRED_PACKAGE_NAME,
    URI_ROS_TYPE_PACKAGE,
)

__ALLOWED_MAPPINGS = {URI_GEOM_TYPE_RIGID_BODY, URI_GEOM_TYPE_KTREE}


@dataclass
class KinematicMapping:
    target_id: URIRef
    target_type: URIRef
    entity: str | None = None


def get_kinematic_mapping(mapping_id: URIRef, graph: Graph) -> KinematicMapping:
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

    return KinematicMapping(target_uri, target_types.pop(), entity=entity)


def load_attr_kinematic_mappings(graph: Graph, model: ModelBase, **kwargs: object) -> None:
    mappings = []
    targets = set()
    for mapping_id in graph.objects(model.id, URI_EXEC_PRED_HAS_MAPPING):
        if not isinstance(mapping_id, URIRef):
            raise TypeError(f"model '{model}' has non-URI mapping: {mapping_id}")
        mapping = get_kinematic_mapping(mapping_id, graph)
        if mapping.target_id in targets:
            raise ValueError(f"multiple mappings found for target {mapping.target_id}")
        targets.add(mapping.target_id)
        mappings.append(mapping)
    model.set_attr(URI_EXEC_PRED_HAS_MAPPING, tuple(mappings))


def get_kinematic_mappings(
    model: ModelBase, target_type: URIRef | None = None
) -> tuple[KinematicMapping, ...]:
    mappings = model.get_attr(URI_EXEC_PRED_HAS_MAPPING)
    if not isinstance(mappings, tuple) or not all(
        isinstance(mapping, KinematicMapping) for mapping in mappings
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
    load_attr_has_config,
    load_attr_kinematic_mappings,
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
    object_models: dict[URIRef, dict[URIRef, ModelBase]]
    agent_models: dict[URIRef, dict[URIRef, ModelBase]]

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

        model_loader = ModelLoader()
        for loader in DEFAULT_MODEL_LOADERS if loaders is None else loaders:
            model_loader.register(loader)

        self.models = self._load_models(
            graph=graph,
            model_loader=model_loader,
            owner_id=self.id,
            predicate=URI_EXEC_PRED_MODEL,
        )

        self.object_models = self._load_element_models(
            graph=graph,
            model_loader=model_loader,
            has_modelled_pred=URI_EXEC_PRED_HAS_MODELLED_OBJ,
            has_modelled_type=URI_ENV_TYPE_MOD_OBJ,
            of_elem_pred=URI_ENV_PRED_OF_OBJ,
            has_model_pred=URI_ENV_PRED_HAS_OBJ_MODEL,
        )
        self.agent_models = self._load_element_models(
            graph=graph,
            model_loader=model_loader,
            has_modelled_pred=URI_EXEC_PRED_HAS_MODELLED_AGN,
            has_modelled_type=URI_AGN_TYPE_MOD_AGN,
            of_elem_pred=URI_AGN_PRED_OF_AGN,
            has_model_pred=URI_AGN_PRED_HAS_AGN_MODEL,
        )

    def get_body_for_resource_entity(self, name: str, graph: Graph) -> RigidBodyModel | None:
        matched_ids = [
            mapping.target_id
            for model in self.models.values()
            for mapping in get_kinematic_mappings(model, URI_GEOM_TYPE_RIGID_BODY)
            if mapping.entity == name
        ]

        if not matched_ids:
            return None

        if len(matched_ids) > 1:
            raise ValueError(f"entity name '{name}' matched multiple bodies: {matched_ids}")

        return RigidBodyModel(body_id=matched_ids[0], graph=graph)

    def _load_models(
        self,
        graph: Graph,
        model_loader: ModelLoader,
        owner_id: URIRef,
        predicate: URIRef,
    ) -> dict[URIRef, ModelBase]:
        models = {}

        for model_id in graph.objects(subject=owner_id, predicate=predicate):
            if not isinstance(model_id, URIRef):
                raise TypeError(
                    f"Model owner ({owner_id.n3(self._ns_manager)})'s model ID is not a URIRef: {model_id}"
                )

            model = ModelBase(node_id=model_id, graph=graph)
            model_loader.load_attributes(graph=graph, model=model)
            models[model_id] = model

        return models

    def _load_element_models(
        self,
        graph: Graph,
        model_loader: ModelLoader,
        has_modelled_pred: URIRef,
        has_modelled_type: URIRef,
        of_elem_pred: URIRef,
        has_model_pred: URIRef,
    ) -> dict[URIRef, dict[URIRef, ModelBase]]:
        elements: dict[URIRef, dict[URIRef, ModelBase]] = {}
        configured: dict[URIRef, URIRef] = {}
        modelled_type_rep = has_modelled_type.n3(self._ns_manager)
        for modelled_id in graph.objects(subject=self.id, predicate=has_modelled_pred):
            if not isinstance(modelled_id, URIRef):
                raise TypeError(f"SceneInstance '{self.id}' has non-URI {modelled_type_rep}")
            modelled_id_rep = modelled_id.n3(self._ns_manager)
            if (modelled_id, RDF.type, has_modelled_type) not in graph:
                raise TypeError(f"{modelled_type_rep} '{modelled_id_rep}' has the wrong type")

            element_id = ensure_one_obj_uri(
                graph=graph, subject=modelled_id, predicate=of_elem_pred
            )
            if element_id is None:
                raise ValueError(
                    f"{modelled_type_rep} '{modelled_id_rep}' doens't link to a scene element"
                )

            elem_models = elements.setdefault(element_id, {})
            for model_id in graph.objects(modelled_id, has_model_pred):
                if not isinstance(model_id, URIRef):
                    raise TypeError(f"{modelled_type_rep} '{modelled_id}' has a non-URI model")
                if model_id in elem_models:
                    continue

                model = ModelBase(node_id=model_id, graph=graph)
                model_loader.load_attributes(graph=graph, model=model)
                if model.has_attr(URI_EXEC_PRED_HAS_CONFIG):
                    previous = configured.get(element_id)
                    if previous is not None:
                        raise ValueError(
                            f"Scene element '{element_id.n3(self._ns_manager)}' has multiple model configurations"
                        )
                    configured[element_id] = model_id
                elem_models[model_id] = model

        return elements
