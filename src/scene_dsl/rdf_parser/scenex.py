# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.common import AttrLoaderProtocol, ModelBase, ModelLoader
from rdf_utils.models.execution import load_attr_path
from rdf_utils.models.python import load_py_module_attr
from rdf_utils.models.vocab import (
    URI_AGN_PRED_HAS_AGN_MODEL,
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_EXEC_PRED_HAS_MODELLED_AGN,
    URI_EXEC_PRED_HAS_MODELLED_OBJ,
    URI_EXEC_PRED_MODEL,
    URI_EXEC_PRED_PATH,
    URI_EXEC_TYPE_SCENE_INST,
)
from rdflib import Graph, Literal, URIRef

from scene_dsl.rdf_parser.vocab import URI_ROS_PRED_PACKAGE_NAME, URI_ROS_TYPE_PACKAGE


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
    modelled_objects: dict[URIRef, dict[URIRef, ModelBase]]
    modelled_agents: dict[URIRef, dict[URIRef, ModelBase]]
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

        self.modelled_objects = {}
        for modelled_obj_id in graph.objects(
            subject=self.id, predicate=URI_EXEC_PRED_HAS_MODELLED_OBJ
        ):
            if not isinstance(modelled_obj_id, URIRef):
                raise TypeError(
                    f"SceneInstance ({self.id.n3(self._ns_manager)})'s modelled obj ID is not a URIRef: {modelled_obj_id}"
                )

            self.modelled_objects[modelled_obj_id] = self._load_models(
                graph=graph, owner_id=modelled_obj_id, predicate=URI_ENV_PRED_HAS_OBJ_MODEL
            )

        self.modelled_agents = {}
        for modelled_agn_id in graph.objects(
            subject=self.id, predicate=URI_EXEC_PRED_HAS_MODELLED_AGN
        ):
            if not isinstance(modelled_agn_id, URIRef):
                raise TypeError(
                    f"SceneInstance ({self.id.n3(self._ns_manager)})'s modelled agn ID is not a URIRef: {modelled_agn_id}"
                )

            self.modelled_agents[modelled_agn_id] = self._load_models(
                graph=graph, owner_id=modelled_agn_id, predicate=URI_AGN_PRED_HAS_AGN_MODEL
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
            self._model_loader.load_attributes(graph=graph, model=model)
            models[model_id] = model

        return models
