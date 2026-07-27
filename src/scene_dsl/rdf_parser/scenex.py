# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.common import AttrLoaderProtocol, ModelBase, ModelLoader
from rdf_utils.models.execution import load_attr_path
from rdf_utils.models.python import load_py_module_attr
from rdf_utils.models.vocab import URI_EXEC_PRED_MODEL, URI_EXEC_TYPE_SCENE_INST
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


class SceneInstanceModel(ModelBase):
    models: dict[URIRef, ModelBase]
    _model_loader: ModelLoader

    def __init__(
        self,
        scn_inst_id: URIRef,
        graph: Graph,
        loaders: list[AttrLoaderProtocol] | None = None,
    ) -> None:
        super().__init__(node_id=scn_inst_id, graph=graph)
        assert (
            URI_EXEC_TYPE_SCENE_INST in self.types
        ), f"node '{scn_inst_id}' is not a scene instance"

        self._model_loader = ModelLoader()
        for loader in DEFAULT_MODEL_LOADERS if loaders is None else loaders:
            self._model_loader.register(loader)

        self.models = {}
        for model_id in graph.objects(subject=scn_inst_id, predicate=URI_EXEC_PRED_MODEL):
            assert isinstance(model_id, URIRef), f"unexpected scene model ID type: {type(model_id)}"
            model = ModelBase(node_id=model_id, graph=graph)
            assert (
                model_id not in self.models
            ), f"scene instance '{self.id}' has duplicate model '{model_id}'"
            self._model_loader.load_attributes(graph=graph, model=model)
            self.models[model_id] = model

        assert self.models, f"scene instance '{self.id}' has no model"
