# SPDX-License-Identifier: MPL-2.0
from collections.abc import Generator, Iterable
from typing import Any

from rdf_utils.models.common import ModelBase, ModelLoader, get_node_types
from rdf_utils.models.vocab import (
    URI_ENV_PRED_HAS_OBJ,
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_ENV_PRED_HAS_WS,
    URI_ENV_PRED_OF_OBJ,
    URI_ENV_PRED_OF_WS,
    URI_ENV_TYPE_MOD_OBJ,
    URI_ENV_TYPE_OBJ,
    URI_ENV_TYPE_WS_OBJ,
    URI_ENV_TYPE_WS_WS,
    URI_EXEC_PRED_HAS_CONFIG,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.rdf_parser.common import load_attr_has_config


class ObjectModel(ModelBase):
    models: dict[URIRef, ModelBase]
    model_types: set[URIRef]
    model_type_to_id: dict[URIRef, set[URIRef]]

    def __init__(
        self,
        graph: Graph,
        obj_id: URIRef,
        modelled_ids: Iterable[URIRef] | None = None,
    ):
        types = None
        if modelled_ids is not None:
            types = get_node_types(graph, obj_id) | {URI_ENV_TYPE_OBJ}
        super().__init__(graph=graph, node_id=obj_id, types=types)
        self.models = {}
        self.model_types = set()
        self.model_type_to_id = {}
        self._config: URIRef | None = None

        modelled_objects = (
            graph.subjects(URI_ENV_PRED_OF_OBJ, obj_id, unique=True)
            if modelled_ids is None
            else modelled_ids
        )
        for modelled in modelled_objects:
            if not isinstance(modelled, URIRef):
                raise TypeError(
                    f"'{self}' has a non-URI modelled wrapper: {modelled.n3(self._ns_manager)}"
                )
            if (modelled, RDF.type, URI_ENV_TYPE_MOD_OBJ) not in graph:
                continue
            if (modelled, URI_ENV_PRED_OF_OBJ, obj_id) not in graph:
                raise ValueError(f"ModelledObject '{modelled}' does not model object '{obj_id}'")
            self._load_obj_models(modelled, graph)

        if not self.model_types:
            raise ValueError(f"object '{obj_id}' has no model type")

    def _load_obj_models(self, modelled_id: URIRef, graph: Graph) -> None:
        for model_id in graph.objects(modelled_id, URI_ENV_PRED_HAS_OBJ_MODEL):
            if not isinstance(model_id, URIRef):
                raise TypeError(f"'{self}' has a non-URI model: {model_id}")
            if model_id in self.models:
                continue
            model = ModelBase(graph=graph, node_id=model_id)

            if graph.value(model.id, URI_EXEC_PRED_HAS_CONFIG) is not None:
                if self._config is not None:
                    raise ValueError(f"object '{self}' has multiple model configurations")
                self._config = model.id
                load_attr_has_config(graph=graph, model=model)

            self.models[model.id] = model
            for model_type in model.types:
                self.model_types.add(model_type)
                self.model_type_to_id.setdefault(model_type, set()).add(model.id)

    @property
    def config(self) -> ModelBase | None:
        if self._config is None:
            return None
        return self.models[self._config]

    def load_model_attrs(self, graph: Graph, model_loader: ModelLoader, **kwargs: Any) -> None:
        model_loader.load_attributes(graph=graph, model=self, **kwargs)
        for model in self.models.values():
            model_loader.load_attributes(graph=graph, model=model, **kwargs)

    def load_first_model_by_type(self, model_type: URIRef) -> ModelBase:
        for model_uri in self.model_type_to_id[model_type]:
            return self.models[model_uri]
        raise RuntimeError(f"object '{self.id}' doesn't have a model of type '{model_type}'")


class WorkspaceModel(ModelBase):
    def __init__(self, ws_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=ws_id, graph=graph)
        self.workspaces: set[URIRef] = set()
        self.objects: set[URIRef] = set()


def _get_ws_objects_re(
    ws_id: URIRef,
    ws_dict: dict[URIRef, WorkspaceModel],
    ws_path: set[URIRef] | None = None,
) -> Generator[URIRef, None, None]:
    path = set() if ws_path is None else set(ws_path)
    if ws_id in path:
        raise RuntimeError(f"workspace loop detected at '{ws_id}'")
    path.add(ws_id)

    ws_model = ws_dict[ws_id]
    for sub_ws_id in ws_model.workspaces:
        yield from _get_ws_objects_re(sub_ws_id, ws_dict, path)
    yield from ws_model.objects


def _load_ws_re(
    ws_id: URIRef,
    graph: Graph,
    ws_dict: dict[URIRef, WorkspaceModel],
    ws_path: set[URIRef] | None = None,
) -> None:
    path = set() if ws_path is None else set(ws_path)
    if ws_id in path:
        raise RuntimeError(f"workspace loop detected at '{ws_id}'")
    if ws_id in ws_dict:
        return
    path.add(ws_id)

    ws_model = WorkspaceModel(ws_id=ws_id, graph=graph)
    ws_dict[ws_id] = ws_model
    compositions = list(graph.subjects(URI_ENV_PRED_OF_WS, ws_id, unique=True))
    if len(compositions) > 1:
        raise RuntimeError(f"multiple compositions for workspace '{ws_id}'")
    if not compositions or not isinstance(compositions[0], URIRef):
        raise TypeError(f"workspace '{ws_id}' has no URI composition")
    ws_comp_id = compositions[0]

    if (ws_comp_id, RDF.type, URI_ENV_TYPE_WS_OBJ) in graph:
        for obj_id in graph.objects(ws_comp_id, URI_ENV_PRED_HAS_OBJ):
            if not isinstance(obj_id, URIRef):
                raise TypeError(f"workspace '{ws_id}' contains a non-URI object: {obj_id}")
            ws_model.objects.add(obj_id)
    if (ws_comp_id, RDF.type, URI_ENV_TYPE_WS_WS) in graph:
        for sub_ws_id in graph.objects(ws_comp_id, URI_ENV_PRED_HAS_WS):
            if not isinstance(sub_ws_id, URIRef):
                raise TypeError(f"workspace '{ws_id}' contains a non-URI workspace: {sub_ws_id}")
            ws_model.workspaces.add(sub_ws_id)
            _load_ws_re(sub_ws_id, graph, ws_dict, path)
