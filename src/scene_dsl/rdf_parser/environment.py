# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.common import ModelBase, ModelLoader
from rdf_utils.models.vocab import (
    URI_ENV_PRED_HAS_OBJ,
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_ENV_PRED_HAS_WS,
    URI_ENV_PRED_OF_OBJ,
    URI_ENV_PRED_OF_WS,
    URI_ENV_TYPE_MOD_OBJ,
    URI_ENV_TYPE_OBJ,
    URI_ENV_TYPE_WS,
    URI_ENV_TYPE_WS_OBJ,
    URI_ENV_TYPE_WS_WS,
    URI_EXEC_PRED_HAS_CONFIG,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.rdf_parser.common import load_attr_has_config


class ObjectModel(ModelBase):
    def __init__(self, graph: Graph, obj_id: URIRef):
        super().__init__(graph=graph, node_id=obj_id)
        self._models: dict[URIRef, ModelBase] | None = None
        self._model_types: set[URIRef] = set()
        self._model_type_to_id: dict[URIRef, set[URIRef]] = {}
        self._config: URIRef | None = None

    def load_models(
        self,
        graph: Graph,
        model_loader: ModelLoader,
        override: bool = False,
        modelled_ids: set[URIRef] | None = None,
        **kwargs,
    ) -> dict[URIRef, ModelBase]:
        if self._models is not None and not override:
            return self._models

        if override:
            self._model_types.clear()
            self._model_type_to_id.clear()
            self._config = None
            self._attributes.clear()
        self._models = {}

        modelled_objects = (
            graph.subjects(URI_ENV_PRED_OF_OBJ, self.id, unique=True)
            if modelled_ids is None
            else modelled_ids
        )
        model_loader.load_attributes(graph=graph, model=self, **kwargs)
        for modelled in modelled_objects:
            if not isinstance(modelled, URIRef):
                raise TypeError(
                    f"'{self}' has a non-URI modelled wrapper: {modelled.n3(self._ns_manager)}"
                )
            if (modelled, RDF.type, URI_ENV_TYPE_MOD_OBJ) not in graph:
                continue
            if (modelled, URI_ENV_PRED_OF_OBJ, self.id) not in graph:
                raise ValueError(f"ModelledObject '{modelled}' does not model '{self}'")
            self._load_obj_models(modelled, graph, model_loader=model_loader, **kwargs)

        if not self._model_types:
            raise ValueError(f"'{self}' has no model type")

        return self._models

    def _load_obj_models(
        self, modelled_id: URIRef, graph: Graph, model_loader: ModelLoader, **kwargs
    ) -> None:
        if self._models is None:
            raise RuntimeError(f"{self}._load_obj_models() called without initializing ._models")

        for model_id in graph.objects(modelled_id, URI_ENV_PRED_HAS_OBJ_MODEL):
            if not isinstance(model_id, URIRef):
                raise TypeError(f"'{self}' has a non-URI model: {model_id}")

            if model_id in self._models:
                continue

            model = ModelBase(graph=graph, node_id=model_id)
            model_loader.load_attributes(graph=graph, model=model, **kwargs)

            if graph.value(model.id, URI_EXEC_PRED_HAS_CONFIG) is not None:
                if self._config is not None:
                    raise ValueError(f"object '{self}' has multiple model configurations")
                self._config = model.id
                load_attr_has_config(graph=graph, model=model)

            self._models[model.id] = model
            for model_type in model.types:
                self._model_types.add(model_type)
                self._model_type_to_id.setdefault(model_type, set()).add(model.id)

    @property
    def models_loaded(self) -> bool:
        return self._models is not None

    @property
    def config(self) -> ModelBase | None:
        if self._models is None:
            raise RuntimeError(f"{self}.config accessed without calling .load_models()")

        if self._config is None:
            return None
        return self._models[self._config]

    def load_first_model_by_type(self, model_type: URIRef) -> ModelBase:
        if self._models is None:
            raise RuntimeError(
                f"{self}.load_first_model_by_type called without initializing with load_models()"
            )

        for model_uri in self._model_type_to_id[model_type]:
            return self._models[model_uri]
        raise RuntimeError(f"object '{self.id}' doesn't have a model of type '{model_type}'")

    @property
    def models(self) -> dict[URIRef, ModelBase]:
        if self._models is None:
            raise RuntimeError(f"{self}.models accessed without calling .load_models()")
        return self._models

    @property
    def model_types(self) -> set[URIRef]:
        if self._models is None:
            raise RuntimeError(f"{self}.model_types accessed without calling .load_models()")
        return self._model_types

    @property
    def model_type_to_id(self) -> dict[URIRef, set[URIRef]]:
        if self._models is None:
            raise RuntimeError(f"{self}.model_type_to_id accessed without calling .load_models()")
        return self._model_type_to_id


class WorkspaceModel(ModelBase):
    def __init__(self, ws_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=ws_id, graph=graph)
        self.objects: set[URIRef] = set()
        self.workspaces: set[URIRef] = set()
        self.ws_comps: set[URIRef] = set()

    def load_ws_comp(self, ws_comp_id: URIRef, graph: Graph) -> None:
        if (ws_comp_id, URI_ENV_PRED_OF_WS, self.id) not in graph:
            raise ValueError(f"composition '{ws_comp_id.n3(self._ns_manager)}' is not of '{self}'")

        has_objects = (ws_comp_id, RDF.type, URI_ENV_TYPE_WS_OBJ) in graph
        has_workspaces = (ws_comp_id, RDF.type, URI_ENV_TYPE_WS_WS) in graph
        if not has_objects and not has_workspaces:
            raise TypeError(f"Composition for {self} does not have a composition type")

        if has_objects:
            for obj_id in graph.objects(ws_comp_id, URI_ENV_PRED_HAS_OBJ):
                if not isinstance(obj_id, URIRef):
                    raise TypeError(
                        f"'{self}' contains a non-URI object: {obj_id.n3(self._ns_manager)}"
                    )
                if (obj_id, RDF.type, URI_ENV_TYPE_OBJ) not in graph:
                    raise TypeError(
                        f"'{self}' links to '{obj_id.n3(self._ns_manager)}', which is not an Object"
                    )

                self.objects.add(obj_id)

        if has_workspaces:
            for sub_ws_id in graph.objects(ws_comp_id, URI_ENV_PRED_HAS_WS):
                if not isinstance(sub_ws_id, URIRef):
                    raise TypeError(
                        f"'{self}' contains a non-URI workspace: {sub_ws_id.n3(self._ns_manager)}"
                    )

                if graph.value(sub_ws_id, URI_ENV_PRED_OF_WS) is not None:
                    self.ws_comps.add(sub_ws_id)
                elif (sub_ws_id, RDF.type, URI_ENV_TYPE_WS) in graph:
                    self.workspaces.add(sub_ws_id)
                else:
                    raise TypeError(
                        f"'{self}' links to '{sub_ws_id.n3(self._ns_manager)}', which is not a WS or composition"
                    )
