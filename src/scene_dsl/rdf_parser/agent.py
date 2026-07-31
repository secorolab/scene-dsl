# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.common import ModelBase, ModelLoader, get_node_types
from rdf_utils.models.vocab import (
    URI_AGN_PRED_HAS_AGN_MODEL,
    URI_AGN_PRED_OF_AGN,
    URI_AGN_TYPE_AGN,
    URI_AGN_TYPE_MOD_AGN,
    URI_EXEC_PRED_HAS_CONFIG,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.rdf_parser.common import load_attr_has_config


class AgentModel(ModelBase):
    def __init__(self, graph: Graph, agent_id: URIRef) -> None:
        super().__init__(
            graph=graph,
            node_id=agent_id,
            types=get_node_types(graph, agent_id) | {URI_AGN_TYPE_AGN},
        )
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
        modelled_agents = (
            graph.subjects(URI_AGN_PRED_OF_AGN, self.id, unique=True)
            if modelled_ids is None
            else modelled_ids
        )
        model_loader.load_attributes(graph=graph, model=self, **kwargs)
        for modelled in modelled_agents:
            if not isinstance(modelled, URIRef):
                raise TypeError(
                    f"'{self}' has a non-URI modelled wrapper: {modelled.n3(self._ns_manager)}"
                )

            if (modelled, RDF.type, URI_AGN_TYPE_MOD_AGN) not in graph:
                continue
            if (modelled, URI_AGN_PRED_OF_AGN, self.id) not in graph:
                raise ValueError(f"ModelledAgent '{modelled}' does not model agent '{self.id}'")

            self._load_agn_models(
                modelled_id=modelled, graph=graph, model_loader=model_loader, **kwargs
            )

        return self._models

    def _load_agn_models(
        self, modelled_id: URIRef, graph: Graph, model_loader: ModelLoader, **kwargs
    ) -> None:
        if self._models is None:
            raise RuntimeError(f"{self}._load_agn_models() called without initializing ._models")

        for model_id in graph.objects(modelled_id, URI_AGN_PRED_HAS_AGN_MODEL):
            if not isinstance(model_id, URIRef):
                raise TypeError(f"'{self}' has a non-URI model: {model_id}")
            if model_id in self._models:
                continue
            model = ModelBase(graph=graph, node_id=model_id)
            model_loader.load_attributes(graph=graph, model=model, **kwargs)

            if graph.value(model.id, URI_EXEC_PRED_HAS_CONFIG) is not None:
                if self._config is not None:
                    raise ValueError(f"agent '{self}' has multiple model configurations")
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
        raise RuntimeError(f"agent '{self.id}' doesn't have a model of type '{model_type}'")

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
