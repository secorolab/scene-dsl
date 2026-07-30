# SPDX-License-Identifier: MPL-2.0
from collections.abc import Iterable
from typing import Any

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
    def __init__(
        self,
        graph: Graph,
        agent_id: URIRef,
        modelled_ids: Iterable[URIRef] | None = None,
    ) -> None:
        types = None
        if modelled_ids is not None:
            types = get_node_types(graph, agent_id) | {URI_AGN_TYPE_AGN}
        super().__init__(graph=graph, node_id=agent_id, types=types)
        self.models: dict[URIRef, ModelBase] = {}
        self.model_types: set[URIRef] = set()
        self.model_type_to_id: dict[URIRef, set[URIRef]] = {}
        self._config: URIRef | None = None

        modelled_agents = (
            graph.subjects(URI_AGN_PRED_OF_AGN, agent_id, unique=True)
            if modelled_ids is None
            else modelled_ids
        )
        for modelled in modelled_agents:
            if not isinstance(modelled, URIRef):
                raise TypeError(
                    f"'{self}' has a non-URI modelled wrapper: {modelled.n3(self._ns_manager)}"
                )

            if (modelled, RDF.type, URI_AGN_TYPE_MOD_AGN) not in graph:
                continue
            if (modelled, URI_AGN_PRED_OF_AGN, agent_id) not in graph:
                raise ValueError(f"ModelledAgent '{modelled}' does not model agent '{agent_id}'")

            self._load_agn_models(modelled_id=modelled, graph=graph)

    def _load_agn_models(self, modelled_id: URIRef, graph: Graph) -> None:
        for model_id in graph.objects(modelled_id, URI_AGN_PRED_HAS_AGN_MODEL):
            if not isinstance(model_id, URIRef):
                raise TypeError(f"'{self}' has a non-URI model: {model_id}")
            if model_id in self.models:
                continue
            model = ModelBase(graph=graph, node_id=model_id)

            if graph.value(model.id, URI_EXEC_PRED_HAS_CONFIG) is not None:
                if self._config is not None:
                    raise ValueError(f"agent '{self}' has multiple model configurations")
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
        raise RuntimeError(f"agent '{self.id}' doesn't have a model of type '{model_type}'")
