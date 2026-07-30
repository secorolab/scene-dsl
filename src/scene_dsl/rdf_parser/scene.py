# SPDX-License-Identifier: MPL-2.0
from collections.abc import Generator, Iterable, Mapping
from typing import Any

from rdf_utils.models.common import ModelBase, ModelLoader, get_node_types
from rdf_utils.models.vocab import (
    URI_AGN_PRED_HAS_AGN,
    URI_ENV_PRED_HAS_OBJ,
    URI_ENV_PRED_HAS_WS,
)
from rdflib import Graph, URIRef

from scene_dsl.rdf_parser.agent import AgentModel
from scene_dsl.rdf_parser.environment import (
    ObjectModel,
    WorkspaceModel,
    _get_ws_objects_re,
    _load_ws_re,
)
from scene_dsl.rdf_parser.vocab import (
    URI_BDD_PRED_OF_SCENE,
    URI_BDD_TYPE_SCENE_AGN,
    URI_BDD_TYPE_SCENE_OBJ,
    URI_BDD_TYPE_SCENE_WS,
)


class SceneElementLoader(ModelLoader):
    def __init__(self) -> None:
        super().__init__()
        self.object_models: dict[URIRef, ObjectModel] = {}
        self.agent_models: dict[URIRef, AgentModel] = {}
        self.workspace_models: dict[URIRef, WorkspaceModel] = {}

    def load_object_model(
        self,
        graph: Graph,
        obj_id: URIRef,
        override: bool = False,
        modelled_ids: Iterable[URIRef] | None = None,
        **kwargs: Any,
    ) -> ObjectModel:
        if obj_id in self.object_models and not override:
            return self.object_models[obj_id]
        model = ObjectModel(graph=graph, obj_id=obj_id, modelled_ids=modelled_ids)
        model.load_model_attrs(graph=graph, model_loader=self, **kwargs)
        self.object_models[obj_id] = model
        return model

    def load_agent_model(
        self,
        graph: Graph,
        agent_id: URIRef,
        override: bool = False,
        modelled_ids: Iterable[URIRef] | None = None,
        **kwargs: Any,
    ) -> AgentModel:
        if agent_id in self.agent_models and not override:
            return self.agent_models[agent_id]
        model = AgentModel(graph=graph, agent_id=agent_id, modelled_ids=modelled_ids)
        model.load_model_attrs(graph=graph, model_loader=self, **kwargs)
        self.agent_models[agent_id] = model
        return model

    def load_ws_model(
        self,
        graph: Graph,
        ws_id: URIRef,
        override: bool = False,
        modelled_objects: Mapping[URIRef, Iterable[URIRef]] | None = None,
        **kwargs: Any,
    ) -> WorkspaceModel:
        if ws_id in self.workspace_models and not override:
            return self.workspace_models[ws_id]
        loaded_ws_ids = set(self.workspace_models)
        _load_ws_re(ws_id, graph, self.workspace_models)
        for loaded_ws_id in self.workspace_models.keys() - loaded_ws_ids:
            self.load_attributes(graph=graph, model=self.workspace_models[loaded_ws_id], **kwargs)
        for obj_id in _get_ws_objects_re(ws_id, self.workspace_models):
            if modelled_objects is not None and obj_id not in modelled_objects:
                continue
            self.load_object_model(
                graph,
                obj_id,
                override=override,
                modelled_ids=None if modelled_objects is None else modelled_objects[obj_id],
                **kwargs,
            )
        return self.workspace_models[ws_id]

    def load_ws_objects(
        self, graph: Graph, ws_id: URIRef, override: bool = False, **kwargs: Any
    ) -> Generator[ObjectModel, None, None]:
        if ws_id not in self.workspace_models:
            self.load_ws_model(graph, ws_id, override=override, **kwargs)
        for obj_id in _get_ws_objects_re(ws_id, self.workspace_models):
            yield self.object_models[obj_id]


class SceneModel(ModelBase):
    """RDF view of the objects, workspaces, and agents in a scene."""

    def __init__(self, graph: Graph, scene_id: URIRef) -> None:
        super().__init__(graph=graph, node_id=scene_id)
        self.objects: set[URIRef] = set()
        self.workspaces: set[URIRef] = set()
        self.agents: set[URIRef] = set()
        self.element_loader = SceneElementLoader()

        categories = {
            URI_BDD_TYPE_SCENE_OBJ: (URI_ENV_PRED_HAS_OBJ, self.objects),
            URI_BDD_TYPE_SCENE_WS: (URI_ENV_PRED_HAS_WS, self.workspaces),
            URI_BDD_TYPE_SCENE_AGN: (URI_AGN_PRED_HAS_AGN, self.agents),
        }
        for component_id in graph.subjects(URI_BDD_PRED_OF_SCENE, scene_id, unique=True):
            if not isinstance(component_id, URIRef):
                raise TypeError(f"scene '{scene_id}' has a non-URI component: {component_id}")
            component_types = get_node_types(graph=graph, node_id=component_id)
            matched = component_types & categories.keys()
            if len(matched) > 1:
                raise ValueError(f"scene component '{component_id}' has multiple categories")
            if not matched:
                continue
            predicate, elements = categories[matched.pop()]
            for element_id in graph.objects(component_id, predicate):
                if not isinstance(element_id, URIRef):
                    raise TypeError(
                        f"scene component '{component_id}' has a non-URI element: {element_id}"
                    )
                elements.add(element_id)

    def load_obj_model(
        self, graph: Graph, obj_id: URIRef, override: bool = False, **kwargs: Any
    ) -> ObjectModel:
        if obj_id not in self.objects:
            raise ValueError(f"object '{obj_id}' is not in scene '{self.id}'")
        return self.element_loader.load_object_model(graph, obj_id, override, **kwargs)

    def load_ws_model(
        self, graph: Graph, ws_id: URIRef, override: bool = False, **kwargs: Any
    ) -> WorkspaceModel:
        if ws_id not in self.workspaces:
            raise ValueError(f"workspace '{ws_id}' is not in scene '{self.id}'")
        return self.element_loader.load_ws_model(graph, ws_id, override, **kwargs)

    def load_agn_model(
        self, graph: Graph, agent_id: URIRef, override: bool = False, **kwargs: Any
    ) -> AgentModel:
        if agent_id not in self.agents:
            raise ValueError(f"agent '{agent_id}' is not in scene '{self.id}'")
        return self.element_loader.load_agent_model(graph, agent_id, override, **kwargs)

    def has_invariant_elem(self, elem_id: URIRef) -> bool:
        return elem_id in self.objects or elem_id in self.workspaces or elem_id in self.agents
