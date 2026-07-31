# SPDX-License-Identifier: MPL-2.0
from collections.abc import Generator
from typing import Any

from rdf_utils.models.common import ModelBase, ModelLoader, get_node_types
from rdf_utils.models.vocab import (
    URI_AGN_PRED_HAS_AGN,
    URI_ENV_PRED_HAS_OBJ,
    URI_ENV_PRED_HAS_WS,
    URI_ENV_PRED_OF_WS,
    URI_ENV_TYPE_WS,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.rdf_parser.agent import AgentModel
from scene_dsl.rdf_parser.environment import ObjectModel, WorkspaceModel
from scene_dsl.rdf_parser.vocab import (
    URI_BDD_PRED_OF_SCENE,
    URI_BDD_TYPE_SCENE_AGN,
    URI_BDD_TYPE_SCENE_OBJ,
    URI_BDD_TYPE_SCENE_WS,
)


class SceneModel(ModelBase):
    """RDF view of the objects, workspaces, and agents in a scene."""

    def __init__(self, graph: Graph, scene_id: URIRef) -> None:
        super().__init__(graph=graph, node_id=scene_id)
        self.objects: dict[URIRef, ObjectModel] = {}
        self.workspaces: dict[URIRef, WorkspaceModel] = {}
        # Map each selected composition to its underlying workspace.
        self._ws_comps: dict[URIRef, URIRef] = {}
        self.agents: dict[URIRef, AgentModel] = {}
        self.element_loader = ModelLoader()

        categories = {
            URI_BDD_TYPE_SCENE_OBJ: URI_ENV_PRED_HAS_OBJ,
            URI_BDD_TYPE_SCENE_WS: URI_ENV_PRED_HAS_WS,
            URI_BDD_TYPE_SCENE_AGN: URI_AGN_PRED_HAS_AGN,
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

            predicate = categories[matched.pop()]
            for element_id in graph.objects(component_id, predicate):
                if not isinstance(element_id, URIRef):
                    raise TypeError(
                        f"scene component '{component_id}' has a non-URI element: {element_id}"
                    )

                if predicate == URI_ENV_PRED_HAS_OBJ:
                    self.objects[element_id] = ObjectModel(graph=graph, obj_id=element_id)
                elif predicate == URI_ENV_PRED_HAS_WS:
                    if (element_id, RDF.type, URI_ENV_TYPE_WS) in graph:
                        self.workspaces[element_id] = WorkspaceModel(ws_id=element_id, graph=graph)
                    else:
                        # assume this is a workspace compositions
                        self._load_ws_comp_re(ws_comp_id=element_id, graph=graph)
                elif predicate == URI_AGN_PRED_HAS_AGN:
                    self.agents[element_id] = AgentModel(graph=graph, agent_id=element_id)

    def _load_ws_comp_re(
        self, ws_comp_id: URIRef, graph: Graph, ws_path: set[URIRef] | None = None
    ):
        ws_ids = list(graph.objects(subject=ws_comp_id, predicate=URI_ENV_PRED_OF_WS, unique=True))
        if len(ws_ids) != 1 or not isinstance(ws_ids[0], URIRef):
            raise ValueError(
                f"WorkspaceComposition '{ws_comp_id.n3(self._ns_manager)}' for '{self}' doesn't link to 1 WS, found: {ws_ids}"
            )

        ws_id: URIRef = ws_ids[0]

        path = set() if ws_path is None else set(ws_path)
        if ws_id in path:
            raise RuntimeError(f"workspace loop detected at '{ws_id}'")
        path.add(ws_id)

        if ws_comp_id in self._ws_comps:
            return

        self._ws_comps[ws_comp_id] = ws_id

        if ws_id not in self.workspaces:
            self.workspaces[ws_id] = WorkspaceModel(ws_id=ws_id, graph=graph)

        ws_model = self.workspaces[ws_id]
        ws_model.load_ws_comp(ws_comp_id=ws_comp_id, graph=graph)

        for obj_id in ws_model.objects:
            if obj_id not in self.objects:
                self.objects[obj_id] = ObjectModel(obj_id=obj_id, graph=graph)

        for sub_ws_id in ws_model.workspaces:
            if sub_ws_id not in self.workspaces:
                self.workspaces[sub_ws_id] = WorkspaceModel(ws_id=sub_ws_id, graph=graph)

        for sub_comp_id in ws_model.ws_comps:
            self._load_ws_comp_re(ws_comp_id=sub_comp_id, graph=graph, ws_path=path)

    def load_obj_model(
        self, graph: Graph, obj_id: URIRef, override: bool = False, **kwargs: Any
    ) -> ObjectModel:
        if obj_id not in self.objects:
            raise ValueError(f"object '{obj_id}' is not in scene '{self.id}'")
        model = self.objects[obj_id]
        if not model.models_loaded or override:
            model.load_models(
                graph=graph,
                model_loader=self.element_loader,
                override=override,
                **kwargs,
            )
        return model

    def load_ws_model(self, ws_id: URIRef) -> WorkspaceModel:
        if ws_id not in self.workspaces:
            raise ValueError(f"workspace '{ws_id}' is not in scene '{self.id}'")
        return self.workspaces[ws_id]

    def load_ws_objects(
        self, graph: Graph, ws_id: URIRef, override: bool = False, **kwargs: Any
    ) -> Generator[ObjectModel, None, None]:
        if ws_id not in self.workspaces:
            raise ValueError(f"workspace '{ws_id}' is not in scene '{self.id}'")

        ws_model = self.workspaces[ws_id]
        for obj_id in ws_model.objects:
            yield self.load_obj_model(graph, obj_id, override=override, **kwargs)
        for child_ws_id in ws_model.workspaces:
            yield from self.load_ws_objects(graph, child_ws_id, override=override, **kwargs)
        for sub_comp_id in ws_model.ws_comps:
            yield from self.load_ws_objects(
                graph, self._ws_comps[sub_comp_id], override=override, **kwargs
            )

    def load_agn_model(
        self, graph: Graph, agent_id: URIRef, override: bool = False, **kwargs: Any
    ) -> AgentModel:
        if agent_id not in self.agents:
            raise ValueError(f"agent '{agent_id}' is not in scene '{self.id}'")
        model = self.agents[agent_id]
        if not model.models_loaded or override:
            model.load_models(
                graph=graph,
                model_loader=self.element_loader,
                override=override,
                **kwargs,
            )
        return model

    def has_invariant_elem(self, elem_id: URIRef) -> bool:
        return elem_id in self.objects or elem_id in self.workspaces or elem_id in self.agents
