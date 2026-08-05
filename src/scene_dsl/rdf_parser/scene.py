# SPDX-License-Identifier: MPL-2.0
from collections.abc import Generator

from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.vocab import (
    URI_AGN_PRED_HAS_AGN,
    URI_ENV_PRED_HAS_OBJ,
    URI_ENV_PRED_HAS_WS,
    URI_ENV_PRED_OF_WS,
    URI_ENV_TYPE_OBJ,
    URI_ENV_TYPE_WS,
    URI_ENV_TYPE_WS_OBJ,
    URI_ENV_TYPE_WS_WS,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.rdf_parser.vocab import (
    URI_BDD_PRED_OF_SCENE,
    URI_BDD_TYPE_SCENE_AGN,
    URI_BDD_TYPE_SCENE_OBJ,
    URI_BDD_TYPE_SCENE_WS,
)


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


class SceneModel(ModelBase):
    """RDF view of the objects, workspaces, and agents in a scene."""

    def __init__(self, graph: Graph, scene_id: URIRef) -> None:
        super().__init__(graph=graph, node_id=scene_id)
        self.objects: set[URIRef] = set()
        self.workspaces: dict[URIRef, WorkspaceModel] = {}
        # Map each selected composition to its underlying workspace.
        self._ws_comps: dict[URIRef, URIRef] = {}
        self.agents: set[URIRef] = set()

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
                    self.objects.add(element_id)
                elif predicate == URI_ENV_PRED_HAS_WS:
                    if (element_id, RDF.type, URI_ENV_TYPE_WS) in graph:
                        self.workspaces.setdefault(
                            element_id, WorkspaceModel(ws_id=element_id, graph=graph)
                        )
                    else:
                        # assume this is a workspace compositions
                        self._load_ws_comp_re(ws_comp_id=element_id, graph=graph)
                elif predicate == URI_AGN_PRED_HAS_AGN:
                    self.agents.add(element_id)

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
            self.objects.add(obj_id)

        for sub_ws_id in ws_model.workspaces:
            if sub_ws_id not in self.workspaces:
                self.workspaces[sub_ws_id] = WorkspaceModel(ws_id=sub_ws_id, graph=graph)

        for sub_comp_id in ws_model.ws_comps:
            self._load_ws_comp_re(ws_comp_id=sub_comp_id, graph=graph, ws_path=path)

    def iter_workspace_objects(self, ws_id: URIRef) -> Generator[URIRef, None, None]:
        if ws_id not in self.workspaces:
            raise ValueError(f"workspace '{ws_id}' is not in scene '{self.id}'")

        ws_model = self.workspaces[ws_id]
        yield from ws_model.objects
        for child_ws_id in ws_model.workspaces:
            yield from self.iter_workspace_objects(child_ws_id)
        for sub_comp_id in ws_model.ws_comps:
            yield from self.iter_workspace_objects(self._ws_comps[sub_comp_id])

    def has_invariant_elem(self, elem_id: URIRef) -> bool:
        return elem_id in self.objects or elem_id in self.workspaces or elem_id in self.agents
