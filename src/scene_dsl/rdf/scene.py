# SPDX-License-Identifier: MPL-2.0
from typing import Any, Optional

from bdd_dsl.models.urirefs import (
    URI_AGN_PRED_HAS_AGN,
    URI_AGN_TYPE_AGN,
    URI_BDD_PRED_ELEMS,
    URI_BDD_TYPE_CONST_SET,
    URI_BDD_TYPE_SCENE,
    URI_BDD_TYPE_SCENE_AGN,
    URI_BDD_TYPE_SCENE_OBJ,
    URI_BDD_TYPE_SCENE_WS,
    URI_BDD_TYPE_SET,
    URI_ENV_PRED_HAS_OBJ,
    URI_ENV_PRED_HAS_WS,
    URI_ENV_PRED_OF_WS,
    URI_ENV_TYPE_OBJ,
    URI_ENV_TYPE_WS,
    URI_ENV_TYPE_WS_OBJ,
    URI_ENV_TYPE_WS_WS,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.classes.scene import (
    AgentSet,
    ObjectSet,
    SceneModel,
    SceneSet,
    SimilarAgentSet,
    SimilarObjectSet,
    WorkspaceComposition,
    WorkspaceSet,
)


def _add_scene_set_common(graph: Graph, scene_set: SceneSet, set_uris: set[URIRef]):
    if scene_set.uri in set_uris:
        return
    set_uris.add(scene_set.uri)

    graph.add(triple=(scene_set.uri, RDF.type, URI_BDD_TYPE_SET))
    graph.add(triple=(scene_set.uri, RDF.type, URI_BDD_TYPE_CONST_SET))
    graph.bind(prefix=scene_set.ns_prefix, namespace=scene_set.namespace)


def add_obj_set(
    graph: Graph,
    obj_set: ObjectSet | SimilarObjectSet,
    set_uris: set[URIRef],
    scn_comp_uri: Optional[URIRef] = None,
) -> None:
    _add_scene_set_common(graph=graph, scene_set=obj_set, set_uris=set_uris)

    for obj in obj_set.objects:
        graph.add(triple=(obj.uri, RDF.type, URI_ENV_TYPE_OBJ))
        graph.add(triple=(obj_set.uri, URI_BDD_PRED_ELEMS, obj.uri))
        if scn_comp_uri is not None:
            graph.add(triple=(scn_comp_uri, URI_ENV_PRED_HAS_OBJ, obj.uri))


def add_ws_set(
    graph: Graph, ws_set: WorkspaceSet, set_uris: set[URIRef], scn_comp_uri: Optional[URIRef] = None
) -> None:
    _add_scene_set_common(graph=graph, scene_set=ws_set, set_uris=set_uris)

    for ws in ws_set.workspaces:
        graph.add(triple=(ws.uri, RDF.type, URI_ENV_TYPE_WS))
        graph.add(triple=(ws_set.uri, URI_BDD_PRED_ELEMS, ws.uri))
        if scn_comp_uri is not None:
            graph.add(triple=(scn_comp_uri, URI_ENV_PRED_HAS_WS, ws.uri))


def add_agn_set(
    graph: Graph,
    agn_set: AgentSet | SimilarAgentSet,
    set_uris: set[URIRef],
    scn_comp_uri: Optional[URIRef] = None,
) -> None:
    _add_scene_set_common(graph=graph, scene_set=agn_set, set_uris=set_uris)

    for agn in agn_set.agents:
        graph.add(triple=(agn.uri, RDF.type, URI_AGN_TYPE_AGN))
        graph.add(triple=(agn_set.uri, URI_BDD_PRED_ELEMS, agn.uri))
        if scn_comp_uri is not None:
            graph.add(triple=(scn_comp_uri, URI_AGN_PRED_HAS_AGN, agn.uri))


def add_scene_set(
    graph: Graph, scene_set: SceneSet, set_uris: set[URIRef], scn_comp_uri: Optional[URIRef] = None
):
    if isinstance(scene_set, (ObjectSet, SimilarObjectSet)):
        add_obj_set(graph=graph, obj_set=scene_set, set_uris=set_uris, scn_comp_uri=scn_comp_uri)
    elif isinstance(scene_set, WorkspaceSet):
        add_ws_set(graph=graph, ws_set=scene_set, set_uris=set_uris, scn_comp_uri=scn_comp_uri)
    elif isinstance(scene_set, (AgentSet, SimilarAgentSet)):
        add_agn_set(graph=graph, agn_set=scene_set, set_uris=set_uris, scn_comp_uri=scn_comp_uri)
    else:
        raise ValueError(f"Unhandled SceneSet type: {type(scene_set)}")


def add_ws_comp(
    graph: Graph,
    scene: SceneModel,
    ws_comp: WorkspaceComposition,
    set_uris: set[URIRef],
    seen_ws_comp_uris: set[URIRef],
) -> None:
    # Keep scene composition a tree for now; reject both cycles and shared DAG reuse.
    if ws_comp.uri in seen_ws_comp_uris:
        raise RuntimeError(
            f"Workspace composition '{ws_comp.uri}' is reused. "
            "Shared or cyclic workspace compositions are not supported."
        )
    seen_ws_comp_uris.add(ws_comp.uri)

    graph.add(triple=(scene.scene_ws_uri, URI_ENV_PRED_HAS_WS, ws_comp.ws.uri))
    graph.add(triple=(ws_comp.ws.uri, RDF.type, URI_ENV_TYPE_WS))

    graph.add(triple=(ws_comp.uri, URI_ENV_PRED_OF_WS, ws_comp.ws.uri))
    if len(ws_comp.objects) > 0:
        graph.add(triple=(ws_comp.uri, RDF.type, URI_ENV_TYPE_WS_OBJ))
    if len(ws_comp.workspaces) > 0 or len(ws_comp.ws_comps):
        graph.add(triple=(ws_comp.uri, RDF.type, URI_ENV_TYPE_WS_WS))

    for obj in ws_comp.objects:
        if not isinstance(obj.parent, (ObjectSet, SimilarObjectSet)):
            raise TypeError(f"parent of obj not an object set: {obj.parent}")
        if obj.parent.uri not in set_uris:
            add_obj_set(graph=graph, obj_set=obj.parent, set_uris=set_uris)
            graph.add(triple=(scene.scene_obj_uri, URI_ENV_PRED_HAS_OBJ, obj.uri))
        graph.add(triple=(ws_comp.uri, URI_ENV_PRED_HAS_OBJ, obj.uri))

    for ws in ws_comp.workspaces:
        if not isinstance(ws.parent, WorkspaceSet):
            raise TypeError(f"parent of ws not a workspace set: {ws.parent}")
        if ws.parent.uri not in set_uris:
            add_ws_set(graph=graph, ws_set=ws.parent, set_uris=set_uris)
            graph.add(triple=(scene.scene_ws_uri, URI_ENV_PRED_HAS_WS, ws.uri))
        graph.add(triple=(ws_comp.uri, URI_ENV_PRED_HAS_WS, ws.uri))

    for child_comp in ws_comp.ws_comps:
        graph.add(triple=(ws_comp.uri, URI_ENV_PRED_HAS_WS, child_comp.ws.uri))
        add_ws_comp(
            graph=graph,
            scene=scene,
            ws_comp=child_comp,
            set_uris=set_uris,
            seen_ws_comp_uris=seen_ws_comp_uris,
        )


def add_scene_model(
    graph: Graph, scene: SceneModel, set_uris: set[URIRef]
) -> tuple[bool, bool, bool]:
    graph.bind(prefix=scene.ns_prefix, namespace=scene.namespace)
    graph.add(triple=(scene.uri, RDF.type, URI_BDD_TYPE_SCENE))

    for obj_set in scene.obj_sets:
        add_obj_set(
            graph=graph,
            obj_set=obj_set,
            set_uris=set_uris,
            scn_comp_uri=scene.scene_obj_uri,
        )

    for ws_set in scene.ws_sets:
        add_ws_set(
            graph=graph,
            ws_set=ws_set,
            set_uris=set_uris,
            scn_comp_uri=scene.scene_ws_uri,
        )

    for agn_set in scene.agn_sets:
        add_agn_set(
            graph=graph,
            agn_set=agn_set,
            set_uris=set_uris,
            scn_comp_uri=scene.scene_agn_uri,
        )

    for ws_comp in scene.ws_comps:
        add_ws_comp(
            graph=graph,
            scene=scene,
            ws_comp=ws_comp,
            set_uris=set_uris,
            seen_ws_comp_uris=set(),
        )

    scene_has_obj = (
        graph.value(subject=scene.scene_obj_uri, predicate=URI_ENV_PRED_HAS_OBJ) is not None
    )
    scene_has_ws = (
        graph.value(subject=scene.scene_ws_uri, predicate=URI_ENV_PRED_HAS_WS) is not None
    )
    scene_has_agn = (
        graph.value(subject=scene.scene_agn_uri, predicate=URI_AGN_PRED_HAS_AGN) is not None
    )
    if scene_has_obj:
        graph.add(triple=(scene.scene_obj_uri, RDF.type, URI_BDD_TYPE_SCENE_OBJ))
    if scene_has_ws:
        graph.add(triple=(scene.scene_ws_uri, RDF.type, URI_BDD_TYPE_SCENE_WS))
    if scene_has_agn:
        graph.add(triple=(scene.scene_agn_uri, RDF.type, URI_BDD_TYPE_SCENE_AGN))
    return scene_has_obj, scene_has_ws, scene_has_agn


def create_scene_model_graph(model: Any, g: Optional[Graph] = None) -> Graph:
    if g is None:
        g = Graph()

    scene_models = getattr(model, "scene_models", None)
    if scene_models is None or not isinstance(scene_models, list):
        raise ValueError("no 'scene_models' attr of type 'list' in model")
    set_uris = set()
    for scn in scene_models:
        add_scene_model(graph=g, scene=scn, set_uris=set_uris)

    return g
