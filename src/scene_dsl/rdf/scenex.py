# SPDX-License-Identifier: MPL-2.0
from typing import Any, Optional

from bdd_dsl.models.namespace import NS_MM_ROS
from bdd_dsl.models.urirefs import (
    URI_AGN_PRED_HAS_AGN_MODEL,
    URI_AGN_PRED_OF_AGN,
    URI_AGN_TYPE_AGN_MODEL,
    URI_AGN_TYPE_MOD_AGN,
    URI_BDD_PRED_OF_SCENE,
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_ENV_PRED_OF_OBJ,
    URI_ENV_TYPE_MOD_OBJ,
    URI_ENV_TYPE_OBJ_MODEL,
)
from rdf_utils.models.vocab import (
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_HAS_MODELLED_AGN,
    URI_EXEC_PRED_HAS_MODELLED_OBJ,
    URI_EXEC_PRED_MAPS,
    URI_EXEC_PRED_MODEL,
    URI_EXEC_PRED_MODEL_ENTITY,
    URI_EXEC_PRED_PATH,
    URI_EXEC_TYPE_RES_PATH,
    URI_EXEC_TYPE_SCENE_INST,
    URI_EXEC_TYPE_SCENE_MODEL,
    URI_EXEC_TYPE_SYS_RES,
)
from rdf_utils.namespace import (
    NS_MM_GEOM,
    NS_MM_GEOM_COORD,
    NS_MM_GEOM_REL,
    NS_MM_QUDT_QTY,
    NS_MM_QUDT_UNIT,
)
from rdflib import RDF, Graph, Literal, Namespace, URIRef

from scene_dsl.classes.scenex import (
    ElementModel,
    ModelledAgent,
    ModelledAgentSet,
    ModelledObject,
    ModelledObjectSet,
    SceneInstance,
    TreeMapping,
)
from scene_dsl.rdf.common import add_py_module_attr
from scene_dsl.rdf.distrib import add_distribution
from scene_dsl.rdf.ktree import add_kinematic_graph, add_kinematic_tree
from scene_dsl.rdf.scene import add_scene_model
from scene_dsl.rdf.sensors import add_sensors


NS_XML = Namespace("https://www.w3.org/TR/2006/REC-xml11-20060816#")
NS_URDF = Namespace("https://wiki.ros.org/urdf/XML/")
NS_MJCF = Namespace("https://mujoco.readthedocs.io/en/stable/XMLreference.html#")
NS_USD = Namespace("https://openusd.org/release/spec.html#")

URI_XML_DOCUMENT = NS_XML["document"]
URI_URDF_ROBOT = NS_URDF["robot"]
URI_MJCF_MUJOCO = NS_MJCF["mujoco"]
URI_USD_STAGE = NS_USD["stage"]


def _bind_model_kind_namespaces(graph: Graph) -> None:
    graph.bind(prefix="xml", namespace=NS_XML)
    graph.bind(prefix="urdf", namespace=NS_URDF)
    graph.bind(prefix="mjcf", namespace=NS_MJCF)
    graph.bind(prefix="usd", namespace=NS_USD)
    graph.bind(prefix="geom", namespace=NS_MM_GEOM)
    graph.bind(prefix="geom-rel", namespace=NS_MM_GEOM_REL)
    graph.bind(prefix="geom-coord", namespace=NS_MM_GEOM_COORD)
    graph.bind(prefix="quantitykind", namespace=NS_MM_QUDT_QTY)
    graph.bind(prefix="unit", namespace=NS_MM_QUDT_UNIT)


def add_model_spec(graph: Graph, elem_model: ElementModel) -> None:
    model_spec_type = elem_model.model_spec.__class__.__name__
    if "PyModuleAttr" in model_spec_type:
        add_py_module_attr(graph=graph, node_uri=elem_model.uri, py_model=elem_model.model_spec)
    elif "SystemPath" in model_spec_type:
        graph.add((elem_model.uri, RDF.type, URI_EXEC_TYPE_RES_PATH))
        graph.add((elem_model.uri, RDF.type, URI_EXEC_TYPE_SYS_RES))
        graph.add((elem_model.uri, URI_EXEC_PRED_PATH, Literal(elem_model.model_spec.path)))
    elif "RosPath" in model_spec_type:
        graph.add((elem_model.uri, RDF.type, URI_EXEC_TYPE_RES_PATH))
        graph.add((elem_model.uri, RDF.type, URI_EXEC_TYPE_SYS_RES))
        graph.add((elem_model.uri, RDF.type, NS_MM_ROS["Package"]))
        graph.add((elem_model.uri, NS_MM_ROS["package-name"], Literal(elem_model.model_spec.pkg)))
        graph.add((elem_model.uri, URI_EXEC_PRED_PATH, Literal(elem_model.model_spec.path)))
    else:
        raise ValueError(f"Unhandled model specification type: {model_spec_type}")

    if elem_model.model_kind == "urdf":
        graph.add((elem_model.uri, RDF.type, URI_XML_DOCUMENT))
        graph.add((elem_model.uri, RDF.type, URI_URDF_ROBOT))
    elif elem_model.model_kind == "mjcf":
        graph.add((elem_model.uri, RDF.type, URI_XML_DOCUMENT))
        graph.add((elem_model.uri, RDF.type, URI_MJCF_MUJOCO))
    elif elem_model.model_kind == "usd":
        graph.add((elem_model.uri, RDF.type, URI_USD_STAGE))
    elif elem_model.model_kind is not None:
        raise ValueError(f"Unhandled model kind: {elem_model.model_kind}")


def _ensure_unique_scene_uri(
    node_uri: URIRef, scene_inst: SceneInstance, seen_uris: set[URIRef]
) -> None:
    if node_uri in seen_uris:
        raise ValueError(
            f"Duplicate model URI '{node_uri}' in scene instance '{scene_inst.uri}'. "
            "Use unique model names within a scene instance."
        )
    seen_uris.add(node_uri)


def add_element_model(
    graph: Graph,
    scene_inst: SceneInstance,
    elem_model: ElementModel,
    type_uri: URIRef,
    seen_model_uris: set[URIRef],
    seen_ktrees: set[URIRef],
) -> None:
    """A model file: what it is, where it lives, and what of the scene it stands for."""
    _ensure_unique_scene_uri(elem_model.uri, scene_inst, seen_model_uris)
    graph.add((elem_model.uri, RDF.type, type_uri))
    add_model_spec(graph, elem_model)

    # A node per mapping: one model may map several targets, so `entity` cannot hang off
    # the model itself. What it maps says its kind, so it needs no type of its own.
    for mapping in elem_model.mappings:
        target = mapping.target
        graph.add((elem_model.uri, URI_EXEC_PRED_HAS_MAPPING, mapping.uri))
        graph.add((mapping.uri, URI_EXEC_PRED_MAPS, target.uri))
        if mapping.entity is not None:
            graph.add((mapping.uri, URI_EXEC_PRED_MODEL_ENTITY, Literal(mapping.entity)))
        if isinstance(mapping, TreeMapping) and target.uri not in seen_ktrees:
            _ensure_unique_scene_uri(target.uri, scene_inst, seen_model_uris)
            add_kinematic_tree(graph, target, seen_trees=seen_ktrees)


def add_modelled_obj(
    graph: Graph,
    scene_inst: SceneInstance,
    obj_model: ModelledObject,
    seen_model_uris: set[URIRef],
    seen_ktrees: set[URIRef],
) -> None:
    graph.add((obj_model.modelled_uri, RDF.type, URI_ENV_TYPE_MOD_OBJ))
    graph.add((obj_model.modelled_uri, URI_ENV_PRED_OF_OBJ, obj_model.obj.uri))
    graph.add((scene_inst.uri, URI_EXEC_PRED_HAS_MODELLED_OBJ, obj_model.modelled_uri))

    for model in obj_model.models:
        add_element_model(
            graph, scene_inst, model, URI_ENV_TYPE_OBJ_MODEL, seen_model_uris, seen_ktrees
        )
        graph.add((obj_model.modelled_uri, URI_ENV_PRED_HAS_OBJ_MODEL, model.uri))


def add_modelled_agn(
    graph: Graph,
    scene_inst: SceneInstance,
    agn_model: ModelledAgent,
    seen_model_uris: set[URIRef],
    seen_ktrees: set[URIRef],
) -> None:
    graph.add((agn_model.modelled_uri, RDF.type, URI_AGN_TYPE_MOD_AGN))
    graph.add((agn_model.modelled_uri, URI_AGN_PRED_OF_AGN, agn_model.agn.uri))
    graph.add((scene_inst.uri, URI_EXEC_PRED_HAS_MODELLED_AGN, agn_model.modelled_uri))

    for model in agn_model.models:
        add_element_model(
            graph, scene_inst, model, URI_AGN_TYPE_AGN_MODEL, seen_model_uris, seen_ktrees
        )
        graph.add((agn_model.modelled_uri, URI_AGN_PRED_HAS_AGN_MODEL, model.uri))

    add_sensors(graph, agn_model)


def add_modelled_agn_set(
    graph: Graph,
    scene_inst: SceneInstance,
    agn_model_set: ModelledAgentSet,
    seen_model_uris: set[URIRef],
    seen_ktrees: set[URIRef],
) -> None:
    # The grouping is a claim of membership, checked in check_agent_set_membership: the
    # agents are already tied to their set in the scene model, so it adds no triple.
    for agn_model in agn_model_set.models:
        add_modelled_agn(graph, scene_inst, agn_model, seen_model_uris, seen_ktrees)


def add_modelled_obj_set(
    graph: Graph,
    scene_inst: SceneInstance,
    obj_model_set: ModelledObjectSet,
    seen_model_uris: set[URIRef],
    seen_ktrees: set[URIRef],
) -> None:
    for model in obj_model_set.models:
        add_element_model(
            graph, scene_inst, model, URI_ENV_TYPE_OBJ_MODEL, seen_model_uris, seen_ktrees
        )
        for index, obj in enumerate(obj_model_set.obj_set.objects):
            modelled_uri = obj_model_set.modelled_uri(index=index)
            graph.add((modelled_uri, RDF.type, URI_ENV_TYPE_MOD_OBJ))
            graph.add((modelled_uri, URI_ENV_PRED_OF_OBJ, obj.uri))
            graph.add((scene_inst.uri, URI_EXEC_PRED_HAS_MODELLED_OBJ, modelled_uri))
            graph.add((modelled_uri, URI_ENV_PRED_HAS_OBJ_MODEL, model.uri))


def add_modelled_scene(graph: Graph, scene_inst: SceneInstance) -> None:
    graph.bind(prefix=scene_inst.ns_prefix, namespace=scene_inst.namespace)
    graph.add((scene_inst.uri, RDF.type, URI_EXEC_TYPE_SCENE_INST))
    graph.add((scene_inst.uri, URI_BDD_PRED_OF_SCENE, scene_inst.scene.uri))

    add_scene_model(graph=graph, scene=scene_inst.scene, set_uris=set())

    seen_model_uris = set()
    seen_ktrees = set()
    if scene_inst.kgraph is not None:
        _ensure_unique_scene_uri(scene_inst.kgraph.uri, scene_inst, seen_model_uris)
        add_kinematic_graph(graph, scene_inst.kgraph, seen_trees=seen_ktrees)

    for model in scene_inst.models:
        add_element_model(
            graph, scene_inst, model, URI_EXEC_TYPE_SCENE_MODEL, seen_model_uris, seen_ktrees
        )
        graph.add((scene_inst.uri, URI_EXEC_PRED_MODEL, model.uri))

    for obj_model in scene_inst.modelled_objs:
        add_modelled_obj(graph, scene_inst, obj_model, seen_model_uris, seen_ktrees)
    for obj_model_set in scene_inst.modelled_obj_sets:
        add_modelled_obj_set(graph, scene_inst, obj_model_set, seen_model_uris, seen_ktrees)
    for agn_model in scene_inst.modelled_agns:
        add_modelled_agn(graph, scene_inst, agn_model, seen_model_uris, seen_ktrees=seen_ktrees)
    for agn_model_set in scene_inst.modelled_agn_sets:
        add_modelled_agn_set(graph, scene_inst, agn_model_set, seen_model_uris, seen_ktrees)


def create_scenex_model_graph(model: Any, g: Optional[Graph] = None) -> Graph:
    if g is None:
        g = Graph()

    _bind_model_kind_namespaces(graph=g)

    for distribution in getattr(model, "distributions", []):
        add_distribution(graph=g, distribution=distribution)

    for scene_inst in getattr(model, "scene_insts", []):
        add_modelled_scene(graph=g, scene_inst=scene_inst)

    return g
