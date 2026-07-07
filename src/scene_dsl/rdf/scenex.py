# SPDX-License-Identifier: MPL-2.0
from typing import Any, Optional

from bdd_dsl.models.namespace import NS_MM_EXEC, NS_MM_ROS
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
    URI_EXEC_PRED_PATH,
    URI_EXEC_TYPE_RES_PATH,
    URI_EXEC_TYPE_SYS_RES,
)
from rdf_utils.namespace import (
    NS_MM_AGN,
    NS_MM_GEOM,
    NS_MM_GEOM_COORD,
    NS_MM_GEOM_REL,
    NS_MM_QUDT,
    NS_MM_QUDT_QTY,
    NS_MM_QUDT_UNIT,
)
from rdf_utils.models.geometry import (
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_OF,
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_WRT,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_POINT,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_TYPE_DEG,
    URI_QUDT_TYPE_RAD,
)
from rdflib import RDF, Graph, Literal, Namespace, URIRef
from textx.scoping import get_included_models

from scene_dsl.classes.scenex import (
    BodySpec,
    ElementModel,
    EulerOrientationSpec,
    Frame,
    GeometrySpec,
    ModelledAgent,
    ModelledAgentSet,
    ModelledObject,
    ModelledObjectSet,
    SceneInstance,
)
from scene_dsl.rdf.common import add_py_module_attr
from scene_dsl.rdf.scene import add_scene_model


URI_EXEC_TYPE_SCENE_INST = NS_MM_EXEC["SceneInstance"]
URI_EXEC_PRED_HAS_MODELLED_OBJ = NS_MM_EXEC["has-modelled-object"]
URI_EXEC_PRED_HAS_MODELLED_AGN = NS_MM_EXEC["has-modelled-agent"]
URI_EXEC_PRED_HAS_FIXED_ATTACHMENT = NS_MM_EXEC["has-fixed-attachment"]
URI_GEOM_TYPE_SIMPLICIAL_COMPLEX = NS_MM_GEOM["SimplicialComplex"]
URI_GEOM_TYPE_GEOMETRY_MODEL = NS_MM_GEOM["GeometryModel"]
URI_GEOM_TYPE_RIGID_BODY = NS_MM_GEOM["RigidBody"]
URI_GEOM_PRED_HAS_FRAME = NS_MM_GEOM["has-frame"]
URI_GEOM_PRED_SIMPLICES = NS_MM_GEOM["simplices"]
URI_AGN_TYPE_ATTACHMENT_MODEL = NS_MM_AGN["AttachmentModel"]
NS_MM_KC = Namespace("https://comp-rob2b.github.io/metamodels/kinematic-chain/structural-entities#")
NS_MM_DYN_ENT = Namespace(
    "https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/structural-entities#"
)
NS_MM_DYN_COORD = Namespace(
    "https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/coordinates#"
)
URI_KC_TYPE_KINEMATIC_CHAIN = NS_MM_KC["KinematicChain"]
URI_DYN_TYPE_RIGID_BODY_INERTIA = NS_MM_DYN_ENT["RigidBodyInertia"]
URI_DYN_TYPE_INERTIA_REFERENCE = NS_MM_DYN_COORD["InertiaReference"]
URI_DYN_TYPE_RIGID_BODY_INERTIA_COORD = NS_MM_DYN_COORD["RigidBodyInertiaCoordinate"]
URI_DYN_TYPE_MASS_SCALAR = NS_MM_DYN_COORD["MassScalar"]
URI_DYN_PRED_OF_BODY = NS_MM_DYN_ENT["of-body"]
URI_DYN_PRED_ABOUT = NS_MM_DYN_ENT["about"]
URI_DYN_PRED_OF_INERTIA = NS_MM_DYN_COORD["of-inertia"]
URI_DYN_PRED_AS_SEEN_BY = NS_MM_DYN_COORD["as-seen-by"]
URI_DYN_PRED_MASS = NS_MM_DYN_COORD["mass"]
URI_QUDT_PRED_UNIT = NS_MM_QUDT["unit"]
URI_QUDT_PRED_QUANTITY_KIND = NS_MM_QUDT["hasQuantityKind"]
URI_QUDT_QK_MASS = NS_MM_QUDT_QTY["Mass"]
URI_QUDT_UNIT_KG = NS_MM_QUDT_UNIT["KiloGM"]
URI_QUDT_UNIT_G = NS_MM_QUDT_UNIT["GM"]
URI_QUDT_UNIT_M = NS_MM_QUDT_UNIT["M"]
URI_QUDT_UNIT_CM = NS_MM_QUDT_UNIT["CentiM"]
URI_QUDT_UNIT_MM = NS_MM_QUDT_UNIT["MilliM"]
LENGTH_UNITS = {"m": URI_QUDT_UNIT_M, "cm": URI_QUDT_UNIT_CM, "mm": URI_QUDT_UNIT_MM}
MASS_UNITS = {"kg": URI_QUDT_UNIT_KG, "g": URI_QUDT_UNIT_G}

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
    graph.bind(prefix="kc", namespace=NS_MM_KC)
    graph.bind(prefix="dyn-ent", namespace=NS_MM_DYN_ENT)
    graph.bind(prefix="dyn-coord", namespace=NS_MM_DYN_COORD)
    graph.bind(prefix="quantitykind", namespace=NS_MM_QUDT_QTY)
    graph.bind(prefix="unit", namespace=NS_MM_QUDT_UNIT)


def add_frame(graph: Graph, frame: Frame) -> None:
    graph.add(triple=(frame.uri, RDF.type, URI_GEOM_TYPE_FRAME))
    graph.add(triple=(frame.origin_uri, RDF.type, URI_GEOM_TYPE_POINT))
    graph.add(triple=(frame.uri, URI_GEOM_PRED_ORIGIN, frame.origin_uri))


def add_geometry_model(
    graph: Graph,
    geom_spec: GeometrySpec,
    node_id: Optional[URIRef] = None,
    scene_geom: Optional[GeometrySpec] = None,
) -> None:
    if node_id is None:
        node_id = geom_spec.uri

    graph.add(triple=(node_id, RDF.type, URI_GEOM_TYPE_GEOMETRY_MODEL))

    for frame in [geom_spec.root, *geom_spec.frames]:
        add_frame(graph=graph, frame=frame)
        graph.add(triple=(node_id, URI_GEOM_PRED_HAS_FRAME, frame.uri))

    if geom_spec.pose is None:
        return

    pose = geom_spec.pose
    if geom_spec.pose.wrt is not None:
        wrt_frame = geom_spec.pose.wrt
    elif scene_geom is not None:
        wrt_frame = scene_geom.root
    else:
        raise ValueError(
            f"No reference frame specified and no default global pose for GeometrySpec {geom_spec.uri}"
        )

    if geom_spec.root.uri == wrt_frame.uri:
        raise ValueError(
            f"GeometrySpec {geom_spec.uri}: reference pose URI is the same with target pose: {wrt_frame.uri}"
        )

    pose_uri = geom_spec.pose_uri(wrt=wrt_frame)
    graph.add(triple=(pose_uri, RDF.type, URI_GEOM_TYPE_POSE))
    graph.add(triple=(pose_uri, URI_GEOM_PRED_OF, geom_spec.root.uri))
    graph.add(triple=(pose_uri, URI_GEOM_PRED_WRT, wrt_frame.uri))

    # Coordinate
    coord_uri = geom_spec.pose_coord_uri(wrt=wrt_frame)
    graph.add(triple=(coord_uri, RDF.type, URI_GEOM_TYPE_POSE_REF))
    graph.add(triple=(coord_uri, URI_GEOM_PRED_OF_POSE, pose_uri))
    graph.add(triple=(coord_uri, RDF.type, URI_GEOM_TYPE_POSE_COORD))
    graph.add(triple=(coord_uri, URI_GEOM_PRED_SEEN_BY, wrt_frame.uri))
    # Position Coords
    graph.add(triple=(coord_uri, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    graph.add(triple=(coord_uri, URI_GEOM_PRED_X, Literal(pose.x)))
    graph.add(triple=(coord_uri, URI_GEOM_PRED_Y, Literal(pose.y)))
    graph.add(triple=(coord_uri, URI_GEOM_PRED_Z, Literal(pose.z)))
    graph.add(triple=(coord_uri, URI_QUDT_PRED_UNIT, LENGTH_UNITS[pose.length_unit]))

    if isinstance(pose.orientation, EulerOrientationSpec):
        graph.add(triple=(coord_uri, RDF.type, URI_GEOM_TYPE_EULER_ANGLES))

        graph.add(triple=(coord_uri, URI_GEOM_PRED_AXES_SEQ, Literal(pose.orientation.axes)))

        in_ex_type = (
            URI_GEOM_TYPE_EXTRINSIC if pose.orientation.extrinsic else URI_GEOM_TYPE_INTRINSIC
        )
        graph.add(triple=(coord_uri, RDF.type, in_ex_type))

        graph.add(triple=(coord_uri, RDF.type, URI_GEOM_TYPE_ANGLES_ABG))
        graph.add(triple=(coord_uri, URI_GEOM_PRED_ALPHA, Literal(pose.orientation.alpha)))
        graph.add(triple=(coord_uri, URI_GEOM_PRED_BETA, Literal(pose.orientation.beta)))
        graph.add(triple=(coord_uri, URI_GEOM_PRED_GAMMA, Literal(pose.orientation.gamma)))

        unit_uri = URI_QUDT_TYPE_DEG if pose.orientation.unit == "deg" else URI_QUDT_TYPE_RAD
        graph.add(triple=(coord_uri, URI_QUDT_PRED_UNIT, unit_uri))
    else:
        raise ValueError(f"Unsupported orientation: {pose.orientation}")


def add_body(graph: Graph, body: BodySpec) -> None:
    graph.add(triple=(body.uri, RDF.type, URI_GEOM_TYPE_RIGID_BODY))
    graph.add(triple=(body.uri, RDF.type, URI_GEOM_TYPE_SIMPLICIAL_COMPLEX))
    graph.add(triple=(body.uri, URI_GEOM_PRED_SIMPLICES, body.frame.uri))
    graph.add(triple=(body.uri, URI_GEOM_PRED_SIMPLICES, body.frame.origin_uri))

    if body.mass is None:
        return

    graph.add(triple=(body.inertia_uri, RDF.type, URI_DYN_TYPE_RIGID_BODY_INERTIA))
    graph.add(triple=(body.inertia_uri, URI_DYN_PRED_OF_BODY, body.uri))
    graph.add(triple=(body.inertia_uri, URI_DYN_PRED_ABOUT, body.frame.origin_uri))
    graph.add(triple=(body.inertia_uri, URI_QUDT_PRED_QUANTITY_KIND, URI_QUDT_QK_MASS))

    graph.add(triple=(body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_INERTIA_REFERENCE))
    graph.add(triple=(body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_RIGID_BODY_INERTIA_COORD))
    graph.add(triple=(body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MASS_SCALAR))
    graph.add(triple=(body.inertia_coord_uri, URI_DYN_PRED_OF_INERTIA, body.inertia_uri))
    graph.add(triple=(body.inertia_coord_uri, URI_DYN_PRED_AS_SEEN_BY, body.frame.uri))
    graph.add(triple=(body.inertia_coord_uri, URI_QUDT_PRED_UNIT, MASS_UNITS[body.mass.unit]))
    graph.add(triple=(body.inertia_coord_uri, URI_DYN_PRED_MASS, Literal(body.mass.value)))


def add_model_spec(graph: Graph, elem_model: ElementModel) -> None:
    model_spec_type = elem_model.model_spec.__class__.__name__
    if "PyModuleAttr" in model_spec_type:
        add_py_module_attr(graph=graph, node_uri=elem_model.uri, py_model=elem_model.model_spec)
    elif "SystemPath" in model_spec_type:
        graph.add(triple=(elem_model.uri, RDF.type, URI_EXEC_TYPE_RES_PATH))
        graph.add(triple=(elem_model.uri, RDF.type, URI_EXEC_TYPE_SYS_RES))
        path_l = Literal(elem_model.model_spec.path)
        graph.add(triple=(elem_model.uri, URI_EXEC_PRED_PATH, path_l))
    elif "RosPath" in model_spec_type:
        graph.add(triple=(elem_model.uri, RDF.type, URI_EXEC_TYPE_RES_PATH))
        graph.add(triple=(elem_model.uri, RDF.type, URI_EXEC_TYPE_SYS_RES))
        graph.add(triple=(elem_model.uri, RDF.type, NS_MM_ROS["Package"]))
        pkg_name_l = Literal(elem_model.model_spec.pkg)
        graph.add(triple=(elem_model.uri, NS_MM_ROS["package-name"], pkg_name_l))
        path_l = Literal(elem_model.model_spec.path)
        graph.add(triple=(elem_model.uri, URI_EXEC_PRED_PATH, path_l))
    else:
        raise ValueError(f"Unhandled model specification type: {model_spec_type}")

    if elem_model.model_kind == "urdf":
        graph.add(triple=(elem_model.uri, RDF.type, URI_XML_DOCUMENT))
        graph.add(triple=(elem_model.uri, RDF.type, URI_URDF_ROBOT))
    elif elem_model.model_kind == "mjcf":
        graph.add(triple=(elem_model.uri, RDF.type, URI_XML_DOCUMENT))
        graph.add(triple=(elem_model.uri, RDF.type, URI_MJCF_MUJOCO))
    elif elem_model.model_kind == "usd":
        graph.add(triple=(elem_model.uri, RDF.type, URI_USD_STAGE))
    elif elem_model.model_kind is not None:
        raise ValueError(f"Unhandled model kind: {elem_model.model_kind}")


def _ensure_unique_scene_uri(
    node_uri: URIRef,
    scene_inst: SceneInstance,
    seen_uris: set[URIRef],
) -> None:
    if node_uri in seen_uris:
        raise ValueError(
            f"Duplicate model URI '{node_uri}' in scene instance '{scene_inst.uri}'. "
            "Use unique model names within a scene instance."
        )
    seen_uris.add(node_uri)


def add_modelled_obj(
    graph: Graph,
    scene_inst: SceneInstance,
    obj_model: ModelledObject,
    seen_model_uris: set[URIRef],
) -> None:
    graph.add(triple=(obj_model.modelled_uri, RDF.type, URI_ENV_TYPE_MOD_OBJ))
    graph.add(triple=(obj_model.modelled_uri, URI_ENV_PRED_OF_OBJ, obj_model.obj.uri))
    graph.add(triple=(scene_inst.uri, URI_EXEC_PRED_HAS_MODELLED_OBJ, obj_model.modelled_uri))
    obj_geom = obj_model.geometry
    if obj_geom is not None:
        _ensure_unique_scene_uri(
            node_uri=obj_geom.uri,
            scene_inst=scene_inst,
            seen_uris=seen_model_uris,
        )
        graph.add(triple=(obj_model.modelled_uri, URI_ENV_PRED_HAS_OBJ_MODEL, obj_geom.uri))
        add_geometry_model(
            graph=graph,
            node_id=obj_geom.uri,
            geom_spec=obj_geom,
            scene_geom=scene_inst.geometry,
        )
    if obj_model.body is not None:
        _ensure_unique_scene_uri(
            node_uri=obj_model.body.uri,
            scene_inst=scene_inst,
            seen_uris=seen_model_uris,
        )
        add_body(graph=graph, body=obj_model.body)

    for model in obj_model.models:
        _ensure_unique_scene_uri(
            node_uri=model.uri, scene_inst=scene_inst, seen_uris=seen_model_uris
        )
        graph.add(triple=(obj_model.modelled_uri, URI_ENV_PRED_HAS_OBJ_MODEL, model.uri))
        graph.add(triple=(model.uri, RDF.type, URI_ENV_TYPE_OBJ_MODEL))
        add_model_spec(graph=graph, elem_model=model)


def add_modelled_agn(
    graph: Graph,
    scene_inst: SceneInstance,
    agn_model: ModelledAgent,
    seen_model_uris: set[URIRef],
) -> None:
    graph.add(triple=(agn_model.modelled_uri, RDF.type, URI_AGN_TYPE_MOD_AGN))
    graph.add(triple=(agn_model.modelled_uri, URI_AGN_PRED_OF_AGN, agn_model.agn.uri))
    graph.add(triple=(scene_inst.uri, URI_EXEC_PRED_HAS_MODELLED_AGN, agn_model.modelled_uri))

    # Kinematic chain model
    kc_model = agn_model.kinematic.model
    _ensure_unique_scene_uri(
        node_uri=kc_model.uri, scene_inst=scene_inst, seen_uris=seen_model_uris
    )
    graph.add(triple=(agn_model.modelled_uri, URI_AGN_PRED_HAS_AGN_MODEL, kc_model.uri))
    graph.add(triple=(kc_model.uri, RDF.type, URI_AGN_TYPE_AGN_MODEL))
    graph.add(triple=(kc_model.uri, RDF.type, URI_KC_TYPE_KINEMATIC_CHAIN))
    add_model_spec(graph=graph, elem_model=kc_model)

    # Add geometry model to the same node
    _ensure_unique_scene_uri(
        node_uri=agn_model.kinematic.geometry.uri,
        scene_inst=scene_inst,
        seen_uris=seen_model_uris,
    )
    add_geometry_model(
        graph=graph,
        geom_spec=agn_model.kinematic.geometry,
        node_id=kc_model.uri,
        scene_geom=scene_inst.geometry,
    )

    for attachment in agn_model.attachments:
        model = attachment.model
        if model is None:
            continue
        _ensure_unique_scene_uri(
            node_uri=model.uri, scene_inst=scene_inst, seen_uris=seen_model_uris
        )
        graph.add(triple=(agn_model.modelled_uri, URI_AGN_PRED_HAS_AGN_MODEL, model.uri))
        graph.add(triple=(agn_model.modelled_uri, URI_EXEC_PRED_HAS_FIXED_ATTACHMENT, model.uri))
        graph.add(triple=(model.uri, RDF.type, URI_AGN_TYPE_AGN_MODEL))
        graph.add(triple=(model.uri, RDF.type, URI_AGN_TYPE_ATTACHMENT_MODEL))
        add_model_spec(graph=graph, elem_model=model)
        if attachment.geometry is not None:
            _ensure_unique_scene_uri(
                node_uri=attachment.geometry.uri,
                scene_inst=scene_inst,
                seen_uris=seen_model_uris,
            )
            add_geometry_model(
                graph=graph,
                node_id=model.uri,
                geom_spec=attachment.geometry,
                scene_geom=scene_inst.geometry,
            )
        if attachment.body is not None:
            _ensure_unique_scene_uri(
                node_uri=attachment.body.uri,
                scene_inst=scene_inst,
                seen_uris=seen_model_uris,
            )
            add_body(graph=graph, body=attachment.body)


def add_modelled_obj_set(
    graph: Graph,
    scene_inst: SceneInstance,
    obj_model_set: ModelledObjectSet,
    seen_model_uris: set[URIRef],
) -> None:
    for model in obj_model_set.models:
        _ensure_unique_scene_uri(
            node_uri=model.uri, scene_inst=scene_inst, seen_uris=seen_model_uris
        )
        graph.add(triple=(model.uri, RDF.type, URI_ENV_TYPE_OBJ_MODEL))
        add_model_spec(graph=graph, elem_model=model)

        for index, obj in enumerate(obj_model_set.obj_set.objects):
            modelled_uri = obj_model_set.modelled_uri(index=index)
            graph.add(triple=(modelled_uri, RDF.type, URI_ENV_TYPE_MOD_OBJ))
            graph.add(triple=(modelled_uri, URI_ENV_PRED_OF_OBJ, obj.uri))
            graph.add(triple=(scene_inst.uri, URI_EXEC_PRED_HAS_MODELLED_OBJ, modelled_uri))
            graph.add(triple=(modelled_uri, URI_ENV_PRED_HAS_OBJ_MODEL, model.uri))


def add_modelled_agn_set(
    graph: Graph,
    scene_inst: SceneInstance,
    agn_model_set: ModelledAgentSet,
    seen_model_uris: set[URIRef],
) -> None:
    for model in agn_model_set.models:
        _ensure_unique_scene_uri(
            node_uri=model.uri, scene_inst=scene_inst, seen_uris=seen_model_uris
        )
        graph.add(triple=(model.uri, RDF.type, URI_AGN_TYPE_AGN_MODEL))
        add_model_spec(graph=graph, elem_model=model)

        for index, agn in enumerate(agn_model_set.agn_set.agents):
            modelled_uri = agn_model_set.modelled_uri(index=index)
            graph.add(triple=(modelled_uri, RDF.type, URI_AGN_TYPE_MOD_AGN))
            graph.add(triple=(modelled_uri, URI_AGN_PRED_OF_AGN, agn.uri))
            graph.add(triple=(scene_inst.uri, URI_EXEC_PRED_HAS_MODELLED_AGN, modelled_uri))
            graph.add(triple=(modelled_uri, URI_AGN_PRED_HAS_AGN_MODEL, model.uri))


def add_modelled_scene(graph: Graph, scene_inst: SceneInstance) -> None:
    graph.bind(prefix=scene_inst.ns_prefix, namespace=scene_inst.namespace)
    graph.add(triple=(scene_inst.uri, RDF.type, URI_EXEC_TYPE_SCENE_INST))
    graph.add(triple=(scene_inst.uri, URI_BDD_PRED_OF_SCENE, scene_inst.scene.uri))

    seen_model_uris = set()

    if scene_inst.geometry is not None:
        _ensure_unique_scene_uri(
            node_uri=scene_inst.geometry.uri,
            scene_inst=scene_inst,
            seen_uris=seen_model_uris,
        )
        add_geometry_model(graph=graph, geom_spec=scene_inst.geometry)

    for obj_model in scene_inst.modelled_objs:
        add_modelled_obj(
            graph=graph,
            scene_inst=scene_inst,
            obj_model=obj_model,
            seen_model_uris=seen_model_uris,
        )

    for obj_model_set in scene_inst.modelled_obj_sets:
        add_modelled_obj_set(
            graph=graph,
            scene_inst=scene_inst,
            obj_model_set=obj_model_set,
            seen_model_uris=seen_model_uris,
        )

    for agn_model in scene_inst.modelled_agns:
        add_modelled_agn(
            graph=graph,
            scene_inst=scene_inst,
            agn_model=agn_model,
            seen_model_uris=seen_model_uris,
        )

    for agn_model_set in scene_inst.modelled_agn_sets:
        add_modelled_agn_set(
            graph=graph,
            scene_inst=scene_inst,
            agn_model_set=agn_model_set,
            seen_model_uris=seen_model_uris,
        )


def _iter_scene_models(model: Any):
    yield from getattr(model, "scene_models", []) or []
    for included_model in get_included_models(model):
        yield from getattr(included_model, "scene_models", []) or []


def create_scenex_model_graph(model: Any, g: Optional[Graph] = None) -> Graph:
    if g is None:
        g = Graph()

    _bind_model_kind_namespaces(graph=g)

    set_uris = set()
    for scn in _iter_scene_models(model):
        add_scene_model(graph=g, scene=scn, set_uris=set_uris)

    scene_insts = getattr(model, "scene_insts", None)
    if scene_insts is None or not isinstance(scene_insts, list):
        raise ValueError("no 'scene_insts' attr of type 'list' in model")
    for scene_inst in scene_insts:
        add_modelled_scene(graph=g, scene_inst=scene_inst)

    return g
