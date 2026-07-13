# SPDX-License-Identifier: MPL-2.0
from rdf_utils.namespace import NS_MM_DYN_COORD, NS_MM_GEOM, NS_MM_QUDT_UNIT
from rdflib import Namespace, RDF, Graph, Literal, URIRef, XSD
from rdf_utils.models.geometry import (
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_PRED_COMMON_AXIS,
    URI_KC_PRED_JOINTS,
    URI_KC_PRED_ORIGIN_OFFSET,
    URI_KC_TYPE_JOINT,
    URI_KC_TYPE_REVOLUTE_JOINT,
    URI_QUDT_UNIT_G,
    URI_QUDT_UNIT_KG,
    URI_GEOM_TYPE_COLLINEAR,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_GEOM_TYPE_SIMPLICIAL_COMPLEX,
    URI_GEOM_PRED_LINES,
    URI_GEOM_PRED_SIMPLICES,
    URI_KC_TYPE_KC,
    URI_KC_TYPE_SERIAL,
    URI_DYN_PRED_ABOUT,
    URI_DYN_PRED_AS_SEEN_BY,
    URI_DYN_PRED_MASS,
    URI_DYN_PRED_OF_BODY,
    URI_DYN_PRED_OF_INERTIA,
    URI_DYN_TYPE_INERTIA_REFERENCE,
    URI_DYN_TYPE_MASS_SCALAR,
    URI_DYN_TYPE_RIGID_BODY_INERTIA,
    URI_DYN_TYPE_RIGID_BODY_INERTIA_COORD,
)

from scene_dsl.classes.ktree import (
    Actuation,
    FixedJoint,
    JointBase,
    KinematicTreeModel,
    RevoluteJoint,
    SerialJoints,
)
from scene_dsl.rdf.geom import (
    URI_QUDT_PRED_UNIT,
    add_frame,
    add_position_coord,
)

URI_GEOM_TYPE_KTREE = NS_MM_GEOM["KinematicTree"]
NS_MM_KC_STAT = Namespace("https://comp-rob2b.github.io/metamodels/kinematic-chain/state#")
NS_MM_ACT = Namespace("https://secorolab.github.io/metamodels/robot/actuation#")

URI_ACT_TYPE_ACTUATION = NS_MM_ACT["Actuation"]
URI_ACT_PRED_JOINT = NS_MM_ACT["joint"]
URI_ACT_PRED_GEAR_RATIO = NS_MM_ACT["gear-ratio"]
URI_ACT_PRED_COMMAND_INTERFACE = NS_MM_ACT["command-interface"]
URI_ACT_PRED_STATE_INTERFACE = NS_MM_ACT["state-interface"]
URI_ACT_TYPE_JOINT_CURRENT = NS_MM_ACT["JointCurrent"]
ACTUATION_INTERFACE_TYPES = {
    "position": NS_MM_KC_STAT["JointPosition"],
    "velocity": NS_MM_KC_STAT["JointVelocity"],
    "torque": NS_MM_KC_STAT["JointForce"],
    "current": URI_ACT_TYPE_JOINT_CURRENT,
}

MASS_UNITS = {"kg": URI_QUDT_UNIT_KG, "g": URI_QUDT_UNIT_G}
INERTIA_UNITS = {"kg*m^2": NS_MM_QUDT_UNIT["KiloGM-M2"]}
URI_DYN_TYPE_MOMENT_OF_INERTIA_XYZ = NS_MM_DYN_COORD["MomentOfInertiaXYZ"]
URI_DYN_TYPE_PRODUCT_OF_INERTIA_XYZ = NS_MM_DYN_COORD["ProductOfInertiaXYZ"]
URI_DYN_PRED_IXX = NS_MM_DYN_COORD["ixx"]
URI_DYN_PRED_IXY = NS_MM_DYN_COORD["ixy"]
URI_DYN_PRED_IXZ = NS_MM_DYN_COORD["ixz"]
URI_DYN_PRED_IYY = NS_MM_DYN_COORD["iyy"]
URI_DYN_PRED_IYZ = NS_MM_DYN_COORD["iyz"]
URI_DYN_PRED_IZZ = NS_MM_DYN_COORD["izz"]


def add_body(graph: Graph, body) -> None:
    graph.add((body.uri, RDF.type, URI_GEOM_TYPE_RIGID_BODY))
    graph.add((body.uri, RDF.type, URI_GEOM_TYPE_SIMPLICIAL_COMPLEX))
    for frame in body.frames:
        add_frame(graph, frame)
        graph.add((body.uri, URI_GEOM_PRED_SIMPLICES, frame.uri))
        graph.add((body.uri, URI_GEOM_PRED_SIMPLICES, frame.origin_uri))

    if body.inertia is not None:
        inertia = body.inertia
        graph.add((body.inertia_uri, RDF.type, URI_DYN_TYPE_RIGID_BODY_INERTIA))
        graph.add((body.inertia_uri, URI_DYN_PRED_OF_BODY, body.uri))
        graph.add((body.inertia_uri, URI_DYN_PRED_ABOUT, inertia.frame.origin_uri))
        graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_INERTIA_REFERENCE))
        graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_RIGID_BODY_INERTIA_COORD))
        graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MASS_SCALAR))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_OF_INERTIA, body.inertia_uri))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_AS_SEEN_BY, inertia.frame.uri))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_MASS, Literal(inertia.mass)))
        graph.add((body.inertia_coord_uri, URI_QUDT_PRED_UNIT, MASS_UNITS[inertia.mass_unit]))
        graph.add((body.inertia_coord_uri, URI_QUDT_PRED_UNIT, INERTIA_UNITS[inertia.inertia_unit]))
        graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MOMENT_OF_INERTIA_XYZ))
        graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_PRODUCT_OF_INERTIA_XYZ))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_IXX, Literal(inertia.matrix[0][0])))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_IXY, Literal(inertia.matrix[0][1])))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_IXZ, Literal(inertia.matrix[0][2])))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_IYY, Literal(inertia.matrix[1][1])))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_IYZ, Literal(inertia.matrix[1][2])))
        graph.add((body.inertia_coord_uri, URI_DYN_PRED_IZZ, Literal(inertia.matrix[2][2])))


def add_revolute_joint(graph: Graph, joint: RevoluteJoint) -> None:
    graph.add((joint.uri, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT))
    graph.add((joint.uri, URI_KC_PRED_BETWEEN_ATTACHMENTS, joint.parent_frame_axis.frame.uri))
    graph.add((joint.uri, URI_KC_PRED_BETWEEN_ATTACHMENTS, joint.child_frame_axis.frame.uri))

    graph.add((joint.uri, URI_KC_PRED_COMMON_AXIS, joint.common_axis_uri))
    graph.add((joint.common_axis_uri, RDF.type, URI_GEOM_TYPE_COLLINEAR))
    parent_axis_uri = joint.parent_frame_axis.frame.axis_vector_uri(
        axis=joint.parent_frame_axis.axis
    )
    child_axis_uri = joint.child_frame_axis.frame.axis_vector_uri(axis=joint.child_frame_axis.axis)
    graph.add((joint.common_axis_uri, URI_GEOM_PRED_LINES, parent_axis_uri))
    graph.add((joint.common_axis_uri, URI_GEOM_PRED_LINES, child_axis_uri))

    if joint.offset is not None:
        graph.add((joint.uri, URI_KC_PRED_ORIGIN_OFFSET, joint.offset_uri))
        add_position_coord(
            graph=graph,
            pos_uri=joint.offset_uri,
            pos_coord_uri=joint.offset_coord_uri,
            as_seen_by=joint.parent_frame_axis.frame.uri,
            of_uri=joint.child_frame_axis.frame.origin_uri,
            wrt_uri=joint.parent_frame_axis.frame.origin_uri,
            xyz_vector=joint.offset.xyz,
            unit=joint.offset.length_unit,
        )

    if joint.actuation is not None:
        add_actuation(
            graph=graph,
            joint_uri=joint.uri,
            actuation_uri=joint.actuation_uri,
            actuation=joint.actuation,
        )
    # TODO(minhnh): add_joint_limits()
    # TODO(minhnh): add_joint_mimics()


def add_actuation(
    graph: Graph, joint_uri: URIRef, actuation_uri: URIRef, actuation: Actuation
) -> None:
    graph.add((actuation_uri, RDF.type, URI_ACT_TYPE_ACTUATION))
    graph.add((actuation_uri, URI_ACT_PRED_JOINT, joint_uri))
    graph.add(
        (actuation_uri, URI_ACT_PRED_GEAR_RATIO, Literal(actuation.gear_ratio, datatype=XSD.double))
    )
    for interface in actuation.cmd_interfaces:
        graph.add(
            (
                actuation_uri,
                URI_ACT_PRED_COMMAND_INTERFACE,
                ACTUATION_INTERFACE_TYPES[interface],
            )
        )
    for interface in actuation.state_interfaces:
        graph.add(
            (actuation_uri, URI_ACT_PRED_STATE_INTERFACE, ACTUATION_INTERFACE_TYPES[interface])
        )


def add_joint(graph: Graph, joint: JointBase) -> None:
    graph.add((joint.uri, RDF.type, URI_KC_TYPE_JOINT))

    if isinstance(joint, FixedJoint):
        graph.add((joint.uri, URI_KC_PRED_BETWEEN_ATTACHMENTS, joint.parent_frame.uri))
        graph.add((joint.uri, URI_KC_PRED_BETWEEN_ATTACHMENTS, joint.child_frame.uri))
        return

    if isinstance(joint, RevoluteJoint):
        add_revolute_joint(graph=graph, joint=joint)
        return

    raise ValueError(f"Unsupported joint type: {joint}")


def add_kinematic_tree(graph: Graph, tree: KinematicTreeModel, seen_trees: set[URIRef]) -> None:
    if tree.uri in seen_trees:
        raise ValueError(f"add_kinematic_tree: duplicate KinematicTree URI: {tree.uri}")
    seen_trees.add(tree.uri)

    graph.add((tree.uri, RDF.type, URI_GEOM_TYPE_KTREE))
    for linked_tree in tree.trees:
        add_kinematic_tree(graph=graph, tree=linked_tree, seen_trees=seen_trees)

    for body in tree.bodies:
        add_body(graph=graph, body=body)

    for joint in tree.joints_spec.joints:
        add_joint(graph=graph, joint=joint)

    if tree.joints_spec.joint_comp is not None:
        if isinstance(tree.joints_spec.joint_comp, SerialJoints):
            graph.add((tree.uri, RDF.type, URI_KC_TYPE_KC))
            graph.add((tree.uri, RDF.type, URI_KC_TYPE_SERIAL))
            for joint in tree.joints_spec.joint_comp.joints:
                graph.add((tree.uri, URI_KC_PRED_JOINTS, joint.uri))
