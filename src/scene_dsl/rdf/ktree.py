# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.vocab import (
    URI_ACT_PRED_COMMAND_INTERFACE,
    URI_ACT_PRED_GEAR_RATIO,
    URI_ACT_PRED_JOINT,
    URI_ACT_PRED_STATE_INTERFACE,
    URI_ACT_TYPE_ACTUATION,
    URI_ACT_TYPE_JOINT_CURRENT,
    URI_DYN_PRED_ABOUT,
    URI_DYN_PRED_AS_SEEN_BY,
    URI_DYN_PRED_IXX,
    URI_DYN_PRED_IXY,
    URI_DYN_PRED_IXZ,
    URI_DYN_PRED_IYY,
    URI_DYN_PRED_IYZ,
    URI_DYN_PRED_IZZ,
    URI_DYN_PRED_MASS,
    URI_DYN_PRED_OF_BODY,
    URI_DYN_PRED_OF_INERTIA,
    URI_DYN_TYPE_INERTIA_REFERENCE,
    URI_DYN_TYPE_MASS_SCALAR,
    URI_DYN_TYPE_MOMENT_OF_INERTIA_XYZ,
    URI_DYN_TYPE_PRODUCT_OF_INERTIA_XYZ,
    URI_DYN_TYPE_RIGID_BODY_INERTIA,
    URI_DYN_TYPE_RIGID_BODY_INERTIA_COORD,
    URI_GEOM_PRED_LINES,
    URI_GEOM_PRED_SIMPLICES,
    URI_GEOM_TYPE_COLLINEAR,
    URI_GEOM_TYPE_KGRAPH,
    URI_GEOM_TYPE_KTREE,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_GEOM_TYPE_SIMPLICIAL_COMPLEX,
    URI_KC_EXT_PRED_DEPENDENT_JOINT,
    URI_KC_EXT_PRED_INDEPENDENT_JOINT,
    URI_KC_EXT_PRED_MULTIPLIER,
    URI_KC_EXT_PRED_LOWER,
    URI_KC_EXT_PRED_OFFSET,
    URI_KC_EXT_PRED_OF_JOINT,
    URI_KC_EXT_PRED_UPPER,
    URI_KC_EXT_PRED_ROOT,
    URI_KC_EXT_PRED_TIP,
    URI_KC_EXT_TYPE_JOINT_COUPLING,
    URI_KC_EXT_TYPE_SCLERONOMIC,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_PRED_COMMON_AXIS,
    URI_KC_PRED_JOINTS,
    URI_KC_PRED_ORIGIN_OFFSET,
    URI_KC_STAT_JNT_FORCE,
    URI_KC_STAT_JNT_POSITION,
    URI_KC_STAT_JNT_VEL,
    URI_KC_TYPE_JOINT,
    URI_KC_TYPE_KC,
    URI_KC_TYPE_REVOLUTE_JOINT,
    URI_KC_TYPE_REVOLUTE_JOINT_ORIENTED_AXIS,
    URI_KC_TYPE_SERIAL,
    URI_KC_EXT_TYPE_JOINT_LIMIT,
    URI_KC_STAT_JNT_ACC,
    URI_QUDT_QK_ANG_ACCEL,
    URI_QUDT_QK_ANG_VEL,
    URI_QUDT_QK_TORQUE,
    URI_QUDT_UNIT_DEG_PER_SEC,
    URI_QUDT_UNIT_DEG_PER_SEC2,
    URI_QUDT_UNIT_N_M,
    URI_QUDT_UNIT_RAD_PER_SEC,
    URI_QUDT_UNIT_RAD_PER_SEC2,
    URI_QUDT_UNIT_DEG,
    URI_QUDT_PRED_QUANTITY_KIND,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_PRED_VALUE,
    URI_QUDT_QK_ANGLE,
    URI_QUDT_TYPE_QUANTITY,
    URI_QUDT_UNIT_G,
    URI_QUDT_UNIT_KG,
    URI_QUDT_UNIT_KG_M2,
    URI_QUDT_UNIT_RAD,
)
from rdflib import RDF, XSD, Graph, Literal, URIRef

from scene_dsl.classes.ktree import (
    JointLimits,
    Actuation,
    FixedJoint,
    JointBase,
    JointMimicSpec,
    KinematicGraph,
    KinematicTreeModel,
    RevoluteJoint,
    SerialJoints,
)
from scene_dsl.rdf.geom import (
    add_frame,
    add_position_coord,
)

ACTUATION_INTERFACE_TYPES = {
    "position": URI_KC_STAT_JNT_POSITION,
    "velocity": URI_KC_STAT_JNT_VEL,
    "torque": URI_KC_STAT_JNT_FORCE,
    "current": URI_ACT_TYPE_JOINT_CURRENT,
}

MASS_UNITS = {"kg": URI_QUDT_UNIT_KG, "g": URI_QUDT_UNIT_G}
INERTIA_UNITS = {"kg*m^2": URI_QUDT_UNIT_KG_M2}

# The joint-limit kinds kc-ext:JointLimitShape admits: which state the bound constrains, the
# quantity kind its values carry, and the units it may be authored in. Revolute joints only --
# a prismatic kind would add Length/LinearVelocity rows here.
JOINT_LIMIT_KINDS = (
    ("position", URI_KC_STAT_JNT_POSITION, URI_QUDT_QK_ANGLE,
     {"rad": URI_QUDT_UNIT_RAD, "deg": URI_QUDT_UNIT_DEG}),
    ("velocity", URI_KC_STAT_JNT_VEL, URI_QUDT_QK_ANG_VEL,
     {"rad/s": URI_QUDT_UNIT_RAD_PER_SEC, "deg/s": URI_QUDT_UNIT_DEG_PER_SEC}),
    ("acceleration", URI_KC_STAT_JNT_ACC, URI_QUDT_QK_ANG_ACCEL,
     {"rad/s^2": URI_QUDT_UNIT_RAD_PER_SEC2, "deg/s^2": URI_QUDT_UNIT_DEG_PER_SEC2}),
    ("effort", URI_KC_STAT_JNT_FORCE, URI_QUDT_QK_TORQUE, {"N*m": URI_QUDT_UNIT_N_M}),
)


def add_body(graph: Graph, body) -> None:
    graph.add((body.uri, RDF.type, URI_GEOM_TYPE_RIGID_BODY))
    graph.add((body.uri, RDF.type, URI_GEOM_TYPE_SIMPLICIAL_COMPLEX))
    for frame in body.frames:
        add_frame(graph, frame)
        graph.add((body.uri, URI_GEOM_PRED_SIMPLICES, frame.uri))
        graph.add((body.uri, URI_GEOM_PRED_SIMPLICES, frame.origin_uri))

    graph.add(triple=(body.uri, URI_KC_EXT_PRED_ROOT, body.default_frame.uri))

    if body.inertia is not None:
        inertia = body.inertia
        graph.add((body.inertia_uri, RDF.type, URI_DYN_TYPE_RIGID_BODY_INERTIA))
        graph.add((body.inertia_uri, URI_DYN_PRED_OF_BODY, body.uri))
        graph.add((body.inertia_uri, URI_DYN_PRED_ABOUT, inertia.frame.origin_uri))

        if inertia.mass is not None or inertia.matrix is not None:
            graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_INERTIA_REFERENCE))
            graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_RIGID_BODY_INERTIA_COORD))
            graph.add((body.inertia_coord_uri, URI_DYN_PRED_OF_INERTIA, body.inertia_uri))
            graph.add((body.inertia_coord_uri, URI_DYN_PRED_AS_SEEN_BY, inertia.frame.uri))

        if inertia.mass is not None:
            graph.add((body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MASS_SCALAR))
            graph.add((body.inertia_coord_uri, URI_DYN_PRED_MASS, Literal(inertia.mass)))
            graph.add((body.inertia_coord_uri, URI_QUDT_PRED_UNIT, MASS_UNITS[inertia.mass_unit]))

        if inertia.matrix is not None:
            graph.add(
                (body.inertia_coord_uri, URI_QUDT_PRED_UNIT, INERTIA_UNITS[inertia.inertia_unit])
            )
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
    # The axis of rotation is given by the collinearity of the parent and child frame
    # axes emitted below, so the joint is always an oriented-axis revolute joint.
    graph.add((joint.uri, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT_ORIENTED_AXIS))
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
            position_spec=joint.offset.xyz,
            unit=joint.offset.length_unit,
        )

    if joint.actuation is not None:
        add_actuation(
            graph=graph,
            joint_uri=joint.uri,
            actuation_uri=joint.actuation_uri,
            actuation=joint.actuation,
        )
    if joint.mimic is not None:
        add_mimic(
            graph=graph,
            joint_uri=joint.uri,
            mimic_uri=joint.mimic_uri,
            offset_uri=joint.mimic_offset_uri,
            mimic=joint.mimic,
        )
    if joint.limits is not None:
        add_joint_limits(graph=graph, joint=joint, limits=joint.limits)


def add_joint_limits(graph: Graph, joint, limits: JointLimits) -> None:
    """One kc-ext:JointLimit per authored bound: what it limits, on which joint, between which
    values. A revolute joint that authors no position bound is continuous -- consumers read that
    from the absence, so nothing is emitted to say it.
    """
    for field, limit_type, quantity_kind, units in JOINT_LIMIT_KINDS:
        bounds = getattr(limits, field, None)
        if bounds is None:
            continue
        unit_name = getattr(limits, f"{field}_unit", None)
        unit = units.get(unit_name)
        if unit is None:
            raise ValueError(
                f"joint '{joint.name}': {field} limit has unit '{unit_name}',"
                f" expected one of {sorted(units)}"
            )
        lower_value, upper_value = bounds
        if lower_value > upper_value:
            raise ValueError(
                f"joint '{joint.name}': {field} limit lower {lower_value}"
                f" exceeds upper {upper_value}"
            )
        limit_uri = joint.namespace[joint.scoped(f"-limit-{field}")]
        graph.add((limit_uri, RDF.type, URI_KC_EXT_TYPE_JOINT_LIMIT))
        graph.add((limit_uri, RDF.type, limit_type))
        graph.add((limit_uri, URI_KC_EXT_PRED_OF_JOINT, joint.uri))
        for pred, value in (
            (URI_KC_EXT_PRED_LOWER, lower_value),
            (URI_KC_EXT_PRED_UPPER, upper_value),
        ):
            bound_uri = joint.namespace[joint.scoped(f"-limit-{field}-{'lower' if pred == URI_KC_EXT_PRED_LOWER else 'upper'}")]
            graph.add((bound_uri, RDF.type, URI_QUDT_TYPE_QUANTITY))
            graph.add((bound_uri, URI_QUDT_PRED_VALUE, Literal(value, datatype=XSD.double)))
            graph.add((bound_uri, URI_QUDT_PRED_UNIT, unit))
            graph.add((bound_uri, URI_QUDT_PRED_QUANTITY_KIND, quantity_kind))
            graph.add((limit_uri, pred, bound_uri))


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


def add_mimic(
    graph: Graph,
    joint_uri: URIRef,
    mimic_uri: URIRef,
    offset_uri: URIRef,
    mimic: JointMimicSpec,
) -> None:
    graph.add((mimic_uri, RDF.type, URI_KC_EXT_TYPE_JOINT_COUPLING))
    graph.add((mimic_uri, RDF.type, URI_KC_EXT_TYPE_SCLERONOMIC))
    graph.add((mimic_uri, URI_KC_EXT_PRED_INDEPENDENT_JOINT, mimic.joint.uri))
    graph.add((mimic_uri, URI_KC_EXT_PRED_DEPENDENT_JOINT, joint_uri))
    graph.add(
        (mimic_uri, URI_KC_EXT_PRED_MULTIPLIER, Literal(mimic.multiplier, datatype=XSD.double))
    )
    graph.add((mimic_uri, URI_KC_EXT_PRED_OFFSET, offset_uri))
    graph.add((offset_uri, RDF.type, URI_QUDT_TYPE_QUANTITY))
    graph.add((offset_uri, URI_QUDT_PRED_VALUE, Literal(mimic.offset, datatype=XSD.double)))
    graph.add((offset_uri, URI_QUDT_PRED_QUANTITY_KIND, URI_QUDT_QK_ANGLE))
    graph.add((offset_uri, URI_QUDT_PRED_UNIT, URI_QUDT_UNIT_RAD))


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


def add_joints_spec(graph: Graph, owner: KinematicGraph) -> None:
    """The joints a graph declares and the composition over them, hung off that graph."""
    if owner.joints_spec is None:
        return

    for joint in owner.joints_spec.joints:
        graph.add(triple=(owner.uri, URI_KC_PRED_JOINTS, joint.uri))
        add_joint(graph=graph, joint=joint)

    joint_comp = owner.joints_spec.joint_comp
    if joint_comp is None:
        return
    if not isinstance(joint_comp, SerialJoints):
        raise TypeError(f"JointComposition type not handled for: {joint_comp}")

    # A graph may branch and still carry a chain: the chain is the composition, not it.
    graph.add(triple=(joint_comp.uri, RDF.type, URI_KC_TYPE_KC))
    graph.add(triple=(joint_comp.uri, RDF.type, URI_KC_TYPE_SERIAL))
    for chain_joint in joint_comp.joints:
        graph.add(triple=(joint_comp.uri, URI_KC_PRED_JOINTS, chain_joint.uri))
    graph.add(triple=(joint_comp.uri, URI_KC_EXT_PRED_ROOT, joint_comp.root_frame.uri))
    graph.add(triple=(joint_comp.uri, URI_KC_EXT_PRED_TIP, joint_comp.tip_frame.uri))


def add_kinematic_tree(graph: Graph, tree: KinematicTreeModel, seen_trees: set[URIRef]) -> None:
    if tree.uri in seen_trees:
        raise ValueError(f"add_kinematic_tree: duplicate KinematicTree URI: {tree.uri}")
    seen_trees.add(tree.uri)

    graph.add(triple=(tree.uri, RDF.type, URI_GEOM_TYPE_KTREE))
    for linked_tree in tree.trees:
        add_kinematic_tree(graph=graph, tree=linked_tree, seen_trees=seen_trees)

    for body in tree.bodies:
        add_body(graph=graph, body=body)

    # Derived from the joints, so a tree has a root with or without a chain over it.
    graph.add(triple=(tree.uri, URI_KC_EXT_PRED_ROOT, tree.roots[0].default_frame.uri))

    add_joints_spec(graph=graph, owner=tree)


def add_kinematic_graph(graph: Graph, kgraph: KinematicGraph, seen_trees: set[URIRef]) -> None:
    """A scene's kinematics: the trees it composes, and the bodies hanging from nothing."""
    graph.add(triple=(kgraph.uri, RDF.type, URI_GEOM_TYPE_KGRAPH))
    for tree in kgraph.trees:
        add_kinematic_tree(graph=graph, tree=tree, seen_trees=seen_trees)
    for body in kgraph.bodies:
        add_body(graph=graph, body=body)

    # Every body no joint attaches, a graph hanging from many where a tree has one. This
    # is what says the bodies and the trees standing on them are this graph's.
    for root in kgraph.roots:
        graph.add(triple=(kgraph.uri, URI_KC_EXT_PRED_ROOT, root.default_frame.uri))

    add_joints_spec(graph=graph, owner=kgraph)
