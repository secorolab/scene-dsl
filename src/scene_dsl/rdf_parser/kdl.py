# SPDX-License-Identifier: MPL-2.0
"""Lower a scene graph's kinematics into segments a KDL backend can emit."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rdf_utils.models.common import ModelBase
from rdf_utils.models.geom_coord import get_coord_vectorxyz, get_pose_coord_vals, get_pose_coords
from rdf_utils.models.geom_rel import find_pose_path
from rdf_utils.models.vocab import (
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
    URI_GEOM_PRED_LINES,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_SIMPLICES,
    URI_GEOM_PRED_VECT_X,
    URI_GEOM_PRED_VECT_Y,
    URI_GEOM_PRED_VECT_Z,
    URI_GEOM_TYPE_KGRAPH,
    URI_GEOM_TYPE_KTREE,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_KC_EXT_PRED_ROOT,
    URI_KC_EXT_PRED_TIP,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_PRED_COMMON_AXIS,
    URI_KC_PRED_JOINTS,
    URI_KC_PRED_ORIGIN_OFFSET,
    URI_KC_TYPE_JOINT,
    URI_KC_TYPE_REVOLUTE_JOINT,
    URI_KC_TYPE_SERIAL,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_G,
    URI_QUDT_UNIT_KG,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM,
)
from rdflib import RDF, Graph, URIRef
from scipy.spatial.transform import RigidTransform

from scene_dsl.rdf_parser.common import ensure_one_obj_uri
from scene_dsl.rdf_parser.model_inertia import ModelFileError, read_body_inertia

LENGTH_SCALE = {URI_QUDT_UNIT_M: 1.0, URI_QUDT_UNIT_CM: 1e-2, URI_QUDT_UNIT_MM: 1e-3}
MASS_SCALE = {URI_QUDT_UNIT_KG: 1.0, URI_QUDT_UNIT_G: 1e-3}
AXIS_PREDS = {"x": URI_GEOM_PRED_VECT_X, "y": URI_GEOM_PRED_VECT_Y, "z": URI_GEOM_PRED_VECT_Z}


@dataclass(frozen=True)
class JointIR:
    name: str
    kind: str  # "None" | "RotAxis"
    origin: tuple[float, float, float]
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class InertiaIR:
    mass: float
    cog: tuple[float, float, float]
    rot: tuple[float, float, float, float, float, float]  # Ixx, Iyy, Izz, Ixy, Ixz, Iyz


@dataclass(frozen=True)
class SegmentIR:
    name: str
    hook: str
    joint: JointIR
    quat: tuple[float, float, float, float]  # f_tip rotation, xyzw
    pos: tuple[float, float, float]  # f_tip translation, metres
    inertia: InertiaIR | None


@dataclass(frozen=True)
class ChainIR:
    """A slice of its tree: the segments between two of them."""

    name: str
    root: str
    tip: str


@dataclass(frozen=True)
class TreeIR:
    name: str
    root: str
    segments: tuple[SegmentIR, ...]
    chains: tuple[ChainIR, ...]


class KinematicsError(Exception):
    """The graph does not describe kinematics a KDL tree can be built from."""


def _floats(values) -> tuple[float, ...]:
    """The IR is printed by a template, so it holds plain floats, not numpy scalars."""
    return tuple(float(value) for value in values)


def _moments(matrix) -> tuple[float, ...]:
    """A symmetric tensor in the order KDL's RotationalInertia takes it."""
    return _floats(
        (matrix[0][0], matrix[1][1], matrix[2][2], matrix[0][1], matrix[0][2], matrix[1][2])
    )


def transform_between(of_frame: URIRef, wrt_frame: URIRef, graph: Graph) -> RigidTransform | None:
    """The pose of one frame in another, in metres.

    `rdf_utils` composes the pose path but leaves lengths in their authored unit, so scale
    each pose as it is composed -- which also lifts its one-unit-per-path restriction.
    """
    path = find_pose_path(of_frame, wrt_frame, graph)
    if path is None:
        return None

    result = RigidTransform.identity()
    for pose, coords in get_pose_coords(graph=graph, poses=path):
        if len(coords) != 1:
            raise KinematicsError(f"pose '{pose.id}' must have one coordinate, found {len(coords)}")
        unit = coords[0].position_coord.unit
        if unit not in LENGTH_SCALE:
            raise KinematicsError(f"pose '{pose.id}' has unhandled length unit '{unit}'")
        transform = get_pose_coord_vals(coords[0], graph)
        scaled = RigidTransform.from_components(
            np.asarray(transform.translation) * LENGTH_SCALE[unit], transform.rotation
        )
        result = scaled * result
    return result


def _in_body(frame: URIRef, body: URIRef, graph: Graph) -> RigidTransform:
    """Where a frame sits on its body. Declaring no pose puts it at the body's root frame."""
    transform = transform_between(frame, _root_frame(body, graph), graph)
    return RigidTransform.identity() if transform is None else transform


def _scopes(graph: Graph) -> list[URIRef]:
    """Tree and graph IRIs, longest first: an element's IRI nests under the one scoping it."""
    scopes = set(graph.subjects(RDF.type, URI_GEOM_TYPE_KTREE))
    scopes |= set(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH))
    return sorted((s for s in scopes if isinstance(s, URIRef)), key=lambda s: -len(str(s)))


def _name(uri: URIRef, scopes: list[URIRef]) -> str:
    """The element's path below the tree that scopes it, e.g. 'arm1/base_link'."""
    for scope in scopes:
        prefix = f"{scope}/"
        if str(uri).startswith(prefix):
            return f"{str(scope).rsplit('/', 1)[-1]}/{str(uri).removeprefix(prefix)}"
    return str(uri).rsplit("/", 1)[-1]


def _body_of(frame: URIRef, graph: Graph) -> URIRef:
    bodies = [
        body
        for body in graph.subjects(URI_GEOM_PRED_SIMPLICES, frame)
        if (body, RDF.type, URI_GEOM_TYPE_RIGID_BODY) in graph
    ]
    if len(bodies) != 1:
        raise KinematicsError(f"frame '{frame}' must belong to one rigid body, found {bodies}")
    return bodies[0]


def _root_frame(body: URIRef, graph: Graph) -> URIRef:
    frame = ensure_one_obj_uri(graph=graph, subject=body, predicate=URI_KC_EXT_PRED_ROOT)
    if frame is None:
        raise KinematicsError(f"rigid body '{body}' declares no root frame")
    return frame


@dataclass
class _Joint:
    uri: URIRef
    frames: tuple[URIRef, URIRef]
    bodies: tuple[URIRef, URIRef]
    revolute: bool


def _joints(graph: Graph) -> dict[URIRef, _Joint]:
    joints = {}
    for uri in graph.subjects(RDF.type, URI_KC_TYPE_JOINT):
        frames = tuple(graph.objects(uri, URI_KC_PRED_BETWEEN_ATTACHMENTS))
        if len(frames) != 2:
            raise KinematicsError(f"joint '{uri}' must join two attachments, found {len(frames)}")
        joints[uri] = _Joint(
            uri=uri,
            frames=frames,
            bodies=tuple(_body_of(frame, graph) for frame in frames),
            revolute=(uri, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT) in graph,
        )
    return joints


def _axis_of(vector: URIRef, graph: Graph) -> tuple[URIRef, str]:
    """The frame a bound axis vector belongs to, and which of its axes it is."""
    for axis, predicate in AXIS_PREDS.items():
        for frame in graph.subjects(predicate, vector):
            return frame, axis
    raise KinematicsError(f"axis vector '{vector}' is no frame's axis")


def _revolute_axis(joint: _Joint, parent_frame: URIRef, graph: Graph) -> str:
    """The axis letter the joint turns about, rejecting an under-determined pairing."""
    common = ensure_one_obj_uri(graph=graph, subject=joint.uri, predicate=URI_KC_PRED_COMMON_AXIS)
    if common is None:
        raise KinematicsError(f"revolute joint '{joint.uri}' declares no common axis")
    axes = dict(_axis_of(vector, graph) for vector in graph.objects(common, URI_GEOM_PRED_LINES))
    if len(axes) != 2:
        raise KinematicsError(f"common axis of '{joint.uri}' must relate two frame axes: {axes}")
    if len(set(axes.values())) != 1:
        raise KinematicsError(
            f"revolute joint '{joint.uri}' makes '{'' .join(sorted(set(axes.values())))}' axes "
            f"collinear: which way the child frame then faces about them is undetermined"
        )
    return axes[parent_frame]


def _offset(joint: _Joint, graph: Graph) -> np.ndarray:
    """The child frame's displacement from the anchor, seen by the anchor, in metres."""
    position = ensure_one_obj_uri(
        graph=graph, subject=joint.uri, predicate=URI_KC_PRED_ORIGIN_OFFSET
    )
    if position is None:
        return np.zeros(3)

    coords = list(graph.subjects(URI_GEOM_PRED_OF_POSITION, position))
    if len(coords) != 1:
        raise KinematicsError(f"offset '{position}' must have one coordinate, found {coords}")
    values = get_coord_vectorxyz(ModelBase(node_id=coords[0], graph=graph), graph)
    if values is None:
        raise KinematicsError(f"offset '{position}' has no coordinate values")
    unit = graph.value(coords[0], URI_QUDT_PRED_UNIT)
    if unit not in LENGTH_SCALE:
        raise KinematicsError(f"offset '{position}' has unhandled length unit '{unit}'")
    return np.asarray(values) * LENGTH_SCALE[unit]


def _from_model_file(body: URIRef, graph: Graph, base_dir: Path | None) -> InertiaIR | None:
    """What the mapped model file states, when the scene itself states no inertia."""
    read = read_body_inertia(body, graph, base_dir)
    if read is None:
        return None
    mass, cog, matrix = read
    return InertiaIR(
        mass=mass,
        cog=_floats(cog),
        rot=_moments(matrix),
    )


def _inertia(body: URIRef, graph: Graph, base_dir: Path | None) -> InertiaIR | None:
    """The body's inertia in its own frame: about the centre of mass, in body orientation."""
    inertias = list(graph.subjects(URI_DYN_PRED_OF_BODY, body))
    if not inertias:
        return _from_model_file(body, graph, base_dir)
    if len(inertias) > 1:
        raise KinematicsError(f"body '{body}' has {len(inertias)} inertias")

    coords = list(graph.subjects(URI_DYN_PRED_OF_INERTIA, inertias[0]))
    if len(coords) != 1:
        raise KinematicsError(f"inertia '{inertias[0]}' must have one coordinate: {coords}")
    coord = coords[0]

    mass_literal = graph.value(coord, URI_DYN_PRED_MASS)
    moments = [
        graph.value(coord, predicate)
        for predicate in (
            URI_DYN_PRED_IXX,
            URI_DYN_PRED_IYY,
            URI_DYN_PRED_IZZ,
            URI_DYN_PRED_IXY,
            URI_DYN_PRED_IXZ,
            URI_DYN_PRED_IYZ,
        )
    ]
    if mass_literal is None or any(moment is None for moment in moments):
        # A frame-only inertia claims where the mass is, not how much: the file states that.
        return _from_model_file(body, graph, base_dir)

    units = set(graph.objects(coord, URI_QUDT_PRED_UNIT)) & set(MASS_SCALE)
    if len(units) != 1:
        raise KinematicsError(f"inertia coordinate '{coord}' must have one mass unit: {units}")
    mass = float(mass_literal.toPython()) * MASS_SCALE[units.pop()]

    inertial_frame = ensure_one_obj_uri(graph=graph, subject=coord, predicate=URI_DYN_PRED_AS_SEEN_BY)
    if inertial_frame is None:
        raise KinematicsError(f"inertia coordinate '{coord}' is seen by no frame")
    in_body = _in_body(inertial_frame, body, graph)

    ixx, iyy, izz, ixy, ixz, iyz = (float(moment.toPython()) for moment in moments)
    matrix = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    rotation = in_body.rotation.as_matrix()
    rotated = rotation @ matrix @ rotation.T
    return InertiaIR(
        mass=mass,
        cog=_floats(in_body.translation),
        rot=_moments(rotated),
    )


def _joint_owners(graph: Graph) -> dict[URIRef, set[URIRef]]:
    """The tree each joint belongs to. A serial chain also lists joints, and owns none."""
    owners: dict[URIRef, set[URIRef]] = {}
    for scope in _scopes(graph):
        for joint in graph.objects(scope, URI_KC_PRED_JOINTS):
            owners.setdefault(joint, set()).add(scope)
    return owners


def _roots(joints: dict[URIRef, _Joint], graph: Graph) -> dict[URIRef, URIRef | None]:
    """The body each tree hangs from, and the tree that names it.

    A tree declares its own root, but a composed tree's root is attached by the tree
    composing it -- so the roots left are those no other tree's joint touches.
    """
    owners = _joint_owners(graph)
    attached: dict[URIRef, set[URIRef]] = {}
    for joint in joints.values():
        for body in joint.bodies:
            attached.setdefault(body, set()).update(owners.get(joint.uri, set()))

    roots: dict[URIRef, URIRef | None] = {}
    for tree in _scopes(graph):
        frame = ensure_one_obj_uri(graph=graph, subject=tree, predicate=URI_KC_EXT_PRED_ROOT)
        if frame is None:
            continue
        body = _body_of(frame, graph)
        if attached.get(body, set()) - {tree}:
            continue
        roots[body] = tree

    # A body no joint attaches floats: it is placed, not articulated, so KDL holds nothing.
    return roots


def _segment(
    joint: _Joint,
    parent: URIRef,
    scopes: list[URIRef],
    graph: Graph,
    base_dir: Path | None,
) -> SegmentIR:
    """The child body as a KDL segment: its joint, and where it sits at zero position."""
    child_index = 1 if joint.bodies[0] == parent else 0
    child = joint.bodies[child_index]
    parent_frame, child_frame = joint.frames[1 - child_index], joint.frames[child_index]

    in_parent = _in_body(parent_frame, parent, graph)
    in_child = _in_body(child_frame, child, graph)
    # An attachment makes two frames coincide unless a pose says how they differ.
    attach = transform_between(child_frame, parent_frame, graph)
    if attach is None:
        attach = RigidTransform.identity()

    if joint.revolute:
        axis = _revolute_axis(joint, parent_frame, graph)
        attach = RigidTransform.from_translation(_offset(joint, graph)) * attach
        kind, direction = "RotAxis", in_parent.rotation.as_matrix()[:, "xyz".index(axis)]
    else:
        kind, direction = "None", np.zeros(3)

    f_tip = in_parent * attach * in_child.inv()
    return SegmentIR(
        name=_name(child, scopes),
        hook=_name(parent, scopes),
        joint=JointIR(
            name=_name(joint.uri, scopes),
            kind=kind,
            origin=_floats(in_parent.translation),
            axis=_floats(direction),
        ),
        quat=_floats(f_tip.rotation.as_quat()),
        pos=_floats(f_tip.translation),
        inertia=_body_inertia(child, graph, base_dir, moves=joint.revolute),
    )


def _body_inertia(
    body: URIRef, graph: Graph, base_dir: Path | None, moves: bool
) -> InertiaIR | None:
    """What a body weighs, if anything has to know.

    A body a fixed joint holds carries the tree without moving in it -- a sensor's frame,
    a table, a mounting plate -- so having no mass is a thing it may legitimately be, and
    KDL carries that as a zero inertia. A body a joint moves is another matter: a missing
    mass there is a hole in the dynamics, and is reported rather than assumed away.
    """
    try:
        return _inertia(body, graph, base_dir)
    except ModelFileError:
        if moves:
            raise
        return None


def _tip_segment(tip_frame: URIRef, tip: URIRef, scopes: list[URIRef], graph: Graph) -> SegmentIR:
    """A frame a chain ends at, but its body is not rooted at, is a fixed segment of its own."""
    in_body = _in_body(tip_frame, tip, graph)
    return SegmentIR(
        name=f"{_name(tip, scopes)}/{str(tip_frame).rsplit('/', 1)[-1]}",
        hook=_name(tip, scopes),
        joint=JointIR(name="", kind="None", origin=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 0.0)),
        quat=_floats(in_body.rotation.as_quat()),
        pos=_floats(in_body.translation),
        inertia=None,
    )


def _chains(
    root: URIRef,
    parents: dict[URIRef, URIRef],
    scopes: list[URIRef],
    graph: Graph,
) -> tuple[tuple[ChainIR, ...], list[SegmentIR]]:
    """Every declared serial composition whose bodies are this tree's, and the segments
    its endpoints add.

    A chain is the tree between two of its segments, so it names them rather than
    repeating them. A tip frame is no body, so nothing has made a segment of it yet:
    it joins the tree as a fixed leaf, and the chain then slices out to it.
    """
    chains, tips = [], {}
    for serial in graph.subjects(RDF.type, URI_KC_TYPE_SERIAL):
        root_frame = ensure_one_obj_uri(graph=graph, subject=serial, predicate=URI_KC_EXT_PRED_ROOT)
        tip_frame = ensure_one_obj_uri(graph=graph, subject=serial, predicate=URI_KC_EXT_PRED_TIP)
        if root_frame is None or tip_frame is None:
            raise KinematicsError(f"serial chain '{serial}' declares no root or no tip")

        chain_root, tip = _body_of(root_frame, graph), _body_of(tip_frame, graph)
        if tip not in parents and tip != root:
            continue
        if root_frame != _root_frame(chain_root, graph):
            raise KinematicsError(
                f"serial chain '{serial}' starts at '{root_frame}', which is not the root frame "
                f"of body '{chain_root}': a KDL chain carries no offset before its first segment"
            )

        body = tip
        while body != chain_root:
            parent = parents.get(body)
            if parent is None:
                raise KinematicsError(f"serial chain '{serial}' tip is not below its root")
            body = parent

        tip_name = _name(tip, scopes)
        if tip_frame != _root_frame(tip, graph):
            segment = _tip_segment(tip_frame, tip, scopes, graph)
            # Two chains may end at one frame, and the tree holds it once.
            tips.setdefault(segment.name, segment)
            tip_name = segment.name
        chains.append(
            ChainIR(
                name=_name(serial, scopes).removesuffix("/chain"),
                root=_name(chain_root, scopes),
                tip=tip_name,
            )
        )
    return tuple(chains), list(tips.values())


def build_kdl_model(graph: Graph, base_dir: Path | None = None) -> list[TreeIR]:
    """Every tree the graph's kinematics hang from, with the chains declared over them."""
    scopes = _scopes(graph)
    joints = _joints(graph)
    trees = []

    for root, owner in sorted(_roots(joints, graph).items(), key=lambda item: str(item[0])):
        pending, seen, ordered = [root], {root}, []
        parents: dict[URIRef, URIRef] = {}
        while pending:
            body = pending.pop(0)
            for joint in joints.values():
                if body not in joint.bodies:
                    continue
                child = joint.bodies[1 if joint.bodies[0] == body else 0]
                if child in seen:
                    continue
                seen.add(child)
                parents[child] = body
                ordered.append(_segment(joint, body, scopes, graph, base_dir))
                pending.append(child)

        # A fixed segment may weigh nothing; one a joint moves may not (see _body_inertia).
        missing = [
            segment.name
            for segment in ordered
            if segment.inertia is None and segment.joint.kind != "None"
        ]
        if missing:
            raise KinematicsError(
                f"no inertia for {', '.join(sorted(missing))}: state mass and inertia-matrix "
                f"in the model, or map the bodies to a model file that does"
            )

        # Appended last, so each tip frame follows the body it hangs from.
        chains, tips = _chains(root, parents, scopes, graph)
        trees.append(
            TreeIR(
                name=_name(owner, scopes) if owner is not None else _name(root, scopes),
                root=_name(root, scopes),
                segments=tuple(ordered + tips),
                chains=chains,
            )
        )
    return trees
