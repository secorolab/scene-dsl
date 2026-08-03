# SPDX-License-Identifier: MPL-2.0
"""Lower a scene graph into a kinematic model: bodies, the joints between them, chains.

The model is representation-neutral -- it names joints `fixed` and `revolute`, carries
transforms as `scipy` `RigidTransform`, and knows nothing of any solver library. A
backend template turns it into whatever that library spells these things.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase
from rdf_utils.models.geom_coord import (
    get_transform_between_frames,
    get_translation_between_points,
)
from rdf_utils.models.geom_rel import FrameModel
from rdf_utils.models.vocab import (
    URI_DYN_PRED_OF_BODY,
    URI_GEOM_PRED_LINES,
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
    URI_QUDT_UNIT_G,
    URI_QUDT_UNIT_KG,
)
from rdflib.namespace import split_uri
from rdflib import RDF, Graph, URIRef
from scipy.spatial.transform import RigidTransform

from scene_dsl.rdf_parser.common import ensure_one_obj_uri
from scene_dsl.rdf_parser.ktree import InertiaModel, get_root_frame
from scene_dsl.rdf_parser.mapped_inertia import read_body_inertia

MASS_SCALE = {URI_QUDT_UNIT_KG: 1.0, URI_QUDT_UNIT_G: 1e-3}
AXIS_PREDS = {"x": URI_GEOM_PRED_VECT_X, "y": URI_GEOM_PRED_VECT_Y, "z": URI_GEOM_PRED_VECT_Z}

class JointKind(StrEnum):
    """What a joint is, not what any one library calls it."""

    FIXED = "fixed"
    REVOLUTE = "revolute"


@dataclass
class Inertia:
    """A body's mass and how it is distributed, in the body's own frame.

    Not a graph model: a body whose inertia is read from its model file has no inertia
    node to be one of.
    """

    mass: float
    cog: np.ndarray  # centre of mass in the body frame
    moments: np.ndarray  # Ixx, Iyy, Izz, Ixy, Ixz, Iyz, about the centre of mass


class JointModel(ModelBase):
    """A joint: the two frames it attaches, and what it does between them.

    `origin` and `axis` are the line it moves about, resolved in the parent body's
    frame; they are set once the joint is oriented, since which frame is the parent
    is a property of the tree and not of the joint.
    """

    def __init__(self, joint_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=joint_id, graph=graph)
        frames = tuple(
            frame
            for frame in graph.objects(joint_id, URI_KC_PRED_BETWEEN_ATTACHMENTS)
            if isinstance(frame, URIRef)
        )
        if len(frames) != 2:
            raise ConstraintViolation(
                "kinematics", f"joint '{joint_id}' must join two attachments, found {len(frames)}"
            )
        self.frames = frames
        self.bodies = tuple(body_of_frame(frame, graph) for frame in frames)
        self.kind = (
            JointKind.REVOLUTE
            if URI_KC_TYPE_REVOLUTE_JOINT in self.types
            else JointKind.FIXED
        )
        self.name = ""
        self.origin = np.zeros(3)
        self.axis = np.zeros(3)


class SegmentModel(ModelBase):
    """A body, and how it hangs off the one before it.

    `transform` places this body's frame in its parent's when every joint is at zero.
    """

    def __init__(
        self,
        body_id: URIRef,
        graph: Graph,
        name: str,
        hook: str,
        joint: JointModel | None,
        transform: RigidTransform,
        inertia: Inertia | None,
    ) -> None:
        super().__init__(node_id=body_id, graph=graph)
        self.name = name
        self.hook = hook
        self.joint = joint
        self.transform = transform
        self.inertia = inertia


class ChainModel(ModelBase):
    """A serial composition: the segments of its tree between two of them."""

    def __init__(self, chain_id: URIRef, graph: Graph, name: str, root: str, tip: str) -> None:
        super().__init__(node_id=chain_id, graph=graph)
        self.name = name
        self.root = root
        self.tip = tip


class TreeModel(ModelBase):
    """A kinematic tree: what hangs from its root, and the chains declared over it."""

    def __init__(
        self,
        tree_id: URIRef,
        graph: Graph,
        name: str,
        root: str,
        root_id: URIRef,
        segments: tuple[SegmentModel, ...],
        chains: tuple[ChainModel, ...],
    ) -> None:
        super().__init__(node_id=tree_id, graph=graph)
        self.name = name
        self.root = root
        # Nothing attaches the root, so it is the one name no segment carries.
        self.root_id = root_id
        self.segments = segments
        self.chains = chains


def as_moments(matrix: np.ndarray) -> np.ndarray:
    """A symmetric tensor read out as Ixx, Iyy, Izz, Ixy, Ixz, Iyz."""
    return np.array(
        [
            matrix[0][0],
            matrix[1][1],
            matrix[2][2],
            matrix[0][1],
            matrix[0][2],
            matrix[1][2],
        ]
    )


def frame_in_body(frame: URIRef, body: URIRef, graph: Graph) -> RigidTransform:
    """Where a frame sits on its body.

    A body's root frame is where the body is, so it needs no pose. Any other frame is
    somewhere, and a model that does not say where cannot be placed: report that rather
    than assume the two coincide, which is a difference no later error would point at.
    """
    root = get_root_frame(body, graph).id
    if frame == root:
        return RigidTransform.identity()

    transform = get_transform_between_frames(frame, root, graph)
    if transform is None:
        raise ConstraintViolation(
            "kinematics",
            f"no pose places frame '{frame}' on body '{body}': a frame that is not the "
            f"body's root frame needs a pose leading to it",
        )
    return transform


def trees_in_graph(graph: Graph) -> set[URIRef]:
    """Every kinematic tree and graph the model declares."""
    return {
        tree
        for tree in set(graph.subjects(RDF.type, URI_GEOM_TYPE_KTREE))
        | set(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH))
        if isinstance(tree, URIRef)
    }


def body_owners(graph: Graph) -> dict[URIRef, URIRef]:
    """The tree each body belongs to, from the edges that say so.

    A tree declares the body it is rooted at and owns the joints reaching the rest. The
    body a tree is rooted at is its own, and so is every body its own joints reach from
    there -- a composing tree owns only the joints attaching the trees below it, so it
    reaches none of their bodies from its root and claims none of them. Two trees may
    declare one root, since a composing tree inherits the root of the tree it composes;
    the one whose own joints touch it is the one describing it.
    """
    declared: dict[URIRef, set[URIRef]] = {}
    reaches: dict[URIRef, set[URIRef]] = {}
    touches: dict[URIRef, set[URIRef]] = {}

    for tree in trees_in_graph(graph):
        attached: dict[URIRef, list[URIRef]] = {}
        for joint in graph.objects(tree, URI_KC_PRED_JOINTS):
            if (joint, RDF.type, URI_KC_TYPE_JOINT) not in graph:
                continue
            bodies = [
                body_of_frame(frame, graph)
                for frame in graph.objects(joint, URI_KC_PRED_BETWEEN_ATTACHMENTS)
                if isinstance(frame, URIRef)
            ]
            for body in bodies:
                touches.setdefault(body, set()).add(tree)
                attached.setdefault(body, []).extend(other for other in bodies if other != body)

        roots = [
            body_of_frame(frame, graph)
            for frame in graph.objects(tree, URI_KC_EXT_PRED_ROOT)
            if isinstance(frame, URIRef)
        ]
        for root in roots:
            declared.setdefault(root, set()).add(tree)

        pending, seen = deque(roots), set(roots)
        while pending:
            body = pending.popleft()
            reaches.setdefault(body, set()).add(tree)
            for other in attached.get(body, []):
                if other not in seen:
                    seen.add(other)
                    pending.append(other)

    owners: dict[URIRef, URIRef] = {}
    for body in set(declared) | set(reaches):
        for candidates in (
            declared.get(body, set()) & touches.get(body, set()),
            declared.get(body, set()),
            reaches.get(body, set()),
        ):
            if len(candidates) == 1:
                owners[body] = next(iter(candidates))
                break
    return owners


def scoped_name(uri: URIRef, owner: URIRef | None) -> str:
    """What a backend calls an element: its own name under the tree that owns it.

    Two instances of one device share every local name, so the tree that owns the
    element is what tells them apart.
    """
    _, name = split_uri(uri)
    if owner is None:
        return name
    return f"{split_uri(owner)[1]}/{name}"


def body_of_frame(frame: URIRef, graph: Graph) -> URIRef:
    bodies = [
        body
        for body in graph.subjects(URI_GEOM_PRED_SIMPLICES, frame)
        if isinstance(body, URIRef) and (body, RDF.type, URI_GEOM_TYPE_RIGID_BODY) in graph
    ]
    if len(bodies) != 1:
        raise ConstraintViolation(
            "kinematics", f"frame '{frame}' must belong to one rigid body, found {bodies}"
        )
    return bodies[0]


def joints_in_graph(graph: Graph) -> dict[URIRef, JointModel]:
    """Every joint the graph declares, by IRI."""
    return {
        uri: JointModel(joint_id=uri, graph=graph)
        for uri in graph.subjects(RDF.type, URI_KC_TYPE_JOINT)
        if isinstance(uri, URIRef)
    }


def axis_of_vector(vector: URIRef, graph: Graph) -> tuple[URIRef, str]:
    """The frame a bound axis vector belongs to, and which of its axes it is."""
    for axis, predicate in AXIS_PREDS.items():
        for frame in graph.subjects(predicate, vector):
            if isinstance(frame, URIRef):
                return frame, axis
    raise ConstraintViolation("kinematics", f"axis vector '{vector}' is no frame's axis")


def revolute_axis(joint: JointModel, parent_frame: URIRef, graph: Graph) -> str:
    """The axis letter the joint turns about, rejecting an under-determined pairing."""
    common = ensure_one_obj_uri(graph=graph, subject=joint.id, predicate=URI_KC_PRED_COMMON_AXIS)
    if common is None:
        raise ConstraintViolation(
            "kinematics", f"revolute joint '{joint.id}' declares no common axis"
        )
    axes = dict(
        axis_of_vector(vector, graph)
        for vector in graph.objects(common, URI_GEOM_PRED_LINES)
        if isinstance(vector, URIRef)
    )
    if len(axes) != 2:
        raise ConstraintViolation(
            "kinematics", f"common axis of '{joint.id}' must relate two frame axes: {axes}"
        )
    if len(set(axes.values())) != 1:
        raise ConstraintViolation(
            "kinematics",
            f"revolute joint '{joint.id}' makes '{''.join(sorted(set(axes.values())))}' axes "
            f"collinear: which way the child frame then faces about them is undetermined",
        )
    if parent_frame not in axes:
        raise ConstraintViolation(
            "kinematics",
            f"common axis of '{joint.id}' does not mention its parent attachment "
            f"'{parent_frame}': it relates {sorted(str(frame) for frame in axes)}",
        )
    return axes[parent_frame]


def joint_offset(
    joint: JointModel, parent_frame: URIRef, child_frame: URIRef, graph: Graph
) -> tuple[float, float, float]:
    """The child frame's displacement from the anchor, seen by the anchor."""
    position = ensure_one_obj_uri(
        graph=graph, subject=joint.id, predicate=URI_KC_PRED_ORIGIN_OFFSET
    )
    if position is None:
        return (0.0, 0.0, 0.0)

    values = get_translation_between_points(
        FrameModel(frame_id=child_frame, graph=graph).origin,
        FrameModel(frame_id=parent_frame, graph=graph).origin,
        graph,
    )
    if values is None:
        raise ConstraintViolation("kinematics", f"offset '{position}' relates no two points")
    return values


def inertia_from_model_file(
    body: URIRef, graph: Graph, owner: URIRef | None, base_dir: Path | None
) -> Inertia | None:
    """What the mapped model file states, when the scene itself states no inertia."""
    read = read_body_inertia(body, graph, owner, base_dir)
    if read is None:
        return None
    mass, cog, matrix = read
    return Inertia(mass=mass, cog=cog, moments=as_moments(matrix))


def declared_inertia(
    body: URIRef, graph: Graph, owner: URIRef | None, base_dir: Path | None
) -> Inertia | None:
    """The body's inertia in its own frame: about the centre of mass, in body orientation."""
    inertias = [
        inertia
        for inertia in graph.subjects(URI_DYN_PRED_OF_BODY, body)
        if isinstance(inertia, URIRef)
    ]
    if not inertias:
        return inertia_from_model_file(body, graph, owner, base_dir)
    if len(inertias) > 1:
        raise ConstraintViolation("kinematics", f"body '{body}' has {len(inertias)} inertias")

    inertia = InertiaModel(inertias[0], graph)
    if inertia.mass is None or inertia.moments is None:
        # A frame-only inertia claims where the mass is, not how much: the file states that.
        return inertia_from_model_file(body, graph, owner, base_dir)

    ixx, iyy, izz, ixy, ixz, iyz = inertia.moments
    in_body = frame_in_body(inertia.inertial_frame.id, body, graph)
    rotation = in_body.rotation.as_matrix()
    matrix = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    return Inertia(
        mass=inertia.mass,
        cog=in_body.translation,
        moments=as_moments(rotation @ matrix @ rotation.T),
    )


def declaring_tree(joint: URIRef, graph: Graph) -> URIRef | None:
    """The tree that declares a joint. A chain lists joints too, and declares none."""
    trees = [tree for tree in trees_in_graph(graph) if (tree, URI_KC_PRED_JOINTS, joint) in graph]
    return trees[0] if len(trees) == 1 else None


def joint_owners(graph: Graph) -> dict[URIRef, set[URIRef]]:
    """The tree each joint belongs to. A serial chain also lists joints, and owns none."""
    owners: dict[URIRef, set[URIRef]] = {}
    for scope in trees_in_graph(graph):
        for joint in graph.objects(scope, URI_KC_PRED_JOINTS):
            if isinstance(joint, URIRef):
                owners.setdefault(joint, set()).add(scope)
    return owners


def tree_roots(joints: dict[URIRef, JointModel], graph: Graph) -> dict[URIRef, URIRef]:
    """The body each tree hangs from, and the tree that names it.

    A tree declares its own root, but a composed tree's root is attached by the tree
    composing it -- so the roots left are those no other tree's joint touches.
    """
    owners = joint_owners(graph)
    attached: dict[URIRef, set[URIRef]] = {}
    for joint in joints.values():
        for body in joint.bodies:
            attached.setdefault(body, set()).update(owners.get(joint.id, set()))

    roots: dict[URIRef, URIRef] = {}
    for tree in trees_in_graph(graph):
        frame = ensure_one_obj_uri(graph=graph, subject=tree, predicate=URI_KC_EXT_PRED_ROOT)
        if frame is None:
            continue
        body = body_of_frame(frame, graph)
        if attached.get(body, set()) - {tree}:
            continue
        roots[body] = tree

    # A body no joint attaches floats: it is placed, not articulated, so the model holds nothing.
    return roots


def segment_for(
    joint: JointModel,
    parent: URIRef,
    owners: dict[URIRef, URIRef],
    graph: Graph,
    base_dir: Path | None,
) -> SegmentModel:
    """The child body as a segment: its joint, and where it sits at zero position."""
    child_index = 1 if joint.bodies[0] == parent else 0
    child = joint.bodies[child_index]
    parent_frame, child_frame = joint.frames[1 - child_index], joint.frames[child_index]

    in_parent = frame_in_body(parent_frame, parent, graph)
    in_child = frame_in_body(child_frame, child, graph)
    # An attachment makes two frames coincide unless a pose says how they differ.
    attach = get_transform_between_frames(child_frame, parent_frame, graph)
    if attach is None:
        attach = RigidTransform.identity()

    direction = np.zeros(3)
    if joint.kind is JointKind.REVOLUTE:
        axis = revolute_axis(joint, parent_frame, graph)
        offset = joint_offset(joint, parent_frame, child_frame, graph)
        attach = RigidTransform.from_translation(offset) * attach
        direction = in_parent.rotation.as_matrix()[:, "xyz".index(axis)]

    joint.name = scoped_name(joint.id, declaring_tree(joint.id, graph))
    joint.origin = in_parent.translation
    joint.axis = direction

    return SegmentModel(
        body_id=child,
        graph=graph,
        name=scoped_name(child, owners.get(child)),
        hook=scoped_name(parent, owners.get(parent)),
        joint=joint,
        transform=in_parent * attach * in_child.inv(),
        inertia=segment_inertia(
            child, graph, owners.get(child), base_dir, moves=joint.kind is JointKind.REVOLUTE
        ),
    )


def segment_inertia(
    body: URIRef, graph: Graph, owner: URIRef | None, base_dir: Path | None, moves: bool
) -> Inertia | None:
    """What a body weighs, if anything has to know.

    A body a fixed joint holds carries the tree without moving in it -- a sensor's frame,
    a table, a mounting plate -- so having no mass is a thing it may legitimately be, and
    A backend carries that as a zero inertia. A body a joint moves is another matter: a missing
    mass there is a hole in the dynamics, and is reported rather than assumed away.
    """
    try:
        return declared_inertia(body, graph, owner, base_dir)
    except ConstraintViolation:
        if moves:
            raise
        return None


def tip_frame_segment(tip_frame: URIRef, tip: URIRef, owners: dict[URIRef, URIRef], graph: Graph) -> SegmentModel:
    """A frame a chain ends at, but its body is not rooted at, is a fixed segment of its own."""
    return SegmentModel(
        body_id=tip_frame,
        graph=graph,
        name=f"{scoped_name(tip, owners.get(tip))}/{split_uri(tip_frame)[1]}",
        hook=scoped_name(tip, owners.get(tip)),
        joint=None,
        transform=frame_in_body(tip_frame, tip, graph),
        inertia=None,
    )


def chains_over(
    root: URIRef,
    parents: dict[URIRef, URIRef],
    owners: dict[URIRef, URIRef],
    graph: Graph,
) -> tuple[tuple[ChainModel, ...], list[SegmentModel]]:
    """Every declared serial composition whose bodies are this tree's, and the segments
    its endpoints add.

    A chain is the tree between two of its segments, so it names them rather than
    repeating them. A tip frame is no body, so nothing has made a segment of it yet:
    it joins the tree as a fixed leaf, and the chain then slices out to it.
    """
    chains, tips = [], {}
    for serial in graph.subjects(RDF.type, URI_KC_TYPE_SERIAL):
        if not isinstance(serial, URIRef):
            continue
        root_frame = ensure_one_obj_uri(graph=graph, subject=serial, predicate=URI_KC_EXT_PRED_ROOT)
        tip_frame = ensure_one_obj_uri(graph=graph, subject=serial, predicate=URI_KC_EXT_PRED_TIP)
        if root_frame is None or tip_frame is None:
            raise ConstraintViolation(
                "kinematics", f"serial chain '{serial}' declares no root or no tip"
            )

        chain_root, tip = body_of_frame(root_frame, graph), body_of_frame(tip_frame, graph)
        if tip not in parents and tip != root:
            continue
        if root_frame != get_root_frame(chain_root, graph).id:
            raise ConstraintViolation(
                "kinematics",
                f"serial chain '{serial}' starts at '{root_frame}', which is not the root frame "
                f"of body '{chain_root}': a chain carries no offset before its first segment",
            )

        body = tip
        while body != chain_root:
            parent = parents.get(body)
            if parent is None:
                raise ConstraintViolation(
                    "kinematics", f"serial chain '{serial}' tip is not below its root"
                )
            body = parent

        tip_name = scoped_name(tip, owners.get(tip))
        if tip_frame != get_root_frame(tip, graph).id:
            segment = tip_frame_segment(tip_frame, tip, owners, graph)
            # Two chains may end at one frame, and the tree holds it once.
            tips.setdefault(segment.name, segment)
            tip_name = segment.name
        chains.append(
            ChainModel(
                chain_id=serial,
                graph=graph,
                name="__".join(
                    part.replace("/", "_")
                    for part in (scoped_name(chain_root, owners.get(chain_root)), tip_name)
                ),
                root=scoped_name(chain_root, owners.get(chain_root)),
                tip=tip_name,
            )
        )
    return tuple(chains), list(tips.values())


def build_kinematic_model(graph: Graph, base_dir: Path | None = None) -> list[TreeModel]:
    """Every tree the graph's kinematics hang from, with the chains declared over them."""
    owners = body_owners(graph)
    joints = joints_in_graph(graph)
    trees = []

    for root, owner in sorted(tree_roots(joints, graph).items(), key=lambda item: str(item[0])):
        pending, seen, ordered = deque([root]), {root}, []
        parents: dict[URIRef, URIRef] = {}
        while pending:
            body = pending.popleft()
            for joint in joints.values():
                if body not in joint.bodies:
                    continue
                child = joint.bodies[1 if joint.bodies[0] == body else 0]
                if child in seen:
                    continue
                seen.add(child)
                parents[child] = body
                ordered.append(segment_for(joint, body, owners, graph, base_dir))
                pending.append(child)

        # A fixed segment may weigh nothing; one a joint moves may not (see segment_inertia).
        missing = [
            segment.name
            for segment in ordered
            if segment.inertia is None and segment.joint is not None
            and segment.joint.kind is JointKind.REVOLUTE
        ]
        if missing:
            raise ConstraintViolation(
                "kinematics",
                f"no inertia for {', '.join(sorted(missing))}: state mass and inertia-matrix "
                f"in the model, or map the bodies to a model file that does",
            )

        # Appended last, so each tip frame follows the body it hangs from.
        chains, tips = chains_over(root, parents, owners, graph)
        trees.append(
            TreeModel(
                tree_id=owner,
                graph=graph,
                name=split_uri(owner)[1],
                root=scoped_name(root, owners.get(root)),
                root_id=root,
                segments=tuple(ordered + tips),
                chains=chains,
            )
        )
    return trees
