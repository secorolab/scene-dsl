# SPDX-License-Identifier: MPL-2.0
"""What KDL builds a tree from, lowered out of the scene's kinematics.

A KDL segment is `pose(q) = joint.pose(q) * f_tip`: KDL applies the joint position
itself, so what a segment is built with is the constant part -- where the child body
sits in its parent while the joint contributes nothing. The graph relates frames rather
than bodies, so that constant is composed here. The names are here too: KDL knows a
segment by a string, and nothing but a backend needs one.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.geom_coord import get_transform_between_frames
from rdf_utils.models.vocab import URI_KC_TYPE_REVOLUTE_JOINT
from rdflib import Graph, URIRef
from rdflib.namespace import split_uri
from scipy.spatial.transform import RigidTransform

from scene_dsl.rdf_parser.ktree import (
    Attachment,
    Inertia,
    KinematicTreeModel,
    body_owners,
    build_kinematic_trees,
    declared_inertia,
    declaring_tree,
    frame_in_body,
    get_root_frame,
    joint_offset,
    revolute_axis,
)


@dataclass
class Joint:
    """The line a joint moves about, in the parent body's frame.

    `axis` is None for a joint that does not move: KDL builds it as `Joint::None`, and
    the segment is then only the pose its parent holds it at.
    """

    iri: URIRef
    name: str
    origin: np.ndarray
    axis: np.ndarray | None


@dataclass
class Segment:
    """A body, its joint, and where it sits in the segment before it at zero position."""

    iri: URIRef
    name: str
    parent: str
    joint: Joint | None
    transform: RigidTransform
    inertia: Inertia | None


@dataclass
class Chain:
    """A slice of a tree: the names KDL is asked to cut between."""

    iri: URIRef
    name: str
    root: str
    tip: str


@dataclass
class Tree:
    """One `KDL::Tree` to build, and the chains sliced out of it."""

    iri: URIRef
    name: str
    root: str
    root_iri: URIRef
    segments: tuple[Segment, ...]
    chains: tuple[Chain, ...]


def scoped_name(uri: URIRef, owner: URIRef | None) -> str:
    """What KDL calls an element: its own name under the tree that owns it.

    Two instances of one device share every local name and may hang in one tree, where
    KDL tells its segments apart by name alone -- so the owning tree qualifies it.
    """
    _, name = split_uri(uri)
    if owner is None:
        return name
    return f"{split_uri(owner)[1]}/{name}"


def segment_inertia(
    body: URIRef, graph: Graph, owner: URIRef | None, base_dir: Path | None, moves: bool
) -> Inertia | None:
    """What a body weighs, if anything has to know.

    A body a fixed joint holds carries the tree without moving in it -- a sensor's frame,
    a table, a mounting plate -- so having no mass is a thing it may legitimately be, and
    KDL carries that as a zero inertia. A body a joint moves is another matter: a missing
    mass there is a hole in the dynamics, and is reported rather than assumed away.
    """
    try:
        return declared_inertia(body, graph, owner, base_dir)
    except ConstraintViolation:
        if moves:
            raise
        return None


def segment_for(
    attachment: Attachment,
    owners: dict[URIRef, URIRef],
    graph: Graph,
    base_dir: Path | None,
) -> Segment:
    """The child body as a segment: its joint, and where it sits at zero position."""
    parent, joint, child = attachment
    child_index = 1 if joint.bodies[0] == parent else 0
    parent_frame, child_frame = joint.frames[1 - child_index], joint.frames[child_index]

    in_parent = frame_in_body(parent_frame, parent, graph)
    in_child = frame_in_body(child_frame, child, graph)
    # An attachment makes two frames coincide unless a pose says how they differ.
    attach = get_transform_between_frames(child_frame, parent_frame, graph)
    if attach is None:
        attach = RigidTransform.identity()

    direction = None
    if URI_KC_TYPE_REVOLUTE_JOINT in joint.types:
        axis = revolute_axis(joint, parent_frame, graph)
        offset = joint_offset(joint, parent_frame, child_frame, graph)
        attach = RigidTransform.from_translation(offset) * attach
        direction = in_parent.rotation.as_matrix()[:, "xyz".index(axis)]

    return Segment(
        iri=child,
        name=scoped_name(child, owners.get(child)),
        parent=scoped_name(parent, owners.get(parent)),
        joint=Joint(
            iri=joint.id,
            name=scoped_name(joint.id, declaring_tree(joint.id, graph)),
            origin=in_parent.translation,
            axis=direction,
        ),
        transform=in_parent * attach * in_child.inv(),
        inertia=segment_inertia(
            child, graph, owners.get(child), base_dir, moves=direction is not None
        ),
    )


def tip_frame_segment(
    tip_frame: URIRef, tip: URIRef, owners: dict[URIRef, URIRef], graph: Graph
) -> Segment:
    """A frame a chain ends at, but its body is not rooted at, is a fixed segment of its own."""
    return Segment(
        iri=tip_frame,
        name=f"{scoped_name(tip, owners.get(tip))}/{split_uri(tip_frame)[1]}",
        parent=scoped_name(tip, owners.get(tip)),
        joint=None,
        transform=frame_in_body(tip_frame, tip, graph),
        inertia=None,
    )


def chains_of(
    tree: KinematicTreeModel, owners: dict[URIRef, URIRef], graph: Graph
) -> tuple[list[Chain], list[Segment]]:
    """The tree's chains, and the segments their endpoints add.

    A chain is the tree between two of its segments, so it names them rather than
    repeating them. A tip frame is no body, so nothing has made a segment of it yet:
    it joins the tree as a fixed leaf, and the chain then slices out to it.
    """
    chains, tips = [], {}
    for chain in tree.chains:
        root_name = scoped_name(chain.root_body, owners.get(chain.root_body))
        tip_name = scoped_name(chain.tip_body, owners.get(chain.tip_body))
        if chain.tip_frame != get_root_frame(chain.tip_body, graph).id:
            segment = tip_frame_segment(chain.tip_frame, chain.tip_body, owners, graph)
            # Two chains may end at one frame, and the tree holds it once.
            tips.setdefault(segment.name, segment)
            tip_name = segment.name
        chains.append(
            Chain(
                iri=chain.id,
                name="__".join(part.replace("/", "_") for part in (root_name, tip_name)),
                root=root_name,
                tip=tip_name,
            )
        )
    return chains, list(tips.values())


def build_kdl_trees(graph: Graph, base_dir: Path | None = None) -> list[Tree]:
    """Every tree KDL has to build for this scene, with the chains declared over them."""
    owners = body_owners(graph)
    trees = []

    for tree in build_kinematic_trees(graph):
        segments = [
            segment_for(attachment, owners, graph, base_dir) for attachment in tree.attachments
        ]
        # A fixed segment may weigh nothing; one a joint moves may not (see segment_inertia).
        missing = [
            segment.name
            for segment in segments
            if segment.inertia is None
            and segment.joint is not None
            and segment.joint.axis is not None
        ]
        if missing:
            raise ConstraintViolation(
                "kinematics",
                f"no inertia for {', '.join(sorted(missing))}: state mass and inertia-matrix "
                f"in the model, or map the bodies to a model file that does",
            )

        # Appended last, so each tip frame follows the body it hangs from.
        chains, tips = chains_of(tree, owners, graph)
        trees.append(
            Tree(
                iri=tree.id,
                name=split_uri(tree.id)[1],
                root=scoped_name(tree.root, owners.get(tree.root)),
                # Nothing attaches the root, so it is the one name no segment carries.
                root_iri=tree.root,
                segments=tuple(segments + tips),
                chains=tuple(chains),
            )
        )
    return trees
