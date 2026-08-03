# SPDX-License-Identifier: MPL-2.0
"""The kinematics a scene graph states: bodies, the joints between them, chains.

Everything here is read: a joint's two frames, the tree they hang in once directed out
of a root, the inertia a body declares. Nothing is computed from it -- a pose composed,
a tensor turned, a name a library knows a segment by are all its backend's to derive.
"""

from collections import deque
from typing import NamedTuple

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase
from rdf_utils.models.geom_coord import get_translation_between_points
from rdf_utils.models.geom_rel import FrameModel
from rdf_utils.models.vocab import (
    URI_DYN_PRED_ABOUT,
    URI_DYN_PRED_IXX,
    URI_DYN_PRED_IXY,
    URI_DYN_PRED_IXZ,
    URI_DYN_PRED_IYY,
    URI_DYN_PRED_IYZ,
    URI_DYN_PRED_IZZ,
    URI_DYN_PRED_MASS,
    URI_DYN_PRED_OF_BODY,
    URI_DYN_PRED_OF_INERTIA,
    URI_DYN_TYPE_RIGID_BODY_INERTIA,
    URI_GEOM_PRED_LINES,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SIMPLICES,
    URI_GEOM_PRED_VECT_X,
    URI_GEOM_PRED_VECT_Y,
    URI_GEOM_PRED_VECT_Z,
    URI_GEOM_TYPE_FRAME,
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
    URI_KC_TYPE_SERIAL,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_G,
    URI_QUDT_UNIT_KG,
)
from rdflib import RDF, Graph, Literal, URIRef

from scene_dsl.rdf_parser.common import ensure_one_obj_uri, ensure_one_typed_subject_uri

MASS_IN_KG = {URI_QUDT_UNIT_KG: 1.0, URI_QUDT_UNIT_G: 1e-3}
MOMENT_PREDS = (
    URI_DYN_PRED_IXX,
    URI_DYN_PRED_IYY,
    URI_DYN_PRED_IZZ,
    URI_DYN_PRED_IXY,
    URI_DYN_PRED_IXZ,
    URI_DYN_PRED_IYZ,
)
AXIS_PREDS = {"x": URI_GEOM_PRED_VECT_X, "y": URI_GEOM_PRED_VECT_Y, "z": URI_GEOM_PRED_VECT_Z}


class InertiaModel(ModelBase):
    """A body's rigid-body inertia: where its mass is, and how much of it.

    `mass` and `moments` are None for an inertia that names only its frame, which is a
    body claiming where its centre of mass sits without saying what it weighs.
    """

    inertial_frame: FrameModel
    mass: float | None
    moments: np.ndarray | None

    def __init__(self, inertia_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=inertia_id, graph=graph)

        if URI_DYN_TYPE_RIGID_BODY_INERTIA not in self.types:
            raise TypeError(f"{self} is not a RigidBodyInertia")

        about_node = ensure_one_obj_uri(graph=graph, subject=self.id, predicate=URI_DYN_PRED_ABOUT)
        if about_node is None:
            raise ValueError(
                f"RigidBodyInertia {self} does not link to an URI via predicate 'about'"
            )
        inertial_frame_id = ensure_one_typed_subject_uri(
            graph=graph,
            obj=about_node,
            predicate=URI_GEOM_PRED_ORIGIN,
            subject_type=URI_GEOM_TYPE_FRAME,
        )
        if inertial_frame_id is None:
            raise ValueError(f"RigidBodyInertia {self} about {about_node} is not a frame origin")
        self.inertial_frame = FrameModel(frame_id=inertial_frame_id, graph=graph)

        self.mass = None
        self.moments = None
        coords = [
            coord
            for coord in graph.subjects(URI_DYN_PRED_OF_INERTIA, self.id)
            if isinstance(coord, URIRef)
        ]
        if len(coords) > 1:
            raise ConstraintViolation(
                "dynamics", f"RigidBodyInertia {self} has {len(coords)} coordinates"
            )
        if not coords:
            return

        mass = graph.value(coords[0], URI_DYN_PRED_MASS)
        moments = [graph.value(coords[0], predicate) for predicate in MOMENT_PREDS]
        stated = [moment for moment in moments if isinstance(moment, Literal)]
        # An inertia may name only its frame: a body says where its mass is, not how much.
        if not isinstance(mass, Literal) or len(stated) != len(moments):
            return

        units = {
            unit
            for unit in graph.objects(coords[0], URI_QUDT_PRED_UNIT)
            if isinstance(unit, URIRef) and unit in MASS_IN_KG
        }
        if len(units) != 1:
            raise ConstraintViolation(
                "dynamics", f"inertia coordinate '{coords[0]}' must have one mass unit: {units}"
            )
        self.mass = float(mass.toPython()) * MASS_IN_KG[units.pop()]
        self.moments = np.array([float(moment.toPython()) for moment in stated])


def get_root_frame(target_id: URIRef, graph: Graph) -> FrameModel:
    """Return the declared root frame of a kinematics model."""

    root_id = ensure_one_obj_uri(graph, target_id, URI_KC_EXT_PRED_ROOT)
    if root_id is None:
        raise ValueError(f"Kinematics model '{target_id}' doesn't link to a root URI")

    return FrameModel(frame_id=root_id, graph=graph)


class RigidBodyModel(ModelBase):
    simplices: set[URIRef]
    root_frame: FrameModel
    inertia: InertiaModel | None

    def __init__(self, body_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=body_id, graph=graph)
        if URI_GEOM_TYPE_RIGID_BODY not in self.types:
            raise TypeError(f"{self} is not a RigidBody")

        self.simplices = set()
        for simplice_node in graph.objects(subject=self.id, predicate=URI_GEOM_PRED_SIMPLICES):
            if not isinstance(simplice_node, URIRef):
                raise TypeError(
                    f"RigidBody '{self}' doesn't link to an URI via 'geom_rel:simplices': {simplice_node}"
                )
            self.simplices.add(simplice_node)

        self.root_frame = get_root_frame(target_id=self.id, graph=graph)
        if self.root_frame.id not in self.simplices:
            raise ValueError(
                f"Root frame '{self.root_frame}' is not a simplicial complex of RigidBody '{self}'"
            )

        inertia_id = ensure_one_typed_subject_uri(
            graph=graph,
            obj=self.id,
            predicate=URI_DYN_PRED_OF_BODY,
            subject_type=URI_DYN_TYPE_RIGID_BODY_INERTIA,
        )
        if not inertia_id:
            self.inertia = None
        else:
            self.inertia = InertiaModel(inertia_id=inertia_id, graph=graph)


class JointModel(ModelBase):
    """A joint: the two frames it attaches, and the bodies those frames are on.

    Which of the two is the parent is a property of the tree the joint hangs in, not of
    the joint, so the pair stays in the order the graph gives it.
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


class SerialChainModel(ModelBase):
    """A serial composition: the frames it runs between, and the bodies they are on."""

    def __init__(self, chain_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=chain_id, graph=graph)
        root_frame = ensure_one_obj_uri(
            graph=graph, subject=chain_id, predicate=URI_KC_EXT_PRED_ROOT
        )
        tip_frame = ensure_one_obj_uri(graph=graph, subject=chain_id, predicate=URI_KC_EXT_PRED_TIP)
        if root_frame is None or tip_frame is None:
            raise ConstraintViolation(
                "kinematics", f"serial chain '{chain_id}' declares no root or no tip"
            )
        self.root_frame = root_frame
        self.tip_frame = tip_frame
        self.root_body = body_of_frame(root_frame, graph)
        self.tip_body = body_of_frame(tip_frame, graph)


class Attachment(NamedTuple):
    """A joint, directed: the body it hangs off, and the body it carries."""

    parent: URIRef
    joint: JointModel
    child: URIRef


class KinematicTreeModel(ModelBase):
    """A tree of bodies out of one root, and the serial chains declared over it.

    `attachments` are ordered root outwards, so a body always follows the one it hangs
    from -- the graph relates the two by an unordered set of attachments, and which way
    round they go is only answered by walking out of the root.
    """

    def __init__(
        self,
        tree_id: URIRef,
        graph: Graph,
        root: URIRef,
        attachments: tuple[Attachment, ...],
        chains: tuple[SerialChainModel, ...],
    ) -> None:
        super().__init__(node_id=tree_id, graph=graph)
        self.root = root
        self.attachments = attachments
        self.chains = chains


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


def trees_in_graph(graph: Graph) -> set[URIRef]:
    """Every kinematic tree and graph the model declares."""
    return {
        tree
        for tree in set(graph.subjects(RDF.type, URI_GEOM_TYPE_KTREE))
        | set(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH))
        if isinstance(tree, URIRef)
    }


def joints_in_graph(graph: Graph) -> dict[URIRef, JointModel]:
    """Every joint the graph declares, by IRI."""
    return {
        uri: JointModel(joint_id=uri, graph=graph)
        for uri in graph.subjects(RDF.type, URI_KC_TYPE_JOINT)
        if isinstance(uri, URIRef)
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


def chains_over(
    root: URIRef, parents: dict[URIRef, URIRef], graph: Graph
) -> tuple[SerialChainModel, ...]:
    """Every declared serial composition whose bodies are this tree's.

    A chain is the tree between two of its bodies, so it declares its endpoints and
    nothing in between: what it holds is only checked to be there, and to be below its
    root.
    """
    chains = []
    for serial in graph.subjects(RDF.type, URI_KC_TYPE_SERIAL):
        if not isinstance(serial, URIRef):
            continue
        chain = SerialChainModel(chain_id=serial, graph=graph)
        if chain.tip_body not in parents and chain.tip_body != root:
            continue
        if chain.root_frame != get_root_frame(chain.root_body, graph).id:
            raise ConstraintViolation(
                "kinematics",
                f"serial chain '{serial}' starts at '{chain.root_frame}', which is not the root "
                f"frame of body '{chain.root_body}': a chain carries no offset before its first "
                f"segment",
            )

        body = chain.tip_body
        while body != chain.root_body:
            parent = parents.get(body)
            if parent is None:
                raise ConstraintViolation(
                    "kinematics", f"serial chain '{serial}' tip is not below its root"
                )
            body = parent
        chains.append(chain)
    return tuple(chains)


def build_kinematic_trees(graph: Graph) -> list[KinematicTreeModel]:
    """Every tree the graph's kinematics hang from, directed out of its root."""
    joints = joints_in_graph(graph)
    trees = []

    for root, owner in sorted(tree_roots(joints, graph).items(), key=lambda item: str(item[0])):
        pending, seen, attachments = deque([root]), {root}, []
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
                attachments.append(Attachment(parent=body, joint=joint, child=child))
                pending.append(child)

        trees.append(
            KinematicTreeModel(
                tree_id=owner,
                graph=graph,
                root=root,
                attachments=tuple(attachments),
                chains=chains_over(root, parents, graph),
            )
        )
    return trees
