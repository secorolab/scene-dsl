# SPDX-License-Identifier: MPL-2.0
"""The kinematics a scene's graph states: bodies, joints, chains, trees, and the graph itself.

Two things the graph leaves for the reader. A joint's attachments are a set, so which of
its bodies carries the other is only settled by walking out of a declared root. And a tree
composing another states no edge to it, so a component is expanded by activating every
tree whose root body the walk reaches.

Trees are views over the kinematics a graph states, and nothing in the metamodel stops two
of them sharing bodies. This reads them as though nothing does: a body has one describing
tree, an element one name, a tree one tree holding it. Views that overlap without one
holding the other are reported rather than read, since the answer would be arbitrary.
"""

from collections import deque
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.geom_coord import get_transform_between_frames
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
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_MAPS,
    URI_EXEC_PRED_MODEL_ENTITY,
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
    URI_KC_TYPE_REVOLUTE_JOINT,
    URI_KC_TYPE_SERIAL,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_G,
    URI_QUDT_UNIT_KG,
)
from rdflib import RDF, Graph, Literal, URIRef
from rdflib.namespace import split_uri
from scipy.spatial.transform import RigidTransform

from scene_dsl.rdf_parser.common import ensure_one_obj_literal, ensure_one_obj_uri
from scene_dsl.rdf_parser.mapped_inertia import read_body_inertia

MASS_IN_KG = {URI_QUDT_UNIT_KG: 1.0, URI_QUDT_UNIT_G: 1e-3}
AXIS_PREDS = {"x": URI_GEOM_PRED_VECT_X, "y": URI_GEOM_PRED_VECT_Y, "z": URI_GEOM_PRED_VECT_Z}
TENSOR_PREDS = (
    (URI_DYN_PRED_IXX, URI_DYN_PRED_IXY, URI_DYN_PRED_IXZ),
    (URI_DYN_PRED_IXY, URI_DYN_PRED_IYY, URI_DYN_PRED_IYZ),
    (URI_DYN_PRED_IXZ, URI_DYN_PRED_IYZ, URI_DYN_PRED_IZZ),
)


def uris(nodes) -> set[URIRef]:
    """The URI nodes of a query, which is all a well-formed graph holds here."""
    return {node for node in nodes if isinstance(node, URIRef)}


def typed(nodes, node_type: URIRef, graph: Graph) -> set[URIRef]:
    """The nodes of a query that carry a type.

    Asked of each candidate, which the store answers from its index -- listing every node
    of the type and intersecting would walk the whole graph to filter two or three nodes.
    """
    return {node for node in uris(nodes) if (node, RDF.type, node_type) in graph}


def _ensure_one_tree(trees: set[URIRef], claim: str) -> URIRef:
    """The tree a narrowing left, once it left one. Which of several it is, if it left
    several, is the model's to say and not a reader's."""
    if len(trees) != 1:
        raise ConstraintViolation("kinematics", f"{claim}: {sorted(trees, key=str)}")
    return next(iter(trees))


def body_of_frame(frame: URIRef, graph: Graph) -> URIRef:
    """The body a frame is a simplex of. A frame sits on one body, or on none."""
    bodies = typed(graph.subjects(URI_GEOM_PRED_SIMPLICES, frame), URI_GEOM_TYPE_RIGID_BODY, graph)
    if len(bodies) != 1:
        raise ConstraintViolation(
            "kinematics", f"frame '{frame}' must be a simplex of one rigid body, found {bodies}"
        )
    return bodies.pop()


def root_frame_of(node: URIRef, graph: Graph) -> FrameModel:
    """The frame a body or a tree is rooted at."""
    frame = ensure_one_obj_uri(graph=graph, subject=node, predicate=URI_KC_EXT_PRED_ROOT)
    if frame is None:
        raise ConstraintViolation("kinematics", f"'{node}' declares no root frame")
    return FrameModel(frame_id=frame, graph=graph)


# What a model may map: a scene names a body or a whole tree, nothing else.
MAPPABLE_TYPES = {URI_GEOM_TYPE_RIGID_BODY, URI_GEOM_TYPE_KTREE}


@dataclass
class KinematicMapping:
    target_id: URIRef
    target_type: URIRef
    entity: str | None = None


def get_kinematic_mapping(mapping_id: URIRef, graph: Graph) -> KinematicMapping:
    target_uri = ensure_one_obj_uri(graph=graph, subject=mapping_id, predicate=URI_EXEC_PRED_MAPS)
    if target_uri is None:
        raise ValueError(f"mapping '{mapping_id}' does not map to an URI target")

    target_types = get_node_types(graph=graph, node_id=target_uri) & MAPPABLE_TYPES
    if len(target_types) != 1:
        raise ValueError(f"mapping '{mapping_id}' target '{target_uri}' must be one body or tree")

    entity_literal = ensure_one_obj_literal(
        graph=graph, subject=mapping_id, predicate=URI_EXEC_PRED_MODEL_ENTITY
    )

    entity = None
    if entity_literal is not None:
        entity = entity_literal.toPython()

        if not isinstance(entity, str):
            raise TypeError(f"mapping '{mapping_id}' entity must be a string")

    return KinematicMapping(target_uri, target_types.pop(), entity=entity)


def load_attr_kinematic_mappings(graph: Graph, model: ModelBase, **kwargs: object) -> None:
    mappings = []
    targets = set()
    for mapping_id in graph.objects(model.id, URI_EXEC_PRED_HAS_MAPPING):
        if not isinstance(mapping_id, URIRef):
            raise TypeError(f"model '{model}' has non-URI mapping: {mapping_id}")
        mapping = get_kinematic_mapping(mapping_id, graph)
        if mapping.target_id in targets:
            raise ValueError(f"multiple mappings found for target {mapping.target_id}")
        targets.add(mapping.target_id)
        mappings.append(mapping)
    model.set_attr(URI_EXEC_PRED_HAS_MAPPING, tuple(mappings))


def get_kinematic_mappings(
    model: ModelBase, target_type: URIRef | None = None
) -> tuple[KinematicMapping, ...]:
    mappings = model.get_attr(URI_EXEC_PRED_HAS_MAPPING)
    if not isinstance(mappings, tuple) or not all(
        isinstance(mapping, KinematicMapping) for mapping in mappings
    ):
        raise TypeError(f"model '{model}' has no loaded mappings")
    if target_type is None:
        return mappings
    return tuple(mapping for mapping in mappings if mapping.target_type == target_type)


def mapped_model(body: URIRef, graph: Graph, owner: URIRef | None) -> tuple[URIRef, str] | None:
    """The model describing this body, and the name it knows the body by.

    A mapping of the body names it outright; a mapping of the tree that owns it leaves the
    body's own name to stand, since a tree mapping names the tree, not each body under it.
    Which tree owns the body is the caller's to say -- the graph answers it from the joints
    a tree reaches its bodies through, not from how the IRIs happen to nest.
    """
    by_tree: tuple[URIRef, str] | None = None
    for model in graph.subjects(URI_EXEC_PRED_HAS_MAPPING, None):
        if not isinstance(model, URIRef):
            continue
        for mapping_id in graph.objects(model, URI_EXEC_PRED_HAS_MAPPING):
            if not isinstance(mapping_id, URIRef):
                continue
            mapping = get_kinematic_mapping(mapping_id, graph)
            if mapping.target_id == body:
                return model, mapping.entity or split_uri(body)[1]
            if mapping.target_type == URI_GEOM_TYPE_KTREE and mapping.target_id == owner:
                by_tree = (model, split_uri(body)[1])
    return by_tree


@dataclass
class MassProperties:
    """What a body weighs, in the body's own frame.

    Not a model: a body whose mass its scene leaves to a URDF or an MJCF has no node in
    the graph to read this off.
    """

    mass: float
    com: np.ndarray  # centre of mass, in the body frame
    tensor: np.ndarray  # inertia tensor about the centre of mass, along the body's axes


class InertiaModel(ModelBase):
    """The inertia a scene states for a body: the frame it is taken in, and its values.

    A scene may name the frame and stop there, claiming where a body's mass sits without
    saying how much of it there is; `mass` and `tensor` are then None.
    """

    inertial_frame: FrameModel
    mass: float | None
    tensor: np.ndarray | None

    def __init__(self, inertia_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=inertia_id, graph=graph)
        if URI_DYN_TYPE_RIGID_BODY_INERTIA not in self.types:
            raise TypeError(f"{self} is not a RigidBodyInertia")

        about = ensure_one_obj_uri(graph=graph, subject=self.id, predicate=URI_DYN_PRED_ABOUT)
        if about is None:
            raise ConstraintViolation("dynamics", f"{self} is about no point")
        frames = typed(graph.subjects(URI_GEOM_PRED_ORIGIN, about), URI_GEOM_TYPE_FRAME, graph)
        if len(frames) != 1:
            raise ConstraintViolation(
                "dynamics", f"{self} is about '{about}', which is the origin of {frames}"
            )
        self.inertial_frame = FrameModel(frame_id=frames.pop(), graph=graph)

        self.mass = None
        self.tensor = None
        coords = uris(graph.subjects(URI_DYN_PRED_OF_INERTIA, self.id))
        if len(coords) > 1:
            raise ConstraintViolation("dynamics", f"{self} has {len(coords)} coordinates")
        if coords:
            self._load_coord(coords.pop(), graph)

    def _load_coord(self, coord: URIRef, graph: Graph) -> None:
        mass = graph.value(coord, URI_DYN_PRED_MASS)
        values = [[graph.value(coord, pred) for pred in row] for row in TENSOR_PREDS]
        stated = [value for row in values for value in row if isinstance(value, Literal)]
        # Values are written together or not at all: a partial one is a body half described.
        if not isinstance(mass, Literal) or not stated:
            return
        if len(stated) != 9:
            raise ConstraintViolation(
                "dynamics", f"inertia coordinate '{coord}' states {len(stated)} of 9 tensor terms"
            )

        units = uris(graph.objects(coord, URI_QUDT_PRED_UNIT)) & MASS_IN_KG.keys()
        if len(units) != 1:
            raise ConstraintViolation(
                "dynamics", f"inertia coordinate '{coord}' must state one mass unit, has {units}"
            )
        self.mass = float(mass.toPython()) * MASS_IN_KG[units.pop()]
        # `stated` holds the nine terms in the order the rows name them.
        self.tensor = np.array([float(term.toPython()) for term in stated], dtype=float).reshape(
            3, 3
        )


class RigidBodyModel(ModelBase):
    """A body: the frames it carries, the one it is at, and what it weighs."""

    frames: set[URIRef]
    root_frame: FrameModel
    inertia: InertiaModel | None

    def __init__(self, body_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=body_id, graph=graph)
        if URI_GEOM_TYPE_RIGID_BODY not in self.types:
            raise TypeError(f"{self} is not a RigidBody")

        self.frames = uris(graph.objects(self.id, URI_GEOM_PRED_SIMPLICES))
        self.root_frame = root_frame_of(self.id, graph)
        if self.root_frame.id not in self.frames:
            raise ConstraintViolation(
                "kinematics", f"{self} is rooted at '{self.root_frame.id}', which is not its frame"
            )

        inertia = uris(graph.subjects(URI_DYN_PRED_OF_BODY, self.id))
        if len(inertia) > 1:
            raise ConstraintViolation("dynamics", f"{self} has {len(inertia)} inertias")
        self.inertia = InertiaModel(inertia.pop(), graph) if inertia else None

    def pose_of(self, frame: URIRef, graph: Graph) -> RigidTransform:
        """Where one of the body's frames sits on it.

        The root frame is where the body is, so it needs no pose. Any other frame is
        somewhere, and a scene that does not say where cannot be placed -- report it,
        rather than assume the two coincide.
        """
        if frame == self.root_frame.id:
            return RigidTransform.identity()

        pose = get_transform_between_frames(frame, self.root_frame.id, graph)
        if pose is None:
            raise ConstraintViolation(
                "kinematics",
                f"no pose places frame '{frame}' on body '{self.id}': a frame other than "
                f"the body's root needs a pose leading to it",
            )
        return pose

    def mass_properties(
        self, graph: Graph, owner: URIRef | None = None, base_dir: Path | None = None
    ) -> MassProperties:
        """What the body weighs, in its own frame.

        The scene answers, or the URDF or MJCF its model maps the body into does. A body
        is matter, so neither answering is an error -- a frame is what carries no mass.
        """
        stated = self.inertia
        if stated is not None and stated.mass is not None and stated.tensor is not None:
            # The inertia is stated about its frame; a solver wants it about the body's.
            in_body = self.pose_of(stated.inertial_frame.id, graph)
            rotation = in_body.rotation.as_matrix()
            return MassProperties(
                mass=stated.mass,
                # The frame is where the mass is, so its origin is the centre of mass.
                com=in_body.translation,
                # I_body = R I_frame R^T, R being the inertia frame's orientation in the
                # body: a second-order tensor changes axes by a similarity transform. No
                # parallel axis term -- both frames sit at the centre of mass.
                tensor=rotation @ stated.tensor @ rotation.T,
            )

        mapped = mapped_model(self.id, graph, owner)
        if mapped is None:
            raise ConstraintViolation(
                "dynamics",
                f"nothing states the inertia of body '{self.id}': give it mass and "
                f"inertia-matrix in the scene, or map it to a model file that states them",
            )
        mass, com, tensor = read_body_inertia(mapped, graph, base_dir)
        return MassProperties(mass=mass, com=com, tensor=tensor)


class JointModel(ModelBase):
    """A joint and the two frames it holds together.

    The graph states the pair unordered, since which side carries the other is the tree's
    to say, not the joint's -- see `KinematicTreeModel`.
    """

    frames: tuple[URIRef, ...]
    bodies: tuple[URIRef, ...]

    def __init__(self, joint_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=joint_id, graph=graph)
        if URI_KC_TYPE_JOINT not in self.types:
            raise TypeError(f"{self} is not a Joint")

        self.frames = tuple(sorted(uris(graph.objects(self.id, URI_KC_PRED_BETWEEN_ATTACHMENTS))))
        if len(self.frames) != 2:
            raise ConstraintViolation(
                "kinematics", f"{self} holds {len(self.frames)} attachments, and a joint holds two"
            )
        self.bodies = tuple(body_of_frame(frame, graph) for frame in self.frames)

    def other_body(self, body: URIRef) -> URIRef:
        """The body across the joint from this one."""
        if body not in self.bodies:
            raise ValueError(f"joint '{self.id}' does not attach body '{body}'")
        return self.bodies[1] if self.bodies[0] == body else self.bodies[0]

    def frame_on(self, body: URIRef) -> URIRef:
        """The attachment this joint holds on the given body."""
        if body not in self.bodies:
            raise ValueError(f"joint '{self.id}' does not attach body '{body}'")
        return self.frames[self.bodies.index(body)]


class RevoluteJointModel(JointModel):
    """A joint turning about an axis its two attachments share.

    The graph states the axis as a collinearity of two bound vectors, one per attachment;
    `axis_on` names which of a frame's own axes that is. An offset, if stated, displaces
    the child attachment from the parent one.
    """

    axes: dict[URIRef, str]
    offset: URIRef | None

    def __init__(self, joint_id: URIRef, graph: Graph) -> None:
        super().__init__(joint_id=joint_id, graph=graph)

        common = ensure_one_obj_uri(graph=graph, subject=self.id, predicate=URI_KC_PRED_COMMON_AXIS)
        if common is None:
            raise ConstraintViolation("kinematics", f"{self} turns about no common axis")
        self.axes = dict(
            _axis_of_vector(vector, graph)
            for vector in uris(graph.objects(common, URI_GEOM_PRED_LINES))
        )
        if set(self.axes) != set(self.frames):
            raise ConstraintViolation(
                "kinematics",
                f"the common axis of {self} relates the axes of {sorted(self.axes)}, and not "
                f"those of the frames it attaches",
            )
        if len(set(self.axes.values())) != 1:
            raise ConstraintViolation(
                "kinematics",
                f"{self} makes unlike axes collinear ({', '.join(sorted(self.axes.values()))}): "
                f"which way the child then faces about them is left undetermined",
            )
        self.offset = ensure_one_obj_uri(
            graph=graph, subject=self.id, predicate=URI_KC_PRED_ORIGIN_OFFSET
        )

    def axis_on(self, frame: URIRef) -> str:
        """Which of the frame's own axes the joint turns about."""
        if frame not in self.axes:
            raise ValueError(f"joint '{self.id}' does not turn about an axis of '{frame}'")
        return self.axes[frame]


def _axis_of_vector(vector: URIRef, graph: Graph) -> tuple[URIRef, str]:
    """The frame a bound axis vector belongs to, and which of its axes it is."""
    for axis, predicate in AXIS_PREDS.items():
        frames = uris(graph.subjects(predicate, vector))
        if frames:
            return frames.pop(), axis
    raise ConstraintViolation("kinematics", f"'{vector}' is no frame's axis")


def joint_model(joint_id: URIRef, graph: Graph) -> JointModel:
    """The joint, as whichever kind of joint the graph types it."""
    if (joint_id, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT) in graph:
        return RevoluteJointModel(joint_id=joint_id, graph=graph)
    return JointModel(joint_id=joint_id, graph=graph)


class KinematicChainModel(ModelBase):
    """A serial composition: the frames it runs between, and the joints it runs through.

    The graph states the joints as a set. In what order they run is the component's to
    say, not the chain's, so the ordered path lives there -- see `KinematicTreeModel.path`.
    """

    root_frame: URIRef
    tip_frame: URIRef
    root_body: URIRef
    tip_body: URIRef
    joints: set[URIRef]

    def __init__(self, chain_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=chain_id, graph=graph)
        if URI_KC_TYPE_SERIAL not in self.types:
            raise TypeError(f"{self} is not a SerialComposition")

        root = ensure_one_obj_uri(graph=graph, subject=self.id, predicate=URI_KC_EXT_PRED_ROOT)
        tip = ensure_one_obj_uri(graph=graph, subject=self.id, predicate=URI_KC_EXT_PRED_TIP)
        if root is None or tip is None:
            raise ConstraintViolation("kinematics", f"{self} declares no root or no tip")
        self.root_frame = root
        self.tip_frame = tip
        self.root_body = body_of_frame(root, graph)
        self.tip_body = body_of_frame(tip, graph)
        self.joints = uris(graph.objects(self.id, URI_KC_PRED_JOINTS))


class _KinematicIndex:
    """Graph-level index used to reconstruct directed, composed kinematics."""

    def __init__(self, graph: Graph) -> None:
        self.joints = {
            joint_id: joint_model(joint_id, graph)
            for joint_id in sorted(uris(graph.subjects(RDF.type, URI_KC_TYPE_JOINT)), key=str)
        }
        self.chains = [
            KinematicChainModel(chain_id, graph)
            for chain_id in sorted(uris(graph.subjects(RDF.type, URI_KC_TYPE_SERIAL)), key=str)
        ]

        self.joints_by_body: dict[URIRef, set[URIRef]] = {}
        for joint in self.joints.values():
            for body in joint.bodies:
                self.joints_by_body.setdefault(body, set()).add(joint.id)

        self.joints_by_tree: dict[URIRef, set[URIRef]] = {}
        self.trees_by_root_body: dict[URIRef, set[URIRef]] = {}
        self.kgraphs = uris(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH))
        tree_ids = uris(graph.subjects(RDF.type, URI_GEOM_TYPE_KTREE)) | self.kgraphs
        for tree_id in sorted(tree_ids, key=str):
            self.joints_by_tree[tree_id] = (
                uris(graph.objects(tree_id, URI_KC_PRED_JOINTS)) & self.joints.keys()
            )
            for root_frame in uris(graph.objects(tree_id, URI_KC_EXT_PRED_ROOT)):
                root_body = body_of_frame(root_frame, graph)
                self.trees_by_root_body.setdefault(root_body, set()).add(tree_id)

    def component_roots(self) -> list[URIRef]:
        """Bodies with no joint declared by a tree rooted elsewhere."""
        roots = []
        for body, rooted_trees in self.trees_by_root_body.items():
            local_joints: set[URIRef] = set()
            for tree_id in rooted_trees:
                local_joints.update(self.joints_by_tree[tree_id])
            if self.joints_by_body.get(body, set()) <= local_joints:
                roots.append(body)
        return sorted(roots, key=str)

    def activate_rooted_trees(self, body: URIRef, active_joints: dict[URIRef, URIRef]) -> None:
        """Activate each tree rooted at a body reached by the walk."""
        for tree_id in sorted(self.trees_by_root_body.get(body, set()), key=str):
            for joint_id in sorted(self.joints_by_tree[tree_id], key=str):
                active_joints.setdefault(joint_id, tree_id)

    def defining_tree(self, body: URIRef, declaring_tree: URIRef | None) -> URIRef:
        """Tree whose model mapping defines a body.

        A tree rooted at the body defines it when one of that tree's own joints touches
        the body. Otherwise the body remains in the tree whose joint reached it.
        """
        rooted_trees = self.trees_by_root_body.get(body)
        if not rooted_trees:
            if declaring_tree is None:
                raise ConstraintViolation(
                    "kinematics", f"body '{body}' is not rooted in or reached by a tree"
                )
            return declaring_tree

        touching = self.joints_by_body.get(body, set())
        candidates = {
            tree_id for tree_id in rooted_trees if self.joints_by_tree[tree_id] & touching
        }
        return _ensure_one_tree(
            candidates or rooted_trees,
            f"body '{body}' is the root of several trees whose joints touch it, so which "
            f"one describes it cannot be told",
        )


@dataclass
class _ExpandedComponent:
    """One directed component expanded from a top-level root body."""

    root_body: URIRef
    bodies: dict[URIRef, RigidBodyModel]
    parent_by_body: dict[URIRef, URIRef]
    parent_joint_by_body: dict[URIRef, URIRef]
    defining_tree_by_body: dict[URIRef, URIRef]
    declaring_tree_by_joint: dict[URIRef, URIRef]
    body_order: list[URIRef]
    chains: list[KinematicChainModel]
    chain_paths: dict[URIRef, tuple[URIRef, ...]]


def _expand_component(
    root_body: URIRef,
    index: _KinematicIndex,
    graph: Graph,
    claimed_bodies: set[URIRef],
) -> _ExpandedComponent:
    """Direct one component by walking outwards from its root."""
    component = _ExpandedComponent(
        root_body=root_body,
        bodies={root_body: RigidBodyModel(root_body, graph)},
        parent_by_body={},
        parent_joint_by_body={},
        defining_tree_by_body={root_body: index.defining_tree(root_body, declaring_tree=None)},
        declaring_tree_by_joint={},
        body_order=[],
        chains=[],
        chain_paths={},
    )
    active_joints = component.declaring_tree_by_joint  # joint -> declaring tree
    pending = deque([root_body])

    while pending:
        body = pending.popleft()
        component.body_order.append(body)
        index.activate_rooted_trees(body, active_joints)

        available = index.joints_by_body.get(body, set()) & active_joints.keys()
        for joint_id in sorted(available, key=str):
            # The joint this body hangs from, met again from the other side.
            if joint_id == component.parent_joint_by_body.get(body):
                continue

            child = index.joints[joint_id].other_body(body)
            if child in component.bodies:
                raise ConstraintViolation(
                    "kinematics",
                    f"joint '{joint_id}' creates a cycle or a second path to body '{child}': "
                    f"a tree reaches each of its bodies exactly one way",
                )
            if child in claimed_bodies:
                raise ConstraintViolation(
                    "kinematics",
                    f"joint '{joint_id}' connects the component rooted at '{root_body}' to "
                    f"'{child}', which hangs from a root of its own",
                )

            declaring_tree = active_joints[joint_id]
            claimed_bodies.add(child)
            component.bodies[child] = RigidBodyModel(child, graph)
            component.parent_by_body[child] = body
            component.parent_joint_by_body[child] = joint_id
            component.defining_tree_by_body[child] = index.defining_tree(child, declaring_tree)
            pending.append(child)

    return component


def _expand_components(index: _KinematicIndex, graph: Graph) -> list[_ExpandedComponent]:
    """Expand every top-level component exactly once."""
    roots = index.component_roots()
    claimed_bodies = set(roots)  # reserve all roots before any component is walked
    components = [_expand_component(root, index, graph, claimed_bodies) for root in roots]

    # A joint that placed no body is kinematics gone missing. Asked of the scene, not of
    # each walk: a graph rooted at several bodies activates its joints in all of them.
    active_joints: set[URIRef] = set()
    parent_joints: set[URIRef] = set()
    for component in components:
        active_joints.update(component.declaring_tree_by_joint)
        parent_joints.update(component.parent_joint_by_body.values())

    unreached = active_joints - parent_joints
    if unreached:
        raise ConstraintViolation(
            "kinematics",
            f"nothing reaches {sorted(unreached, key=str)}: those joints are the parent of "
            f"no body the walk out of any root found",
        )

    return components


def _chain_path(chain: KinematicChainModel, component: _ExpandedComponent) -> tuple[URIRef, ...]:
    """Ordered root-to-tip joints recovered from directed parent links."""
    path = []
    body = chain.tip_body
    while body != chain.root_body:
        if body not in component.parent_by_body:
            raise ConstraintViolation("kinematics", f"{chain} has a tip not below its root")
        path.append(component.parent_joint_by_body[body])
        body = component.parent_by_body[body]
    return tuple(reversed(path))


def _assign_chains(index: _KinematicIndex, components: list[_ExpandedComponent]) -> None:
    """Assign each declared chain to the component containing both endpoints."""
    component_by_body = {body: component for component in components for body in component.bodies}

    for chain in index.chains:
        component = component_by_body.get(chain.root_body)
        # A chain runs through joints, so both its ends hang from one root or it runs
        # through nothing. Dropping it would leave the model claiming a chain and the
        # picture, the header and every solver reading them showing none.
        if component is None or component_by_body.get(chain.tip_body) is not component:
            raise ConstraintViolation(
                "kinematics",
                f"{chain} runs from body '{chain.root_body}' to body '{chain.tip_body}', but "
                f"no joints lead from the one to the other: a chain is a path through a tree",
            )
        if chain.root_frame != component.bodies[chain.root_body].root_frame.id:
            raise ConstraintViolation(
                "kinematics",
                f"{chain} starts at '{chain.root_frame}', which is not the root frame of "
                f"body '{chain.root_body}': a chain carries no offset before its first joint",
            )

        path = _chain_path(chain, component)
        if set(path) != chain.joints:
            raise ConstraintViolation(
                "kinematics",
                f"{chain} declares joints {sorted(chain.joints, key=str)}, but its "
                f"root-to-tip path contains {sorted(path, key=str)}",
            )
        component.chain_paths[chain.id] = path
        component.chains.append(chain)


def _trees_held(tree: URIRef, component: _ExpandedComponent, index: _KinematicIndex) -> set[URIRef]:
    """The trees a tree's joints reach into: a device touches its own bodies, an assembly
    the devices it joins. No edge states it."""
    held = set()
    for joint_id in index.joints_by_tree[tree]:
        for body in index.joints[joint_id].bodies:
            defining_tree = component.defining_tree_by_body.get(body)
            if defining_tree is not None and defining_tree != tree:
                held.add(defining_tree)
    return held


def _top_level_tree(component: _ExpandedComponent, index: _KinematicIndex) -> URIRef | None:
    """Outermost tree naming a component; the graph itself where it articulates bodies of
    its own, and nothing for a body it only places."""
    root = component.root_body
    rooted = index.trees_by_root_body[root]
    candidates = rooted - index.kgraphs
    if not candidates:
        if not component.parent_joint_by_body:
            return None
        return _ensure_one_tree(
            rooted,
            f"body '{root}' is joined to others, and is the root of several graphs, so "
            f"which of them the kinematics under it belongs to cannot be told",
        )
    if len(candidates) == 1:
        return next(iter(candidates))

    composed = {held for tree in candidates for held in _trees_held(tree, component, index)}
    return _ensure_one_tree(
        candidates - composed or candidates,
        f"several trees are rooted at body '{root}' and none of them holds the others, so "
        f"which one names the kinematics under it cannot be told",
    )


def _nested_trees(
    component: _ExpandedComponent, index: _KinematicIndex
) -> dict[URIRef, tuple[URIRef, ...]]:
    """The trees each tree composes. Trees sharing a root body nest by how much each
    holds; the outermost hangs from the tree whose joint attached that body."""
    parent: dict[URIRef, URIRef] = {}
    for body in component.body_order:
        rooted = index.trees_by_root_body.get(body, set()) - index.kgraphs
        holds = {tree: _trees_held(tree, component, index) & rooted for tree in rooted}
        if len({len(held) for held in holds.values()}) != len(rooted):
            counts = ", ".join(f"'{tree}' holds {len(held)}" for tree, held in sorted(holds.items()))
            raise ConstraintViolation(
                "kinematics",
                f"several trees are rooted at body '{body}' and each holds as many other "
                f"trees as the next, so which of them composes the other cannot be told "
                f"({counts}). An assembly is the tree whose joints reach into the ones it "
                f"holds together",
            )

        # Trees at one root nest, so each holds the one below it. Two that hold each
        # other's bodies without either holding the other are views laid over the same
        # kinematics, which the rest of this -- one describing tree per body, one name
        # per element, a picture of clusters inside clusters -- has no way to express.
        order = sorted(rooted, key=lambda tree: len(holds[tree]))
        for inner, outer in pairwise(order):
            if inner not in holds[outer]:
                raise ConstraintViolation(
                    "kinematics",
                    f"trees '{inner}' and '{outer}' are both rooted at body '{body}' and "
                    f"neither is composed into the other, so they overlap: a tree may share "
                    f"the bodies of another only by holding it",
                )
            parent[inner] = outer

        joint = component.parent_joint_by_body.get(body)
        if order and joint is not None:
            holder = component.declaring_tree_by_joint[joint]
            if holder != order[-1]:
                parent[order[-1]] = holder

    nested: dict[URIRef, list[URIRef]] = {}
    for tree, holder in sorted(parent.items(), key=lambda pair: str(pair[0])):
        nested.setdefault(holder, []).append(tree)
    return {holder: tuple(trees) for holder, trees in nested.items()}


def _declaring_trees(
    component: _ExpandedComponent, nested: dict[URIRef, tuple[URIRef, ...]], default: URIRef
) -> dict[URIRef, URIRef]:
    """What declares each joint and chain: the walk states a joint's tree, and a chain
    belongs to the innermost tree holding both the bodies it runs between."""
    holder = {sub: tree for tree, subs in nested.items() for sub in subs}
    declaring = dict(component.declaring_tree_by_joint)
    for chain in component.chains:
        above = set()
        tree = component.defining_tree_by_body[chain.root_body]
        while tree is not None:
            above.add(tree)
            tree = holder.get(tree)

        tree = component.defining_tree_by_body[chain.tip_body]
        while tree is not None and tree not in above:
            tree = holder.get(tree)
        declaring[chain.id] = default if tree is None else tree
    return declaring


class KinematicTreeModel(ModelBase):
    """One directed kinematic component, including all composed trees."""

    root_frame: FrameModel
    root: URIRef
    bodies: dict[URIRef, RigidBodyModel]
    joints: dict[URIRef, JointModel]
    parent: dict[URIRef, URIRef]
    parent_joint: dict[URIRef, URIRef]
    defining_tree_by_body: dict[URIRef, URIRef]
    declaring_tree: dict[URIRef, URIRef]
    topological_order: tuple[URIRef, ...]
    chains: tuple[KinematicChainModel, ...]
    chain_paths: dict[URIRef, tuple[URIRef, ...]]
    nested_trees: dict[URIRef, tuple[URIRef, ...]]

    def __init__(
        self,
        tree_id: URIRef,
        graph: Graph,
        component: _ExpandedComponent,
        index: _KinematicIndex,
        base_dir: Path | None = None,
    ) -> None:
        super().__init__(node_id=tree_id, graph=graph)
        if not self.types & {URI_GEOM_TYPE_KTREE, URI_GEOM_TYPE_KGRAPH}:
            raise TypeError(f"{self} is not a KinematicTree")

        self.base_dir = base_dir
        self.root = component.root_body
        # Off the root body: a graph rooted at several bodies states no one frame.
        self.root_frame = component.bodies[self.root].root_frame
        self.topological_order = tuple(component.body_order)
        self.bodies = dict(component.bodies)
        self.parent = dict(component.parent_by_body)
        self.parent_joint = dict(component.parent_joint_by_body)
        self.joints = {joint_id: index.joints[joint_id] for joint_id in self.parent_joint.values()}
        self.defining_tree_by_body = dict(component.defining_tree_by_body)
        self.chains = tuple(component.chains)
        self.chain_paths = dict(component.chain_paths)
        self.nested_trees = _nested_trees(component, index)
        self.declaring_tree = _declaring_trees(component, self.nested_trees, default=self.id)

    def path(self, chain: KinematicChainModel) -> tuple[URIRef, ...]:
        """The joints of a chain, root outwards, as the walk out of this root ordered them."""
        return self.chain_paths[chain.id]

    def mass_properties(self, body: URIRef, graph: Graph) -> MassProperties:
        """Mass properties using the body's defining tree for mapped-model lookup."""
        return self.bodies[body].mass_properties(
            graph,
            owner=self.defining_tree_by_body.get(body),
            base_dir=self.base_dir,
        )


def kinematic_trees(graph: Graph, base_dir: Path | None = None) -> list[KinematicTreeModel]:
    """Read all top-level kinematic components from the graph. A body the scene only
    places is articulated by nothing, so no tree is built for it."""
    index = _KinematicIndex(graph)
    components = _expand_components(index, graph)
    _assign_chains(index, components)
    trees = ((component, _top_level_tree(component, index)) for component in components)
    return [
        KinematicTreeModel(tree_id, graph, component, index, base_dir)
        for component, tree_id in trees
        if tree_id is not None
    ]


def root_bodies(node: URIRef, graph: Graph) -> set[URIRef]:
    """The bodies a graph hangs from: what its trees stand on, and what it only places."""
    return {
        body_of_frame(frame, graph) for frame in uris(graph.objects(node, URI_KC_EXT_PRED_ROOT))
    }


def is_attached(body: URIRef, graph: Graph) -> bool:
    """Whether any joint holds this body, which is what makes it part of a tree."""
    return any(
        typed(graph.subjects(URI_KC_PRED_BETWEEN_ATTACHMENTS, frame), URI_KC_TYPE_JOINT, graph)
        for frame in uris(graph.objects(body, URI_GEOM_PRED_SIMPLICES))
    )


class KinematicGraphModel(ModelBase):
    """The kinematics of one scene: its trees, and the free bodies it places itself.

    A graph states a root for every body no joint attaches, which is what says the body is
    this graph's and not another's. A tree stands on one of them; the rest are only placed.
    """

    trees: tuple[KinematicTreeModel, ...]
    free_bodies: dict[URIRef, RigidBodyModel]
    nested_trees: dict[URIRef, tuple[URIRef, ...]]

    def __init__(
        self,
        kgraph_id: URIRef,
        graph: Graph,
        base_dir: Path | None = None,
        trees: list[KinematicTreeModel] | None = None,
    ) -> None:
        super().__init__(node_id=kgraph_id, graph=graph)
        if URI_GEOM_TYPE_KGRAPH not in self.types:
            raise TypeError(f"{self} is not a KinematicGraph")

        self.base_dir = base_dir
        roots = root_bodies(self.id, graph)
        built = kinematic_trees(graph, base_dir) if trees is None else trees
        self.trees = tuple(tree for tree in built if tree.root in roots)

        # A body a joint holds is carried by a tree, and belongs to it; what is left hangs
        # from nothing -- placed by a pose, not articulated.
        self.free_bodies = {
            body: RigidBodyModel(body, graph)
            for body in sorted(roots, key=str)
            if not is_attached(body, graph)
        }

        # A component the graph itself names is no tree below it: those bodies are its own.
        self.nested_trees = {self.id: tuple(tree.id for tree in self.trees if tree.id != self.id)}
        for tree in self.trees:
            self.nested_trees.update(tree.nested_trees)


def kinematic_graphs(graph: Graph, base_dir: Path | None = None) -> list[KinematicGraphModel]:
    """Every kinematic graph the scene declares, with the trees below each already built."""
    trees = kinematic_trees(graph, base_dir)
    return [
        KinematicGraphModel(kgraph_id, graph, base_dir, trees)
        for kgraph_id in sorted(uris(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH)), key=str)
    ]
