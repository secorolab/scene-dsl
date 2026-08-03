# SPDX-License-Identifier: MPL-2.0
"""The kinematics a scene's graph states: bodies, the joints between them, chains, trees.

Two things the graph leaves for the reader. A joint's attachments are a set, so which of
its bodies carries the other is only settled by walking out of a declared root. And a tree
composing another states no edge to it, so a component is expanded by activating every
tree whose root body the walk reaches.
"""

from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase
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
from scipy.spatial.transform import RigidTransform

from scene_dsl.rdf_parser.common import ensure_one_obj_uri
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

        The scene answers, or the URDF or MJCF its model maps the body into does. Neither
        answering is an error: nothing else can be asked, and a body of unknown mass is
        not a body a solver can carry.
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

        read = read_body_inertia(self.id, graph, owner, base_dir)
        if read is None:
            raise ConstraintViolation(
                "dynamics",
                f"nothing states the inertia of body '{self.id}': give it mass and "
                f"inertia-matrix in the scene, or map it to a model file that states them",
            )
        mass, com, tensor = read
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

    @property
    def moves(self) -> bool:
        """Whether the joint has a degree of freedom, or only holds its two frames together."""
        return False

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

    @property
    def moves(self) -> bool:
        return True

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

    The graph states the joints as a set; `path` is that set ordered root outwards, which
    the tree holding the chain fills in.
    """

    root_frame: URIRef
    tip_frame: URIRef
    root_body: URIRef
    tip_body: URIRef
    joints: set[URIRef]
    path: tuple[URIRef, ...]

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
        self.path = ()


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
        tree_ids = uris(graph.subjects(RDF.type, URI_GEOM_TYPE_KTREE)) | uris(
            graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH)
        )
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
        return min(candidates or rooted_trees, key=str)


@dataclass
class _ExpandedComponent:
    """One directed component expanded from a top-level root body."""

    root_body: URIRef
    bodies: dict[URIRef, RigidBodyModel]
    parent_by_body: dict[URIRef, URIRef]
    parent_joint_by_body: dict[URIRef, URIRef]
    defining_tree_by_body: dict[URIRef, URIRef]
    body_order: list[URIRef]
    chains: list[KinematicChainModel]


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
        body_order=[],
        chains=[],
    )
    active_joints: dict[URIRef, URIRef] = {}  # joint -> declaring tree
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

    # Every joint a tree carries places one of its bodies. One that placed nothing is a
    # piece of the kinematics gone missing, which is worth more than a quieter picture.
    unused_joints = active_joints.keys() - set(component.parent_joint_by_body.values())
    if unused_joints:
        raise ConstraintViolation(
            "kinematics",
            f"the component rooted at '{root_body}' never reaches "
            f"{sorted(unused_joints, key=str)}: those joints attach nothing the walk found",
        )

    return component


def _expand_components(index: _KinematicIndex, graph: Graph) -> list[_ExpandedComponent]:
    """Expand every top-level component exactly once."""
    roots = index.component_roots()
    claimed_bodies = set(roots)  # reserve all roots before any component is walked
    return [_expand_component(root, index, graph, claimed_bodies) for root in roots]


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
        if component is None or component_by_body.get(chain.tip_body) is not component:
            continue
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
        chain.path = path
        component.chains.append(chain)


def _top_level_tree(component: _ExpandedComponent, index: _KinematicIndex) -> URIRef:
    """Outermost tree that names an expanded component."""
    candidates = index.trees_by_root_body[component.root_body]
    if len(candidates) == 1:
        return next(iter(candidates))

    composed = set()
    for declaring_tree in candidates:
        for joint_id in index.joints_by_tree[declaring_tree]:
            for body in index.joints[joint_id].bodies:
                defining_tree = component.defining_tree_by_body.get(body)
                if defining_tree is not None and defining_tree != declaring_tree:
                    composed.add(defining_tree)
    return min(candidates - composed or candidates, key=str)


class KinematicTreeModel(ModelBase):
    """One directed kinematic component, including all composed trees."""

    root_frame: FrameModel
    root: URIRef
    bodies: dict[URIRef, RigidBodyModel]
    joints: dict[URIRef, JointModel]
    parent: dict[URIRef, URIRef]
    parent_joint: dict[URIRef, URIRef]
    defining_tree_by_body: dict[URIRef, URIRef]
    topological_order: tuple[URIRef, ...]
    chains: tuple[KinematicChainModel, ...]

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
        self.root_frame = root_frame_of(self.id, graph)
        self.root = component.root_body
        self.topological_order = tuple(component.body_order)
        self.bodies = dict(component.bodies)
        self.parent = dict(component.parent_by_body)
        self.parent_joint = dict(component.parent_joint_by_body)
        self.joints = {joint_id: index.joints[joint_id] for joint_id in self.parent_joint.values()}
        self.defining_tree_by_body = dict(component.defining_tree_by_body)
        self.chains = tuple(component.chains)

    def mass_properties(self, body: URIRef, graph: Graph) -> MassProperties:
        """Mass properties using the body's defining tree for mapped-model lookup."""
        return self.bodies[body].mass_properties(
            graph,
            owner=self.defining_tree_by_body.get(body),
            base_dir=self.base_dir,
        )


def kinematic_trees(graph: Graph, base_dir: Path | None = None) -> list[KinematicTreeModel]:
    """Read all top-level kinematic components from the graph."""
    index = _KinematicIndex(graph)
    components = _expand_components(index, graph)
    _assign_chains(index, components)
    return [
        KinematicTreeModel(
            _top_level_tree(component, index),
            graph,
            component,
            index,
            base_dir,
        )
        for component in components
    ]
