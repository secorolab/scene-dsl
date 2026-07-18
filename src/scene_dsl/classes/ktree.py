# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from copy import deepcopy
from typing import Any, Optional

import numpy as np
from rdflib import Namespace, URIRef

from scene_dsl.classes.common import (
    FloatVector,
    IHasNamespace,
    IHasNamespaceDeclare,
    IHasParent,
)
from scene_dsl.classes.geom import Frame, FrameAxis, IDefaultFrame


class KinematicTreeTemplate(IHasParent):
    """A device described without being any particular one: it mints no IRI."""

    name: str
    bodies: list[RigidBody]
    joints_spec: JointsSpec

    def __init__(self, parent, name, bodies, joints_spec) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.bodies = bodies
        self.joints_spec = joints_spec


class KinematicStructure(IHasNamespaceDeclare, IDefaultFrame):
    """Bodies, the joints between them, and the trees composed into them.

    Every element under it takes its namespace from it.
    """

    name: str
    trees: list[KinematicTreeModel]
    bodies: list[RigidBody]
    joints_spec: Optional[JointsSpec]

    def __init__(self, parent, ns, name, trees, bodies, joints_spec) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.trees = trees
        self.bodies = bodies
        self.joints_spec = joints_spec

    @property
    def subtrees(self) -> dict[int, KinematicStructure]:
        """This structure and every tree composed below it, keyed by identity."""
        found: dict[int, KinematicStructure] = {}
        stack = [self]
        while stack:
            tree = stack.pop()
            # A tree may be composed by two others, or -- nothing forbids it -- by itself.
            if id(tree) in found:
                continue
            found[id(tree)] = tree
            stack.extend(tree.trees)
        return found

    @property
    def all_joints(self) -> list[JointBase]:
        """Every joint of this structure and of the trees it composes."""
        return [
            joint
            for tree in self.subtrees.values()
            if tree.joints_spec is not None
            for joint in tree.joints_spec.joints
        ]

    @property
    def roots(self) -> list[RigidBody]:
        """The bodies no joint attaches: what the rest of the structure hangs from."""
        attached = {id(joint.child_frame.parent) for joint in self.all_joints}
        return [
            body
            for tree in self.subtrees.values()
            for body in tree.bodies
            if id(body) not in attached
        ]

    @property
    def default_frame(self) -> Frame:
        if not self.bodies:
            raise ValueError(f"default_frame: {self} has no body")
        return self.bodies[0].default_frame


class KinematicGraph(KinematicStructure):
    """The kinematics of a whole scene: trees, and the bodies that hang from nothing."""


class KinematicTreeModel(KinematicStructure):
    """A concrete kinematic tree: one root, and no way back to a body once you leave it."""

    def __init__(self, parent, ns, name, trees, bodies, joints_spec) -> None:
        super().__init__(parent, ns, name, trees, bodies, joints_spec)

    def composition_cycle(self) -> list[KinematicTreeModel]:
        """The chain of composed trees leading from this tree back to itself, if any."""

        def walk(tree, path):
            for linked in tree.trees:
                if linked is self:
                    return path + [linked]
                # A cycle not passing through this tree is reported by one that it does.
                if any(linked is seen for seen in path):
                    continue
                found = walk(linked, path + [linked])
                if found:
                    return found
            return []

        return walk(self, [self])


class KinematicTreeInstance(KinematicTreeModel):
    """A concrete tree copied from a namespace-less template."""

    def __init__(self, parent, ns, name, template) -> None:
        super().__init__(parent, ns, name, trees=[], bodies=[], joints_spec=None)
        self.template = template

    def copy_template(self) -> None:
        """Take the template's structure as this tree's own, under its name and namespace.

        The memo maps the template onto this tree, so every copied element lands with this
        tree as its parent -- which is what scopes their IRIs -- and the template's
        internal references follow onto the copies.
        """
        memo: dict[int, Any] = {id(self.template): self}
        self.bodies = deepcopy(self.template.bodies, memo)
        self.joints_spec = deepcopy(self.template.joints_spec, memo)
        self.copies = memo


class RigidBody(IHasNamespace, IDefaultFrame):
    name: str
    frames: list[Frame]
    inertia: Optional[RigidBodyInertia]

    def __init__(self, parent, name, frames, inertia) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.frames = frames
        self.inertia = inertia
        self._uri = None
        self._inertia_uri = None
        self._inertia_coord_uri = None

    @property
    def namespace(self) -> Namespace:
        if isinstance(self.parent, KinematicTreeTemplate):
            raise AttributeError(f"'{self.parent.name}' is a template, so it has no namespace")
        if not isinstance(self.parent, KinematicStructure):
            raise TypeError(f"parent of RigidBody has no namespace: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.scoped()]
        return self._uri

    @property
    def default_frame(self) -> Frame:
        if len(self.frames) < 1:
            raise ValueError(f"RigidBody.default_frame: {self.uri} has no frame")
        return self.frames[0]

    @property
    def inertia_uri(self) -> URIRef:
        if self._inertia_uri is None:
            self._inertia_uri = self.namespace[self.scoped("-inertia")]
        return self._inertia_uri

    @property
    def inertia_coord_uri(self) -> URIRef:
        if self._inertia_coord_uri is None:
            self._inertia_coord_uri = self.namespace[self.scoped("-inertia-coord")]
        return self._inertia_coord_uri


class RigidBodyInertia:
    frame: Frame
    mass: float
    mass_unit: str
    row1: FloatVector
    row2: FloatVector
    row3: FloatVector
    inertia_unit: str

    matrix: np.ndarray

    def __init__(self, parent, frame, mass, mass_unit, row1, row2, row3, inertia_unit) -> None:
        self.parent = parent
        self.frame = frame
        if mass < 0:
            raise ValueError(f"RigidBodyInertia.mass must be >= 0, got {mass}")
        self.mass = mass
        self.mass_unit = mass_unit
        self.row1 = row1
        self.row2 = row2
        self.row3 = row3
        self.inertia_unit = inertia_unit

        self.matrix = np.array(
            (
                self.row1.as_xyz("RigidBodyInertia.row1"),
                self.row2.as_xyz("RigidBodyInertia.row2"),
                self.row3.as_xyz("RigidBodyInertia.row3"),
            ),
            dtype=float,
        )
        if not np.allclose(self.matrix, self.matrix.T):
            raise ValueError("RigidBodyInertia.matrix must be symmetric")


class JointsSpec(IHasNamespace):
    joints: list[JointBase]
    joint_comp: Optional[Any]

    def __init__(self, parent, joints, joint_comp) -> None:
        super().__init__(parent=parent)
        self.joints = joints
        self.joint_comp = joint_comp

    @property
    def namespace(self) -> Namespace:
        if isinstance(self.parent, KinematicTreeTemplate):
            raise AttributeError(f"'{self.parent.name}' is a template, so it has no namespace")
        if not isinstance(self.parent, KinematicStructure):
            raise TypeError(f"parent of JointsSpec is not a KinematicStructure: {self.parent}")
        return self.parent.namespace


class JointBase(IHasNamespace):
    def __init__(self, parent) -> None:
        super().__init__(parent=parent)

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, JointsSpec):
            raise TypeError(f"parent of joint is not a JointsSpec: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        raise NotImplementedError(f"JointBase.uri property not implemented: {self}")


class FixedJoint(JointBase):
    name: str
    parent_frame: Frame
    child_frame: Frame

    def __init__(self, parent, name, parent_frame, child_frame) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self._uri = None

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.scoped()]
        return self._uri


class RevoluteJoint(JointBase):
    parent_frame_axis: FrameAxis
    child_frame_axis: FrameAxis
    actuation: Optional[Actuation]
    offset: Optional[JointOffset]
    limits: Optional[JointLimits]
    mimic: Optional[JointMimicSpec]
    polarity: str

    _uri: Optional[URIRef]
    _actuation_uri: Optional[URIRef]
    _mimic_uri: Optional[URIRef]
    _mimic_offset_uri: Optional[URIRef]
    _common_axis_uri: Optional[URIRef]
    _offset_uri: Optional[URIRef]
    _offset_coord_uri: Optional[URIRef]

    def __init__(
        self,
        parent,
        name,
        parent_frame_axis,
        child_frame_axis,
        actuation,
        offset,
        limits,
        mimic,
        polarity,
    ) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.parent_frame_axis = parent_frame_axis
        self.child_frame_axis = child_frame_axis
        self.actuation = actuation
        self.offset = offset
        self.limits = limits
        self.mimic = mimic
        self.polarity = polarity

        self._uri = None
        self._actuation_uri = None
        self._mimic_uri = None
        self._mimic_offset_uri = None
        self._common_axis_uri = None
        self._offset_uri = None
        self._offset_coord_uri = None

    @property
    def parent_frame(self) -> Frame:
        """The frame this joint hangs from: a revolute joint turns about its axis."""
        return self.parent_frame_axis.frame

    @property
    def child_frame(self) -> Frame:
        return self.child_frame_axis.frame

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.scoped()]
        return self._uri

    @property
    def actuation_uri(self) -> URIRef:
        if self._actuation_uri is None:
            self._actuation_uri = self.namespace[self.scoped("-actuation")]
        return self._actuation_uri

    @property
    def mimic_uri(self) -> URIRef:
        if self._mimic_uri is None:
            self._mimic_uri = self.namespace[self.scoped("-mimic")]
        return self._mimic_uri

    @property
    def mimic_offset_uri(self) -> URIRef:
        if self._mimic_offset_uri is None:
            self._mimic_offset_uri = self.namespace[self.scoped("-mimic-offset")]
        return self._mimic_offset_uri

    @property
    def common_axis_uri(self) -> URIRef:
        if self._common_axis_uri is None:
            self._common_axis_uri = self.namespace[self.scoped("-common-axis")]
        return self._common_axis_uri

    @property
    def offset_uri(self) -> URIRef:
        if self._offset_uri is None:
            self._offset_uri = self.namespace[self.scoped("-offset")]
        return self._offset_uri

    @property
    def offset_coord_uri(self) -> URIRef:
        if self._offset_coord_uri is None:
            self._offset_coord_uri = self.namespace[self.scoped("-offset-coord")]
        return self._offset_coord_uri


class JointOffset:
    xyz: FloatVector
    length_unit: str

    def __init__(self, parent, xyz, length_unit) -> None:
        self.parent = parent
        self.xyz = xyz
        self.length_unit = length_unit


class JointLimits:
    position: Optional[tuple[float, float]]
    position_unit: Optional[str]
    velocity: Optional[tuple[float, float]]
    velocity_unit: Optional[str]
    acceleration: Optional[tuple[float, float]]
    acceleration_unit: Optional[str]
    effort: Optional[tuple[float, float]]
    effort_unit: Optional[str]

    def __init__(
        self,
        parent,
        position,
        position_unit,
        velocity,
        velocity_unit,
        acceleration,
        acceleration_unit,
        effort,
        effort_unit,
    ) -> None:
        self.parent = parent
        self.position = position.as_low_high("JointLimits.position") if position else None
        self.position_unit = position_unit
        self.velocity = velocity.as_low_high("JointLimits.velocity") if velocity else None
        self.velocity_unit = velocity_unit
        self.acceleration = (
            acceleration.as_low_high("JointLimits.acceleration") if acceleration else None
        )
        self.acceleration_unit = acceleration_unit
        self.effort = effort.as_low_high("JointLimits.effort") if effort else None
        self.effort_unit = effort_unit


class JointMimicSpec:
    joint: JointBase

    def __init__(self, parent, joint, multiplier, offset) -> None:
        self.parent = parent
        self.joint = joint
        self.multiplier = multiplier
        self.offset = offset


class JointComposition:
    pass


class SerialJoints(JointComposition):
    root_frame: Frame
    tip_frame: Frame

    def __init__(self, parent, root_frame, tip_frame) -> None:
        self.parent = parent
        self.root_frame = root_frame
        self.tip_frame = tip_frame


class Actuation:
    gear_ratio: float
    cmd_interfaces: list[str]
    state_interfaces: list[str]

    def __init__(self, parent, gear_ratio, cmd_interfaces, state_interfaces) -> None:
        self.parent = parent
        self.gear_ratio = gear_ratio
        self.cmd_interfaces = cmd_interfaces
        self.state_interfaces = state_interfaces
