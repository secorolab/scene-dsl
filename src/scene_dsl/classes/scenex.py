# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from typing import Any, Optional

from rdflib import Namespace, URIRef

from scene_dsl.classes.common import IHasNamespace, IHasNamespaceDeclare
from scene_dsl.classes.scene import (
    Agent,
    SimilarAgentSet,
    SimilarObjectSet,
    Object,
    SceneModel,
)


class ElementModel(IHasNamespace):
    model_spec: Any
    model_kind: Optional[str]
    _uri: Optional[URIRef]

    def __init__(self, parent, name, model_kind, model_spec) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.model_kind = model_kind
        self.model_spec = model_spec
        self._uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, IHasNamespace):
            raise TypeError(f"parent of ElementModel not an 'IHasNamespace': {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri


class OrientationSpec:
    pass


class EulerOrientationSpec(OrientationSpec):
    extrinsic: bool

    def __init__(self, parent, axes, extrinsic, alpha, beta, gamma, unit) -> None:
        self.parent = parent
        self.extrinsic = extrinsic
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.axes = axes or "zyx"
        self.unit = unit or "rad"


class PoseSpec:
    wrt: Optional["Frame"]
    orientation: OrientationSpec

    def __init__(self, parent, wrt, x, y, z, length_unit, orientation) -> None:
        self.parent = parent
        self.wrt = wrt
        self.x = x
        self.y = y
        self.z = z
        self.length_unit = length_unit
        self.orientation = orientation


class ModelledObject(IHasNamespace):
    obj: Object
    models: list[ElementModel]
    geometry: "GeometrySpec"
    body: Optional["BodySpec"]
    _modelled_uri: Optional[URIRef]

    def __init__(self, parent, obj, models, geometry, body=None) -> None:
        super().__init__(parent=parent)
        self.obj = obj
        self.models = models
        self.geometry = geometry
        self.body = body
        self._modelled_uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of modelled obj not a 'SceneInstance': {self.parent}")
        return self.parent.namespace

    @property
    def modelled_uri(self) -> URIRef:
        if self._modelled_uri is None:
            if not isinstance(self.parent, SceneInstance):
                raise TypeError(f"parent of modelled obj not a 'SceneInstance': {self.parent}")
            self._modelled_uri = self.namespace[f"modelled-obj-{self.parent.name}-{self.obj.name}"]
        return self._modelled_uri


class ModelledObjectSet(IHasNamespace):
    obj_set: SimilarObjectSet
    models: list[ElementModel]

    def __init__(self, parent, obj_set, models) -> None:
        super().__init__(parent=parent)
        self.obj_set = obj_set
        self.models = models

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of modelled obj set not a 'SceneInstance': {self.parent}")
        return self.parent.namespace

    def modelled_uri(self, index: int) -> URIRef:
        obj = self.obj_set.objects[index]
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of modelled obj set not a 'SceneInstance': {self.parent}")
        return self.namespace[f"modelled-obj-{self.parent.name}-{obj.name}"]


class Frame(IHasNamespace):
    _uri: Optional[URIRef]
    _origin_uri: Optional[URIRef]

    def __init__(self, parent=None, name=None) -> None:
        super().__init__(parent=parent)
        self.name = name
        self._uri = None
        self._origin_uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, IHasNamespace):
            raise TypeError(f"parent of frame has no namespace: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            if not isinstance(self.parent, GeometrySpec):
                raise TypeError(f"parent of frame not a GeometrySpec: {self.parent}")
            self._uri = self.namespace[f"{self.parent.name}-{self.name}"]
        return self._uri

    @property
    def origin_uri(self) -> URIRef:
        if self._origin_uri is None:
            self._origin_uri = URIRef(f"{self.uri}-origin")
        return self._origin_uri


class GeometrySpec(IHasNamespace):
    name: str
    root: Frame
    frames: list[Frame]
    pose: Optional[PoseSpec]
    _uri: Optional[URIRef]

    def __init__(self, parent, name, root, frames=None, pose=None) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.root = root
        self.frames = frames or []
        self.pose = pose
        self._uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, IHasNamespace):
            raise TypeError(f"parent of geometry spec has no namespace: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri

    def pose_uri(self, wrt: Frame) -> URIRef:
        return self.namespace[f"{self.root.name}-wrt-{wrt.name}"]

    def pose_coord_uri(self, wrt: Frame) -> URIRef:
        return self.namespace[f"{self.root.name}-wrt-{wrt.name}-coord"]


class MassQuantity:
    value: float
    unit: str

    def __init__(self, parent, value, unit) -> None:
        self.parent = parent
        if value < 0:
            raise ValueError(f"MassQuantity must have value >= 0, got {value}")
        self.value = value
        self.unit = unit


class BodySpec(IHasNamespace):
    frame: Frame
    mass: Optional[MassQuantity]
    _uri: Optional[URIRef]
    _inertia_uri: Optional[URIRef]
    _inertia_coord_uri: Optional[URIRef]

    def __init__(self, parent, name, frame, mass=None) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.frame = frame
        self.mass = mass
        self._uri = None
        self._inertia_uri = None
        self._inertia_coord_uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, (ModelledObject, FixedAttachment)):
            raise TypeError(
                f"parent of body spec not a modelled object or attachment: {self.parent}"
            )
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri

    @property
    def inertia_uri(self) -> URIRef:
        if self._inertia_uri is None:
            self._inertia_uri = self.namespace[f"{self.name}-inertia"]
        return self._inertia_uri

    @property
    def inertia_coord_uri(self) -> URIRef:
        if self._inertia_coord_uri is None:
            self._inertia_coord_uri = self.namespace[f"{self.name}-inertia-coord"]
        return self._inertia_coord_uri


class KinematicSpec(IHasNamespace):
    model: ElementModel
    geometry: GeometrySpec

    def __init__(self, parent, model, geometry) -> None:
        super().__init__(parent=parent)
        self.model = model
        self.geometry = geometry

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, ModelledAgent):
            raise TypeError(f"parent of kinematic spec not a 'ModelledAgent': {self.parent}")
        return self.parent.namespace


class FixedAttachment(IHasNamespace):
    model: ElementModel
    geometry: GeometrySpec
    body: Optional[BodySpec]

    def __init__(self, parent, name, model, geometry, body=None) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.model = model
        self.geometry = geometry
        self.body = body

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, ModelledAgent):
            raise TypeError(f"parent of fixed attachment not a 'ModelledAgent': {self.parent}")
        return self.parent.namespace

    @property
    def agent_model(self):
        return self.parent


class ModelledAgent(IHasNamespace):
    agn: Agent
    kinematic: KinematicSpec
    attachments: list[FixedAttachment]
    _modelled_uri: Optional[URIRef]

    def __init__(self, parent, agn, kinematic, attachments=None) -> None:
        super().__init__(parent=parent)
        self.agn = agn
        self.kinematic = kinematic
        self.attachments = attachments or []
        self._modelled_uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of modelled agn not a 'SceneInstance': {self.parent}")
        return self.parent.namespace

    @property
    def modelled_uri(self) -> URIRef:
        if self._modelled_uri is None:
            if not isinstance(self.parent, SceneInstance):
                raise TypeError(f"parent of modelled agn not a 'SceneInstance': {self.parent}")
            self._modelled_uri = self.namespace[f"modelled-agn-{self.parent.name}-{self.agn.name}"]
        return self._modelled_uri


class ModelledAgentSet(IHasNamespace):
    agn_set: SimilarAgentSet
    models: list[ElementModel]

    def __init__(self, parent, agn_set, models) -> None:
        super().__init__(parent=parent)
        self.agn_set = agn_set
        self.models = models

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of modelled agn set not a 'SceneInstance': {self.parent}")
        return self.parent.namespace

    def modelled_uri(self, index: int) -> URIRef:
        agn = self.agn_set.agents[index]
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of modelled agn set not a 'SceneInstance': {self.parent}")
        return self.namespace[f"modelled-agn-{self.parent.name}-{agn.name}"]


class SceneInstance(IHasNamespaceDeclare):
    scene: SceneModel
    geometry: Optional[GeometrySpec]
    modelled_objs: list[ModelledObject]
    modelled_obj_sets: list[ModelledObjectSet]
    modelled_agns: list[ModelledAgent]
    modelled_agn_sets: list[ModelledAgentSet]

    def __init__(
        self,
        parent,
        ns,
        name,
        scene,
        geometry,
        modelled_objs,
        modelled_obj_sets,
        modelled_agns,
        modelled_agn_sets,
    ) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.scene = scene
        self.geometry = geometry
        self.modelled_objs = modelled_objs
        self.modelled_obj_sets = modelled_obj_sets
        self.modelled_agns = modelled_agns
        self.modelled_agn_sets = modelled_agn_sets
