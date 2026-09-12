# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from typing import Any, ClassVar

from rdflib import Namespace, URIRef

from scene_dsl.classes.common import IHasNamespace, IHasNamespaceDeclare
from scene_dsl.classes.ktree import KinematicGraph, KinematicTreeModel, RigidBody
from scene_dsl.classes.scene import (
    Agent,
    AgentSet,
    Object,
    SceneModel,
    SimilarAgentSet,
    SimilarObjectSet,
)


class ElementModel(IHasNamespace):
    model_spec: Any
    model_kind: str | None
    color: Color | None
    mappings: list[ElementMapping]
    _uri: URIRef | None

    def __init__(self, parent, name, model_kind, model_spec, color=None, mappings=None) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.model_kind = model_kind
        self.model_spec = model_spec
        self.color = color
        self.mappings = mappings or []
        self._uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, IHasNamespace):
            raise TypeError(f"parent of ElementModel has no namespace: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.scoped()]
        return self._uri


class Color(IHasNamespace):
    """The color a model is drawn in, in the notation it was written in.

    rgb and rgba are sRGB channels in [0, 1]; hsv is hue in [0, 360) then saturation and
    value in [0, 1]. The bounds are per notation, so each states its own.
    """

    # Channels each notation takes, and the range every channel after the first must hold.
    NOTATIONS: ClassVar[dict[str, tuple[int, tuple[float, float]]]] = {
        "rgb": (3, (0.0, 1.0)),
        "rgba": (4, (0.0, 1.0)),
        "hsv": (3, (0.0, 1.0)),
    }
    HUE_HIGH = 360.0

    notation: str
    channels: tuple[float, ...]
    _uri: URIRef | None

    def __init__(self, parent, notation, channels) -> None:
        super().__init__(parent=parent)
        self.notation = notation
        count, (low, high) = self.NOTATIONS[notation]
        self.channels = channels.as_channels(count, f"Color.{notation}")
        bounded = self.channels[1:] if notation == "hsv" else self.channels
        if any(not low <= channel <= high for channel in bounded):
            raise ValueError(
                f"Color.{notation} channels must be within [{low:g}, {high:g}], got {self.channels}"
            )
        if notation == "hsv" and not 0.0 <= self.channels[0] < self.HUE_HIGH:
            raise ValueError(f"Color.hsv hue must be within [0, 360), got {self.channels[0]}")
        self._uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, ElementModel):
            raise TypeError(f"parent of Color is not an ElementModel: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.parent.scoped("-color")]
        return self._uri


class ElementMapping(IHasNamespace):
    """A scene element, and the name its model file knows it by."""

    entity: str | None
    _uri: URIRef | None

    def __init__(self, parent, entity=None) -> None:
        super().__init__(parent=parent)
        self.entity = entity or None
        self._uri = None

    @property
    def target(self) -> Any:
        raise NotImplementedError(f"'target' not implemented for '{type(self).__name__}'")

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, ElementModel):
            raise TypeError(f"parent of ElementMapping is not an ElementModel: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        """Nested under the model that declares it, then under the target's own path."""
        if self._uri is None:
            target = self.target
            local = str(target.uri).removeprefix(str(target.namespace))
            self._uri = self.namespace[f"{self.parent.scoped()}/maps/{local}"]
        return self._uri


class TreeMapping(ElementMapping):
    tree: KinematicTreeModel

    def __init__(self, parent, tree, entity=None) -> None:
        super().__init__(parent=parent, entity=entity)
        self.tree = tree

    @property
    def target(self) -> KinematicTreeModel:
        return self.tree


class BodyMapping(ElementMapping):
    body: RigidBody

    def __init__(self, parent, body, entity=None) -> None:
        super().__init__(parent=parent, entity=entity)
        self.body = body

    @property
    def target(self) -> RigidBody:
        return self.body


class ElementModelRef:
    """A logical object backed by one body in an already-loaded scene model."""

    model: ElementModel
    body: RigidBody
    entity: str | None
    _mapping_uri: URIRef | None

    def __init__(self, parent, model, body, entity=None) -> None:
        self.parent = parent
        self.model = model
        self.body = body
        self.entity = entity or None
        self._mapping_uri = None

    @property
    def mapping_uri(self) -> URIRef:
        if self._mapping_uri is None:
            local = str(self.body.uri).removeprefix(str(self.body.namespace))
            self._mapping_uri = self.model.namespace[f"{self.model.scoped()}/maps/{local}"]
        return self._mapping_uri


class ModelledObject(IHasNamespace):
    obj: Object
    models: list[ElementModel | ElementModelRef]
    _modelled_uri: URIRef | None

    def __init__(self, parent, obj, models) -> None:
        super().__init__(parent=parent)
        self.obj = obj
        self.name = obj.name
        self.models = models
        self._modelled_uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of ModelledObject is not a SceneInstance: {self.parent}")
        return self.parent.namespace

    @property
    def modelled_uri(self) -> URIRef:
        if self._modelled_uri is None:
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
            raise TypeError(f"parent of ModelledObjectSet is not a SceneInstance: {self.parent}")
        return self.parent.namespace

    def modelled_uri(self, index: int) -> URIRef:
        obj = self.obj_set.objects[index]
        return self.namespace[f"modelled-obj-{self.parent.name}-{obj.name}"]


class ModelledAgent(IHasNamespace):
    agn: Agent
    models: list[ElementModel]
    sensors: list
    _modelled_uri: URIRef | None

    def __init__(self, parent, agn, models, sensors=None) -> None:
        super().__init__(parent=parent)
        self.agn = agn
        self.name = agn.name
        self.models = models
        self.sensors = sensors or []
        self._modelled_uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, (SceneInstance, ModelledAgentSet)):
            raise TypeError(f"parent of ModelledAgent is not a scene or agent set: {self.parent}")
        return self.parent.namespace

    @property
    def scene_inst(self) -> SceneInstance:
        """The scene declaring it, whether directly or through an agent set."""
        return self.parent if isinstance(self.parent, SceneInstance) else self.parent.parent

    @property
    def modelled_uri(self) -> URIRef:
        if self._modelled_uri is None:
            self._modelled_uri = self.namespace[
                f"modelled-agn-{self.scene_inst.name}-{self.agn.name}"
            ]
        return self._modelled_uri


class ModelledAgentSet(IHasNamespace):
    """Concrete modelled agents grouped by their abstract agent set."""

    agn_set: AgentSet | SimilarAgentSet
    models: list[ModelledAgent]

    def __init__(self, parent, agn_set, models) -> None:
        super().__init__(parent=parent)
        self.agn_set = agn_set
        self.models = models

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of ModelledAgentSet is not a SceneInstance: {self.parent}")
        return self.parent.namespace


class SceneInstance(IHasNamespaceDeclare):
    scene: SceneModel
    kgraph: KinematicGraph | None
    models: list[ElementModel]
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
        kgraph,
        models,
        modelled_objs,
        modelled_obj_sets,
        modelled_agns,
        modelled_agn_sets,
    ) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.scene = scene
        self.kgraph = kgraph
        self.models = models
        self.modelled_objs = modelled_objs
        self.modelled_obj_sets = modelled_obj_sets
        self.modelled_agns = modelled_agns
        self.modelled_agn_sets = modelled_agn_sets
