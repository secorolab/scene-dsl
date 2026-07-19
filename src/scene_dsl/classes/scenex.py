# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from typing import Any, Optional

from rdflib import Namespace, URIRef

from scene_dsl.classes.common import IHasNamespace, IHasNamespaceDeclare
from scene_dsl.classes.ktree import KinematicGraph, RigidBody
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
            raise TypeError(f"parent of ElementModel has no namespace: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri


class AgentModel(IHasNamespace):
    """An agent model and the kinematics represented by its file."""

    def __init__(self, parent, model, tree, entity=None) -> None:
        super().__init__(parent=parent)
        self.model = model
        self.tree = tree
        self.entity = entity or None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, ModelledAgent):
            raise TypeError(f"parent of AgentModel is not a ModelledAgent: {self.parent}")
        return self.parent.namespace


class ModelledObject(IHasNamespace):
    obj: Object
    models: list[ElementModel]
    body: Optional[RigidBody]
    _modelled_uri: Optional[URIRef]

    def __init__(self, parent, obj, models, body) -> None:
        super().__init__(parent=parent)
        self.obj = obj
        self.models = models
        self.body = body
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
    models: list[AgentModel]
    sensors: list
    _modelled_uri: Optional[URIRef]

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
    def modelled_uri(self) -> URIRef:
        if self._modelled_uri is None:
            scene = self.parent if isinstance(self.parent, SceneInstance) else self.parent.parent
            self._modelled_uri = self.namespace[f"modelled-agn-{scene.name}-{self.agn.name}"]
        return self._modelled_uri


class ModelledAgentSet(IHasNamespace):
    """Concrete modelled agents grouped by their abstract agent set."""

    agn_set: AgentSet | SimilarAgentSet
    agents: list[ModelledAgent]

    def __init__(self, parent, agn_set, agents) -> None:
        super().__init__(parent=parent)
        self.agn_set = agn_set
        self.agents = agents

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of ModelledAgentSet is not a SceneInstance: {self.parent}")
        return self.parent.namespace


class SceneInstance(IHasNamespaceDeclare):
    scene: SceneModel
    kgraph: Optional[KinematicGraph]
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
