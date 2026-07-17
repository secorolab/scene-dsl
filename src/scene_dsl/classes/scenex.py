# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from typing import Any, Optional

from rdflib import Namespace, URIRef

from scene_dsl.classes.common import IHasNamespace, IHasNamespaceDeclare
from scene_dsl.classes.ktree import KinematicTreeModel, RigidBody
from scene_dsl.classes.scene import (
    Agent,
    Object,
    SceneModel,
    SimilarAgentSet,
    SimilarObjectSet,
)


class ElementModel(IHasNamespace):
    model_spec: Any
    model_kind: Optional[str]
    _uri: Optional[URIRef]

    def __init__(self, parent, name, model_kind, model_spec, tree=None) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.model_kind = model_kind
        self.model_spec = model_spec
        self.tree = tree
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
            raise TypeError(f"parent of modelled obj not a 'SceneInstance': {self.parent}")
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
            raise TypeError(f"parent of modelled obj set not a 'SceneInstance': {self.parent}")
        return self.parent.namespace

    def modelled_uri(self, index: int) -> URIRef:
        obj = self.obj_set.objects[index]
        return self.namespace[f"modelled-obj-{self.parent.name}-{obj.name}"]


class ModelledAgent(IHasNamespace):
    agn: Agent
    models: list[ElementModel]
    sensors: list
    _modelled_uri: Optional[URIRef]

    def __init__(
        self, parent, agn, models, ktree_inline=None, ktree_ref=None, sensors=None
    ) -> None:
        super().__init__(parent=parent)
        self.agn = agn
        # What scopes the IRIs of everything this agent carries.
        self.name = agn.name
        self.models = models
        self.ktree_inline = ktree_inline
        self.ktree_ref = ktree_ref
        self.sensors = sensors or []
        self._modelled_uri = None

    @property
    def ktree(self) -> Optional[KinematicTreeModel]:
        """The agent's kinematics, however they were written: defined here, or named."""
        return self.ktree_inline or self.ktree_ref

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, SceneInstance):
            raise TypeError(f"parent of modelled agn not a 'SceneInstance': {self.parent}")
        return self.parent.namespace

    @property
    def modelled_uri(self) -> URIRef:
        if self._modelled_uri is None:
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
        return self.namespace[f"modelled-agn-{self.parent.name}-{agn.name}"]


class SceneInstance(IHasNamespaceDeclare):
    scene: SceneModel
    ktree: Optional[KinematicTreeModel]
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
        ktree,
        models,
        modelled_objs,
        modelled_obj_sets,
        modelled_agns,
        modelled_agn_sets,
    ) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.scene = scene
        self.ktree = ktree
        self.models = models
        self.modelled_objs = modelled_objs
        self.modelled_obj_sets = modelled_obj_sets
        self.modelled_agns = modelled_agns
        self.modelled_agn_sets = modelled_agn_sets
