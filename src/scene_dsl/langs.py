# SPDX-License-Identifier: MPL-2.0
from importlib import resources
from textx import get_children_of_type, get_model, metamodel_from_file
import textx.scoping.providers as scoping_providers

from scene_dsl.classes.scene import (
    Agent,
    AgentSet,
    Object,
    ObjectSet,
    SceneModel,
    SimilarAgentSet,
    SimilarObjectSet,
    Workspace,
    WorkspaceComposition,
    WorkspaceSet,
)
from scene_dsl.classes.scenex import (
    BodySpec,
    ElementModel,
    EulerOrientationSpec,
    FixedAttachment,
    Frame,
    GeometrySpec,
    KinematicSpec,
    MassQuantity,
    ModelledAgent,
    ModelledAgentSet,
    ModelledObject,
    ModelledObjectSet,
    OrientationSpec,
    PoseSpec,
    SceneInstance,
)


def _geometry_frame(geometry, frame_name):
    matches = [frame for frame in [geometry.root, *geometry.frames] if frame.name == frame_name]
    return matches[0] if len(matches) == 1 else None


class FrameRefScopeProvider:
    def __call__(self, obj, attr, obj_ref):
        parts = obj_ref.obj_name.split(".")
        if len(parts) != 2:
            return None
        geometry_name, frame_name = parts
        model = get_model(obj)
        for geometry in get_children_of_type(GeometrySpec, model):
            if geometry.name == geometry_name:
                return _geometry_frame(geometry, frame_name)
        return None


def scene_metamodel():
    mm_scene = metamodel_from_file(
        resources.files("scene_dsl") / "grammars" / "scene.tx",
        classes=[
            Object,
            Workspace,
            Agent,
            ObjectSet,
            SimilarObjectSet,
            WorkspaceSet,
            AgentSet,
            SimilarAgentSet,
            WorkspaceComposition,
            SceneModel,
        ],
    )
    mm_scene.register_scope_providers({"*.*": scoping_providers.FQNImportURI()})
    return mm_scene


def scenex_metamodel():
    mm_scenex = metamodel_from_file(
        resources.files("scene_dsl") / "grammars" / "scenex.tx",
        classes=[
            ElementModel,
            ModelledObject,
            ModelledObjectSet,
            ModelledAgent,
            ModelledAgentSet,
            SceneInstance,
            KinematicSpec,
            GeometrySpec,
            BodySpec,
            MassQuantity,
            PoseSpec,
            OrientationSpec,
            EulerOrientationSpec,
            Frame,
            FixedAttachment,
        ],
    )
    mm_scenex.register_scope_providers(
        {
            "*.frame": FrameRefScopeProvider(),
            "PoseSpec.wrt": FrameRefScopeProvider(),
            "BodySpec.frame": FrameRefScopeProvider(),
            "*.*": scoping_providers.FQNImportURI(),
        }
    )
    return mm_scenex
