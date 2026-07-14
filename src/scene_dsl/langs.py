# SPDX-License-Identifier: MPL-2.0
from importlib import resources

import textx.scoping.providers as scoping_providers
from textx import get_children_of_type, get_model, metamodel_from_file

from scene_dsl.classes.common import FloatVector, IntVector
from scene_dsl.classes.geom import (
    DirectionCosineOrientationSpec,
    EulerOrientationSpec,
    Frame,
    FrameAxis,
    OrientationSpec,
    PoseSpec,
)
from scene_dsl.classes.ktree import (
    Actuation,
    FixedJoint,
    JointLimits,
    JointMimicSpec,
    JointOffset,
    KinematicTreeModel,
    JointsSpec,
    JointBase,
    JointComposition,
    RevoluteJoint,
    RigidBody,
    RigidBodyInertia,
    SerialJoints,
)
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
    ElementModel,
    ModelledAgent,
    ModelledAgentSet,
    ModelledObject,
    ModelledObjectSet,
    SceneInstance,
)
from scene_dsl.classes.sensors import (
    CameraSensorSpec,
    ForceTorqueSensorSpec,
    ImuSensorSpec,
)


class KinematicTreeRefScopeProvider:
    def __call__(self, obj, attr, obj_ref):
        matches = [
            tree
            for tree in get_children_of_type(KinematicTreeModel, get_model(obj))
            if tree.name == obj_ref.obj_name
        ]
        num_matches = len(matches)
        if num_matches > 1:
            raise ValueError(f"multiple trees found for ref name '{obj_ref.obj_name}': {matches}")

        if num_matches == 1:
            return matches[0]

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
            FloatVector,
            IntVector,
            ElementModel,
            ModelledObject,
            ModelledObjectSet,
            ModelledAgent,
            ModelledAgentSet,
            SceneInstance,
            PoseSpec,
            OrientationSpec,
            EulerOrientationSpec,
            DirectionCosineOrientationSpec,
            Frame,
            FrameAxis,
            KinematicTreeModel,
            RigidBody,
            RigidBodyInertia,
            JointsSpec,
            JointBase,
            FixedJoint,
            RevoluteJoint,
            JointOffset,
            JointLimits,
            JointMimicSpec,
            JointComposition,
            SerialJoints,
            Actuation,
            CameraSensorSpec,
            ForceTorqueSensorSpec,
            ImuSensorSpec,
        ],
    )
    mm_scenex.register_scope_providers(
        {
            "KinematicTreeModel.trees": KinematicTreeRefScopeProvider(),
            "*.*": scoping_providers.FQNImportURI(),
        }
    )
    return mm_scenex
