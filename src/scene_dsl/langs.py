# SPDX-License-Identifier: MPL-2.0
from importlib import resources

from rdflib import URIRef

import textx.scoping.providers as scoping_providers
from textx import get_children, get_children_of_type, get_location, get_model, metamodel_from_file
from textx.exceptions import TextXSemanticError

from scene_dsl.classes.common import FloatVector, IntVector
from scene_dsl.classes.distrib import (
    Distribution,
    DistributionRef,
    FloatMatrix,
    NormalDistribution,
    UniformDistribution,
    UniformRotationDistribution,
)
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


def check_unique_uris(model, metamodel):
    # Elements minting the same IRI silently become one node with merged types.
    seen: dict[URIRef, object] = {}

    for obj in get_children(
        lambda x: getattr(x, "uri", None) is not None
        and getattr(x, "name", None) is not None,
        model,
    ):
        uri = obj.uri
        first = seen.get(uri)

        if first is not None:
            loc = get_location(first)
            raise TextXSemanticError(
                f"IRI '{uri}' is minted by both "
                f"{first.__class__.__name__} '{first.name}' "
                f"({loc['filename']}:{loc['line']}) and "
                f"{obj.__class__.__name__} '{obj.name}' "
                "-- names must be unique per IRI",
                **get_location(obj),
            )

        seen[uri] = obj


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
            Distribution,
            DistributionRef,
            FloatMatrix,
            NormalDistribution,
            UniformDistribution,
            UniformRotationDistribution,
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
    mm_scenex.register_model_processor(check_unique_uris)
    return mm_scenex
