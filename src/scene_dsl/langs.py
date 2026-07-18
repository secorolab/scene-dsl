# SPDX-License-Identifier: MPL-2.0
from importlib import resources

from rdflib import URIRef

import textx.scoping.providers as scoping_providers
from textx import get_children, get_children_of_type, get_location, get_model, metamodel_from_file
from textx.exceptions import TextXSemanticError
from textx.model import ObjCrossRef
from textx.scoping import Postponed

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
    KinematicGraph,
    KinematicStructure,
    FixedJoint,
    JointLimits,
    JointMimicSpec,
    JointOffset,
    KinematicTreeModel,
    KinematicTreeInstance,
    KinematicTreeTemplate,
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


class InstancedRefScopeProvider(scoping_providers.FQNImportURI):
    """Resolves references qualified by an instanced tree, e.g. '<arm1.base.origin>'.

    The tree's copy cannot exist yet, so resolve against the template -- see `_pending_refs`.
    """

    def __call__(self, obj, attr, obj_ref):
        head, _, path = obj_ref.obj_name.partition(".")
        model = get_model(obj)
        # Duplicate tree names collide on their IRI, and are reported there.
        trees = getattr(model, "ktrees", [])
        tree = next((t for t in trees if t.name == head and getattr(t, "template", None)), None)
        if tree is None or not path:
            return super().__call__(obj, attr, obj_ref)
        if not isinstance(tree.template, KinematicTreeTemplate):
            return Postponed()
        in_template = ObjCrossRef(
            f"{tree.template.name}.{path}",
            obj_ref.cls,
            obj_ref.position,
            obj_ref.scope_provider,
            obj_ref.match_rule_name,
        )
        target = super().__call__(obj, attr, in_template)
        if target is not None:
            _pending_refs(model).append((obj, attr.name, tree, target))
        return target


def _pending_refs(model) -> list:
    """References written into an instanced tree, to be landed once its copy exists.

    A template can only be copied after the model is resolved. Until then textx keeps a
    user class's attributes in a side-table keyed by object id, redirecting reads to it:
    so `template.bodies` reads fine, but `deepcopy` copies `__dict__` -- still empty --
    and yields bodies with no frames. Hence resolve against the template now, record the
    ref here, and rebind it onto the copy in `build_instance_trees`.
    """
    refs = getattr(model, "_instanced_refs", None)
    if refs is None:
        refs = model._instanced_refs = []
    return refs


def check_self_contained(template) -> None:
    """Reject a template referencing outside itself: the copy would drag the world in."""
    inside = {id(obj) for obj in get_children(lambda _: True, template)}
    for obj in get_children(lambda _: True, template):
        for attr_name, attr in type(obj)._tx_attrs.items():
            if attr.cont or not attr.ref:
                continue
            value = getattr(obj, attr_name, None)
            for target in value if isinstance(value, list) else [value]:
                if target is None or id(target) in inside:
                    continue
                raise TextXSemanticError(
                    f"kinematic tree template '{template.name}' must be self-contained, "
                    f"since instancing copies it -- but its "
                    f"{type(obj).__name__} '{getattr(obj, 'name', attr_name)}' references "
                    f"'{getattr(target, 'name', target)}', which is outside the tree",
                    **get_location(obj),
                )


def build_instance_trees(model, metamodel):
    """Fill each instanced tree from its template, then land the refs written into it.

    An imported model can reach this processor before its template reference resolves;
    leave that instance for the caller's later resolution pass.
    """
    for tree in get_children_of_type(KinematicTreeInstance, model):
        if isinstance(tree.parent, ModelledAgent):
            raise TextXSemanticError(
                "a template instance must be declared at model level and named by the agent",
                **get_location(tree),
            )
        if tree.template is None or tree.bodies:
            continue
        check_self_contained(tree.template)
        tree.copy_template()

    for obj, attr_name, tree, target in _pending_refs(model):
        setattr(obj, attr_name, tree.copies[id(target)])


def check_tree_composition(model, metamodel):
    """A tree composing itself, however far around, describes a device without end."""
    for tree in get_children_of_type(KinematicTreeModel, model):
        cycle = tree.composition_cycle()
        if cycle:
            raise TextXSemanticError(
                f"kinematic tree '{tree.name}' composes itself: "
                f"{' -> '.join(t.name for t in cycle)}",
                **get_location(tree),
            )


def _parent_joints(tree) -> dict[int, object]:
    """The joint attaching each body to its parent. A body may have only one."""
    parent_joint: dict[int, object] = {}
    for joint in tree.all_joints:
        child = joint.child_frame.parent
        first = parent_joint.get(id(child))
        if first is not None:
            raise TextXSemanticError(
                f"body '{child.name}' is attached by both joint '{first.name}' and joint "
                f"'{joint.name}': a tree closes no loops, so a body has one parent",
                **get_location(joint),
            )
        parent_joint[id(child)] = joint
    return parent_joint


def _up(body, parent_joint) -> list:
    """The bodies from `body` up towards its root, stopping if it comes back around."""
    seen: set[int] = set()
    chain = []
    while body is not None and id(body) not in seen:
        seen.add(id(body))
        chain.append(body)
        joint = parent_joint.get(id(body))
        body = joint.parent_frame.parent if joint is not None else None
    return chain if body is None else chain + [body]


def _reachable(root, tip, joints) -> bool:
    """Whether directed joints connect `root` to `tip`, even through a graph cycle."""
    children: dict[int, list] = {}
    for joint in joints:
        children.setdefault(id(joint.parent_frame.parent), []).append(joint.child_frame.parent)

    seen: set[int] = set()
    pending = [root]
    while pending:
        body = pending.pop()
        if body is tip:
            return True
        if id(body) in seen:
            continue
        seen.add(id(body))
        pending.extend(children.get(id(body), []))
    return False


def check_tree_topology(model, metamodel):
    """A tree has one root and closes no loops; any chain runs root to tip.

    A graph promises neither, so only the chain is its business. An imported model's
    processors run before its own references resolve; its joints are checked anyway,
    through the copy or the composing tree, in the model that imports it.
    """
    for tree in get_children(lambda x: isinstance(x, KinematicStructure), model):
        if isinstance(tree, KinematicTreeInstance) and tree.template is None:
            continue
        if any(j.parent_frame is None or j.child_frame is None for j in tree.all_joints):
            continue

        if isinstance(tree, KinematicTreeModel):
            parent_joint = _parent_joints(tree)
            roots = tree.roots
            if len(roots) > 1:
                raise TextXSemanticError(
                    f"kinematic tree '{tree.name}' has {len(roots)} bodies attached to "
                    f"nothing ({', '.join(sorted(b.name for b in roots))}), but a tree has "
                    f"one root -- join them, or hold them in a kgraph",
                    **get_location(tree),
                )

            for joint in tree.all_joints:
                chain = _up(joint.child_frame.parent, parent_joint)
                # Coming back to a body already passed means the joints close a loop.
                if len(chain) > len({id(b) for b in chain}):
                    raise TextXSemanticError(
                        f"joints of kinematic tree '{tree.name}' close a loop: "
                        f"{' -> '.join(b.name for b in reversed(chain))}",
                        **get_location(joint),
                    )

        comp = tree.joints_spec.joint_comp if tree.joints_spec is not None else None
        if not isinstance(comp, SerialJoints):
            continue
        root, tip = comp.root_frame.parent, comp.tip_frame.parent
        if not _reachable(root, tip, tree.all_joints):
            raise TextXSemanticError(
                f"serial chain of '{tree.name}': tip '{comp.tip_frame.name}' is not below "
                f"root '{comp.root_frame.name}' -- no joints lead from one to the other",
                **get_location(comp),
            )


def check_agent_models(model, metamodel):
    """A model's 'for' must name kinematics the agent has, and only one model may.

    Several devices means a file each, so each must say which; one file needs no 'for'.
    """
    for elem_model in get_children_of_type(ElementModel, model):
        if elem_model.tree is None:
            continue
        if not isinstance(elem_model.parent, ModelledAgent):
            raise TextXSemanticError(
                f"model '{elem_model.name}' says 'for <{elem_model.tree.name}>', but only "
                f"an agent has kinematics for a model to describe",
                **get_location(elem_model),
            )

    for agent in get_children_of_type(ModelledAgent, model):
        # An agent with no kinematics has nothing for a model to describe, so an empty
        # set is the right answer here, not a reason to skip the check.
        available = agent.ktree.subtrees if agent.ktree is not None else {}
        described: dict[int, ElementModel] = {}
        for elem_model in agent.models:
            if elem_model.tree is None:
                if len(agent.models) > 1:
                    raise TextXSemanticError(
                        f"agent '{agent.agn.name}' has {len(agent.models)} models, so each "
                        f"must say which kinematics it describes -- model "
                        f"'{elem_model.name}' names none",
                        **get_location(elem_model),
                    )
                continue
            if id(elem_model.tree) not in available:
                has = f"kinematics '{agent.ktree.name}'" if agent.ktree else "no kinematics"
                raise TextXSemanticError(
                    f"model '{elem_model.name}' describes tree "
                    f"'{elem_model.tree.name}', but agent '{agent.agn.name}' has {has}",
                    **get_location(elem_model),
                )
            first = described.get(id(elem_model.tree))
            if first is not None:
                raise TextXSemanticError(
                    f"models '{first.name}' and '{elem_model.name}' both describe tree "
                    f"'{elem_model.tree.name}': which one drives it is then undecided",
                    **get_location(elem_model),
                )
            described[id(elem_model.tree)] = elem_model


def check_unique_uris(model, metamodel):
    # Elements minting the same IRI silently become one node with merged types.
    seen: dict[URIRef, object] = {}

    def record(obj) -> None:
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

    for obj in get_children(
        lambda x: getattr(x, "uri", None) is not None and getattr(x, "name", None) is not None,
        model,
    ):
        record(obj)


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
            KinematicTreeInstance,
            KinematicTreeTemplate,
            KinematicGraph,
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
            # Falls back to plain FQN when the ref is not into an instanced tree.
            "*.*": InstancedRefScopeProvider(),
            # Trees are named flatly, but nest under scene instances and imports.
            "KinematicTreeModel.trees": scoping_providers.PlainNameImportURI(),
        }
    )
    mm_scenex.register_model_processor(check_tree_composition)
    mm_scenex.register_model_processor(build_instance_trees)
    mm_scenex.register_model_processor(check_tree_topology)
    mm_scenex.register_model_processor(check_agent_models)
    mm_scenex.register_model_processor(check_unique_uris)
    return mm_scenex
