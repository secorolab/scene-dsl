# SPDX-License-Identifier: MPL-2.0
"""Render a scene's kinematics as graphviz: bodies as nodes, joints as edges."""

from textx import get_children_of_type

from scene_dsl.classes.ktree import KinematicTreeModel, RevoluteJoint, SerialJoints

_FILL = {True: "#e8f0fe", False: "#ffffff"}  # keyed by 'is on a serial chain'


def _node(body) -> str:
    return body.scoped()


def _parent_joints(tree) -> dict[int, object]:
    """The joint attaching each body to its parent. Loops are rejected before this runs."""
    return {id(joint.child_frame.parent): joint for joint in tree.all_joints}


def _chain(tree) -> list:
    """The bodies of this tree's serial chain, tip first, or [] if it declares none."""
    comp = tree.joints_spec.joint_comp if tree.joints_spec is not None else None
    if not isinstance(comp, SerialJoints):
        return []
    parent_joint, chain, body = _parent_joints(tree), [], comp.tip_frame.parent
    root = comp.root_frame.parent
    while body is not None:
        chain.append(body)
        if body is root:
            return chain
        joint = parent_joint.get(id(body))
        body = joint.parent_frame.parent if joint is not None else None
    return []


def _clusters(tree, on_chain: set, lines: list, depth: int, prefix: str = "") -> None:
    # A tree is itself a namespace root, so it has no scoped() path: build one on the way
    # down, which also keeps the cluster ids stable between runs.
    path = f"{prefix}{tree.name}"
    pad = "  " * depth
    lines.append(f'{pad}subgraph "cluster_{path}" {{')
    lines.append(f'{pad}  label="{tree.name}";')
    lines.append(f'{pad}  style=rounded; color="#9aa0a6"; fontsize=11;')
    for body in tree.bodies:
        fill = _FILL[_node(body) in on_chain]
        lines.append(f'{pad}  "{_node(body)}" [label="{body.name}", fillcolor="{fill}"];')
    for sub in tree.trees:
        _clusters(sub, on_chain, lines, depth + 1, f"{path}/")
    lines.append(f"{pad}}}")


def create_dot(model) -> str:
    """One digraph per tree that nothing else composes, with a cluster per device."""
    trees = get_children_of_type(KinematicTreeModel, model)
    composed = {id(sub) for tree in trees for sub in tree.trees}
    roots = [tree for tree in trees if id(tree) not in composed]

    lines = ["digraph kinematics {", "  rankdir=LR;", "  compound=true;"]
    lines.append('  node [shape=box, style="rounded,filled", fontsize=10, fillcolor="#ffffff"];')
    lines.append('  edge [fontsize=9, color="#5f6368"];')
    for root in roots:
        on_chain = {_node(b) for tree in root.subtrees.values() for b in _chain(tree)}
        _clusters(root, on_chain, lines, 1)
        for joint in root.all_joints:
            parent, child = joint.parent_frame.parent, joint.child_frame.parent
            # What a joint is and whether it is on a chain are separate things, so they
            # are drawn separately: dashed means fixed, blue means on the declared chain.
            style = "" if isinstance(joint, RevoluteJoint) else "style=dashed, arrowhead=diamond, "
            if _node(parent) in on_chain and _node(child) in on_chain:
                style += 'color="#1a73e8", penwidth=2'
            else:
                style += 'color="#5f6368"'
            lines.append(
                f'  "{_node(parent)}" -> "{_node(child)}" [label="{joint.name}", {style}];'
            )
    lines.append("}")
    return "\n".join(lines) + "\n"
