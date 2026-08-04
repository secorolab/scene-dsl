# SPDX-License-Identifier: MPL-2.0
"""Render a scene's kinematics as graphviz: bodies as nodes, joints as edges.

Drawn from the graph the scene emits, so what is pictured is what a solver would read:
each body under the tree that defines it, every joint directed the way the walk out of a
root settled it, and the bodies a graph places without articulating.
"""

from rdf_utils.models.vocab import (
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_ENV_PRED_OF_OBJ,
    URI_ENV_TYPE_MOD_OBJ,
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_MAPS,
)
from rdflib import Graph, URIRef
from rdflib.namespace import split_uri

from scene_dsl.rdf_parser.common import ensure_one_obj_uri
from scene_dsl.rdf_parser.kgraph import (
    KinematicGraphModel,
    kinematic_graphs,
)
from scene_dsl.rdf_parser.ktree import KinematicTreeModel, typed, uris

_FILL = {True: "#e8f0fe", False: "#ffffff"}  # keyed by 'is on a serial chain'


def _node(body: URIRef, owner: URIRef) -> str:
    """What a body is called in the picture: its name under the one that defines it.

    Two copies of one device share every local name, so the owner has to qualify it.
    """
    return f"{split_uri(owner)[1]}/{split_uri(body)[1]}"


def _frame_node(frame: URIRef, body: URIRef, owner: URIRef) -> str:
    """What a frame is called: its name under the body carrying it."""
    return f"{_node(body, owner)}/{split_uri(frame)[1]}"


def _stands_for(graph: Graph) -> dict[URIRef, str]:
    """The scene object each body stands for, where a model maps one to it.

    Body <- mapping <- model <- the modelled object naming it, which is the only way round:
    a mapping states what it maps, and the object states what models it.
    """
    labels = {}
    for modelled in typed(graph.subjects(URI_ENV_PRED_OF_OBJ, None), URI_ENV_TYPE_MOD_OBJ, graph):
        obj = ensure_one_obj_uri(graph=graph, subject=modelled, predicate=URI_ENV_PRED_OF_OBJ)
        for model in uris(graph.objects(modelled, URI_ENV_PRED_HAS_OBJ_MODEL)):
            for mapping in uris(graph.objects(model, URI_EXEC_PRED_HAS_MAPPING)):
                for body in uris(graph.objects(mapping, URI_EXEC_PRED_MAPS)):
                    if obj is not None:
                        labels[body] = split_uri(obj)[1]
    return labels


def _label(body: URIRef, stands_for: dict[URIRef, str]) -> str:
    obj = stands_for.get(body)
    return f"{split_uri(body)[1]}\\n({obj})" if obj is not None else split_uri(body)[1]


def _chain_bodies(tree: KinematicTreeModel) -> set[URIRef]:
    """Every body a declared chain runs through."""
    bodies = set()
    for chain in tree.chains:
        body = chain.root_body
        bodies.add(body)
        for joint in chain.path:
            body = tree.joints[joint].other_body(body)
            bodies.add(body)
    return bodies


def _tip_frames(tree: KinematicTreeModel) -> dict[URIRef, URIRef]:
    """Each chain tip that is a place on a body rather than the body itself.

    A chain may end anywhere on its last body, and where that is not the body's own root
    frame the endpoint is nothing the walk drew -- so it is hung off that body, which is
    also what a solver does with it.
    """
    return {
        chain.tip_frame: chain.tip_body
        for chain in tree.chains
        if chain.tip_frame != tree.bodies[chain.tip_body].root_frame.id
    }


def _owners(kgraph: KinematicGraphModel) -> dict[URIRef, URIRef]:
    """The tree defining each body, and the graph itself for the bodies it hangs."""
    owners = {body: kgraph.id for body in kgraph.free_bodies}
    for tree in kgraph.trees:
        owners.update(tree.defining_tree_by_body)
    return owners


def _cluster(
    node: URIRef,
    owners: dict[URIRef, URIRef],
    on_chain: set[URIRef],
    stands_for: dict[URIRef, str],
    tips: dict[URIRef, URIRef],
    nested: dict[URIRef, tuple[URIRef, ...]],
    lines: list,
    depth: int,
    prefix: str,
) -> None:
    """One cluster per graph or tree, nested the way each is composed of the next."""
    path = f"{prefix}{split_uri(node)[1]}"
    pad = "  " * depth
    lines.append(f'{pad}subgraph "cluster_{path}" {{')
    lines.append(f'{pad}  label="{split_uri(node)[1]}";')
    lines.append(f'{pad}  style=rounded; color="#9aa0a6"; fontsize=11;')
    for body, owner in owners.items():
        if owner == node:
            fill = _FILL[body in on_chain]
            lines.append(
                f'{pad}  "{_node(body, owner)}" '
                f'[label="{_label(body, stands_for)}", fillcolor="{fill}"];'
            )
            for frame, tip_body in tips.items():
                if tip_body == body:
                    # A frame is no body, so it is drawn as one that carries no mass.
                    lines.append(
                        f'{pad}  "{_frame_node(frame, body, owner)}" [label="{split_uri(frame)[1]}", '
                        f'style="rounded,dashed,filled", fillcolor="{_FILL[True]}"];'
                    )
    for sub in nested.get(node, ()):
        _cluster(sub, owners, on_chain, stands_for, tips, nested, lines, depth + 1, f"{path}/")
    lines.append(f"{pad}}}")


def _edges(
    tree: KinematicTreeModel,
    owners: dict[URIRef, URIRef],
    on_chain: set[URIRef],
    tips: dict[URIRef, URIRef],
    lines: list,
) -> None:
    for child, parent in tree.parent.items():
        joint = tree.joints[tree.parent_joint[child]]
        # What a joint is and whether it is on a chain are separate things, so they are
        # drawn separately: dashed means fixed, blue means on the declared chain.
        style = "" if joint.moves else "style=dashed, arrowhead=diamond, "
        if parent in on_chain and child in on_chain:
            style += 'color="#1a73e8", penwidth=2'
        else:
            style += 'color="#5f6368"'
        lines.append(
            f'  "{_node(parent, owners[parent])}" -> "{_node(child, owners[child])}" '
            f'[label="{split_uri(joint.id)[1]}", {style}];'
        )

    # Nothing moves to a frame, so the chain reaches its tip without a joint.
    for frame, body in tips.items():
        owner = owners[body]
        lines.append(
            f'  "{_node(body, owner)}" -> "{_frame_node(frame, body, owner)}" '
            f'[style=dotted, arrowhead=none, color="#1a73e8"];'
        )


def create_dot(graph: Graph) -> str:
    """One digraph per kinematic graph, with a cluster per device composed into it."""
    lines = ["digraph kinematics {", "  rankdir=LR;", "  compound=true;"]
    lines.append('  node [shape=box, style="rounded,filled", fontsize=10, fillcolor="#ffffff"];')
    lines.append('  edge [fontsize=9, color="#5f6368"];')

    stands_for = _stands_for(graph)
    for kgraph in kinematic_graphs(graph):
        owners = _owners(kgraph)
        on_chain = {body for tree in kgraph.trees for body in _chain_bodies(tree)}
        tips = {frame: body for tree in kgraph.trees for frame, body in _tip_frames(tree).items()}
        _cluster(kgraph.id, owners, on_chain, stands_for, tips, kgraph.nested_trees, lines, 1, "")
        for tree in kgraph.trees:
            _edges(tree, owners, on_chain, _tip_frames(tree), lines)
    lines.append("}")
    return "\n".join(lines) + "\n"
