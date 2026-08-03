# SPDX-License-Identifier: MPL-2.0
"""A scene's whole kinematics: the trees it composes, and the bodies hanging from nothing.

A tree has one root and reaches every body of its own through a joint. A body no joint
attaches is therefore no tree's -- it is placed, not articulated -- and belongs to the
graph composing it, which is what this reads.
"""

from collections import deque
from pathlib import Path

from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_SIMPLICES,
    URI_GEOM_TYPE_KGRAPH,
    URI_GEOM_TYPE_KTREE,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.rdf_parser.ktree import (
    KinematicTreeModel,
    RigidBodyModel,
    kinematic_trees,
    typed,
    uris,
)
from scene_dsl.rdf_parser.vocab import URI_KC_EXT_PRED_COMPOSES


def composed_trees(node: URIRef, graph: Graph) -> set[URIRef]:
    """The trees a graph or a tree is built from, directly."""
    return typed(graph.objects(node, URI_KC_EXT_PRED_COMPOSES), URI_GEOM_TYPE_KTREE, graph)


def composed_bodies(node: URIRef, graph: Graph) -> set[URIRef]:
    """The bodies a graph hangs itself, rather than through a tree below it."""
    return typed(graph.objects(node, URI_KC_EXT_PRED_COMPOSES), URI_GEOM_TYPE_RIGID_BODY, graph)


def trees_below(node: URIRef, graph: Graph) -> set[URIRef]:
    """Every tree under a graph or tree, composed directly or through another."""
    found: set[URIRef] = set()
    pending = deque([node])
    while pending:
        for tree in composed_trees(pending.popleft(), graph):
            if tree not in found:
                found.add(tree)
                pending.append(tree)
    return found


def is_attached(body: URIRef, graph: Graph) -> bool:
    """Whether any joint holds this body, which is what makes it part of a tree."""
    return any(
        typed(graph.subjects(URI_KC_PRED_BETWEEN_ATTACHMENTS, frame), URI_KC_TYPE_JOINT, graph)
        for frame in uris(graph.objects(body, URI_GEOM_PRED_SIMPLICES))
    )


class KinematicGraphModel(ModelBase):
    """The kinematics of one scene: its trees, and the free bodies it places itself."""

    trees: tuple[KinematicTreeModel, ...]
    free_bodies: dict[URIRef, RigidBodyModel]

    def __init__(
        self,
        kgraph_id: URIRef,
        graph: Graph,
        base_dir: Path | None = None,
        trees: list[KinematicTreeModel] | None = None,
    ) -> None:
        super().__init__(node_id=kgraph_id, graph=graph)
        if URI_GEOM_TYPE_KGRAPH not in self.types:
            raise TypeError(f"{self} is not a KinematicGraph")

        self.base_dir = base_dir
        below = trees_below(self.id, graph)
        built = kinematic_trees(graph, base_dir) if trees is None else trees
        self.trees = tuple(tree for tree in built if tree.id in below)

        # A body a joint holds is carried by a tree, and belongs to it; what is left hangs
        # from nothing -- placed by a pose, not articulated.
        self.free_bodies = {
            body: RigidBodyModel(body, graph)
            for body in sorted(composed_bodies(self.id, graph), key=str)
            if not is_attached(body, graph)
        }


def kinematic_graphs(graph: Graph, base_dir: Path | None = None) -> list[KinematicGraphModel]:
    """Every kinematic graph the scene declares, with the trees below each already built."""
    trees = kinematic_trees(graph, base_dir)
    return [
        KinematicGraphModel(kgraph_id, graph, base_dir, trees)
        for kgraph_id in sorted(uris(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH)), key=str)
    ]
