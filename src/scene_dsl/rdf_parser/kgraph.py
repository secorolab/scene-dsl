# SPDX-License-Identifier: MPL-2.0
"""A scene's whole kinematics: the trees it carries, and the bodies hanging from nothing.

A graph states a root for every body no joint attaches, which is what says the body is
this graph's and not another's. A tree stands on one of them; the rest are only placed.
"""

from pathlib import Path

from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_SIMPLICES,
    URI_GEOM_TYPE_KGRAPH,
    URI_KC_EXT_PRED_ROOT,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_JOINT,
)
from rdflib import RDF, Graph, URIRef

from scene_dsl.rdf_parser.ktree import (
    KinematicTreeModel,
    RigidBodyModel,
    body_of_frame,
    kinematic_trees,
    typed,
    uris,
)


def root_bodies(node: URIRef, graph: Graph) -> set[URIRef]:
    """The bodies a graph hangs from: what its trees stand on, and what it only places."""
    return {
        body_of_frame(frame, graph) for frame in uris(graph.objects(node, URI_KC_EXT_PRED_ROOT))
    }


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
    nested_trees: dict[URIRef, tuple[URIRef, ...]]

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
        roots = root_bodies(self.id, graph)
        built = kinematic_trees(graph, base_dir) if trees is None else trees
        self.trees = tuple(tree for tree in built if tree.root in roots)

        # A body a joint holds is carried by a tree, and belongs to it; what is left hangs
        # from nothing -- placed by a pose, not articulated.
        self.free_bodies = {
            body: RigidBodyModel(body, graph)
            for body in sorted(roots, key=str)
            if not is_attached(body, graph)
        }

        # A component the graph itself names is no tree below it: those bodies are its own.
        self.nested_trees = {self.id: tuple(tree.id for tree in self.trees if tree.id != self.id)}
        for tree in self.trees:
            self.nested_trees.update(tree.nested_trees)


def kinematic_graphs(graph: Graph, base_dir: Path | None = None) -> list[KinematicGraphModel]:
    """Every kinematic graph the scene declares, with the trees below each already built."""
    trees = kinematic_trees(graph, base_dir)
    return [
        KinematicGraphModel(kgraph_id, graph, base_dir, trees)
        for kgraph_id in sorted(uris(graph.subjects(RDF.type, URI_GEOM_TYPE_KGRAPH)), key=str)
    ]
