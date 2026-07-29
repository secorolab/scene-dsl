# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.common import ModelBase
from rdf_utils.models.geom_rel import FrameModel
from rdf_utils.models.vocab import (
    URI_DYN_PRED_ABOUT,
    URI_DYN_PRED_OF_BODY,
    URI_DYN_TYPE_RIGID_BODY_INERTIA,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SIMPLICES,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_KC_EXT_PRED_ROOT,
)
from rdflib import Graph, URIRef

from scene_dsl.rdf_parser.common import ensure_one_obj_uri, ensure_one_typed_subject_uri


class InertiaModel(ModelBase):
    inertial_frame: FrameModel

    def __init__(self, inertia_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=inertia_id, graph=graph)

        if URI_DYN_TYPE_RIGID_BODY_INERTIA not in self.types:
            raise TypeError(f"{self} is not a RigidBodyInertia")

        about_node = ensure_one_obj_uri(graph=graph, subject=self.id, predicate=URI_DYN_PRED_ABOUT)
        if about_node is None:
            raise ValueError(
                f"RigidBodyInertia {self} does not link to an URI via predicate 'about'"
            )
        inertial_frame_id = ensure_one_typed_subject_uri(
            graph=graph,
            obj=about_node,
            predicate=URI_GEOM_PRED_ORIGIN,
            subject_type=URI_GEOM_TYPE_FRAME,
        )
        if inertial_frame_id is None:
            raise ValueError(f"RigidBodyInertia {self} about {about_node} is not a frame origin")
        self.inertial_frame = FrameModel(frame_id=inertial_frame_id, graph=graph)

        # TODO(minhnh): parse inertia coordinates


def get_root_frame(target_id: URIRef, graph: Graph) -> FrameModel:
    """Return the declared root frame of a kinematics model."""

    root_id = ensure_one_obj_uri(graph, target_id, URI_KC_EXT_PRED_ROOT)
    if root_id is None:
        raise ValueError(f"Kinematics model '{target_id}' doesn't link to a root URI")

    return FrameModel(frame_id=root_id, graph=graph)


class RigidBodyModel(ModelBase):
    simplices: set[URIRef]
    root_frame: FrameModel
    inertia: InertiaModel | None

    def __init__(self, body_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=body_id, graph=graph)
        if URI_GEOM_TYPE_RIGID_BODY not in self.types:
            raise TypeError(f"{self} is not a RigidBody")

        self.simplices = set()
        for simplice_node in graph.objects(subject=self.id, predicate=URI_GEOM_PRED_SIMPLICES):
            if not isinstance(simplice_node, URIRef):
                raise TypeError(
                    f"RigidBody '{self}' doesn't link to an URI via 'geom_rel:simplices': {simplice_node}"
                )
            self.simplices.add(simplice_node)

        self.root_frame = get_root_frame(target_id=self.id, graph=graph)
        if self.root_frame.id not in self.simplices:
            raise ValueError(
                f"Root frame '{self.root_frame}' is not a simplicial complex of RigidBody '{self}'"
            )

        inertia_id = ensure_one_typed_subject_uri(
            graph=graph,
            obj=self.id,
            predicate=URI_DYN_PRED_OF_BODY,
            subject_type=URI_DYN_TYPE_RIGID_BODY_INERTIA,
        )
        if not inertia_id:
            self.inertia = None
        else:
            self.inertia = InertiaModel(inertia_id=inertia_id, graph=graph)
