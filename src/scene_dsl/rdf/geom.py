# SPDX-License-Identifier: MPL-2.0
from rdflib import RDF, Graph, Literal, URIRef

from rdf_utils.collection import add_literal_list_pred
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_START,
    URI_GEOM_PRED_VECT_X,
    URI_GEOM_PRED_VECT_Y,
    URI_GEOM_PRED_VECT_Z,
    URI_GEOM_PRED_DIRECTION_COSINE_X,
    URI_GEOM_PRED_DIRECTION_COSINE_Y,
    URI_GEOM_PRED_DIRECTION_COSINE_Z,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_OF,
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_WRT,
    URI_GEOM_TYPE_BOUND_VECTOR,
    URI_GEOM_TYPE_ORIENT,
    URI_GEOM_TYPE_ORIENT_COORD,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POSITION,
    URI_GEOM_TYPE_POSITION_COORD,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_DIRECTION_COSINE_XYZ,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_FRAME,
    URI_GEOM_TYPE_INTRINSIC,
    URI_GEOM_TYPE_POINT,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_PRED_QUANTITY_KIND,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_QK_ANGLE,
    URI_QUDT_QK_LENGTH,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_DEG,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM,
    URI_QUDT_UNIT_RAD,
)
from scene_dsl.classes.common import FloatVector
from scene_dsl.classes.distrib import (
    DistributionRef,
    NormalDistribution,
    UniformDistribution,
    UniformRotationDistribution,
)
from scene_dsl.classes.geom import (
    DirectionCosineOrientationSpec,
    EulerOrientationSpec,
    Frame,
    PoseSpec,
)
from scene_dsl.rdf.common import add_vector_xyz
from scene_dsl.rdf.distrib import add_sampled_quantity

LENGTH_UNITS = {"m": URI_QUDT_UNIT_M, "cm": URI_QUDT_UNIT_CM, "mm": URI_QUDT_UNIT_MM}
ANGLE_UNITS = {"deg": URI_QUDT_UNIT_DEG, "rad": URI_QUDT_UNIT_RAD}
AXES_PREDS = {"x": URI_GEOM_PRED_VECT_X, "y": URI_GEOM_PRED_VECT_Y, "z": URI_GEOM_PRED_VECT_Z}


def add_frame(graph: Graph, frame: Frame) -> None:
    graph.add((frame.uri, RDF.type, URI_GEOM_TYPE_FRAME))
    graph.add((frame.origin_uri, RDF.type, URI_GEOM_TYPE_POINT))
    graph.add((frame.uri, URI_GEOM_PRED_ORIGIN, frame.origin_uri))

    for axis, pred in AXES_PREDS.items():
        axis_vector_uri = frame.axis_vector_uri(axis=axis)
        graph.add((frame.uri, pred, axis_vector_uri))
        graph.add((axis_vector_uri, URI_GEOM_PRED_START, frame.origin_uri))
        graph.add((axis_vector_uri, RDF.type, URI_GEOM_TYPE_VECTOR))
        graph.add((axis_vector_uri, RDF.type, URI_GEOM_TYPE_BOUND_VECTOR))

    for pose in frame.poses:
        add_pose(graph=graph, pose=pose)


def add_position_coord(
    graph: Graph,
    pos_uri: URIRef,
    pos_coord_uri: URIRef,
    as_seen_by: URIRef,
    of_uri: URIRef,
    wrt_uri: URIRef,
    position_spec: FloatVector | DistributionRef,
    unit: str,
) -> None:
    graph.add(triple=(pos_uri, RDF.type, URI_GEOM_TYPE_POSITION))
    graph.add(triple=(pos_uri, URI_GEOM_PRED_OF, of_uri))
    graph.add(triple=(pos_uri, URI_GEOM_PRED_WRT, wrt_uri))
    graph.add(triple=(pos_uri, URI_QUDT_PRED_QUANTITY_KIND, URI_QUDT_QK_LENGTH))
    graph.add(triple=(pos_coord_uri, RDF.type, URI_GEOM_TYPE_POSITION_COORD))
    graph.add(triple=(pos_coord_uri, URI_GEOM_PRED_SEEN_BY, as_seen_by))
    graph.add(triple=(pos_coord_uri, RDF.type, URI_GEOM_TYPE_POSITION_REF))
    graph.add(triple=(pos_coord_uri, URI_GEOM_PRED_OF_POSITION, pos_uri))
    graph.add(triple=(pos_coord_uri, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))

    if isinstance(position_spec, DistributionRef):
        distribution_spec = position_spec.distribution.spec
        if not isinstance(distribution_spec, (UniformDistribution, NormalDistribution)):
            raise ValueError(
                f"add_position_coord({pos_coord_uri}): sampling requires a uniform or normal distribution"
            )
        if distribution_spec.dimension != 3:
            raise ValueError(
                f"add_position_coord({pos_coord_uri}): XYZ sampling requires a distribution with dimension 3"
            )
        add_sampled_quantity(
            graph=graph,
            quantity_uri=pos_coord_uri,
            distrib_ref=position_spec,
        )
    elif isinstance(position_spec, FloatVector):
        add_vector_xyz(graph=graph, node=pos_coord_uri, vect=position_spec)
    else:
        raise TypeError(
            f"add_position_coord({pos_coord_uri}): Unhandled type for position specification: {position_spec}"
        )

    if unit not in LENGTH_UNITS:
        raise ValueError(f"add_position_coord({pos_coord_uri}): unrecognized length unit '{unit}'")
    graph.add(triple=(pos_coord_uri, URI_QUDT_PRED_UNIT, LENGTH_UNITS[unit]))


def add_orientation_coord(graph: Graph, pose: PoseSpec) -> None:
    graph.add(triple=(pose.orientation_uri, RDF.type, URI_GEOM_TYPE_ORIENT))
    graph.add(triple=(pose.orientation_uri, URI_QUDT_PRED_QUANTITY_KIND, URI_QUDT_QK_ANGLE))
    graph.add(triple=(pose.orientation_uri, URI_GEOM_PRED_OF, pose.of_frame.uri))
    graph.add(triple=(pose.orientation_uri, URI_GEOM_PRED_WRT, pose.wrt.uri))

    graph.add(triple=(pose.orientation_coord_uri, RDF.type, URI_GEOM_TYPE_ORIENT_COORD))
    graph.add(triple=(pose.orientation_coord_uri, RDF.type, URI_GEOM_TYPE_ORIENT_REF))
    graph.add(triple=(pose.orientation_coord_uri, URI_GEOM_PRED_OF_ORIENT, pose.orientation_uri))
    graph.add(triple=(pose.orientation_coord_uri, URI_GEOM_PRED_SEEN_BY, pose.wrt.uri))

    if isinstance(pose.orientation, EulerOrientationSpec):
        graph.add((pose.orientation_coord_uri, RDF.type, URI_GEOM_TYPE_EULER_ANGLES))
        graph.add(
            (pose.orientation_coord_uri, URI_GEOM_PRED_AXES_SEQ, Literal(pose.orientation.axes))
        )
        graph.add(
            (
                pose.orientation_coord_uri,
                RDF.type,
                URI_GEOM_TYPE_EXTRINSIC if pose.orientation.extrinsic else URI_GEOM_TYPE_INTRINSIC,
            )
        )
        graph.add((pose.orientation_coord_uri, RDF.type, URI_GEOM_TYPE_ANGLES_ABG))
        graph.add(
            (pose.orientation_coord_uri, URI_GEOM_PRED_ALPHA, Literal(pose.orientation.alpha))
        )
        graph.add((pose.orientation_coord_uri, URI_GEOM_PRED_BETA, Literal(pose.orientation.beta)))
        graph.add(
            (pose.orientation_coord_uri, URI_GEOM_PRED_GAMMA, Literal(pose.orientation.gamma))
        )
        graph.add(
            (pose.orientation_coord_uri, URI_QUDT_PRED_UNIT, ANGLE_UNITS[pose.orientation.unit])
        )
    elif isinstance(pose.orientation, DirectionCosineOrientationSpec):
        graph.add((pose.orientation_coord_uri, RDF.type, URI_GEOM_TYPE_DIRECTION_COSINE_XYZ))
        add_literal_list_pred(
            graph=graph,
            subject_uri=pose.orientation_coord_uri,
            pred_uri=URI_GEOM_PRED_DIRECTION_COSINE_X,
            values=pose.orientation.x_axis,
        )
        add_literal_list_pred(
            graph=graph,
            subject_uri=pose.orientation_coord_uri,
            pred_uri=URI_GEOM_PRED_DIRECTION_COSINE_Y,
            values=pose.orientation.y_axis,
        )
        add_literal_list_pred(
            graph=graph,
            subject_uri=pose.orientation_coord_uri,
            pred_uri=URI_GEOM_PRED_DIRECTION_COSINE_Z,
            values=pose.orientation.z_axis,
        )
    elif isinstance(pose.orientation, DistributionRef):
        if not isinstance(pose.orientation.distribution.spec, UniformRotationDistribution):
            raise ValueError(
                f"add_orientation_coord({pose.orientation_coord_uri}): sampling requires a UniformRotationDistribution specification"
            )
        add_sampled_quantity(
            graph=graph,
            quantity_uri=pose.orientation_coord_uri,
            distrib_ref=pose.orientation,
        )
    else:
        raise ValueError(
            f"add_orientation_coord({pose.orientation_coord_uri}): Unsupported orientation: {pose.orientation}"
        )


def add_pose(graph: Graph, pose: PoseSpec) -> None:
    if pose.of_frame.uri == pose.wrt.uri:
        raise ValueError(f"PoseSpec '{pose.name}' has same URI for 'of' and 'wrt' frames")

    graph.add((pose.uri, RDF.type, URI_GEOM_TYPE_POSE))
    graph.add((pose.uri, URI_GEOM_PRED_OF, pose.of_frame.uri))
    graph.add((pose.uri, URI_GEOM_PRED_WRT, pose.wrt.uri))
    graph.add((pose.uri_coord, RDF.type, URI_GEOM_TYPE_POSE_REF))
    graph.add((pose.uri_coord, URI_GEOM_PRED_OF_POSE, pose.uri))
    graph.add((pose.uri_coord, RDF.type, URI_GEOM_TYPE_POSE_COORD))
    graph.add((pose.uri_coord, URI_GEOM_PRED_SEEN_BY, pose.wrt.uri))

    add_position_coord(
        graph=graph,
        pos_uri=pose.position_uri,
        pos_coord_uri=pose.position_coord_uri,
        as_seen_by=pose.wrt.uri,
        of_uri=pose.of_frame.origin_uri,
        wrt_uri=pose.wrt.origin_uri,
        position_spec=pose.position_spec,
        unit=pose.length_unit,
    )

    add_orientation_coord(graph=graph, pose=pose)
