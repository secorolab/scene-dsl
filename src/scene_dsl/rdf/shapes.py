# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.color import add_color
from rdf_utils.models.prov import add_entity
from rdf_utils.models.vocab import (
    URI_QUDT_PRED_UNIT,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
    URI_SHAPES_TYPE_FILE_BASED_SHAPE,
    URI_SHAPES_TYPE_SCALED_SHAPE,
)
from rdflib import RDF, XSD, Graph, Literal
from rdflib.namespace import PROV

from scene_dsl.classes.ktree import SceneShape
from scene_dsl.rdf.common import add_file_source
from scene_dsl.rdf.geom import LENGTH_UNITS
from scene_dsl.rdf_parser.shapes import SCALE_PREDS


def add_shape_scale(graph: Graph, shape: SceneShape) -> None:
    if shape.scale is None:
        return
    graph.add((shape.uri, RDF.type, URI_SHAPES_TYPE_SCALED_SHAPE))
    for pred, factor in zip(SCALE_PREDS, shape.scale, strict=True):
        graph.add((shape.uri, pred, Literal(factor, datatype=XSD.double)))


def add_shape(graph: Graph, shape: SceneShape) -> None:
    graph.add((shape.uri, URI_SHAPES_PRED_ATTACHED_TO_FRAME, shape.frame.uri))
    graph.add((shape.uri, URI_QUDT_PRED_UNIT, LENGTH_UNITS[shape.unit]))
    if shape.primitive is not None:
        graph.add((shape.uri, RDF.type, shape.primitive.value))
    else:
        graph.add((shape.uri, RDF.type, URI_SHAPES_TYPE_FILE_BASED_SHAPE))
        add_entity(graph, shape.uri)
        graph.add((shape.uri, PROV.wasDerivedFrom, add_file_source(graph, shape.file)))
    add_shape_scale(graph, shape)
    if shape.color is not None:
        add_color(graph, shape.uri, shape.color_uri, shape.color.scheme, shape.color.literal)
