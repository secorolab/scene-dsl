# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.prov import add_entity
from rdf_utils.models.vocab import (
    URI_QUDT_PRED_UNIT,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
    URI_SHAPES_TYPE_FILE_BASED_SHAPE,
    URI_SHAPES_TYPE_SCALED_SHAPE,
)
from rdf_utils.namespace import NS_MM_SHAPES
from rdflib import RDF, XSD, Graph, Literal
from rdflib.namespace import PROV

from scene_dsl.classes.ktree import SceneShape
from scene_dsl.rdf.color import add_color
from scene_dsl.rdf.common import add_file_source
from scene_dsl.rdf.geom import LENGTH_UNITS


def add_shape_scale(graph: Graph, shape: SceneShape) -> None:
    if shape.scale is None:
        return
    graph.add((shape.uri, RDF.type, URI_SHAPES_TYPE_SCALED_SHAPE))
    for axis, factor in zip("xyz", shape.scale, strict=True):
        graph.add((shape.uri, NS_MM_SHAPES[f"scale-{axis}"], Literal(factor, datatype=XSD.double)))


def add_shape(graph: Graph, shape: SceneShape) -> None:
    graph.add((shape.uri, URI_SHAPES_PRED_ATTACHED_TO_FRAME, shape.frame.uri))
    graph.add((shape.uri, URI_QUDT_PRED_UNIT, LENGTH_UNITS[shape.unit]))
    if shape.primitive is not None:
        graph.add((shape.uri, RDF.type, NS_MM_SHAPES[shape.primitive.value]))
    else:
        graph.add((shape.uri, RDF.type, URI_SHAPES_TYPE_FILE_BASED_SHAPE))
        add_entity(graph, shape.uri)
        graph.add((shape.uri, PROV.wasDerivedFrom, add_file_source(graph, shape.file)))
    add_shape_scale(graph, shape)
    if shape.color is not None:
        add_color(graph, shape.uri, shape.color_uri, shape.color)
