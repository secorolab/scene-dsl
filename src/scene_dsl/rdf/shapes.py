# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.color import add_color
from rdf_utils.models.prov import add_entity
from rdf_utils.models.vocab import (
    URI_QUDT_PRED_UNIT,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
    URI_SHAPES_TYPE_FILE_BASED_SHAPE,
    URI_SHAPES_TYPE_SCALED_SHAPE,
    URI_SHAPES_TYPE_SHAPE,
)
from rdflib import RDF, XSD, Graph, Literal
from rdflib.namespace import PROV

from scene_dsl.classes.ktree import SceneShape
from scene_dsl.rdf.common import add_file_source
from scene_dsl.rdf.geom import LENGTH_UNITS
from scene_dsl.rdf_parser.shapes import SCALE_PREDS
from scene_dsl.rdf_parser.vocab import (
    URI_CONTACT_PRED_HAS_SURFACE_FRICTION,
    URI_CONTACT_PRED_ROLLING,
    URI_CONTACT_PRED_SLIDING,
    URI_CONTACT_PRED_TORSIONAL,
    URI_CONTACT_TYPE_COULOMB_FRICTION,
    URI_CONTACT_TYPE_SURFACE_FRICTION,
)


def add_shape_scale(graph: Graph, shape: SceneShape) -> None:
    if shape.scale is None:
        return
    graph.add((shape.uri, RDF.type, URI_SHAPES_TYPE_SCALED_SHAPE))
    for pred, factor in zip(SCALE_PREDS, shape.scale, strict=True):
        graph.add((shape.uri, pred, Literal(factor, datatype=XSD.double)))


def add_shape(graph: Graph, shape: SceneShape) -> None:
    graph.add((shape.uri, RDF.type, URI_SHAPES_TYPE_SHAPE))
    graph.add((shape.uri, URI_SHAPES_PRED_ATTACHED_TO_FRAME, shape.frame.uri))
    graph.add((shape.uri, URI_QUDT_PRED_UNIT, LENGTH_UNITS[shape.unit]))
    if shape.primitive is not None:
        graph.add((shape.uri, RDF.type, shape.primitive.value))
    else:
        graph.add((shape.uri, RDF.type, URI_SHAPES_TYPE_FILE_BASED_SHAPE))
        add_entity(graph, shape.uri)
        graph.add((shape.uri, PROV.wasDerivedFrom, add_file_source(graph, shape.file)))
    add_shape_scale(graph, shape)
    for role in shape.roles:
        graph.add((shape.uri, RDF.type, role.value))
    if shape.color is not None:
        add_color(graph, shape.uri, shape.color_uri, shape.color.scheme, shape.color.literal)
    if shape.friction is not None:
        add_surface_friction(graph, shape)


def add_surface_friction(graph: Graph, shape: SceneShape) -> None:
    """The friction of the shape's surface, on a node of its own: the shape's unit is its size's,
    and the torsional and rolling coefficients carry a length unit of their own. The node is a
    SurfaceFriction typed with its friction model, CoulombFriction, the one the grammar states."""
    friction, node = shape.friction, shape.friction_uri
    graph.add((shape.uri, URI_CONTACT_PRED_HAS_SURFACE_FRICTION, node))
    graph.add((node, RDF.type, URI_CONTACT_TYPE_SURFACE_FRICTION))
    graph.add((node, RDF.type, URI_CONTACT_TYPE_COULOMB_FRICTION))
    graph.add((node, URI_CONTACT_PRED_SLIDING, Literal(friction.sliding, datatype=XSD.double)))
    if friction.torsional is not None:
        graph.add(
            (node, URI_CONTACT_PRED_TORSIONAL, Literal(friction.torsional, datatype=XSD.double))
        )
    if friction.rolling is not None:
        graph.add((node, URI_CONTACT_PRED_ROLLING, Literal(friction.rolling, datatype=XSD.double)))
    if friction.unit is not None:
        graph.add((node, URI_QUDT_PRED_UNIT, LENGTH_UNITS[friction.unit]))
