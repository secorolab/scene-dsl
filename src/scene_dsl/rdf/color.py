# SPDX-License-Identifier: MPL-2.0
from rdf_utils.models.vocab import (
    URI_COLOR_PRED_HAS_COLOR,
    URI_COLOR_PRED_VALUE,
    URI_COLOR_TYPE_COLOR,
)
from rdf_utils.namespace import NS_MM_COLOR
from rdflib import RDF, Graph, Literal, URIRef

from scene_dsl.classes.color import Color


def add_color(graph: Graph, owner: URIRef, color_uri: URIRef, color: Color) -> None:
    """Draw `owner` in `color`, typed as `col:Color` and its scheme so either can be required."""
    graph.add((owner, URI_COLOR_PRED_HAS_COLOR, color_uri))
    graph.add((color_uri, RDF.type, URI_COLOR_TYPE_COLOR))
    graph.add((color_uri, RDF.type, NS_MM_COLOR[color.scheme.value]))
    graph.add((color_uri, URI_COLOR_PRED_VALUE, Literal(color.literal)))
