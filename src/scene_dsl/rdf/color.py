# SPDX-License-Identifier: MPL-2.0
from rdflib import RDF, Graph, Literal, Namespace, URIRef

from scene_dsl.classes.color import Color

COL = Namespace("https://secorolab.github.io/metamodels/color#")


def add_color(graph: Graph, owner: URIRef, color_uri: URIRef, color: Color) -> None:
    """Draw `owner` in `color`, typed as `col:Color` and its scheme so either can be required."""
    graph.bind("col", COL)
    graph.add((owner, COL["has-color"], color_uri))
    graph.add((color_uri, RDF.type, COL.Color))
    graph.add((color_uri, RDF.type, COL[color.scheme.value]))
    graph.add((color_uri, COL.value, Literal(color.literal)))
