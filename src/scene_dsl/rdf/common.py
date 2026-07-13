# SPDX-License-Identifier: MPL-2.0
from typing import Any

from rdflib import RDF, Graph, Literal, URIRef
from rdf_utils.models.python import (
    URI_PY_PRED_ATTR_NAME,
    URI_PY_PRED_MODULE_NAME,
    URI_PY_TYPE_MODULE_ATTR,
)
from rdf_utils.models.geometry import (
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
)

from scene_dsl.classes.common import FloatVector, parse_py_module_attr


def add_py_module_attr(graph: Graph, node_uri: URIRef, py_model: Any):
    module_name, attr_name = parse_py_module_attr(model=py_model)
    graph.add(triple=(node_uri, RDF.type, URI_PY_TYPE_MODULE_ATTR))
    graph.add(triple=(node_uri, URI_PY_PRED_MODULE_NAME, Literal(module_name)))
    graph.add(triple=(node_uri, URI_PY_PRED_ATTR_NAME, Literal(attr_name)))


def add_vector_xyz(graph: Graph, node: URIRef, vect: FloatVector) -> None:
    x, y, z = vect.as_xyz()
    graph.add((node, RDF.type, URI_GEOM_TYPE_VECTOR_XYZ))
    graph.add((node, URI_GEOM_PRED_X, Literal(x)))
    graph.add((node, URI_GEOM_PRED_Y, Literal(y)))
    graph.add((node, URI_GEOM_PRED_Z, Literal(z)))
