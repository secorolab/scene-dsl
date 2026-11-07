# SPDX-License-Identifier: MPL-2.0
from pathlib import Path
from typing import Any

from rdf_utils.models.prov import add_directory_location, add_file_entity, load_pkg_prov
from rdf_utils.models.python import (
    URI_PY_PRED_ATTR_NAME,
    URI_PY_PRED_MODULE_NAME,
    URI_PY_TYPE_MODULE_ATTR,
)
from rdf_utils.models.vocab import (
    URI_GEOM_PRED_X,
    URI_GEOM_PRED_Y,
    URI_GEOM_PRED_Z,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_ROS_TYPE_PACKAGE,
)
from rdf_utils.namespace import NS_MM_ROS
from rdflib import RDF, Graph, Literal, URIRef
from rdflib.namespace import PROV
from textx import get_model

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


def add_file_spec(graph: Graph, file_spec: Any) -> URIRef:
    """A file named by a FileSpec (SystemPath or RosPath), as a `prov:Entity` at its location.

    A relative system path is under the directory of the model file that names it. Returns the
    file's URI, which rdf-utils derives from its location.
    """
    file_spec_type = file_spec.__class__.__name__
    if "RosPath" in file_spec_type:
        # One IRI per package, its ROS index page, whichever scene names it. No commit is
        # recorded, so there is no identifier to mark dirty.
        root = NS_MM_ROS[file_spec.pkg]
        load_pkg_prov(
            graph, root, file_spec.pkg, additional_types=(URI_ROS_TYPE_PACKAGE,), dirty=False
        )
    elif "SystemPath" in file_spec_type:
        if Path(file_spec.path).expanduser().is_absolute():
            root = None
        else:
            root = add_directory_location(graph, _model_dir(file_spec))
    else:
        raise ValueError(f"Unhandled file specification type: {file_spec_type}")

    file_uri = add_file_entity(graph, root, file_spec.path)
    if root is None:
        # An absolute path is its own location.
        graph.add((graph.value(file_uri, PROV.atLocation), RDF.type, PROV.Location))
    return file_uri


def _model_dir(obj: Any) -> Path:
    """The directory of the model file `obj` came from: what its relative paths are under."""
    filename = get_model(obj)._tx_filename
    return Path(filename).resolve().parent if filename else Path.cwd()
