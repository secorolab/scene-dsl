# SPDX-License-Identifier: MPL-2.0
from pathlib import Path
from typing import Any

from packageurl import PackageURL
from rdf_utils.models.prov import (
    add_directory_location,
    add_file_entity,
    get_pkg_info,
    load_pkg_prov,
)
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
from rdflib import RDF, Graph, Literal, URIRef
from textx import get_location

from scene_dsl.classes.common import (
    FloatVector,
    PackageKind,
    PackagePath,
    SystemPath,
    parse_py_module_attr,
)


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


def add_file_source(graph: Graph, spec: SystemPath | PackagePath) -> URIRef:
    """The file SPEC names, as a `prov:Entity` at its location; returns the file's URI.

    A relative system path is under the directory of the model file declaring it; a package
    path is under the package's purl.
    """
    if isinstance(spec, PackagePath):
        if spec.kind is PackageKind.ROS:
            try:
                from xml.etree import ElementTree

                from ament_index_python.packages import get_package_share_directory

                manifest = Path(get_package_share_directory(spec.pkg)) / "package.xml"
                version = ElementTree.parse(manifest).findtext("version")
            # ament's PackageNotFoundError is a KeyError; naming it would fail when ament is missing.
            except (ImportError, KeyError):
                version = None
            root = URIRef(PackageURL(type="ros", name=spec.pkg, version=version).to_string())
            load_pkg_prov(
                graph,
                root,
                spec.pkg,
                version,
                additional_types=(URI_ROS_TYPE_PACKAGE,),
                dirty=False,
            )
        elif spec.kind is PackageKind.PY:
            root, version, revision, repository, dirty = get_pkg_info(spec.pkg)
            load_pkg_prov(graph, root, spec.pkg, version, revision, repository, dirty=dirty)
        else:
            raise ValueError(f"Unhandled package kind: {spec.kind}")
    elif isinstance(spec, SystemPath):
        if Path(spec.path).expanduser().is_absolute():
            root = None
        else:
            model_file = get_location(spec)["filename"]
            if model_file is None:
                raise ValueError(
                    f"relative path '{spec.path}' is declared in a model that was not read from "
                    "a file to resolve it against"
                )
            root = add_directory_location(graph, Path(model_file).resolve().parent)
    else:
        raise TypeError(f"Unhandled file specification type: {type(spec).__name__}")
    return add_file_entity(graph, root, spec.path)
