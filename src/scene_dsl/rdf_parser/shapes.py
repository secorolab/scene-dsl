# SPDX-License-Identifier: MPL-2.0
"""The shapes a scene's graph draws: what each is, the frame it hangs on, its size and colour.

A shape is a unit base in its declared length unit -- a cube of side 1, a cylinder or sphere
of radius 1, or a mesh file's own coordinates -- scaled per local axis, with its origin and
axes at the frame it is attached to. Where that frame is, is the kinematics' to say. The scale
is read in metres, as rdf-utils reads every length.
"""

import math
from enum import StrEnum
from pathlib import Path

from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.geom_coord import to_metres
from rdf_utils.models.prov import get_file_path
from rdf_utils.models.vocab import (
    URI_QUDT_PRED_UNIT,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
    URI_SHAPES_TYPE_FILE_BASED_SHAPE,
    URI_SHAPES_TYPE_SCALED_SHAPE,
)
from rdf_utils.namespace import NS_MM_SHAPES
from rdflib import RDF, XSD, Graph, URIRef
from rdflib.namespace import PROV

from scene_dsl.rdf_parser.color import ColorModel, get_color
from scene_dsl.rdf_parser.common import ensure_one_obj_literal, ensure_one_obj_uri


class PrimitiveShapeType(StrEnum):
    """A primitive's unit base; the value is its class in the shapes metamodel."""

    CUBE = "UnitCube"
    CYLINDER = "UnitCylinder"
    SPHERE = "UnitSphere"


SCALE_PREDS = tuple(NS_MM_SHAPES[f"scale-{axis}"] for axis in "xyz")
PRIMITIVE_TYPES = {NS_MM_SHAPES[primitive.value]: primitive for primitive in PrimitiveShapeType}
SHAPE_TYPES = {*PRIMITIVE_TYPES, URI_SHAPES_TYPE_FILE_BASED_SHAPE}


class ShapeModel(ModelBase):
    """One shape a scene draws.

    Attributes:
        frame: the frame the shape is attached to
        primitive: the unit base of a primitive shape; None for a file-based one
        path: the mesh file of a file-based shape; None for a primitive
        scale: metres per base unit along local X, Y and Z, the length unit folded in: a
            cube's side lengths, a sphere's radii, a cylinder's two radii and its height, or
            what takes a mesh's coordinates to metres
        color: the colour as the scene states it; None when it states none
    """

    frame: URIRef
    primitive: PrimitiveShapeType | None
    path: Path | None
    scale: tuple[float, float, float]
    color: ColorModel | None

    def __init__(self, shape_id: URIRef, graph: Graph) -> None:
        types = get_node_types(graph=graph, node_id=shape_id)
        kinds = types & SHAPE_TYPES
        if len(kinds) != 1:
            raise ConstraintViolation(
                "shapes",
                f"shape '{shape_id}' needs one type of UnitCube, UnitCylinder, UnitSphere or "
                f"FileBasedShape, found {sorted(kinds, key=str)}",
            )
        [kind] = kinds
        super().__init__(node_id=shape_id, types=types)

        frame = ensure_one_obj_uri(graph, self.id, URI_SHAPES_PRED_ATTACHED_TO_FRAME)
        if frame is None:
            raise ConstraintViolation("shapes", f"shape '{self.id}' is attached to no frame")
        self.frame = frame

        self.primitive = PRIMITIVE_TYPES.get(kind)
        self.path = self._file(graph) if kind == URI_SHAPES_TYPE_FILE_BASED_SHAPE else None
        unit = ensure_one_obj_uri(graph, self.id, URI_QUDT_PRED_UNIT)
        x, y, z = to_metres(self._scale(graph), unit, self.id)
        self.scale = (x, y, z)
        self.color = get_color(graph, self.id)

    def _scale(self, graph: Graph) -> tuple[float, float, float]:
        if URI_SHAPES_TYPE_SCALED_SHAPE not in self.types:
            if any(graph.value(self.id, pred) is not None for pred in SCALE_PREDS):
                raise ConstraintViolation(
                    "shapes", f"shape '{self.id}' states scale factors but is not a ScaledShape"
                )
            return (1.0, 1.0, 1.0)
        factors = []
        for pred in SCALE_PREDS:
            literal = ensure_one_obj_literal(graph, self.id, pred)
            if literal is None or literal.datatype != XSD.double:
                raise ConstraintViolation(
                    "shapes", f"shape '{self.id}' needs one xsd:double '{pred}', found {literal}"
                )
            factor = float(literal)
            if not math.isfinite(factor) or factor <= 0:
                raise ConstraintViolation(
                    "shapes", f"shape '{self.id}' has '{pred}' {factor}, expected finite positive"
                )
            factors.append(factor)
        x, y, z = factors
        return x, y, z

    def _file(self, graph: Graph) -> Path:
        """The mesh file, located as the scene names it: on this system or in a package."""
        source = ensure_one_obj_uri(graph, self.id, PROV.wasDerivedFrom)
        try:
            path = None if source is None else get_file_path(graph, source)
        except (ValueError, LookupError, ImportError) as error:
            raise ConstraintViolation(
                "shapes", f"file shape '{self.id}' cannot be located: {error}"
            ) from error
        if path is None:
            raise ConstraintViolation("shapes", f"file shape '{self.id}' states no file location")
        if not path.is_file():
            raise ConstraintViolation("shapes", f"file shape '{self.id}' is at '{path}', no file")
        return path


def scene_shapes(graph: Graph) -> list[ShapeModel]:
    """Every shape the graph draws, ordered by IRI.

    A node typed as a shape or attached to a frame is one, so a shape missing either is
    reported rather than skipped.
    """
    nodes = set(graph.subjects(URI_SHAPES_PRED_ATTACHED_TO_FRAME))
    for shape_type in SHAPE_TYPES:
        nodes.update(graph.subjects(RDF.type, shape_type))
    for node in nodes:
        if not isinstance(node, URIRef):
            raise ConstraintViolation("shapes", f"shape '{node}' needs an IRI")
    return [ShapeModel(node, graph) for node in sorted(nodes, key=str)]
