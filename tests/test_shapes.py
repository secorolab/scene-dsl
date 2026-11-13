import re

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.color import ColorScheme
from rdf_utils.models.vocab import (
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
    URI_SHAPES_PRED_SCALE_X,
    URI_SHAPES_PRED_SCALE_Y,
    URI_SHAPES_PRED_SCALE_Z,
    URI_SHAPES_TYPE_SCALED_SHAPE,
    URI_SHAPES_TYPE_UNIT_SPHERE,
)
from rdf_utils.namespace import NS_MM_QUDT_UNIT
from rdflib import RDF, XSD, Graph, Literal
from rdflib.namespace import split_uri

from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf.shapes import add_shape
from scene_dsl.rdf_parser.shapes import PrimitiveShapeType, scene_shapes

from .test_common import MODELS_DIR


def _shape_graph(shape_spec: str, model_file=None):
    """The graph of one shape on body `b`, from a model in MODEL_FILE when a path needs one."""
    source = f"""ns n = "https://example.test/"
ktree (ns=n) t {{ root: <b.f>
    body b {{
        frame f {{ }}
        {shape_spec}
    }}
    joints {{ }}
}}
"""
    if model_file is None:
        model = scenex_metamodel().model_from_str(source)
    else:
        model_file.write_text(source)
        model = scenex_metamodel().model_from_file(str(model_file))
    [shape] = model.ktrees[0].bodies[0].shapes
    graph = Graph()
    add_shape(graph, shape)
    return shape.uri, graph


def test_example_shapes_read_back_as_authored():
    model = scenex_metamodel().model_from_file(str(MODELS_DIR / "lab.scenex"))
    shapes = {split_uri(s.id)[1]: s for s in scene_shapes(create_scenex_model_graph(model))}

    # Unnamed, each shape is known by its frame.
    assert {name: s.primitive for name, s in shapes.items()} == {
        "ball_root-shape-0": PrimitiveShapeType.SPHERE,
        "bottle_root-shape-0": PrimitiveShapeType.CYLINDER,
        "box1_root-shape-0": PrimitiveShapeType.CUBE,
        "world-shape-0": None,
    }
    ball, bottle, box, marker = (
        shapes[f"{frame}-shape-0"] for frame in ("ball_root", "bottle_root", "box1_root", "world")
    )
    # Scale and unit come back as authored, unconverted: a sphere's radii, a cylinder's radii
    # and height, a cube's sides, a file shape's length per coordinate unit.
    assert [(s.scale, s.unit) for s in (ball, bottle, box, marker)] == [
        ((0.04, 0.04, 0.04), URI_QUDT_UNIT_M),
        ((35.0, 35.0, 200.0), URI_QUDT_UNIT_MM),
        ((4.0, 4.0, 4.0), URI_QUDT_UNIT_CM),
        ((5.0, 5.0, 5.0), URI_QUDT_UNIT_CM),
    ]
    assert [split_uri(s.frame)[1] for s in (ball, bottle, box, marker)] == [
        "ball_root", "bottle_root", "box1_root", "world",
    ]
    assert marker.path.name == "tetra.stl" and marker.path.is_file()
    assert ball.path is None
    # One colour per form: rgba, hsv and hex, each read in its own scheme. The marker states
    # none, so a consumer keeps its default.
    assert [(s.color.scheme, s.color.value) for s in (ball, bottle, box)] == [
        (ColorScheme.RGBA, "#E63333FF"),
        (ColorScheme.HSV, "210 0.6 0.9"),
        (ColorScheme.RGB, "#1A59FF"),
    ]
    assert marker.color is None


def test_unscaled_shape_has_unit_scale():
    _, graph = _shape_graph("shape sphere { frame: <f> unit: m }")
    [shape] = scene_shapes(graph)
    assert shape.scale == (1.0, 1.0, 1.0)


def test_file_shape_reads_the_file_its_path_names(tmp_path):
    part = tmp_path / "meshes" / "part.obj"
    part.parent.mkdir()
    part.write_text("")
    spec = 'shape file { frame: <f> sys path = "meshes/part.obj" unit: mm scale: 2 }'
    _, graph = _shape_graph(spec, tmp_path / "scene.scenex")

    [shape] = scene_shapes(graph)
    assert (shape.primitive, shape.path) == (None, part.resolve())
    assert (shape.scale, shape.unit) == ((2.0, 2.0, 2.0), URI_QUDT_UNIT_MM)


def test_missing_shape_file_is_reported(tmp_path):
    spec = 'shape file { frame: <f> sys path = "meshes/part.obj" unit: mm }'
    _, graph = _shape_graph(spec, tmp_path / "scene.scenex")
    with pytest.raises(ConstraintViolation, match="no file"):
        scene_shapes(graph)


@pytest.mark.parametrize(
    ("break_graph", "message"),
    [
        # A typed shape without a frame is reported, not skipped.
        (
            lambda g, s: g.remove((s, URI_SHAPES_PRED_ATTACHED_TO_FRAME, None)),
            "attached to no frame",
        ),
        (lambda g, s: g.add((s, RDF.type, URI_SHAPES_TYPE_UNIT_SPHERE)), "needs one type of"),
        (lambda g, s: g.set((s, URI_QUDT_PRED_UNIT, NS_MM_QUDT_UNIT["IN"])), "no supported length unit"),
        (lambda g, s: g.set((s, URI_SHAPES_PRED_SCALE_X, Literal(2))), "needs one xsd:double"),
        (
            lambda g, s: g.set((s, URI_SHAPES_PRED_SCALE_Y, Literal(0.0, datatype=XSD.double))),
            "expected finite positive",
        ),
        (
            lambda g, s: g.remove((s, URI_SHAPES_PRED_SCALE_Z, None)),
            "needs all three scale factors",
        ),
        (
            lambda g, s: g.remove((s, RDF.type, URI_SHAPES_TYPE_SCALED_SHAPE)),
            "states scale factors but is not a ScaledShape",
        ),
    ],
)
def test_malformed_shape_is_reported(break_graph, message):
    uri, graph = _shape_graph("shape cube { frame: <f> unit: cm scale: (1, 2, 3) }")
    break_graph(graph, uri)
    with pytest.raises(ConstraintViolation, match=re.escape(message)):
        scene_shapes(graph)
