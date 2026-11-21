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
    URI_SHAPES_TYPE_SHAPE,
    URI_SHAPES_TYPE_UNIT_SPHERE,
)
from rdf_utils.namespace import NS_MM_QUDT_UNIT
from rdflib import RDF, XSD, Graph, Literal
from rdflib.namespace import split_uri
from textx.exceptions import TextXSyntaxError

from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.ktree import add_body
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.kinematics import RigidBodyModel, kinematic_graphs
from scene_dsl.rdf_parser.shapes import PrimitiveShapeType, ShapeModel, ShapeRole

from .test_common import MODELS_DIR

VISUAL, COLLISION = {ShapeRole.VISUAL}, {ShapeRole.COLLISION}
BOTH = VISUAL | COLLISION
UNSCALED = (1.0, 1.0, 1.0)


def _shape_graph(shape_spec: str, model_file=None):
    """The graph of body `b` carrying one shape, from a model in MODEL_FILE when a path needs
    one."""
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
    [body] = model.ktrees[0].bodies
    [shape] = body.shapes
    graph = Graph()
    add_body(graph, body)
    return body.uri, shape.uri, graph


def test_example_shapes_read_back_as_authored():
    model = scenex_metamodel().model_from_file(str(MODELS_DIR / "lab.scenex"))
    [kgraph] = kinematic_graphs(create_scenex_model_graph(model))
    trees_bodies = (body for tree in kgraph.trees for body in tree.bodies.values())
    bodies = [*kgraph.free_bodies.values(), *trees_bodies]
    shapes = {split_uri(s.id)[1]: s for body in bodies for s in body.shapes}

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
    # One colour per form: rgba, hsv and hex, each read in its own scheme. The marker states none.
    assert [(s.color.scheme, s.color.value) for s in (ball, bottle, box)] == [
        (ColorScheme.RGBA, "#E63333FF"),
        (ColorScheme.HSV, "210 0.6 0.9"),
        (ColorScheme.RGB, "#1A59FF"),
    ]
    assert marker.color is None
    # The marker is visual only, the box states both roles, and the ball and bottle state none,
    # so they get both.
    assert [s.roles for s in (ball, bottle, box, marker)] == [BOTH, BOTH, BOTH, VISUAL]


@pytest.mark.parametrize(
    ("spec", "roles", "color", "scale"),
    [
        # No role is both. The colour is only what the shape states, whatever its roles.
        ("", BOTH, None, UNSCALED),
        ("role: visual", VISUAL, None, UNSCALED),
        ("role: collision visual", BOTH, None, UNSCALED),
        ("role: collision", COLLISION, None, UNSCALED),
        ("color: #E63333 role: collision", COLLISION, (ColorScheme.RGB, "#E63333"), UNSCALED),
        # A scalar scale is the same factor on all three axes.
        ("scale: 2", BOTH, None, (2.0, 2.0, 2.0)),
        ("scale: (0.5, 1.0, 1.5)", BOTH, None, (0.5, 1.0, 1.5)),
        # 0-255 channels go out as hex: exact, where floats in [0, 1] would round.
        ("color: rgb(230, 51, 0)", BOTH, (ColorScheme.RGB, "#E63300"), UNSCALED),
        ("color: rgba(230, 51, 51, 255)", BOTH, (ColorScheme.RGBA, "#E63333FF"), UNSCALED),
        ("color: #33b34d", BOTH, (ColorScheme.RGB, "#33B34D"), UNSCALED),
        ("color: #33B34D80 role: visual", VISUAL, (ColorScheme.RGBA, "#33B34D80"), UNSCALED),
        ("color: hsv(210, 0.6, 0.9)", BOTH, (ColorScheme.HSV, "210 0.6 0.9"), UNSCALED),
        ("color: hsv(359.5, 1, 0)", BOTH, (ColorScheme.HSV, "359.5 1 0"), UNSCALED),
        # No '-0' or exponent, which the metamodel's pattern refuses.
        ("color: hsv(0.0, -0.0, 1e-5)", BOTH, (ColorScheme.HSV, "0.0 0.0 0.00001"), UNSCALED),
    ],
)
def test_shape_spec_reads_back(spec, roles, color, scale):
    body, _, graph = _shape_graph(f"shape cube {{ frame: <f> unit: m {spec} }}")
    [shape] = RigidBodyModel(body, graph).shapes
    read = None if shape.color is None else (shape.color.scheme, shape.color.value)
    assert (shape.roles, read, shape.scale) == (roles, color, scale)


@pytest.mark.parametrize(
    ("spec", "error", "message"),
    [
        ("scale: (2.0)", ValueError, "cube on 'f' scale requires 3 values, got 1"),
        ("scale: (1.0, 2.0)", ValueError, "cube on 'f' scale requires 3 values, got 2"),
        ("scale: 0", ValueError, "cube on 'f' scale is 0.0, expected a finite positive number"),
        ("scale: (1.0, -2.0, 3.0)", ValueError, "scale is -2.0, expected a finite positive number"),
        ("role: visual visual", ValueError, "cube on 'f' states role 'visual' twice"),
        ("color: rgb(256, 0, 0)", ValueError, "red is 256, expected an integer in [0, 255]"),
        ("color: rgba(0, 0, 0, -1)", ValueError, "alpha is -1, expected an integer in [0, 255]"),
        ("color: hsv(360, 0.5, 0.5)", ValueError, "hsv hue is 360, expected degrees in [0, 360)"),
        ("color: hsv(10, 1.5, 0.5)", ValueError, "saturation is 1.5, expected a number in [0, 1]"),
        ("color: hsv(10, 0.5, -0.1)", ValueError, "hsv value is -0.1, expected a number in [0, 1]"),
        # The grammar refuses these: a field stated twice, an unknown role, neither six nor eight
        # hex digits, fractional RGB channels, and a bare vector that names no scheme.
        ("color: #E63333 color: #33B34D", TextXSyntaxError, ""),
        ("role: decorative", TextXSyntaxError, ""),
        ("color: #12345", TextXSyntaxError, ""),
        ("color: rgb(1.5, 0, 0)", TextXSyntaxError, ""),
        ("color: (0.9, 0.2, 0.2, 1.0)", TextXSyntaxError, ""),
    ],
)
def test_bad_shape_spec_is_rejected(spec, error, message):
    with pytest.raises(error, match=re.escape(message)):
        _shape_graph(f"shape cube {{ frame: <f> unit: m {spec} }}")


def test_shape_on_no_frame_of_the_body_is_not_its_shape():
    body, uri, graph = _shape_graph("shape sphere { frame: <f> unit: m }")
    graph.remove((uri, URI_SHAPES_PRED_ATTACHED_TO_FRAME, None))
    assert RigidBodyModel(body, graph).shapes == []
    # Read on its own, a shape without a frame is reported, not placed at some origin.
    with pytest.raises(ConstraintViolation, match="attached to no frame"):
        ShapeModel(uri, graph)


def test_file_shape_reads_the_file_its_path_names(tmp_path):
    part = tmp_path / "meshes" / "part.obj"
    part.parent.mkdir()
    part.write_text("")
    spec = 'shape file { frame: <f> sys path = "meshes/part.obj" unit: mm scale: 2 }'
    body, _, graph = _shape_graph(spec, tmp_path / "scene.scenex")

    [shape] = RigidBodyModel(body, graph).shapes
    assert (shape.primitive, shape.path) == (None, part.resolve())
    assert (shape.scale, shape.unit) == ((2.0, 2.0, 2.0), URI_QUDT_UNIT_MM)


def test_missing_shape_file_is_reported(tmp_path):
    spec = 'shape file { frame: <f> sys path = "meshes/part.obj" unit: mm }'
    body, _, graph = _shape_graph(spec, tmp_path / "scene.scenex")
    with pytest.raises(ConstraintViolation, match="no file"):
        RigidBodyModel(body, graph)


@pytest.mark.parametrize(
    ("break_graph", "message"),
    [
        (lambda g, s: g.remove((s, RDF.type, URI_SHAPES_TYPE_SHAPE)), "is not typed Shape"),
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
        (
            lambda g, s: [g.remove((s, RDF.type, role.value)) for role in ShapeRole],
            "needs a role: VisualShape, CollisionShape or both",
        ),
    ],
)
def test_malformed_shape_is_reported(break_graph, message):
    body, uri, graph = _shape_graph("shape cube { frame: <f> unit: cm scale: (1, 2, 3) }")
    break_graph(graph, uri)
    with pytest.raises(ConstraintViolation, match=re.escape(message)):
        RigidBodyModel(body, graph)
