import re

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_MM,
    URI_SHAPES_PRED_ATTACHED_TO_FRAME,
    URI_SHAPES_TYPE_SCALED_SHAPE,
)
from rdf_utils.namespace import NS_MM_QUDT_UNIT, NS_MM_SHAPES
from rdflib import RDF, XSD, Graph, Literal
from rdflib.namespace import split_uri

from scene_dsl.classes.ktree import PrimitiveShapeType
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf.shapes import add_shape
from scene_dsl.rdf_parser.shapes import scene_shapes

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
    model = scenex_metamodel().model_from_file(str(MODELS_DIR / "kinova_with_objects.scenex"))
    shapes = {split_uri(s.id)[1]: s for s in scene_shapes(create_scenex_model_graph(model))}

    assert {name: s.primitive for name, s in shapes.items()} == {
        "ball": PrimitiveShapeType.SPHERE,
        "cube": PrimitiveShapeType.CUBE,
        "cylinder": PrimitiveShapeType.CYLINDER,
        "static_marker": None,
    }
    ball, cube, cylinder, marker = (
        shapes[n] for n in ("ball", "cube", "cylinder", "static_marker")
    )
    assert (ball.unit, ball.scale) == (URI_QUDT_UNIT_M, (0.07, 0.05, 0.04))
    assert (cube.unit, cube.scale) == (URI_QUDT_UNIT_CM, (7.0, 7.0, 7.0))
    assert (cylinder.unit, cylinder.scale) == (URI_QUDT_UNIT_MM, (50.0, 50.0, 90.0))
    # The unit folded into the scale: a cylinder's radii and height, a mesh's metres per unit.
    assert cylinder.scale_in_metres == pytest.approx((0.05, 0.05, 0.09))
    assert marker.scale_in_metres == pytest.approx((0.05, 0.05, 0.05))
    assert split_uri(cylinder.frame)[1] == "cylinder_center"
    assert marker.path.name == "tetra.stl" and marker.path.is_file()
    assert ball.path is None
    # The marker states no colour, so a consumer keeps its default.
    assert marker.rgba is None
    assert ball.rgba == pytest.approx((230 / 255, 51 / 255, 51 / 255, 1.0))


def test_lab_shapes_read_back_as_authored():
    model = scenex_metamodel().model_from_file(str(MODELS_DIR / "lab.scenex"))
    shapes = {split_uri(s.id)[1]: s for s in scene_shapes(create_scenex_model_graph(model))}

    assert {name: s.primitive for name, s in shapes.items()} == {
        "box1_shape": PrimitiveShapeType.CUBE,
        "static_marker": None,
    }
    box, marker = shapes["box1_shape"], shapes["static_marker"]
    assert box.scale_in_metres == pytest.approx((0.04, 0.04, 0.04))
    assert split_uri(box.frame)[1] == "box1_root"
    assert box.rgba == pytest.approx((0x1A / 255, 0x59 / 255, 1.0, 1.0))
    assert marker.path.name == "tetra.stl" and marker.path.is_file()
    assert split_uri(marker.frame)[1] == "world"


def test_unscaled_shape_has_unit_scale():
    _, graph = _shape_graph("shape sphere ball { frame: <f> unit: m }")
    [shape] = scene_shapes(graph)
    assert shape.scale == (1.0, 1.0, 1.0)


def test_file_shape_reads_the_mesh_its_path_names(tmp_path):
    mesh = tmp_path / "meshes" / "part.obj"
    mesh.parent.mkdir()
    mesh.write_text("")
    spec = 'shape mesh part { frame: <f> sys path = "meshes/part.obj" unit: mm scale: 2 }'
    _, graph = _shape_graph(spec, tmp_path / "scene.scenex")

    [shape] = scene_shapes(graph)
    assert (shape.primitive, shape.path) == (None, mesh.resolve())
    assert shape.scale_in_metres == pytest.approx((0.002, 0.002, 0.002))


def test_missing_mesh_file_is_reported(tmp_path):
    spec = 'shape mesh part { frame: <f> sys path = "meshes/part.obj" unit: mm }'
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
        (lambda g, s: g.add((s, RDF.type, NS_MM_SHAPES.UnitSphere)), "needs one type of"),
        (lambda g, s: g.set((s, URI_QUDT_PRED_UNIT, NS_MM_QUDT_UNIT["IN"])), "m, cm or mm"),
        (lambda g, s: g.set((s, NS_MM_SHAPES["scale-x"], Literal(2))), "needs one xsd:double"),
        (
            lambda g, s: g.set((s, NS_MM_SHAPES["scale-y"], Literal(0.0, datatype=XSD.double))),
            "expected finite positive",
        ),
        (
            lambda g, s: g.remove((s, RDF.type, URI_SHAPES_TYPE_SCALED_SHAPE)),
            "states scale factors but is not a ScaledShape",
        ),
    ],
)
def test_malformed_shape_is_reported(break_graph, message):
    uri, graph = _shape_graph("shape cube box { frame: <f> unit: cm scale: (1, 2, 3) }")
    break_graph(graph, uri)
    with pytest.raises(ConstraintViolation, match=re.escape(message)):
        scene_shapes(graph)
