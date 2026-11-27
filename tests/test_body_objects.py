# SPDX-License-Identifier: MPL-2.0
"""Surface friction on collision shapes, and objects the scene builds from one of its bodies."""

import os
import re
from pathlib import Path

import pytest
from rdf_utils.constraints import ConstraintViolation, check_shacl_constraints
from rdf_utils.models.vocab import (
    URI_ENV_PRED_HAS_OBJ_MODEL,
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_MAPS,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_M,
    URI_SHAPES_TYPE_COLLISION_SHAPE,
)
from rdflib import RDF
from textx.exceptions import TextXSemanticError, TextXSyntaxError

from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.kinematics import RigidBodyModel
from scene_dsl.rdf_parser.shapes import ShapeModel
from scene_dsl.rdf_parser.vocab import (
    URI_CONTACT_PRED_HAS_SURFACE_FRICTION,
    URI_CONTACT_PRED_SLIDING,
    URI_CONTACT_PRED_TORSIONAL,
    URI_CONTACT_TYPE_COULOMB_FRICTION,
    URI_CONTACT_TYPE_SURFACE_FRICTION,
)

from .test_common import write_example_scene
from .test_shapes import _shape_graph


@pytest.mark.parametrize(
    ("spec", "read"),
    [
        ("friction { sliding: 0.8 }", (0.8, None, None, None)),
        (
            "friction { sliding: 0.8 torsional: 0.01 m rolling: 0.0001 m }",
            (0.8, 0.01, 0.0001, URI_QUDT_UNIT_M),
        ),
        # Units are kept as authored, never rescaled.
        ("friction { sliding: 0 rolling: 0.01 cm }", (0.0, None, 0.01, URI_QUDT_UNIT_CM)),
    ],
)
def test_surface_friction_reads_back(spec, read):
    body, _, graph = _shape_graph(f"shape cube {{ frame: <f> unit: m role: collision {spec} }}")
    [shape] = RigidBodyModel(body, graph).shapes
    friction = shape.friction
    assert (friction.sliding, friction.torsional, friction.rolling, friction.unit) == read


def test_shape_without_friction_has_none():
    body, _, graph = _shape_graph("shape cube { frame: <f> unit: m }")
    assert RigidBodyModel(body, graph).shapes[0].friction is None


@pytest.mark.parametrize(
    ("spec", "error", "message"),
    [
        (
            "role: visual friction { sliding: 0.8 }",
            ValueError,
            "cube on 'f' states friction but is no collision shape",
        ),
        (
            "friction { sliding: -0.1 }",
            ValueError,
            "cube on 'f' friction sliding is -0.1, expected finite and >= 0",
        ),
        (
            "friction { sliding: 0.8 torsional: 0.01 m rolling: 0.1 mm }",
            ValueError,
            "cube on 'f' friction states torsional and rolling in ['m', 'mm']",
        ),
        # A torsional or rolling coefficient is a length: the grammar refuses one with no unit,
        # and a friction block with no sliding coefficient.
        ("friction { sliding: 0.8 torsional: 0.01 }", TextXSyntaxError, ""),
        ("friction { torsional: 0.01 m }", TextXSyntaxError, ""),
    ],
)
def test_bad_surface_friction_is_rejected(spec, error, message):
    with pytest.raises(error, match=re.escape(message)):
        _shape_graph(f"shape cube {{ frame: <f> unit: m {spec} }}")


def test_friction_on_a_shape_that_does_not_collide_is_reported():
    _, uri, graph = _shape_graph("shape cube { frame: <f> unit: m friction { sliding: 0.8 } }")
    graph.remove((uri, RDF.type, URI_SHAPES_TYPE_COLLISION_SHAPE))
    with pytest.raises(ConstraintViolation, match="states friction but is no CollisionShape"):
        ShapeModel(uri, graph)


def test_friction_without_a_friction_model_is_reported():
    _, uri, graph = _shape_graph("shape cube { frame: <f> unit: m friction { sliding: 0.8 } }")
    friction = graph.value(uri, URI_CONTACT_PRED_HAS_SURFACE_FRICTION)
    graph.remove((friction, RDF.type, URI_CONTACT_TYPE_COULOMB_FRICTION))
    with pytest.raises(ConstraintViolation, match="is not a CoulombFriction"):
        ShapeModel(uri, graph)


def test_friction_lengths_without_a_unit_are_reported():
    _, uri, graph = _shape_graph(
        "shape cube { frame: <f> unit: m friction { sliding: 0.8 torsional: 0.01 m } }"
    )
    friction = graph.value(uri, URI_CONTACT_PRED_HAS_SURFACE_FRICTION)
    graph.remove((friction, URI_QUDT_PRED_UNIT, None))
    with pytest.raises(ConstraintViolation, match="no supported length unit"):
        ShapeModel(uri, graph)


CONTACT_SHACL = Path("newtonian-rigid-body-dynamics") / "contact.shacl.ttl"


def _contact_shapes() -> Path | None:
    """The contact metamodel's shapes in a local metamodels checkout, if there is one: the one
    `METAMODELS_PATH` names, else a `metamodels` folder above this one.

    It is not published yet, so rdf-utils' resolver cannot fetch it.
    """
    if (named := os.environ.get("METAMODELS_PATH")) is not None:
        candidate = Path(named) / CONTACT_SHACL
        return candidate if candidate.is_file() else None
    for root in Path(__file__).resolve().parents:
        candidate = root / "metamodels" / CONTACT_SHACL
        if candidate.is_file():
            return candidate
    return None


@pytest.mark.skipif(
    _contact_shapes() is None, reason="no metamodels checkout with contact.shacl.ttl"
)
@pytest.mark.parametrize(
    "spec",
    [
        "role: collision friction { sliding: 0.8 torsional: 0.01 m rolling: 0.0001 m }",
        "role: collision friction { sliding: 0.8 }",
    ],
)
def test_emitted_friction_conforms_to_the_contact_shapes(spec):
    _, _, graph = _shape_graph(f"shape cube {{ frame: <f> unit: m {spec} }}")
    # Raises with the report when it does not conform.
    assert check_shacl_constraints(graph, {str(_contact_shapes()): "turtle"})


@pytest.mark.skipif(
    _contact_shapes() is None, reason="no metamodels checkout with contact.shacl.ttl"
)
@pytest.mark.parametrize(
    "break_graph",
    [
        # No friction model.
        lambda g, f: g.remove((f, RDF.type, URI_CONTACT_TYPE_COULOMB_FRICTION)),
        # No sliding coefficient.
        lambda g, f: g.remove((f, URI_CONTACT_PRED_SLIDING, None)),
        # A torsional length with no unit.
        lambda g, f: g.remove((f, URI_QUDT_PRED_UNIT, None)),
        # Friction on a shape that does not collide.
        lambda g, f: g.remove((None, RDF.type, URI_SHAPES_TYPE_COLLISION_SHAPE)),
        # A second torsional coefficient.
        lambda g, f: g.add((f, URI_CONTACT_PRED_TORSIONAL, g.value(f, URI_CONTACT_PRED_SLIDING))),
    ],
)
def test_malformed_friction_violates_the_contact_shapes(break_graph):
    _, uri, graph = _shape_graph(
        "shape cube { frame: <f> unit: m friction { sliding: 0.8 torsional: 0.01 m } }"
    )
    friction = graph.value(uri, URI_CONTACT_PRED_HAS_SURFACE_FRICTION)
    assert (friction, RDF.type, URI_CONTACT_TYPE_SURFACE_FRICTION) in graph
    assert (friction, RDF.type, URI_CONTACT_TYPE_COULOMB_FRICTION) in graph
    break_graph(graph, friction)
    assert not check_shacl_constraints(graph, {str(_contact_shapes()): "turtle"}, quiet=True)


def _body_scene(tmp_path: Path, objects: str) -> Path:
    write_example_scene(tmp_path)
    path = tmp_path / "scene.scenex"
    path.write_text(
        f"""import "example.scene"
ns n = "https://example.test/"
ktree (ns=n) world_tree {{ root: <world_body.world>
    body world_body {{ frame world {{ }} }}
    body table_body {{ frame table_root {{ }} }}
    joints {{
        fixed table_on_world {{ parent: <world_body.world> child: <table_body.table_root> }}
    }}
}}
scene inst (ns=n) s_inst {{
    scene: <s>
    kgraph (ns=n) g {{ anchor: <world_tree.world_body.world>
        tree <world_tree>
        body cup_body {{
            frame cup_root {{ }}
            shape cylinder {{
                frame: <cup_root> unit: m scale: (0.04, 0.04, 0.1)
                friction {{ sliding: 0.8 }}
            }}
            inertia {{
                frame: <cup_root> mass: 0.2 kg
                inertia-matrix: ((0.001, 0.0, 0.0), (0.0, 0.001, 0.0), (0.0, 0.0, 0.001)) kg*m^2
            }}
        }}
    }}
    {objects}
}}
"""
    )
    return path


def test_object_built_from_a_body_maps_it_with_no_model(tmp_path):
    path = _body_scene(tmp_path, "obj <objs.cup> { body <g.cup_body> }")
    model = scenex_metamodel().model_from_file(path)
    modelled = model.scene_insts[0].modelled_objs[0]
    [ref] = modelled.models
    graph = create_scenex_model_graph(model)

    # The object maps its body itself; no object model stands between them.
    assert list(graph.objects(modelled.modelled_uri, URI_ENV_PRED_HAS_OBJ_MODEL)) == []
    [mapping] = graph.objects(modelled.modelled_uri, URI_EXEC_PRED_HAS_MAPPING)
    assert mapping == ref.mapping_uri
    assert list(graph.objects(mapping, URI_EXEC_PRED_MAPS)) == [ref.body.uri]


@pytest.mark.parametrize(
    ("objects", "message"),
    [
        (
            'obj <objs.cup> { body <g.cup_body> model cup_model as urdf { sys path = "cup.urdf" } }',
            "object 'cup' is built from body 'cup_body', so it takes no model besides",
        ),
        (
            "obj <objs.cup> { body <world_tree.table_body> }",
            "not a body of scene instance 's_inst''s kinematic graph",
        ),
        (
            (
                'obj <objs.bowl> { model bowl_model as urdf { sys path = "bowl.urdf" '
                "map body <g.cup_body> } }\n    obj <objs.cup> { body <g.cup_body> }"
            ),
            "which a model already maps",
        ),
    ],
)
def test_bad_body_object_is_rejected(tmp_path, objects, message):
    path = _body_scene(tmp_path, objects)
    with pytest.raises(TextXSemanticError, match=re.escape(message)):
        scenex_metamodel().model_from_file(path)
