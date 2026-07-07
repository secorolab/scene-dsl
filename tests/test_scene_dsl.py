from pathlib import Path

import pytest
from rdflib import RDF

from scene_dsl.langs import scene_metamodel, scenex_metamodel
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import (
    URI_DYN_TYPE_MASS_SCALAR,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_QUDT_PRED_UNIT,
    MASS_UNITS,
    LENGTH_UNITS,
    create_scenex_model_graph,
)

MODELS_DIR = Path(__file__).parents[1] / "examples" / "models"


def test_scene_parses_and_generates_rdf():
    model = scene_metamodel().model_from_file(MODELS_DIR / "lab.scene")

    assert model.scene_models
    assert model.sim_obj_sets
    assert len(create_scene_model_graph(model)) > 0


def test_scenex_references_scene_and_generates_rdf():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_inst = model.scene_insts[0]
    table_obj = next(obj for obj in scene_inst.modelled_objs if obj.geometry.name == "table_geom")

    assert scene_inst.scene.name == "pickplace_scene"
    assert table_obj.body.frame.name == "table_root"
    assert table_obj.body.mass.value == 10.0
    assert table_obj.body.mass.unit == "kg"

    graph = create_scenex_model_graph(model)
    assert (table_obj.body.uri, RDF.type, URI_GEOM_TYPE_RIGID_BODY) in graph
    assert (table_obj.body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MASS_SCALAR) in graph


def test_shared_workspace_composition_is_rejected(tmp_path):
    model_path = tmp_path / "shared_workspace.scene"
    model_path.write_text(
        """ns n = \"https://example.test/\"

obj set (ns=n) objs { object cup }
ws set (ns=n) wss { workspace root, workspace a, workspace b, workspace shared }
agn set (ns=n) agns { agent robot }

comp (ns=n) shared_comp of ws <wss.shared> {
    obj <objs.cup>
}
comp (ns=n) a_comp of ws <wss.a> {
    ws comp <shared_comp>
}
comp (ns=n) b_comp of ws <wss.b> {
    ws comp <shared_comp>
}
comp (ns=n) root_comp of ws <wss.root> {
    ws comp <a_comp>
    ws comp <b_comp>
}
scene (ns=n) dag_scene {
    obj set <objs>
    ws set <wss>
    ws comp <root_comp>
    agn set <agns>
}
"""
    )

    model = scene_metamodel().model_from_file(model_path)
    with pytest.raises(RuntimeError, match="Shared or cyclic workspace compositions"):
        create_scene_model_graph(model)


def _write_collision_scene(tmp_path: Path) -> None:
    (tmp_path / "collision.scene").write_text(
        """ns n = \"https://example.test/\"
obj set (ns=n) objs { object cup, object bowl }
ws set (ns=n) wss { workspace table }
agn set (ns=n) agns { agent robot }
scene (ns=n) s {
    obj set <objs>
    ws set <wss>
    agn set <agns>
}
"""
    )


def test_duplicate_geometry_uri_is_rejected(tmp_path):
    _write_collision_scene(tmp_path)
    model_path = tmp_path / "duplicate_geometry.scenex"
    model_path.write_text(
        """import \"collision.scene\"
ns n = \"https://example.test/\"
scene inst (ns=n) sx {
    scene: <s>
    obj <objs.cup> {
        model cup_model as urdf { sys path = \"cup.urdf\" }
        geom dup { root: frame root }
    }
    obj <objs.bowl> {
        model bowl_model as urdf { sys path = \"bowl.urdf\" }
        geom dup { root: frame root }
    }
}
"""
    )

    model = scenex_metamodel().model_from_file(model_path)
    with pytest.raises(ValueError, match="Duplicate model URI"):
        create_scenex_model_graph(model)


def test_duplicate_body_uri_is_rejected(tmp_path):
    _write_collision_scene(tmp_path)
    model_path = tmp_path / "duplicate_body.scenex"
    model_path.write_text(
        """import \"collision.scene\"
ns n = \"https://example.test/\"
scene inst (ns=n) sx {
    scene: <s>
    obj <objs.cup> {
        model cup_model as urdf { sys path = \"cup.urdf\" }
        geom cup_geom { root: frame root }
        body same_body { inertial frame: <cup_geom.root> mass: 1.0 kg }
    }
    obj <objs.bowl> {
        model bowl_model as urdf { sys path = \"bowl.urdf\" }
        geom bowl_geom { root: frame root }
        body same_body { inertial frame: <bowl_geom.root> mass: 2.0 kg }
    }
}
"""
    )

    model = scenex_metamodel().model_from_file(model_path)
    with pytest.raises(ValueError, match="Duplicate model URI"):
        create_scenex_model_graph(model)


@pytest.mark.parametrize(
    ("length_unit", "mass_unit"),
    [("m", "kg"), ("cm", "g"), ("mm", "kg")],
)
def test_scenex_length_and_mass_units(tmp_path, length_unit, mass_unit):
    _write_collision_scene(tmp_path)
    model_path = tmp_path / "units.scenex"
    model_path.write_text(
        f"""import \"collision.scene\"
ns n = \"https://example.test/\"
scene inst (ns=n) sx {{
    scene: <s>
    geom world_geom {{ root: frame world }}
    obj <objs.cup> {{
        model cup_model as urdf {{ sys path = \"cup.urdf\" }}
        geom cup_geom {{
            root: frame root
            pose {{
                xyz: (1.0, 2.0, 3.0)
                {length_unit}
                orientation: euler {{ angles: (0.0, 0.0, 0.0) }}
            }}
        }}
        body cup_body {{ inertial frame: <cup_geom.root> mass: 10.0 {mass_unit} }}
    }}
}}
"""
    )

    model = scenex_metamodel().model_from_file(model_path)
    cup = model.scene_insts[0].modelled_objs[0]
    assert cup.geometry.pose.length_unit == length_unit
    assert cup.body.mass.value == 10.0
    assert cup.body.mass.unit == mass_unit

    graph = create_scenex_model_graph(model)
    assert (
        cup.geometry.pose_coord_uri(model.scene_insts[0].geometry.root),
        URI_QUDT_PRED_UNIT,
        LENGTH_UNITS[length_unit],
    ) in graph
    assert (cup.body.inertia_coord_uri, URI_QUDT_PRED_UNIT, MASS_UNITS[mass_unit]) in graph


def test_scenex_mass_quantity_validation(tmp_path):
    _write_collision_scene(tmp_path)
    model_path = tmp_path / "mass_quantity.scenex"
    model_path.write_text(
        """import \"collision.scene\"
ns n = \"https://example.test/\"
scene inst (ns=n) sx {
    scene: <s>
    obj <objs.cup> {
        model cup_model as urdf { sys path = \"cup.urdf\" }
        geom cup_geom { root: frame root }
        body cup_body { inertial frame: <cup_geom.root> mass: 0.0 kg }
    }
}
"""
    )

    cup = scenex_metamodel().model_from_file(model_path).scene_insts[0].modelled_objs[0]
    assert cup.body.mass.value == 0.0

    model_path.write_text(model_path.read_text().replace("0.0", "-1.0"))
    with pytest.raises(ValueError, match="MassQuantity must have value"):
        scenex_metamodel().model_from_file(model_path)
