import pytest
from bdd_dsl.models.urirefs import URI_EXEC_PRED_PATH
from rdflib import Literal, Namespace

from scene_dsl.langs import scene_metamodel, scenex_metamodel
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import (
    URI_EXEC_PRED_MODEL_ENTITY,
    URI_USD_STAGE,
    create_scenex_model_graph,
)
from scene_dsl.rdf_parser.scenex import SceneInstanceModel


from .test_common import MODELS_DIR


def test_scene_parses_and_generates_rdf():
    model = scene_metamodel().model_from_file(MODELS_DIR / "lab.scene")

    assert len(model.scene_models) > 0
    assert len(create_scene_model_graph(model)) > 0


def test_scenex_references_scene_and_generates_rdf():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    assert len(model.scene_insts) > 0
    graph = create_scenex_model_graph(model)
    assert len(graph) > 0


def test_scenex_accepts_scene_level_usd_model():
    model = scenex_metamodel().model_from_str(
        """import "lab.scene"
scene inst (ns=scene_lab_mjc) usd_scene {
    scene: <pickplace_scene>
    model usd_stage as usd { sys path = "/tmp/scene.usda" }
}
""",
        file_name=str(MODELS_DIR / "usd_scene.scenex"),
    )

    scene_instance = model.scene_insts[0]
    resource = scene_instance.models[0]
    assert resource.model_kind == "usd"
    assert resource.model_spec.path == "/tmp/scene.usda"

    parsed = SceneInstanceModel(scene_instance.uri, create_scenex_model_graph(model))
    [resource] = parsed.models.values()
    assert URI_USD_STAGE in resource.types
    assert resource.get_attr(URI_EXEC_PRED_PATH) == "/tmp/scene.usda"


def test_shared_workspace_composition_is_rejected(tmp_path):
    model_path = tmp_path / "shared_workspace.scene"
    model_path.write_text(
        """ns n = "https://example.test/"

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


def test_model_entity_names_body_inside_a_scene_file(tmp_path):
    """One scene file may hold many bodies: 'entity' says which one a model stands for."""
    (tmp_path / "objects.scene").write_text(
        """ns n = "https://example.test/"

obj set (ns=n) objs { object table, object box }
scene (ns=n) lab { obj set <objs> }
"""
    )
    model_path = tmp_path / "lab.scenex"
    model_path.write_text(
        """import "objects.scene"

ns nx = "https://example.test/x/"

scene inst (ns=nx) si {
    scene: <lab>
    kgraph (ns=nx) kg {
        body table_body { frame table_root { } }
        body box_body { frame box_root { } }
    }
    obj <objs.table> {
        model table-in-scene as mjcf { sys path = 'lab_scene.xml'
            entity: "table"
        }
        body: <kg.table_body>
    }
    obj <objs.box> {
        model box-in-scene as mjcf { sys path = 'lab_scene.xml'
            entity: "box"
        }
        body: <kg.box_body>
    }
}
"""
    )

    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(model_path))

    nx = Namespace("https://example.test/x/")
    assert (nx["table-in-scene"], URI_EXEC_PRED_MODEL_ENTITY, Literal("table")) in graph
    assert (nx["box-in-scene"], URI_EXEC_PRED_MODEL_ENTITY, Literal("box")) in graph
    # A model without 'entity' says nothing about one.
    assert (nx["table-in-scene"], URI_EXEC_PRED_PATH, Literal("lab_scene.xml")) in graph
