import pytest
from rdf_utils.models.vocab import (
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_MAPS,
    URI_EXEC_PRED_MODEL_ENTITY,
    URI_EXEC_PRED_PATH,
)
from rdflib import Literal, Namespace

from scene_dsl.classes.common import IHasNamespace
from scene_dsl.langs import scene_metamodel, scenex_metamodel
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.scenex import SceneInstanceModel, get_ros_pkg_path
from scene_dsl.rdf_parser.vocab import (
    URI_ROS_TYPE_PACKAGE,
    URI_USD_STAGE,
)

from .test_common import MODELS_DIR


def test_namespace_must_be_declared_by_the_model_class():
    class MissingNamespace(IHasNamespace):
        pass

    with pytest.raises(NotImplementedError, match="MissingNamespace"):
        _ = MissingNamespace(parent=object()).namespace


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


def _parse_example_scene(index=0, loaders=None):
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_instance = model.scene_insts[index]
    return SceneInstanceModel(
        scene_instance.uri,
        create_scenex_model_graph(model),
        loaders=loaders,
    )


def test_scene_parser_loads_modelled_objects_and_agents():
    parsed = _parse_example_scene()

    assert parsed.models == {}
    assert len(parsed.modelled_objects) == 2
    assert len(parsed.modelled_agents) == 4

    object_models = [
        model for models in parsed.modelled_objects.values() for model in models.values()
    ]
    ros_model = next(model for model in object_models if URI_ROS_TYPE_PACKAGE in model.types)
    assert get_ros_pkg_path(ros_model) == ("test_pkg", "assets/table.xml")


def test_modelled_resources_use_custom_loaders_in_order():
    calls = []

    def first(graph, model, **kwargs):
        calls.append(("first", model.id))

    def second(graph, model, **kwargs):
        calls.append(("second", model.id))

    parsed = _parse_example_scene(loaders=[first, second])
    models = [
        model
        for modelled in (parsed.modelled_objects, parsed.modelled_agents)
        for resources in modelled.values()
        for model in resources.values()
    ]

    assert calls == [(name, model.id) for model in models for name in ("first", "second")]


def test_object_set_wrappers_preserve_shared_model_references():
    parsed = _parse_example_scene(index=1)

    assert parsed.models == {}
    assert parsed.modelled_agents == {}
    assert len(parsed.modelled_objects) == 5
    model_ids = [next(iter(models)) for models in parsed.modelled_objects.values()]
    assert len(set(model_ids)) == 2


def test_scene_parser_accepts_scene_without_models():
    model = scenex_metamodel().model_from_str(
        """import "lab.scene"
scene inst (ns=scene_lab_mjc) empty_scene {
    scene: <pickplace_scene>
}
""",
        file_name=str(MODELS_DIR / "empty_scene.scenex"),
    )

    parsed = SceneInstanceModel(model.scene_insts[0].uri, create_scenex_model_graph(model))

    assert parsed.models == {}
    assert parsed.modelled_objects == {}
    assert parsed.modelled_agents == {}


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


def test_agent_kinematics_attach_to_bodies_in_one_model_file(tmp_path):
    (tmp_path / "robot.scene").write_text(
        """ns n = "https://example.test/"
agn set (ns=n) agents { agent robot }
scene (ns=n) lab { agn set <agents> }
"""
    )
    model_path = tmp_path / "lab.scenex"
    model_path.write_text(
        """import "robot.scene"

ns nx = "https://example.test/x/"
ktree (ns=nx) arm { body arm_base { frame arm_root { } } joints { } }
ktree (ns=nx) gripper { body grip_base { frame grip_root { } } joints { } }

scene inst (ns=nx) si {
    scene: <lab>
    agn <agents.robot> {
        model arm-in-scene as mjcf {sys path = "robot.xml" map tree <arm> to "arm_body" }
        model gripper-in-scene as mjcf {sys path = "robot.xml" map tree <gripper> to "gripper_body" }
    }
}
"""
    )

    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(model_path))

    nx = Namespace("https://example.test/x/")
    # `entity` hangs off the mapping, not the model: one file may map several trees.
    for model_name, tree, entity in (
        ("si/robot/arm-in-scene", nx.arm, "arm_body"),
        ("si/robot/gripper-in-scene", nx.gripper, "gripper_body"),
    ):
        [mapping] = list(graph.objects(nx[model_name], URI_EXEC_PRED_HAS_MAPPING))
        assert (mapping, URI_EXEC_PRED_MAPS, tree) in graph
        assert (mapping, URI_EXEC_PRED_MODEL_ENTITY, Literal(entity)) in graph
    assert (nx["si/robot/arm-in-scene"], URI_EXEC_PRED_PATH, Literal("robot.xml")) in graph
