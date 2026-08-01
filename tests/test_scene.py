import pytest
from rdf_utils.models.common import ModelBase
from rdf_utils.models.execution import load_attr_path
from rdf_utils.models.vocab import (
    URI_EXEC_PRED_HAS_CONFIG,
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_MAPS,
    URI_EXEC_PRED_MODEL_ENTITY,
    URI_EXEC_PRED_PATH,
    URI_GEOM_TYPE_KTREE,
    URI_GEOM_TYPE_RIGID_BODY,
)
from rdflib import RDF, Literal, Namespace, URIRef

from scene_dsl.classes.common import IHasNamespace
from scene_dsl.langs import scene_metamodel, scenex_metamodel
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.ktree import RigidBodyModel, get_root_frame
from scene_dsl.rdf_parser.scene import SceneModel
from scene_dsl.rdf_parser.scenex import (
    SceneInstanceModel,
    get_kinematic_mappings,
    get_ros_pkg_path,
    load_attr_kinematic_mappings,
    load_ros_path,
)
from scene_dsl.rdf_parser.vocab import (
    URI_BDD_TYPE_SCENE,
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
    graph = create_scene_model_graph(model)
    parsed = SceneModel(graph, model.scene_models[0].uri)

    assert parsed.objects
    assert parsed.workspaces


def test_scene_parser_collects_agent_uris():
    model = scene_metamodel().model_from_str(
        'ns n="https://example.test/" agn set (ns=n) agents { agent robot } '
        "scene (ns=n) s { agn set <agents> }"
    )
    scene = model.scene_models[0]
    parsed = SceneModel(create_scene_model_graph(model), scene.uri)

    assert parsed.agents == {scene.agn_sets[0].agents[0].uri}


def test_scene_parser_resolves_nested_workspace_compositions():
    model = scene_metamodel().model_from_file(MODELS_DIR / "lab.scene")
    scene = next(scene for scene in model.scene_models if scene.name == "sorting_scene")
    root_comp = scene.ws_comps[0]
    parsed = SceneModel(create_scene_model_graph(model), scene.uri)

    root_ws = parsed.workspaces[root_comp.ws.uri]
    assert root_ws.ws_comps == {comp.uri for comp in root_comp.ws_comps}
    for child_comp in root_comp.ws_comps:
        assert parsed._ws_comps[child_comp.uri] == child_comp.ws.uri
        assert parsed.workspaces[child_comp.ws.uri].objects == {
            obj.uri for obj in child_comp.objects
        }


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


def test_scene_parser_resolves_scene_entity_body_by_name():
    model = scenex_metamodel().model_from_str(
        """import "lab.scene"
scene inst (ns=scene_lab_mjc) usd_scene {
    scene: <pickplace_scene>
    kgraph (ns=scene_lab_mjc) g { body world { frame root { } } }
    model usd_stage as usd { sys path = "/tmp/scene.usda"
        map body <g.world> to "sim-world"
    }
}
""",
        file_name=str(MODELS_DIR / "usd_scene.scenex"),
    )
    graph = create_scenex_model_graph(model)
    parsed = SceneInstanceModel(model.scene_insts[0].uri, graph)

    assert parsed.get_body_for_resource_entity("sim-world", graph).id == URIRef(
        f"{model.scene_insts[0].namespace}g/world"
    )
    assert parsed.get_body_for_resource_entity("missing", graph) is None


def test_scene_instance_accepts_only_its_linked_scene_model():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    graph = create_scenex_model_graph(model)
    scene_instance = model.scene_insts[0]
    scene_model = SceneModel(graph, scene_instance.scene.uri)

    parsed = SceneInstanceModel(
        scene_instance.uri,
        graph,
        scene_model=scene_model,
    )
    assert parsed.scene_model is scene_model

    wrong_scene_id = URIRef("https://example.test/wrong-scene")
    graph.add((wrong_scene_id, RDF.type, URI_BDD_TYPE_SCENE))
    wrong_scene = SceneModel(graph, wrong_scene_id)
    with pytest.raises(ValueError, match="not supplied scene"):
        SceneInstanceModel(
            scene_instance.uri,
            graph,
            scene_model=wrong_scene,
        )


def _parse_example_scene(index=0, loaders=None):
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_instance = model.scene_insts[index]
    return SceneInstanceModel(
        scene_instance.uri,
        create_scenex_model_graph(model),
        loaders=loaders,
    )


def test_scene_parser_loads_modelled_objects_and_agents():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_instance = model.scene_insts[0]
    graph = create_scenex_model_graph(model)
    parsed = SceneInstanceModel(scene_instance.uri, graph)

    assert parsed.models == {}
    assert parsed.scene_model.id == scene_instance.scene.uri
    box = next(
        modelled.obj for modelled in scene_instance.modelled_objs if modelled.obj.name == "box1"
    )
    panda = next(
        modelled.agn for modelled in scene_instance.modelled_agns if modelled.agn.name == "panda"
    )
    assert parsed.object_models[box.uri]
    assert parsed.agent_models[panda.uri]
    assert not hasattr(parsed, "element_loader")
    assert not hasattr(parsed, "mapping_loader")
    assert not hasattr(parsed, "workspace_models")

    ros_model_id = next(graph.subjects(predicate=None, object=URI_ROS_TYPE_PACKAGE))
    ros_model = ModelBase(URIRef(ros_model_id), graph)
    load_attr_path(graph=graph, model=ros_model)
    load_ros_path(graph=graph, model=ros_model)
    assert get_ros_pkg_path(ros_model) == ("test_pkg", "assets/table.xml")


def test_element_configuration_stays_on_its_resource():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_instance = model.scene_insts[0]
    graph = create_scenex_model_graph(model)
    table = next(modelled for modelled in scene_instance.modelled_objs if len(modelled.models) > 1)
    first, second = table.models
    graph.add((first.uri, URI_EXEC_PRED_HAS_CONFIG, Literal('{"enabled": true}')))

    parsed = SceneInstanceModel(scene_instance.uri, graph)
    assert parsed.object_models[table.obj.uri][first.uri].get_attr(URI_EXEC_PRED_HAS_CONFIG) == {
        "enabled": True
    }

    graph.add((second.uri, URI_EXEC_PRED_HAS_CONFIG, Literal("{}")))
    with pytest.raises(ValueError, match="multiple model configurations"):
        SceneInstanceModel(scene_instance.uri, graph)


def test_scene_instances_keep_models_out_of_supplied_scene():
    model = scenex_metamodel().model_from_str(
        """import "lab.scene"
scene inst (ns=scene_lab_mjc) first {
    scene: <pickplace_scene>
    obj <pickplace_objects.box1> {
        model first-box as mjcf { sys path = "/tmp/first-box.xml" }
    }
    obj <ws_objects.container_1> {
        model first-variable as mjcf { sys path = "/tmp/first-variable.xml" }
    }
}
scene inst (ns=scene_lab_mjc) second {
    scene: <pickplace_scene>
    obj <pickplace_objects.box1> {
        model second-box as mjcf { sys path = "/tmp/second-box.xml" }
    }
    obj <ws_objects.container_1> {
        model second-variable as mjcf { sys path = "/tmp/second-variable.xml" }
    }
}
""",
        file_name=str(MODELS_DIR / "isolated_instances.scenex"),
    )
    graph = create_scenex_model_graph(model)
    scene = SceneModel(graph, model.scene_insts[0].scene.uri)
    box_id = model.scene_insts[0].modelled_objs[0].obj.uri
    variable_id = model.scene_insts[0].modelled_objs[1].obj.uri

    first = SceneInstanceModel(model.scene_insts[0].uri, graph, scene_model=scene)
    second = SceneInstanceModel(model.scene_insts[1].uri, graph, scene_model=scene)

    assert box_id in scene.objects
    assert variable_id not in scene.objects
    assert first.object_models[box_id] is not second.object_models[box_id]
    assert first.object_models[variable_id] is not second.object_models[variable_id]
    assert set(first.object_models[box_id]) != set(second.object_models[box_id])
    assert set(first.object_models[variable_id]) != set(second.object_models[variable_id])


def test_scene_parser_loads_mappings_and_resolves_element_roots():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_instance = model.scene_insts[0]
    graph = create_scenex_model_graph(model)
    parsed = SceneInstanceModel(scene_instance.uri, graph)

    resource_ids = set(graph.subjects(predicate=URI_EXEC_PRED_HAS_MAPPING))
    resources = [ModelBase(URIRef(resource_id), graph) for resource_id in resource_ids]
    for resource in resources:
        load_attr_kinematic_mappings(graph, resource)

    box_resource = next(resource for resource in resources if "box1-mjc" in str(resource.id))
    [box_mapping] = get_kinematic_mappings(box_resource, URI_GEOM_TYPE_RIGID_BODY)
    assert box_mapping.entity == "cube"

    box = next(
        modelled.obj for modelled in scene_instance.modelled_objs if modelled.obj.name == "box1"
    )
    panda = next(
        modelled.agn for modelled in scene_instance.modelled_agns if modelled.agn.name == "panda"
    )
    box_resources = parsed.object_models[box.uri].values()
    [box_mapping] = [
        mapping
        for resource in box_resources
        for mapping in get_kinematic_mappings(resource, URI_GEOM_TYPE_RIGID_BODY)
    ]
    box_body = RigidBodyModel(box_mapping.target_id, graph)
    assert str(box_body.root_frame.id).endswith("lab_graph/box1_body/box1_root")
    [panda_mapping] = [
        mapping
        for resource in parsed.agent_models[panda.uri].values()
        for mapping in get_kinematic_mappings(resource, URI_GEOM_TYPE_KTREE)
    ]
    panda_root = get_root_frame(panda_mapping.target_id, graph)
    assert str(panda_root.id).endswith("panda_tree/panda_base_body/base_link")

    arm_gripper = next(
        modelled.agn
        for modelled in scene_instance.modelled_agns
        if modelled.agn.name == "arm1_gripper"
    )
    assert (
        len(
            [
                mapping
                for resource in parsed.agent_models[arm_gripper.uri].values()
                for mapping in get_kinematic_mappings(resource, URI_GEOM_TYPE_KTREE)
            ]
        )
        > 1
    )


def test_scene_parser_rejects_mapping_without_target():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_instance = model.scene_insts[0]
    graph = create_scenex_model_graph(model)
    mapping_id = next(graph.objects(predicate=URI_EXEC_PRED_HAS_MAPPING))
    graph.remove((mapping_id, URI_EXEC_PRED_MAPS, None))

    with pytest.raises(ValueError, match="does not map to an URI"):
        SceneInstanceModel(scene_instance.uri, graph)


def test_modelled_resources_use_custom_loaders_in_order():
    calls = []

    def first(graph, model, **kwargs):
        calls.append(("first", model.id))

    def second(graph, model, **kwargs):
        calls.append(("second", model.id))

    parsed = _parse_example_scene(loaders=[first, second])

    assert calls[::2] == [("first", model_id) for _, model_id in calls[1::2]]
    assert all(name == "second" for name, _ in calls[1::2])
    loaded_ids = {model_id for _, model_id in calls}
    assert all(set(resources) <= loaded_ids for resources in parsed.object_models.values())
    assert all(set(resources) <= loaded_ids for resources in parsed.agent_models.values())
    assert all(
        not resource.has_attr(URI_EXEC_PRED_HAS_MAPPING)
        for resources in (*parsed.object_models.values(), *parsed.agent_models.values())
        for resource in resources.values()
    )


def test_object_set_without_body_mappings_returns_none():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_instance = model.scene_insts[1]
    graph = create_scenex_model_graph(model)
    parsed = SceneInstanceModel(scene_instance.uri, graph)

    assert parsed.models == {}
    assert all(
        not any(
            get_kinematic_mappings(resource, URI_GEOM_TYPE_RIGID_BODY)
            for resource in parsed.object_models[modelled.obj.uri].values()
        )
        for modelled in scene_instance.modelled_objs
    )


def test_scene_parser_accepts_scene_without_models():
    model = scenex_metamodel().model_from_str(
        """import "lab.scene"
scene inst (ns=scene_lab_mjc) empty_scene {
    scene: <pickplace_scene>
}
""",
        file_name=str(MODELS_DIR / "empty_scene.scenex"),
    )

    graph = create_scenex_model_graph(model)
    parsed = SceneInstanceModel(model.scene_insts[0].uri, graph)

    assert parsed.models == {}
    assert parsed.object_models == {}
    assert parsed.agent_models == {}


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
