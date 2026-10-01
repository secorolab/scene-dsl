from importlib import resources

import pytest
from jinja2 import Environment, FileSystemLoader
from rdf_utils.models.vocab import (
    URI_ACT_PRED_JOINT,
    URI_ACT_PRED_ROTOR_INERTIA,
    URI_QUDT_PRED_QUANTITY_KIND,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_PRED_VALUE,
    URI_QUDT_QK_MOMENT_OF_INERTIA,
    URI_QUDT_UNIT_KG_M2,
)
from rdflib import Literal, URIRef

from scene_dsl.classes.ktree import Actuation
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from .test_common import write_example_scene
from .test_kinematics import _graph
from .test_package_import import _scene_importing

JOINT_1 = URIRef("https://example.test/lab/arm/joint_1")


def _kinova(tmp_path):
    write_example_scene(tmp_path)
    path = tmp_path / "lab.scenex"
    path.write_text(_scene_importing("scene_dsl:robots/kinova_gen3_7dof.ktree"))
    return create_scenex_model_graph(scenex_metamodel().model_from_file(path)), path.parent


def _joints(trees):
    return {
        segment["joint"]["name"].rsplit("/", 1)[-1]: segment["joint"]
        for tree in trees
        for segment in tree["segments"]
        if segment["joint"] and segment["joint"]["axis"] is not None
    }


def test_a_stated_rotor_inertia_is_on_the_joints_actuation(tmp_path):
    graph, _ = _kinova(tmp_path)
    actuation = graph.value(predicate=URI_ACT_PRED_JOINT, object=JOINT_1)
    inertia = graph.value(actuation, URI_ACT_PRED_ROTOR_INERTIA)
    assert graph.value(inertia, URI_QUDT_PRED_VALUE) == Literal(0.558)
    assert graph.value(inertia, URI_QUDT_PRED_UNIT) == URI_QUDT_UNIT_KG_M2
    assert graph.value(inertia, URI_QUDT_PRED_QUANTITY_KIND) == URI_QUDT_QK_MOMENT_OF_INERTIA


def test_each_joint_carries_its_rotor_inertia_into_the_kdl_tree(tmp_path):
    graph, base = _kinova(tmp_path)
    joints = _joints(build_kdl_trees(graph, base))
    inertias = [joints[f"joint_{i}"]["rotor_inertia"] for i in range(1, 8)]
    assert inertias == [0.558] * 4 + [0.1389] * 3


def _header(trees) -> str:
    templates = str(resources.files("scene_dsl") / "templates")
    template = Environment(loader=FileSystemLoader(templates)).get_template("kdl.hpp.jinja2")
    return template.render(data={"name": "scene", "source": "scene.scenex", "trees": trees})


def test_the_kdl_header_passes_it_to_the_joint(tmp_path):
    graph, base = _kinova(tmp_path)
    header = _header(build_kdl_trees(graph, base))
    assert header.count("KDL::Joint::RotAxis, 1.0, 0.0, 0.558)") == 4
    assert header.count("KDL::Joint::RotAxis, 1.0, 0.0, 0.1389)") == 3


def test_a_negative_rotor_inertia_is_rejected():
    with pytest.raises(ValueError, match="Actuation.rotor_inertia must be"):
        Actuation(None, 1.0, ["torque"], ["position"], -0.1, "kg*m^2")


def test_an_unstated_rotor_inertia_is_zero_and_not_in_the_graph(tmp_path):
    graph = _graph(tmp_path)
    assert not list(graph.triples((None, URI_ACT_PRED_ROTOR_INERTIA, None)))
    trees = build_kdl_trees(graph, tmp_path)
    joints = _joints(trees)
    assert joints and all(joint["rotor_inertia"] == 0.0 for joint in joints.values())
    assert "KDL::Joint::RotAxis)" in _header(trees)
