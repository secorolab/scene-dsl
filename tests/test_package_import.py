import re
from importlib import resources
from pathlib import Path

import pytest
from rdflib import URIRef

from scene_dsl.langs import package_import, scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from .test_common import write_example_scene

SHIPPED_ROBOTS = ["eddie_base", "kinova_gen3_7dof", "robotiq_2f85", "ur10e"]


def _scene_importing(uri: str) -> str:
    return f"""import "example.scene"
import "{uri}"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm of <kinova_tree>
scene inst (ns=lab) lab {{
    scene: <s>
    kgraph (ns=lab) lab_graph {{ anchor: <arm.base_link.base_link_origin>
        tree <arm>
    }}
}}
"""


def test_an_import_without_the_prefix_stays_relative():
    assert package_import("../device.ktree") == "../device.ktree"


def test_a_prefix_naming_no_installed_package_stays_as_written():
    assert package_import("no_such_package:robots/x.ktree") == "no_such_package:robots/x.ktree"


def test_any_installed_package_can_ship_the_file():
    resolved = package_import("textx:__init__.py")
    assert Path(resolved) == Path(str(resources.files("textx") / "__init__.py"))


@pytest.mark.parametrize("robot", SHIPPED_ROBOTS)
def test_a_prefixed_import_names_the_file_the_package_ships(robot):
    resolved = Path(package_import(f"scene_dsl:robots/{robot}.ktree"))
    assert resolved == Path(str(resources.files("scene_dsl") / "robots" / f"{robot}.ktree"))
    assert resolved.is_file()


def test_a_scene_instances_a_shipped_robot(tmp_path):
    write_example_scene(tmp_path)
    path = tmp_path / "lab.scenex"
    path.write_text(_scene_importing("scene_dsl:robots/kinova_gen3_7dof.ktree"))

    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(path))
    assert URIRef("https://example.test/lab/arm/base_link/base_link_origin") in graph.subjects()


def test_a_shipped_file_that_does_not_exist_names_where_it_was_looked_for(tmp_path):
    write_example_scene(tmp_path)
    path = tmp_path / "lab.scenex"
    path.write_text(_scene_importing("scene_dsl:robots/no_such_robot.ktree"))

    expected = str(resources.files("scene_dsl") / "robots" / "no_such_robot.ktree")
    with pytest.raises(OSError, match=re.escape(expected)):
        scenex_metamodel().model_from_file(path)
