# SPDX-License-Identifier: MPL-2.0
"""The kinematics read back out of the graph a scene model emits."""

import json
from pathlib import Path

import numpy as np
import pytest
from jinja2 import Environment, FileSystemLoader
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_DYN_PRED_OF_BODY,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_PRED_JOINTS,
    URI_KC_TYPE_JOINT,
)
from rdflib import RDF, URIRef

from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.kinematics import (
    RevoluteJointModel,
    kinematic_graphs,
    kinematic_trees,
)

from .test_common import write_example_scene

# An arm carrying a tool: two trees, and a third composing them over a fixed joint.
SCENEX = """import "example.scene"

ns n = "https://example.test/"

ktree (ns=n) arm {{
    body base {{
        frame base_origin {{ }}
        frame j1_anchor {{
            pose j1_anchor_in_base {{
                wrt: <base.base_origin>
                xyz: (0, 0, 0.15) m
                orientation: euler {{ axes: xyz extrinsic angles: (180, 0, 0) unit: deg }}
            }}
        }}
    }}
    body link1 {{
        frame link1_origin {{ }}
        frame tcp {{
            pose tcp_in_link1 {{
                wrt: <link1.link1_origin>
                xyz: (0, 0, 0.4) m
                orientation: euler {{ angles: (0, 0, 0) unit: rad }}
            }}
        }}
    }}
    joints {{
        revolute j1 {{
            parent: <base.j1_anchor>.z
            child: <link1.link1_origin>.z
            polarity: PositivePolarity
        }}
        serial {{ root: <base.base_origin> tip: <link1.tcp> }}
    }}
}}

ktree (ns=n) tool {{
    body tool_base {{
        frame tool_origin {{ }}
        inertia {{
            frame: <tool_origin>
            mass: 0.5 kg
            inertia-matrix: ((0.002, 0.0, 0.0), (0.0, 0.003, 0.0), (0.0, 0.0, 0.004)) kg*m^2
        }}
    }}
    body tool_tip {{
        frame tool_tip_origin {{
            pose tip_in_tool {{
                wrt: <tool_base.tool_origin>
                xyz: (0, 0, 0.05) m
                orientation: euler {{ angles: (0, 0, 0) unit: rad }}
            }}
        }}
        inertia {{
            frame: <tool_tip_origin>
            mass: 0.1 kg
            inertia-matrix: ((1e-4, 0.0, 0.0), (0.0, 1e-4, 0.0), (0.0, 0.0, 1e-4)) kg*m^2
        }}
    }}
    joints {{
        fixed tip_on_base {{
            parent: <tool_base.tool_origin>
            child: <tool_tip.tool_tip_origin>
        }}
    }}
}}

ktree (ns=n) arm_tool {{
    tree <arm>
    tree <tool>
    joints {{
        fixed tool_on_arm {{
            parent: <arm.link1.tcp>
            child: <tool.tool_base.tool_origin>
        }}
        serial {{ root: <arm.base.base_origin> tip: <tool.tool_tip.tool_tip_origin> }}
    }}
}}

scene inst (ns=n) sx {{
    scene: <s>

    kgraph (ns=n) lab {{ tree <arm_tool> }}

    model arm_model as {kind} {{
        sys path = 'arm.model'
        map tree <arm> to "arm"
    }}
}}
"""

MJCF = """<mujoco model="arm">
  <compiler angle="degree" eulerseq="xyz"/>
  <worldbody>
    <body name="link1">
      <inertial mass="0.75" pos="0 0 0.2" euler="0 0 90" diaginertia="0.004 0.005 0.0006"/>
    </body>
  </worldbody>
</mujoco>
"""

URDF = """<robot name="arm">
  <link name="link1">
    <inertial>
      <mass value="0.75"/>
      <origin xyz="0 0 0.2" rpy="0 0 1.5707963267948966"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0006"/>
    </inertial>
  </link>
</robot>
"""


def _graph(tmp_path: Path, kind: str = "mjcf", model: str = MJCF):
    """The RDF the scene model emits."""
    write_example_scene(tmp_path)
    (tmp_path / "arm.model").write_text(model)
    (tmp_path / "sx.scenex").write_text(SCENEX.format(kind=kind))
    scenex = scenex_metamodel().model_from_file(str(tmp_path / "sx.scenex"))
    return create_scenex_model_graph(model=scenex)


def _parse(tmp_path: Path, kind: str = "mjcf", model: str = MJCF):
    """The trees of the scene, and the graph they were read from."""
    graph = _graph(tmp_path, kind, model)
    return kinematic_trees(graph, tmp_path), graph


def _name(uri: URIRef | None) -> str:
    return str(uri).rsplit("/", 1)[-1]


def _scoped(uri: URIRef) -> str:
    """The tree-qualified name, since every tree names its chain "chain"."""
    return "/".join(str(uri).rsplit("/", 2)[-2:])


def _body(tree, name: str) -> URIRef:
    [uri] = [body for body in tree.bodies if _name(body) == name]
    return uri


def test_only_the_outermost_tree_at_a_root_is_built(tmp_path):
    """The arm and the composing tree are rooted at one body; the composing one holds more."""
    [tree], _ = _parse(tmp_path)

    assert _name(tree.id) == "arm_tool"
    assert _name(tree.root) == "base"
    assert {_name(body) for body in tree.bodies} == {"base", "link1", "tool_base", "tool_tip"}


def test_reaching_a_trees_root_brings_in_its_joints(tmp_path):
    """Nothing states composition: the tool is walked because the fixed joint reaches its root."""
    [tree], _ = _parse(tmp_path)

    assert {_name(joint) for joint in tree.joints} == {"j1", "tool_on_arm", "tip_on_base"}
    assert _name(tree.parent[_body(tree, "tool_base")]) == "link1"
    assert _name(tree.parent_joint[_body(tree, "tool_tip")]) == "tip_on_base"
    # Root outwards: a body never comes before the one carrying it.
    order = [_name(body) for body in tree.topological_order]
    assert order[0] == "base"
    assert order.index("link1") < order.index("tool_base") < order.index("tool_tip")


def test_a_body_belongs_to_the_tree_that_describes_it(tmp_path):
    """What a model file's mapping is written against: the tree, not the one composing it."""
    [tree], _ = _parse(tmp_path)

    trees = {_name(body): _name(tree.defining_tree_by_body[body]) for body in tree.bodies}
    assert trees == {"base": "arm", "link1": "arm", "tool_base": "tool", "tool_tip": "tool"}


def test_a_chain_is_ordered_root_outwards(tmp_path):
    """The graph states a chain's joints as a set; the tree carrying them says in what order."""
    [tree], _ = _parse(tmp_path)
    paths = {_scoped(chain.id): [_name(j) for j in tree.path(chain)] for chain in tree.chains}

    assert paths == {
        "arm/chain": ["j1"],  # the arm's own chain, which lies inside the composing tree
        "arm_tool/chain": ["j1", "tool_on_arm", "tip_on_base"],
    }


def test_a_revolute_joint_states_the_axis_its_attachments_share(tmp_path):
    """Both attachments name the same axis letter; the frames differ, so the vectors do."""
    [tree], _ = _parse(tmp_path)
    joint = tree.joints[next(uri for uri in tree.joints if _name(uri) == "j1")]

    assert isinstance(joint, RevoluteJointModel)
    assert set(joint.axes.values()) == {"z"}
    assert joint.axes[joint.frame_on(_body(tree, "base"))] == "z"


def test_a_body_uses_the_inertia_the_scene_states(tmp_path):
    """Stated about the body's own frame here, so it is taken as it stands."""
    [tree], graph = _parse(tmp_path)
    mass = tree.mass_properties(_body(tree, "tool_base"), graph)

    assert mass.mass == pytest.approx(0.5)
    assert np.allclose(mass.tensor, np.diag((0.002, 0.003, 0.004)))
    assert np.allclose(mass.com, (0.0, 0.0, 0.0))


@pytest.mark.parametrize(("kind", "model"), [("mjcf", MJCF), ("urdf", URDF)])
def test_inertia_absent_from_the_scene_is_read_from_the_mapped_file(tmp_path, kind, model):
    """MJCF angles are degrees unless its compiler says otherwise; URDF rpy is always radians."""
    [tree], graph = _parse(tmp_path, kind=kind, model=model)
    mass = tree.mass_properties(_body(tree, "link1"), graph)

    assert mass.mass == pytest.approx(0.75)
    assert np.allclose(mass.com, (0.0, 0.0, 0.2))
    # a quarter turn about z swaps which axis carries which moment
    assert np.allclose(np.diag(mass.tensor), (0.005, 0.004, 0.0006))


def test_a_mapped_file_that_cannot_answer_is_an_error(tmp_path):
    """The arm maps into the file, which describes no body of that name."""
    [tree], graph = _parse(tmp_path)

    with pytest.raises(ConstraintViolation, match="no body named 'base'"):
        tree.mass_properties(_body(tree, "base"), graph)


def test_a_body_nothing_states_an_inertia_for_is_an_error(tmp_path):
    """A body is matter: the tip states none once its own is dropped, and nothing maps it."""
    graph = _graph(tmp_path)
    tip = URIRef("https://example.test/tool/tool_tip")
    graph.remove((None, URI_DYN_PRED_OF_BODY, tip))
    [tree] = kinematic_trees(graph, tmp_path)

    with pytest.raises(ConstraintViolation, match="nothing states the inertia"):
        tree.mass_properties(tip, graph)


def test_a_frame_needs_a_pose_to_be_placed_on_its_body(tmp_path):
    """A frame that is not the body's root is somewhere, and the scene has to say where."""
    [tree], graph = _parse(tmp_path)
    base = tree.bodies[_body(tree, "base")]
    [anchor] = [frame for frame in base.frames if _name(frame) == "j1_anchor"]

    assert np.allclose(base.pose_of(anchor, graph).translation, (0.0, 0.0, 0.15))
    assert base.pose_of(base.root_frame.id, graph).translation.tolist() == [0.0, 0.0, 0.0]
    with pytest.raises(ConstraintViolation, match="no pose places frame"):
        base.pose_of(URIRef("https://example.test/arm/base/nowhere"), graph)


def test_kdl_ir_is_json_and_keeps_chain_endpoint_frames(tmp_path):
    """A KDL chain reaches its named tool frame, rather than stopping at its body root."""
    graph = _graph(tmp_path)
    trees = build_kdl_trees(graph, tmp_path)

    assert json.loads(json.dumps(trees)) == trees
    [tree] = trees
    assert tree["root"] == "arm/base"
    assert tree["cpp_name"] == "arm_tool"
    assert {segment["name"] for segment in tree["segments"]} == {
        "arm/link1",
        "arm/link1/tcp",
        "tool/tool_base",
        "tool/tool_tip",
    }
    link1 = next(segment for segment in tree["segments"] if segment["name"] == "arm/link1")
    assert link1["transform"]["translation"] == pytest.approx((0.0, 0.0, 0.15))
    assert {
        chain["name"]: (chain["root"], chain["tip"], chain["joints"]) for chain in tree["chains"]
    } == {
        "arm/chain": (
            "arm/base",
            "arm/link1/tcp",
            [{"name": "arm/j1", "local_name": "j1"}],
        ),
        "arm_tool/chain": (
            "arm/base",
            "tool/tool_tip",
            [{"name": "arm/j1", "local_name": "j1"}],
        ),
    }
    assert {chain["name"]: chain["cpp_name"] for chain in tree["chains"]} == {
        "arm/chain": "arm_chain",
        "arm_tool/chain": "arm_tool_chain",
    }

    template = Environment(
        loader=FileSystemLoader(Path(__file__).parents[1] / "src" / "scene_dsl" / "templates")
    ).get_template("kdl.hpp.jinja2")
    header = template.render(data={"name": "scene", "source": "scene.scenex", "trees": trees})
    assert 'KDL::Segment("arm/link1/tcp"' in header
    assert 'tree.getChain("arm/base", "arm/link1/tcp", *chain)' in header


def test_a_second_path_to_a_body_is_rejected(tmp_path):
    """A tree reaches each of its bodies one way, so a second joint to one is no tree."""
    graph = _graph(tmp_path)
    ns = "https://example.test/"
    duplicate = URIRef(f"{ns}arm/j1_again")
    graph.add((duplicate, RDF.type, URI_KC_TYPE_JOINT))
    graph.add((duplicate, URI_KC_PRED_BETWEEN_ATTACHMENTS, URIRef(f"{ns}arm/base/j1_anchor")))
    graph.add((duplicate, URI_KC_PRED_BETWEEN_ATTACHMENTS, URIRef(f"{ns}arm/link1/link1_origin")))
    graph.add((URIRef(f"{ns}arm"), URI_KC_PRED_JOINTS, duplicate))

    with pytest.raises(ConstraintViolation, match="cycle or a second path"):
        kinematic_trees(graph, tmp_path)


def test_a_graph_articulating_its_own_bodies_names_that_component(tmp_path):
    """A graph joining bodies of its own names the component they form, no tree rooting
    it. It roots every unattached body, so its joints are reached from one walk of many."""
    write_example_scene(tmp_path)
    (tmp_path / "g.scenex").write_text(
        """import "example.scene"

ns n = "https://example.test/"

scene inst (ns=n) sx {
    scene: <s>
    kgraph (ns=n) g {
        body a { frame a_out { } }
        body b { frame b_in { } }
        body placed { frame placed_root { } }
        joints { fixed a_to_b { parent: <a.a_out> child: <b.b_in> } }
    }
}
"""
    )
    scenex = scenex_metamodel().model_from_file(str(tmp_path / "g.scenex"))
    graph = create_scenex_model_graph(model=scenex)
    [kgraph] = kinematic_graphs(graph, tmp_path)

    [tree] = kgraph.trees
    assert _name(tree.id) == "g"
    assert _name(tree.root) == "a"
    assert _name(tree.root_frame.id) == "a_out"
    assert {_name(body) for body in tree.bodies} == {"a", "b"}

    # A body nothing joins is placed, not articulated, so no component is named for it.
    assert [_name(body) for body in kgraph.free_bodies] == ["placed"]
    assert kgraph.nested_trees == {kgraph.id: ()}


def test_the_generated_header_compiles(tmp_path):
    """The template writes C++, so the check that it is C++ is a compiler.

    Skipped where no compiler or no KDL is installed, which is what the rest of the suite
    assumes: everything above this reads the header as data.
    """
    import shutil
    import subprocess

    compiler = shutil.which("g++") or shutil.which("clang++")
    includes = [Path(base) for base in ("/usr/include", "/usr/local/include")]
    kdl = next((base for base in includes if (base / "kdl" / "tree.hpp").is_file()), None)
    if compiler is None or kdl is None:
        pytest.skip("needs a C++ compiler and the KDL headers")

    graph = _graph(tmp_path)
    env = Environment(
        loader=FileSystemLoader(Path(__file__).parents[1] / "src" / "scene_dsl" / "templates"),
        keep_trailing_newline=True,
    )
    header = tmp_path / "scene.kdl.hpp"
    header.write_text(
        env.get_template("kdl.hpp.jinja2").render(
            data={
                "name": "scene",
                "source": "sx.scenex",
                "trees": build_kdl_trees(graph, tmp_path),
            }
        )
    )
    # Every function is inline, so a translation unit including it twice must still link.
    main = tmp_path / "main.cpp"
    main.write_text(
        '#include "scene.kdl.hpp"\n'
        '#include "scene.kdl.hpp"\n'
        "int main() {\n"
        "  KDL::Tree tree;\n"
        "  KDL::Chain chain;\n"
        "  return scene::make_tree_arm_tool(&tree) &&\n"
        "         scene::make_chain_arm_tool_chain(tree, &chain) &&\n"
        "         scene::kIris.size() > 0 ? 0 : 1;\n"
        "}\n"
    )
    built = subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-I",
            str(kdl),
            str(main),
            "-o",
            str(tmp_path / "main"),
            "-lorocos-kdl",
        ],
        capture_output=True,
        text=True,
        cwd=tmp_path,
        check=False,
    )
    assert built.returncode == 0, built.stderr
    assert subprocess.run([str(tmp_path / "main")], cwd=tmp_path, check=False).returncode == 0
