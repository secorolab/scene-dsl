# SPDX-License-Identifier: MPL-2.0
from pathlib import Path

import numpy as np
import pytest
from rdf_utils.constraints import ConstraintViolation

from scene_dsl.kdl import build_kdl_trees
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

SCENE = """ns t = "https://example.test/"
agn set (ns=t) agents { agent arm }
scene (ns=t) bench { agn set <agents> }
"""

TREE = """import "bench.scene"

ns t = "https://example.test/"

ktree (ns=t) rig {{
    body base {{
        frame base_origin {{ }}
        frame j1_anchor {{
            pose j1_anchor_in_base {{
                wrt: <base.base_origin>
                xyz: (0, 0, 0.15643) m
                orientation: euler {{ axes: xyz extrinsic angles: (180, 0, 0) unit: deg }}
            }}
        }}
    }}
    body link1 {{
        frame link1_origin {{ }}
        frame tcp {{ {tcp_pose} }}
        {inertia}
    }}
    joints {{
        revolute j1 {{
            parent: <base.j1_anchor>.z
            child: <link1.link1_origin>.z
            polarity: PositivePolarity
        }}
        serial {{
            root: <base.base_origin>
            tip: <link1.tcp>
        }}
    }}
}}

scene inst (ns=t) scene_rig {{
    scene: <bench>

    agn <agents.arm> {{
        model arm_model as {kind} {{
            sys path = 'rig.model'
            map tree <rig> to "rig"
        }}
    }}
}}
"""

STATED_INERTIA = """inertia {
            frame: <link1_origin>
            mass: 0.75 kg
            inertia-matrix: ((0.004, 0.0, 0.0), (0.0, 0.005, 0.0), (0.0, 0.0, 0.0006)) kg*m^2
        }"""

MJCF = """<mujoco model="rig">
  <compiler angle="degree" eulerseq="xyz"/>
  <worldbody>
    <body name="link1">
      <inertial mass="0.75" pos="0 0 0.2" euler="0 0 90" diaginertia="0.004 0.005 0.0006"/>
    </body>
  </worldbody>
</mujoco>
"""

URDF = """<robot name="rig">
  <link name="link1">
    <inertial>
      <mass value="0.75"/>
      <origin xyz="0 0 0.2" rpy="0 0 1.5707963267948966"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0006"/>
    </inertial>
  </link>
</robot>
"""


TCP_POSE = """pose tcp_in_link1 {
                wrt: <link1.link1_origin>
                xyz: (0, 0, 0.4) m
                orientation: euler { angles: (0, 0, 0) unit: rad }
            }"""


def _tree(
    tmp_path: Path,
    inertia: str = STATED_INERTIA,
    kind: str = "mjcf",
    body: str = MJCF,
    tcp_pose: str = TCP_POSE,
):
    (tmp_path / "bench.scene").write_text(SCENE)
    (tmp_path / "rig.model").write_text(body)
    (tmp_path / "rig.scenex").write_text(TREE.format(inertia=inertia, kind=kind, tcp_pose=tcp_pose))
    model = scenex_metamodel().model_from_file(str(tmp_path / "rig.scenex"))
    [tree] = build_kdl_trees(create_scenex_model_graph(model=model), tmp_path)
    return tree


def test_a_joint_places_its_child_where_its_anchor_is(tmp_path):
    """The anchor carries both the axis turned about and the child's pose at zero position."""
    tree = _tree(tmp_path)

    assert tree.root == "rig/base"
    segment = tree.segments[0]
    assert (segment.name, segment.parent) == ("rig/link1", "rig/base")
    assert np.allclose(segment.joint.origin, (0.0, 0.0, 0.15643))
    # the anchor is turned 180 degrees about x, so its z points down in the parent body
    assert np.allclose(segment.joint.axis, (0.0, 0.0, -1.0), atol=1e-12)
    assert np.allclose(segment.transform.translation, (0.0, 0.0, 0.15643))
    assert np.allclose(segment.inertia.moments, (0.004, 0.005, 0.0006, 0.0, 0.0, 0.0))


def test_a_chain_is_a_slice_of_its_tree(tmp_path):
    """A tip frame is no body, so the tree gains it as a fixed leaf to be sliced out to."""
    tree = _tree(tmp_path)
    [chain] = tree.chains

    assert (chain.root, chain.tip) == ("rig/base", "rig/link1/tcp")
    tip = tree.segments[-1]
    assert (tip.name, tip.parent) == ("rig/link1/tcp", "rig/link1")
    assert tip.joint is None  # a frame is no body and no joint moves to it
    assert np.allclose(tip.transform.translation, (0.0, 0.0, 0.4))


@pytest.mark.parametrize(("kind", "body"), [("mjcf", MJCF), ("urdf", URDF)])
def test_inertia_absent_from_the_scene_is_read_from_the_model_file(tmp_path, kind, body):
    """MJCF angles are degrees unless its compiler says otherwise; URDF rpy is always radians."""
    inertia = _tree(tmp_path, inertia="", kind=kind, body=body).segments[0].inertia

    assert inertia.mass == pytest.approx(0.75)
    assert np.allclose(inertia.cog, (0.0, 0.0, 0.2))
    # a quarter turn about z swaps which axis carries which moment
    assert np.allclose(inertia.moments[:3], (0.005, 0.004, 0.0006))


def test_a_frame_with_no_pose_is_rejected(tmp_path):
    """A frame that is not the body's root is somewhere, and the model has to say where."""
    with pytest.raises(ConstraintViolation, match="no pose places frame"):
        _tree(tmp_path, tcp_pose="")


def test_a_model_file_stating_no_inertial_is_an_error(tmp_path):
    """MuJoCo would derive it from the geoms; the file is parsed, so this cannot be answered."""
    with pytest.raises(ConstraintViolation, match="states no inertial"):
        _tree(
            tmp_path,
            inertia="",
            body="<mujoco model='rig'><worldbody><body name='link1'/></worldbody></mujoco>",
        )
