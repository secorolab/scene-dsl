# SPDX-License-Identifier: MPL-2.0
from scene_dsl.dot import create_dot
from scene_dsl.langs import scenex_metamodel

from .test_common import MODELS_DIR

CHAIN = 'color="#1a73e8", penwidth=2'


def _edges(dot: str) -> dict[str, str]:
    """Every edge of the graph, keyed by 'from -> to'."""
    return {line.split(" [")[0].strip(): line for line in dot.splitlines() if " -> " in line}


def test_dot_draws_the_chain_across_two_devices():
    """The chain is what the model claims, so it is what the picture picks out."""
    dot = create_dot(scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex"))
    edges = _edges(dot)

    # each device is a cluster of its own, nested in the tree that assembles them
    assert 'subgraph "cluster_lab_graph/world_tree/arm1_gripper"' in dot
    assert 'subgraph "cluster_lab_graph/world_tree/arm1_gripper/gripper"' in dot

    # the chain runs from the arm's base, across the mount, to the gripper's tip
    assert CHAIN in edges['"arm1/base_link" -> "arm1/shoulder_link"']
    assert CHAIN in edges['"arm1/bracelet_link" -> "gripper/g_base_mount"']
    assert CHAIN in edges['"gripper/g_base_mount" -> "gripper/g_base"']

    # a branch off the chain is drawn, but is not the chain
    assert CHAIN not in edges['"gripper/g_base" -> "gripper/g_left_driver"']
    # arm2 declares no chain of its own, so none of it is picked out
    assert CHAIN not in edges['"arm2/base_link" -> "arm2/shoulder_link"']


def test_dot_tells_a_fixed_joint_from_a_revolute_one():
    """What a joint is, and whether it is on a chain, are drawn separately."""
    dot = create_dot(scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex"))
    edges = _edges(dot)

    fixed = edges['"world_tree/table_body" -> "arm1/base_link"']
    assert "style=dashed" in fixed and "arrowhead=diamond" in fixed
    # revolute: solid, whether or not it is on a chain
    assert "style=dashed" not in edges['"arm2/base_link" -> "arm2/shoulder_link"']
    assert "style=dashed" not in edges['"arm1/base_link" -> "arm1/shoulder_link"']
