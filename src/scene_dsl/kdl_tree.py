"""Build the plain-data representation used to render Orocos KDL trees."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.geom_coord import get_transform_between_frames, get_translation_between_points
from rdf_utils.models.vocab import URI_GEOM_PRED_ORIGIN
from rdf_utils.naming import get_valid_var_name
from rdflib import Graph, URIRef
from rdflib.namespace import split_uri
from scipy.spatial.transform import Rotation

from scene_dsl.rdf_parser.kinematics import (
    KinematicTreeModel,
    RevoluteJointModel,
    kinematic_trees,
)


def _body_name(tree: KinematicTreeModel, body: URIRef) -> str:
    """A KDL name unique across trees: the element under the tree the model says owns it."""
    return f"{split_uri(tree.defining_tree_by_body[body])[1]}/{split_uri(body)[1]}"


def _declared_name(tree: KinematicTreeModel, uri: URIRef) -> str:
    """The same, for a joint or a chain, which a tree declares rather than defines."""
    return f"{split_uri(tree.declaring_tree[uri])[1]}/{split_uri(uri)[1]}"


def _pose_matrix(transform) -> np.ndarray:
    return np.asarray(transform.as_matrix(), dtype=float)


def _inverse_pose(pose: np.ndarray) -> np.ndarray:
    inverse = np.eye(4)
    inverse[:3, :3] = pose[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ pose[:3, 3]
    return inverse


def _translation_pose(translation: np.ndarray) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, 3] = translation
    return pose


def _transform_data(pose: np.ndarray) -> dict:
    return {
        "rotation": [float(value) for value in Rotation.from_matrix(pose[:3, :3]).as_quat()],
        "translation": [float(value) for value in pose[:3, 3]],
    }


def _inertia_data(tree: KinematicTreeModel, body: URIRef, graph: Graph) -> dict:
    inertia = tree.mass_properties(body, graph)
    return {
        "mass": float(inertia.mass),
        "com": [float(value) for value in inertia.com],
        "tensor": [[float(value) for value in row] for row in inertia.tensor],
    }


def _origin(frame: URIRef, graph: Graph) -> URIRef:
    origin = graph.value(frame, URI_GEOM_PRED_ORIGIN)
    if not isinstance(origin, URIRef):
        raise ConstraintViolation("kinematics", f"frame '{frame}' has no origin")
    return origin


def _joint_data(
    tree: KinematicTreeModel,
    parent: URIRef,
    child: URIRef,
    joint_id: URIRef,
    graph: Graph,
) -> tuple[dict, dict]:
    """The KDL joint and child segment pose for one directed graph joint."""
    joint = tree.joints[joint_id]
    parent_frame = joint.frame_on(parent)
    child_frame = joint.frame_on(child)
    parent_attachment = _pose_matrix(tree.bodies[parent].pose_of(parent_frame, graph))
    child_attachment = _pose_matrix(tree.bodies[child].pose_of(child_frame, graph))

    if not isinstance(joint, RevoluteJointModel):
        return {
            "name": _declared_name(tree, joint.id),
            "iri": str(joint.id),
            "axis": None,
        }, _transform_data(parent_attachment @ _inverse_pose(child_attachment))

    offset_pose = np.eye(4)
    if joint.offset is not None:
        offset = get_translation_between_points(
            _origin(child_frame, graph), _origin(parent_frame, graph), graph
        )
        if offset is None:
            raise ConstraintViolation(
                "kinematics", f"joint '{joint.id}' has an offset with no coordinates"
            )
        offset_pose[:3, 3] = np.asarray(offset, dtype=float)

    origin = parent_attachment[:3, 3]
    axis = np.zeros(3)
    axis["xyz".index(joint.axis_on(parent_frame))] = 1.0
    return (
        {
            "name": _declared_name(tree, joint.id),
            "iri": str(joint.id),
            "origin": [float(value) for value in origin],
            "axis": [float(value) for value in parent_attachment[:3, :3] @ axis],
        },
        _transform_data(parent_attachment @ offset_pose @ _inverse_pose(child_attachment)),
    )


def _endpoint_segments(
    tree: KinematicTreeModel, graph: Graph
) -> tuple[list[dict], dict[URIRef, str]]:
    """Add fixed KDL leaves for chain endpoints that name a body-local frame."""
    frames = {frame for chain in tree.chains for frame in (chain.root_frame, chain.tip_frame)}
    names: dict[URIRef, str] = {}
    segments = []
    for frame in sorted(frames, key=str):
        body = next(body for body, model in tree.bodies.items() if frame in model.frames)
        body_name = _body_name(tree, body)
        if frame == tree.bodies[body].root_frame.id:
            names[frame] = body_name
            continue
        name = f"{body_name}/{split_uri(frame)[1]}"
        names[frame] = name
        segments.append(
            {
                "name": name,
                "iri": str(frame),
                "parent": body_name,
                "joint": None,
                "transform": _transform_data(_pose_matrix(tree.bodies[body].pose_of(frame, graph))),
                "inertia": None,
            }
        )
    return segments, names


def _chain_segment_order(segments: list[dict], root: str, tip: str) -> list[str]:
    """The segment names KDL's getChain(root, tip) yields, in its order.

    Walk parents from the tip up to the root, the way the chain is sliced, so a caller can
    number a frame exactly as the built chain does instead of searching it by name.
    """
    parent_of = {segment["name"]: segment["parent"] for segment in segments}
    walked: list[str] = []
    current = tip
    while current != root:
        walked.append(current)
        current = parent_of.get(current)
        if current is None:
            raise ConstraintViolation(
                "kinematics", f"chain tip '{tip}' does not descend from its root '{root}'"
            )
    walked.reverse()
    return walked


def _chain_frames(
    tree: KinematicTreeModel, root: str, order: list[str], segments: list[dict], graph: Graph
) -> tuple[dict[str, dict], dict[str, int]]:
    """Every frame and body reachable on a chain, as the index of the segment standing for it
    and, for a frame, its pose on that segment.

    A body carries as many frames as the scene declares, and only a chain endpoint is ever a
    segment of its own, so a frame is found through the body it belongs to rather than by
    matching a segment name. Index 0 is the chain root, matching KDL: segment i of the sliced
    chain is i + 1.
    """
    body_by_segment = {_body_name(tree, body): body for body in tree.bodies}
    frames: dict[str, dict] = {}
    bodies: dict[str, int] = {}
    for index, name in enumerate([root, *order]):
        body = body_by_segment.get(name)
        if body is None:
            continue
        bodies[str(body)] = index
        for frame in tree.bodies[body].frames:
            placed = get_transform_between_frames(frame, tree.bodies[body].root_frame.id, graph)
            # A frame the scene never places -- a joint anchor's derived origin, say -- sits
            # nowhere on the body, so it is not somewhere a pose can be asked for. Leaving it
            # out is what makes a model that names it fail where it names it.
            if placed is None and frame != tree.bodies[body].root_frame.id:
                continue
            pose = _pose_matrix(tree.bodies[body].pose_of(frame, graph))
            frames[str(frame)] = {
                "index": index,
                "offset": None if np.allclose(pose, np.eye(4)) else _transform_data(pose),
            }
    # A chain endpoint is a segment in its own right, and the transform to it is already that
    # segment's. Naming it directly is exact, so it wins over reaching it from its body.
    for index, name in enumerate([root, *order]):
        for segment in segments:
            if segment["name"] == name and segment["iri"] in frames:
                frames[segment["iri"]] = {"index": index, "offset": None}
    return frames, bodies


def _reachable(tree, chain, segments: list[dict], endpoint_names: dict, graph: Graph) -> dict:
    """The chain's `frames` and `bodies` lookups, keyed by IRI."""
    frames, bodies = _chain_frames(
        tree,
        endpoint_names[chain.root_frame],
        _chain_segment_order(
            segments, endpoint_names[chain.root_frame], endpoint_names[chain.tip_frame]
        ),
        segments,
        graph,
    )
    return {
        "frames": frames,
        "bodies": bodies,
        # What the chain ends at, for a consumer that means "the tip" without naming a frame.
        "tip_index": len(
            _chain_segment_order(
                segments, endpoint_names[chain.root_frame], endpoint_names[chain.tip_frame]
            )
        ),
    }


def build_kdl_trees(graph: Graph, base_dir: Path | None = None) -> list[dict]:
    """Read plain-data scene kinematics into a JSON-serializable representation."""
    result = []
    for tree in kinematic_trees(graph, base_dir):
        segments = []
        for child in tree.topological_order[1:]:
            parent = tree.parent[child]
            joint, transform = _joint_data(tree, parent, child, tree.parent_joint[child], graph)
            segments.append(
                {
                    "name": _body_name(tree, child),
                    "iri": str(child),
                    "parent": _body_name(tree, parent),
                    "joint": joint,
                    "transform": transform,
                    "inertia": _inertia_data(tree, child, graph),
                }
            )

        endpoint_segments, endpoint_names = _endpoint_segments(tree, graph)
        segments.extend(endpoint_segments)
        result.append(
            {
                "name": split_uri(tree.id)[1],
                "cpp_name": get_valid_var_name(split_uri(tree.id)[1]),
                "iri": str(tree.id),
                "root": _body_name(tree, tree.root),
                "root_iri": str(tree.root),
                "segments": segments,
                "chains": [
                    {
                        "name": _declared_name(tree, chain.id),
                        "cpp_name": get_valid_var_name(_declared_name(tree, chain.id)),
                        "iri": str(chain.id),
                        "root": endpoint_names[chain.root_frame],
                        "tip": endpoint_names[chain.tip_frame],
                        **_reachable(tree, chain, segments, endpoint_names, graph),
                        "joints": [
                            {
                                "name": _declared_name(tree, joint_id),
                                "local_name": split_uri(joint_id)[1],
                            }
                            for joint_id in tree.path(chain)
                            if isinstance(tree.joints[joint_id], RevoluteJointModel)
                        ],
                    }
                    for chain in tree.chains
                ],
            }
        )
    _ensure_names_are_distinct(result)
    return result


def _ensure_names_are_distinct(trees: list[dict]) -> None:
    """Every name the header holds stands for one element.

    KDL knows a segment or a joint by its name, and the header's IRI table is keyed by
    those names, so two elements sharing one would leave a scene with a name that means
    two things and a table that holds whichever came first.
    """
    named: dict[str, str] = {}
    for tree in trees:
        elements = [(tree["cpp_name"], tree["iri"]), (tree["root"], tree["root_iri"])]
        for segment in tree["segments"]:
            elements.append((segment["name"], segment["iri"]))
            if segment["joint"] is not None:
                elements.append((segment["joint"]["name"], segment["joint"]["iri"]))
        elements.extend((chain["cpp_name"], chain["iri"]) for chain in tree["chains"])

        for name, iri in elements:
            if named.setdefault(name, iri) != iri:
                raise ConstraintViolation(
                    "kinematics",
                    f"'{name}' names both '{named[name]}' and '{iri}': two elements the "
                    f"header must tell apart are called the same thing",
                )
