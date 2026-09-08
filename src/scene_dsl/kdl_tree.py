"""Build the plain-data representation used to render Orocos KDL trees."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import get_node_types
from rdf_utils.models.geom_coord import (
    get_orientation_coord_vals,
    get_pose_coords,
    get_translation_between_points,
)
from rdf_utils.models.geom_rel import PoseModel, relation_neighbors
from rdf_utils.models.vocab import (
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_GEOM_PRED_ORIGIN,
    URI_GEOM_TYPE_POSE,
)
from rdf_utils.naming import get_valid_var_name
from rdflib import Graph, URIRef
from rdflib.namespace import split_uri
from scipy.spatial.transform import Rotation

from scene_dsl.rdf_parser.kinematics import (
    KinematicGraphModel,
    KinematicTreeModel,
    RevoluteJointModel,
    RigidBodyModel,
    kinematic_graphs,
    kinematic_trees,
    pose_between,
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


def drawn_pose(frame: URIRef, graph: Graph) -> tuple[URIRef, str, list[float]] | None:
    """The frame the pose is written against, the position coordinate a run draws, and the
    authored rotation as a quaternion; None when the frame's poses are all numeric."""
    for wrt, pose in relation_neighbors(frame, URI_GEOM_TYPE_POSE, graph, reverse=False):
        for _, coords in get_pose_coords(graph, [PoseModel(pose_id=pose, graph=graph)]):
            for coord in coords:
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY in get_node_types(
                    graph, coord.orientation_coord.id
                ):
                    raise ConstraintViolation(
                        "kinematics", f"frame '{frame}': a drawn orientation is not supported"
                    )
                if URI_DISTRIB_TYPE_SAMPLED_QUANTITY not in get_node_types(
                    graph, coord.position_coord.id
                ):
                    continue
                rotation = get_orientation_coord_vals(coord.orientation_coord, graph)
                if rotation is None:
                    raise ConstraintViolation(
                        "kinematics",
                        f"frame '{frame}': a drawn position needs an authored orientation",
                    )
                return wrt, str(coord.position_coord.id), [float(v) for v in rotation.as_quat()]
    return None


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
        # A fixed joint holds its frames at the pose between them, coincident only without one.
        across = pose_between(child_frame, parent_frame, graph)
        joint_pose = _pose_matrix(across) if across is not None else np.eye(4)
        return {
            "name": _declared_name(tree, joint.id),
            "iri": str(joint.id),
            "axis": None,
        }, _transform_data(parent_attachment @ joint_pose @ _inverse_pose(child_attachment))

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


def _body_frame_segments(
    body_name: str, model: RigidBodyModel, graph: Graph
) -> tuple[list[dict], dict[URIRef, str], list[dict]]:
    """A fixed KDL leaf for every frame the scene places on one body, and the name each got.

    A frame is where something is, so the tree has to hold it: a solver reaches one by name
    only if a segment stands for it. The body's root frame is where the body's own segment
    already is, so it names that rather than a leaf of its own. A frame whose position a run
    draws has no transform yet; it is listed apart, with the segment it hangs off, for the
    controller to add once it has drawn.
    """
    names: dict[URIRef, str] = {model.root_frame.id: body_name}
    segments = []
    drawn = {}
    for frame in sorted(model.frames - {model.root_frame.id}, key=str):
        if (sample := drawn_pose(frame, graph)) is not None:
            drawn[frame] = sample
            continue
        # A frame the scene never places is left out rather than put at the body's origin.
        pose = pose_between(frame, model.root_frame.id, graph)
        if pose is None:
            continue
        names[frame] = f"{body_name}/{split_uri(frame)[1]}"
        segments.append(
            {
                "name": names[frame],
                "iri": str(frame),
                "parent": body_name,
                "joint": None,
                "transform": _transform_data(_pose_matrix(pose)),
                "inertia": None,
            }
        )
    sampled = []
    for frame, (wrt, coord, rotation) in drawn.items():
        if wrt not in names:
            raise ConstraintViolation(
                "kinematics",
                f"the drawn pose of frame '{frame}' is written against '{wrt}', which is not a "
                f"placed frame of body '{model.id}'",
            )
        names[frame] = f"{body_name}/{split_uri(frame)[1]}"
        sampled.append(
            {
                "name": names[frame],
                "iri": str(frame),
                "parent": names[wrt],
                "body": body_name,
                "rotation": rotation,
                "coord_iri": coord,
            }
        )
    return segments, names, sampled


def _frame_segments(
    tree: KinematicTreeModel, graph: Graph
) -> tuple[list[dict], dict[URIRef, str], list[dict]]:
    """The frame leaves of every body of this tree, and the frame a chain end names."""
    names: dict[URIRef, str] = {}
    segments = []
    sampled = []
    for body, model in sorted(tree.bodies.items(), key=lambda item: str(item[0])):
        body_segments, body_names, body_sampled = _body_frame_segments(
            _body_name(tree, body), model, graph
        )
        segments.extend(body_segments)
        names.update(body_names)
        sampled.extend(body_sampled)
    for chain in tree.chains:
        for frame in (chain.root_frame, chain.tip_frame):
            if frame not in names:
                raise ConstraintViolation(
                    "kinematics",
                    f"chain '{chain.id}' runs to frame '{frame}', which no pose places on a "
                    f"body of tree '{tree.id}'",
                )
    return segments, names, sampled


def _chain_segment_order(segments: list[dict], root: str, tip: str) -> list[str]:
    """The segments `getChain(root, tip)` yields, in its order.

    Not the tree's joint path: `_frame_segments` adds leaves the graph has no joint for.
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
    tree: KinematicTreeModel, root: str, order: list[str], segments: list[dict], names: dict
) -> tuple[dict[str, dict], dict[str, int]]:
    """Where the chain's own numbering puts each frame and body it reaches.

    A frame the chain runs through is a segment of it, so it is at that index and nowhere
    else. Any other frame hangs off a segment the chain does have, so it is that segment's
    index and the leaf's own transform -- which is what spares a consumer a second solver
    for a pose the chain already computed. The chain counts its root as 0, so slice segment
    i is i + 1.
    """
    index_by_segment = {name: index for index, name in enumerate([root, *order])}
    parent_of = {segment["name"]: segment["parent"] for segment in segments}
    transform_of = {segment["name"]: segment["transform"] for segment in segments}

    frames: dict[str, dict] = {}
    for frame, name in names.items():
        if name in index_by_segment:
            frames[str(frame)] = {"index": index_by_segment[name], "offset": None}
        elif parent_of.get(name) in index_by_segment:
            frames[str(frame)] = {
                "index": index_by_segment[parent_of[name]],
                "offset": transform_of[name],
            }

    bodies = {
        str(body): index_by_segment[_body_name(tree, body)]
        for body in tree.bodies
        if _body_name(tree, body) in index_by_segment
    }
    return frames, bodies


def _reachable(tree, chain, segments: list[dict], frame_names: dict) -> dict:
    """The chain's `frames` and `bodies` lookups, keyed by IRI."""
    root = frame_names[chain.root_frame]
    order = _chain_segment_order(segments, root, frame_names[chain.tip_frame])
    frames, bodies = _chain_frames(tree, root, order, segments, frame_names)
    return {
        "frames": frames,
        "bodies": bodies,
        # What the chain ends at, for a consumer that means "the tip" without naming a frame.
        "tip_index": len(order),
    }


def _placed_body_tree(kgraph: KinematicGraphModel, body: RigidBodyModel, graph: Graph) -> dict:
    """The one-body tree a body no joint articulates stands as in the world model.

    KDL reaches a segment only through the tree that holds it, so a body the scene merely
    places needs one of its own to be posed and read back at all. It is named after the body,
    since every placed body of a graph would otherwise carry the graph's name.
    """
    local = split_uri(body.id)[1]
    root = f"{split_uri(kgraph.id)[1]}/{local}"
    segments, _, sampled = _body_frame_segments(root, body, graph)
    return {
        "name": local,
        "cpp_name": get_valid_var_name(local),
        "iri": str(body.id),
        "root": root,
        "root_iri": str(body.id),
        "segments": segments,
        "sampled_frames": sampled,
        "chains": [],
    }


def build_kdl_trees(graph: Graph, base_dir: Path | None = None) -> list[dict]:
    """Read scene kinematics into a JSON-serializable representation."""
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

        frame_segments, frame_names, sampled = _frame_segments(tree, graph)
        segments.extend(frame_segments)
        result.append(
            {
                "name": split_uri(tree.id)[1],
                "cpp_name": get_valid_var_name(split_uri(tree.id)[1]),
                "iri": str(tree.id),
                "root": _body_name(tree, tree.root),
                "root_iri": str(tree.root),
                "segments": segments,
                "sampled_frames": sampled,
                "chains": [
                    {
                        "name": _declared_name(tree, chain.id),
                        "cpp_name": get_valid_var_name(_declared_name(tree, chain.id)),
                        "iri": str(chain.id),
                        "root": frame_names[chain.root_frame],
                        "tip": frame_names[chain.tip_frame],
                        **_reachable(tree, chain, segments, frame_names),
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

    for kgraph in kinematic_graphs(graph, base_dir):
        for body in kgraph.free_bodies.values():
            result.append(_placed_body_tree(kgraph, body, graph))
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
        elements.extend((frame["name"], frame["iri"]) for frame in tree["sampled_frames"])
        elements.extend((chain["cpp_name"], chain["iri"]) for chain in tree["chains"])

        for name, iri in elements:
            if named.setdefault(name, iri) != iri:
                raise ConstraintViolation(
                    "kinematics",
                    f"'{name}' names both '{named[name]}' and '{iri}': two elements the "
                    f"header must tell apart are called the same thing",
                )
