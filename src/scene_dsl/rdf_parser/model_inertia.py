# SPDX-License-Identifier: MPL-2.0
"""Read a body's inertia from the model file a scene maps it to.

The file is parsed, not compiled: what it states is what is read. MuJoCo derives a body's
inertia from its geoms when the body states no `<inertial>`, and reproducing that needs the
simulator -- so such a body is reported as missing rather than guessed at.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.execution import get_path_of_node
from rdf_utils.models.vocab import (
    URI_EXEC_PRED_HAS_MAPPING,
    URI_GEOM_TYPE_KTREE,
)
from rdflib import RDF, Graph, URIRef
from scipy.spatial.transform import Rotation

from scene_dsl.rdf_parser.common import get_kinematic_mapping
from scene_dsl.rdf_parser.vocab import URI_MJCF_MUJOCO, URI_URDF_ROBOT

MJCF_ANGLE_SCALE = {"degree": True, "radian": False}


def mapped_model(body: URIRef, graph: Graph) -> tuple[URIRef, str] | None:
    """The model describing this body, and the name it knows the body by.

    A mapping of the body names it outright; a mapping of the tree holding it leaves the
    body's own name to stand, since a tree mapping names the tree, not each body under it.
    """
    by_tree: tuple[URIRef, str] | None = None
    for model in graph.subjects(URI_EXEC_PRED_HAS_MAPPING, None):
        if not isinstance(model, URIRef):
            continue
        for mapping_id in graph.objects(model, URI_EXEC_PRED_HAS_MAPPING):
            if not isinstance(mapping_id, URIRef):
                continue
            mapping = get_kinematic_mapping(mapping_id, graph)
            if mapping.target_id == body:
                return model, mapping.entity or str(body).rsplit("/", 1)[-1]
            if mapping.target_type == URI_GEOM_TYPE_KTREE and str(body).startswith(
                f"{mapping.target_id}/"
            ):
                by_tree = (model, str(body).rsplit("/", 1)[-1])
    return by_tree


def model_path(model: URIRef, graph: Graph, base_dir: Path | None) -> Path:
    """Where a model's path lands.

    A relative path is written either against the model declaring it or against the
    workspace the scene's assets are named from, so try both and take what exists.
    """
    literal = get_path_of_node(graph=graph, node_id=model)
    path = Path(literal).expanduser()
    if path.is_absolute():
        candidates = [path]
    else:
        candidates = [base for base in (base_dir, Path.cwd()) if base is not None]
        candidates = [base / path for base in candidates]

    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise ConstraintViolation(
        "model file",
        f"model '{model}' points at '{literal}', which is no file: tried "
        f"{', '.join(str(candidate) for candidate in candidates)}",
    )


def _floats(text: str | None, count: int, default: tuple[float, ...]) -> np.ndarray:
    if text is None:
        return np.asarray(default, dtype=float)
    values = [float(value) for value in text.split()]
    if len(values) != count:
        raise ConstraintViolation("model file", f"expected {count} numbers, got '{text}'")
    return np.asarray(values, dtype=float)


def _read_urdf(path: Path, entity: str) -> tuple[float, np.ndarray, np.ndarray]:
    root = ElementTree.parse(path).getroot()
    link = root.find(f"./link[@name='{entity}']")
    if link is None:
        raise ConstraintViolation("model file", f"'{path}' has no link named '{entity}'")
    inertial = link.find("inertial")
    if inertial is None:
        raise ConstraintViolation("model file", f"link '{entity}' of '{path}' states no inertial")

    mass_element, tensor = inertial.find("mass"), inertial.find("inertia")
    if mass_element is None or tensor is None:
        raise ConstraintViolation(
            "model file", f"link '{entity}' of '{path}' states no mass or no inertia"
        )
    origin = inertial.find("origin")
    cog = _floats(None if origin is None else origin.get("xyz"), 3, (0.0, 0.0, 0.0))
    rpy = _floats(None if origin is None else origin.get("rpy"), 3, (0.0, 0.0, 0.0))
    # URDF fixes the rotation as extrinsic xyz in radians (REP 103).
    rotation = Rotation.from_euler("xyz", rpy).as_matrix()

    values = {
        key: float(tensor.get(key, 0.0)) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    }
    matrix = np.array(
        [
            [values["ixx"], values["ixy"], values["ixz"]],
            [values["ixy"], values["iyy"], values["iyz"]],
            [values["ixz"], values["iyz"], values["izz"]],
        ]
    )
    return float(mass_element.get("value", 0.0)), cog, rotation @ matrix @ rotation.T


def _mjcf_rotation(inertial: ElementTree.Element, degrees: bool, eulerseq: str) -> np.ndarray:
    if inertial.get("quat") is not None:
        w, x, y, z = _floats(inertial.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))
        return Rotation.from_quat([x, y, z, w]).as_matrix()
    if inertial.get("euler") is not None:
        angles = _floats(inertial.get("euler"), 3, (0.0, 0.0, 0.0))
        return Rotation.from_euler(eulerseq.upper(), angles, degrees=degrees).as_matrix()
    if inertial.get("axisangle") is not None:
        values = _floats(inertial.get("axisangle"), 4, (0.0, 0.0, 1.0, 0.0))
        angle = np.deg2rad(values[3]) if degrees else values[3]
        length = np.linalg.norm(values[:3])
        if length == 0.0:
            raise ConstraintViolation(
                "model file",
                f"'axisangle' has no axis to turn about: '{inertial.get('axisangle')}'",
            )
        return Rotation.from_rotvec(values[:3] / length * angle).as_matrix()
    if inertial.get("zaxis") is not None or inertial.get("xyaxes") is not None:
        raise ConstraintViolation(
            "model file", "'zaxis' and 'xyaxes' inertial orientations are not handled"
        )
    return np.eye(3)


def _read_mjcf(path: Path, entity: str) -> tuple[float, np.ndarray, np.ndarray]:
    root = ElementTree.parse(path).getroot()
    compiler = root.find("compiler")
    angle = "degree" if compiler is None else compiler.get("angle", "degree")
    if angle not in MJCF_ANGLE_SCALE:
        raise ConstraintViolation(
            "model file", f"'{path}' declares an unhandled compiler angle '{angle}'"
        )
    eulerseq = "xyz" if compiler is None else compiler.get("eulerseq", "xyz")

    body = root.find(f".//body[@name='{entity}']")
    if body is None:
        included = " ('<include>' files are not followed)" if root.find(".//include") else ""
        raise ConstraintViolation("model file", f"'{path}' has no body named '{entity}'{included}")
    inertial = body.find("inertial")
    if inertial is None:
        raise ConstraintViolation(
            "model file",
            f"body '{entity}' of '{path}' states no inertial: MuJoCo would derive it from the "
            f"body's geoms at compile time, which this generator does not do",
        )

    mass = inertial.get("mass")
    if mass is None:
        raise ConstraintViolation("model file", f"body '{entity}' of '{path}' states no mass")
    cog = _floats(inertial.get("pos"), 3, (0.0, 0.0, 0.0))
    rotation = _mjcf_rotation(inertial, MJCF_ANGLE_SCALE[angle], eulerseq)

    if inertial.get("fullinertia") is not None:
        ixx, iyy, izz, ixy, ixz, iyz = _floats(inertial.get("fullinertia"), 6, (0.0,) * 6)
        matrix = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    elif inertial.get("diaginertia") is not None:
        matrix = np.diag(_floats(inertial.get("diaginertia"), 3, (0.0, 0.0, 0.0)))
    else:
        raise ConstraintViolation(
            "model file", f"body '{entity}' of '{path}' states no diaginertia or fullinertia"
        )
    return float(mass), cog, rotation @ matrix @ rotation.T


def read_body_inertia(
    body: URIRef, graph: Graph, base_dir: Path | None = None
) -> tuple[float, np.ndarray, np.ndarray] | None:
    """The mass, centre of mass and inertia tensor a mapped model file states for a body.

    Returns None when no model file describes the body at all; raises when one does but
    cannot answer, so a mapping that is wrong is never mistaken for a body nothing models.
    """
    mapped = mapped_model(body, graph)
    if mapped is None:
        return None

    model, entity = mapped
    path = model_path(model, graph, base_dir)
    if (model, RDF.type, URI_MJCF_MUJOCO) in graph:
        return _read_mjcf(path, entity)
    if (model, RDF.type, URI_URDF_ROBOT) in graph:
        return _read_urdf(path, entity)
    raise ConstraintViolation(
        "model file", f"model '{model}' for body '{body}' is neither MJCF nor URDF"
    )
