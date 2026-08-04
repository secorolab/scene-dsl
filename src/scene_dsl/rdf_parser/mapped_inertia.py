# SPDX-License-Identifier: MPL-2.0
"""The inertia a body's mapped URDF or MJCF states, for a scene that states none itself.

A file that cannot answer is rejected; only a body no file describes returns nothing.
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
from rdflib.namespace import split_uri
from scipy.spatial.transform import Rotation

from scene_dsl.rdf_parser.common import get_kinematic_mapping
from scene_dsl.rdf_parser.vocab import URI_MJCF_MUJOCO, URI_URDF_ROBOT


def mapped_model(
    body: URIRef, graph: Graph, owner: URIRef | None
) -> tuple[URIRef, str] | None:
    """The model describing this body, and the name it knows the body by.

    A mapping of the body names it outright; a mapping of the tree that owns it leaves the
    body's own name to stand, since a tree mapping names the tree, not each body under it.
    Which tree owns the body is the caller's to say -- the graph answers it from the joints
    a tree reaches its bodies through, not from how the IRIs happen to nest.
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
                return model, mapping.entity or split_uri(body)[1]
            if mapping.target_type == URI_GEOM_TYPE_KTREE and mapping.target_id == owner:
                by_tree = (model, split_uri(body)[1])
    return by_tree


def model_path(model: URIRef, graph: Graph, base_dir: Path | None) -> Path:
    """Where a model's path lands.

    A relative path is read against the model declaring it, and against nothing else: the
    directory a generator happens to run from is no part of what a scene states, and a
    scene that resolves from one place and not another is not a model of anything.
    """
    literal = get_path_of_node(graph=graph, node_id=model)
    path = Path(literal).expanduser()
    if not path.is_absolute():
        if base_dir is None:
            raise ConstraintViolation(
                "model file",
                f"model '{model}' points at '{literal}', which is relative, and the caller "
                f"named no directory to read it against",
            )
        path = base_dir / path

    if not path.is_file():
        raise ConstraintViolation(
            "model file", f"model '{model}' points at '{literal}', which is no file: '{path}'"
        )
    return path


def _floats(text: str, count: int) -> np.ndarray:
    """The `count` numbers an attribute states; what an absent one means is the caller's."""
    try:
        values = [float(value) for value in text.split()]
    except ValueError as error:
        raise ConstraintViolation("model file", f"'{text}' is not a list of numbers") from error
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
    cog = _floats("0 0 0" if origin is None else origin.get("xyz", "0 0 0"), 3)
    rpy = _floats("0 0 0" if origin is None else origin.get("rpy", "0 0 0"), 3)
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
    quat = inertial.get("quat")
    if quat is not None:
        w, x, y, z = _floats(quat, 4)
        return Rotation.from_quat([x, y, z, w]).as_matrix()
    euler = inertial.get("euler")
    if euler is not None:
        angles = _floats(euler, 3)
        return Rotation.from_euler(eulerseq.upper(), angles, degrees=degrees).as_matrix()
    axisangle = inertial.get("axisangle")
    if axisangle is not None:
        values = _floats(axisangle, 4)
        angle = np.deg2rad(values[3]) if degrees else values[3]
        length = np.linalg.norm(values[:3])
        if length == 0.0:
            raise ConstraintViolation(
                "model file",
                f"'axisangle' has no axis to turn about: '{axisangle}'",
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
    if angle not in ("degree", "radian"):
        raise ConstraintViolation(
            "model file", f"'{path}' declares an unhandled compiler angle '{angle}'"
        )
    degrees = angle == "degree"
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
    cog = _floats(inertial.get("pos", "0 0 0"), 3)
    rotation = _mjcf_rotation(inertial, degrees, eulerseq)

    fullinertia = inertial.get("fullinertia")
    diaginertia = inertial.get("diaginertia")
    if fullinertia is not None:
        ixx, iyy, izz, ixy, ixz, iyz = _floats(fullinertia, 6)
        matrix = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    elif diaginertia is not None:
        matrix = np.diag(_floats(diaginertia, 3))
    else:
        raise ConstraintViolation(
            "model file", f"body '{entity}' of '{path}' states no diaginertia or fullinertia"
        )
    return float(mass), cog, rotation @ matrix @ rotation.T


def read_body_inertia(
    body: URIRef, graph: Graph, owner: URIRef | None, base_dir: Path | None = None
) -> tuple[float, np.ndarray, np.ndarray] | None:
    """The mass, centre of mass and inertia tensor a mapped model file states for a body.

    Returns None when no model file describes the body at all; raises when one does but
    cannot answer, so a mapping that is wrong is never mistaken for a body nothing models.
    """
    mapped = mapped_model(body, graph, owner)
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
