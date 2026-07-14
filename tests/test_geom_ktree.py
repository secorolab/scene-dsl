import pytest
from rdflib import RDF, Literal

from rdf_utils.constraints import check_shacl_constraints
from rdf_utils.models.vocab import (
    URI_QUDT_QK_ANGLE,
    URI_QUDT_PRED_QUANTITY_KIND,
    URI_QUDT_PRED_UNIT,
)
from rdf_utils.models.geometry import (
    OrientCoordModel,
    PoseCoordModel,
    PositionCoordModel,
    get_coord_vectorxyz,
    get_euler_angles_abg,
)

from rdf_utils.namespace import URL_MM_GEOM_SHACL_EXTS, URL_MM_GEOM_SHACL_REL, URL_SECORO_MM
from rdf_utils.resolver import install_resolver

from scene_dsl.classes.common import FloatVector
from scene_dsl.classes.geom import DirectionCosineOrientationSpec
from scene_dsl.classes.ktree import RigidBodyInertia
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.geom import ANGLE_UNITS
from scene_dsl.rdf.ktree import (
    INERTIA_UNITS,
    MASS_UNITS,
    URI_DYN_PRED_IXX,
    URI_DYN_PRED_IXY,
    URI_DYN_PRED_IXZ,
    URI_DYN_PRED_IYY,
    URI_DYN_PRED_IYZ,
    URI_DYN_PRED_IZZ,
    URI_DYN_TYPE_MOMENT_OF_INERTIA_XYZ,
    URI_DYN_TYPE_PRODUCT_OF_INERTIA_XYZ,
    URI_ACT_PRED_COMMAND_INTERFACE,
    URI_ACT_PRED_JOINT,
    URI_KC_EXT_PRED_DEPENDENT_JOINT,
    URI_KC_EXT_PRED_OFFSET,
    URI_QUDT_TYPE_QUANTITY,
    URI_KC_EXT_PRED_INDEPENDENT_JOINT,
    URI_KC_EXT_TYPE_JOINT_COUPLING,
    URI_KC_EXT_TYPE_SCLERONOMIC,
    URI_ACT_PRED_STATE_INTERFACE,
    URI_ACT_TYPE_ACTUATION,
    ACTUATION_INTERFACE_TYPES,
)
from scene_dsl.rdf.scenex import create_scenex_model_graph
from .test_common import MODELS_DIR, write_example_scene

ZERO_POSE = (
    "pose default_pose { xyz: (0.0, 0.0, 0.0) m orientation: euler { angles: (0.0, 0.0, 0.0) } }"
)


def test_duplicate_model_uri_is_rejected(tmp_path):
    write_example_scene(tmp_path)
    model_path = tmp_path / "duplicate_model.scenex"
    model_path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    obj <objs.cup> {
        model dup as urdf { sys path = "cup.urdf" }
    }
    obj <objs.bowl> {
        model dup as urdf { sys path = "bowl.urdf" }
    }
}
"""
    )

    model = scenex_metamodel().model_from_file(model_path)
    with pytest.raises(ValueError, match="Duplicate model URI"):
        create_scenex_model_graph(model)


@pytest.mark.parametrize(
    ("length_unit", "mass_unit", "inertia_unit"),
    [("m", "kg", "kg*m^2"), ("cm", "g", "kg*m^2"), ("mm", "kg", "kg*m^2")],
)
def test_scenex_length_and_mass_units(tmp_path, length_unit, mass_unit, inertia_unit):
    write_example_scene(tmp_path)
    model_path = tmp_path / "units.scenex"
    model_path.write_text(
        f"""import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {{
    scene: <s>
    ktree (ns=n) world_tree {{
        body world_body {{ frame world {{ }} }}
        body cup_body {{
            frame cup_root {{
                pose cup_in_world {{
                    wrt: <world_body.world>
                    xyz: (1.0, 2.0, 3.0) {length_unit}
                    orientation: euler {{ angles: (0.0, 0.0, 0.0) }}
                }}
            }}
            inertia {{
                frame: cup_root
                mass: 10.0 {mass_unit}
                inertia-matrix: (
                    (1.0, 0.1, 0.2),
                    (0.1, 2.0, 0.3),
                    (0.2, 0.3, 3.0)
                ) {inertia_unit}
            }}
        }}
        joints {{ }}
    }}
    obj <objs.cup> {{
        model cup_model as urdf {{ sys path = "cup.urdf" }}
        body: <world_tree.cup_body>
    }}
}}
"""
    )

    model = scenex_metamodel().model_from_file(model_path)
    cup_body = model.scene_insts[0].modelled_objs[0].body
    pose = cup_body.frames[0].poses[0]
    graph = create_scenex_model_graph(model)
    position_coord = PositionCoordModel(pose.position_coord_uri, graph)
    orient_coord = OrientCoordModel(pose.orientation_coord_uri, graph)
    pose_coord = PoseCoordModel(pose.uri_coord, graph)

    assert position_coord.position == pose.position_uri
    assert position_coord.of == pose.of_frame.origin_uri
    assert position_coord.wrt == pose.wrt.origin_uri
    assert position_coord.as_seen_by == pose.wrt.uri
    assert pose_coord.pose == pose.uri
    assert pose_coord.of.id == pose.of_frame.uri
    assert pose_coord.wrt.id == pose.wrt.uri
    assert pose_coord.as_seen_by == pose.wrt.uri

    assert get_coord_vectorxyz(position_coord, graph) == tuple(pose.position_spec.values)

    axes, is_intrinsic, unit_uri, angles = get_euler_angles_abg(orient_coord, graph)
    assert axes == pose.orientation.axes
    assert is_intrinsic != pose.orientation.extrinsic
    assert unit_uri == ANGLE_UNITS[pose.orientation.unit]
    assert angles == tuple(pose.orientation.angles.values)

    assert (cup_body.inertia_coord_uri, URI_QUDT_PRED_UNIT, MASS_UNITS[mass_unit]) in graph
    assert (cup_body.inertia_coord_uri, URI_QUDT_PRED_UNIT, INERTIA_UNITS[inertia_unit]) in graph
    assert (cup_body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MOMENT_OF_INERTIA_XYZ) in graph
    assert (cup_body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_PRODUCT_OF_INERTIA_XYZ) in graph

    matrix = cup_body.inertia.matrix
    for predicate, value in (
        (URI_DYN_PRED_IXX, matrix[0][0]),
        (URI_DYN_PRED_IXY, matrix[0][1]),
        (URI_DYN_PRED_IXZ, matrix[0][2]),
        (URI_DYN_PRED_IYY, matrix[1][1]),
        (URI_DYN_PRED_IYZ, matrix[1][2]),
        (URI_DYN_PRED_IZZ, matrix[2][2]),
    ):
        assert (cup_body.inertia_coord_uri, predicate, Literal(value)) in graph


def test_scenex_mass_quantity_validation(tmp_path):
    write_example_scene(tmp_path)
    model_path = tmp_path / "mass_quantity.scenex"
    model_path.write_text(
        f"""import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {{
    scene: <s>
    ktree (ns=n) world_tree {{
        body cup_body {{
            frame cup_root {{ {ZERO_POSE} }}
            inertia {{
                frame: cup_root
                mass: 0.0 kg
                inertia-matrix: (
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0)
                ) kg*m^2
            }}
        }}
        joints {{ }}
    }}
}}
"""
    )

    cup_body = scenex_metamodel().model_from_file(model_path).scene_insts[0].ktree.bodies[0]
    assert cup_body.inertia.mass == 0.0

    with pytest.raises(ValueError, match="RigidBodyInertia.mass must be"):
        RigidBodyInertia(
            parent=None,
            frame=None,
            mass=-1.0,
            mass_unit="kg",
            row1=FloatVector(None, [1.0, 0.0, 0.0]),
            row2=FloatVector(None, [0.0, 1.0, 0.0]),
            row3=FloatVector(None, [0.0, 0.0, 1.0]),
            inertia_unit="kg*m^2",
        )


def test_rigid_body_inertia_rejects_non_symmetric_matrix():
    with pytest.raises(ValueError, match="RigidBodyInertia.matrix must be symmetric"):
        RigidBodyInertia(
            parent=None,
            frame=None,
            mass=0.0,
            mass_unit="kg",
            row1=FloatVector(None, [1.0, 0.1, 0.0]),
            row2=FloatVector(None, [0.0, 1.0, 0.0]),
            row3=FloatVector(None, [0.0, 0.0, 1.0]),
            inertia_unit="kg*m^2",
        )


def test_direction_cosine_orientation_rejects_non_orthogonal_matrix():
    with pytest.raises(ValueError, match="rotation_matrix must be orthogonal"):
        DirectionCosineOrientationSpec(
            parent=None,
            x_axis=FloatVector(None, [1.0, 0.0, 0.0]),
            y_axis=FloatVector(None, [1.0, 0.0, 0.0]),
            z_axis=FloatVector(None, [0.0, 0.0, 1.0]),
        )


def test_lab_scenex_generated_geometry_validates_against_shacl():
    install_resolver()
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")

    graph = create_scenex_model_graph(model)
    check_shacl_constraints(
        graph=graph,
        shacl_dict={
            URL_MM_GEOM_SHACL_EXTS: "ttl",
            URL_MM_GEOM_SHACL_REL: "ttl",
            # Skipping because of DirectionCosine rule that would fail, potentially
            # because of https://github.com/RDFLib/rdflib/issues/2009
            # URL_MM_GEOM_SHACL_COORD: "ttl",
            f"{URL_SECORO_MM}/robot/actuation.shacl.ttl": "ttl",
            f"{URL_SECORO_MM}/robot/sensors.shacl.ttl": "ttl",
            f"{URL_SECORO_MM}/kinematic-chain/structural-entities-extension.shacl.ttl": "ttl",
        },
        quiet=False,
    )


def test_lab_scenex_generates_panda_joint_actuation():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    panda_tree = next(
        tree for tree in model.scene_insts[0].ktree.trees if tree.name == "panda_tree"
    )
    joint = next(joint for joint in panda_tree.joints_spec.joints if joint.name == "panda_joint1")
    graph = create_scenex_model_graph(model)

    assert (joint.actuation_uri, RDF.type, URI_ACT_TYPE_ACTUATION) in graph
    assert (joint.actuation_uri, URI_ACT_PRED_JOINT, joint.uri) in graph
    for interface in joint.actuation.cmd_interfaces:
        assert (
            joint.actuation_uri,
            URI_ACT_PRED_COMMAND_INTERFACE,
            ACTUATION_INTERFACE_TYPES[interface],
        ) in graph
    for interface in joint.actuation.state_interfaces:
        assert (
            joint.actuation_uri,
            URI_ACT_PRED_STATE_INTERFACE,
            ACTUATION_INTERFACE_TYPES[interface],
        ) in graph

    mimic_joint = next(
        joint for joint in panda_tree.joints_spec.joints if joint.name == "panda_joint2"
    )
    assert (mimic_joint.mimic_uri, RDF.type, URI_KC_EXT_TYPE_JOINT_COUPLING) in graph
    assert (mimic_joint.mimic_uri, RDF.type, URI_KC_EXT_TYPE_SCLERONOMIC) in graph
    assert (
        mimic_joint.mimic_uri,
        URI_KC_EXT_PRED_DEPENDENT_JOINT,
        mimic_joint.uri,
    ) in graph
    assert (
        mimic_joint.mimic_uri,
        URI_KC_EXT_PRED_INDEPENDENT_JOINT,
        mimic_joint.mimic.joint.uri,
    ) in graph
    assert (
        mimic_joint.mimic_uri,
        URI_KC_EXT_PRED_OFFSET,
        mimic_joint.mimic_offset_uri,
    ) in graph
    assert (mimic_joint.mimic_offset_uri, RDF.type, URI_QUDT_TYPE_QUANTITY) in graph
    assert (
        mimic_joint.mimic_offset_uri,
        URI_QUDT_PRED_QUANTITY_KIND,
        URI_QUDT_QK_ANGLE,
    ) in graph


def test_scenex_embedded_kinematic_tree_rejects_bad_frame_ref(tmp_path):
    write_example_scene(tmp_path)
    model_path = tmp_path / "bad_robot.scenex"
    model_path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    ktree (ns=n) world_tree {
        body world_body { frame world { } }
        joints {
            fixed bad_mount {
                parent: <missing>
                child: <world_body.world>
            }
        }
    }
    agn <agns.robot> {
        model robot_mjcf as mjcf { sys path = "robot.xml" }
    }
}
"""
    )

    with pytest.raises(Exception, match="missing"):
        scenex_metamodel().model_from_file(model_path)


def test_direction_cosine_orientation_rejects_reflection():
    with pytest.raises(ValueError, match=r"determinant \+1"):
        DirectionCosineOrientationSpec(
            parent=None,
            x_axis=FloatVector(None, [1.0, 0.0, 0.0]),
            y_axis=FloatVector(None, [0.0, 1.0, 0.0]),
            z_axis=FloatVector(None, [0.0, 0.0, -1.0]),
        )
