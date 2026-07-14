from pathlib import Path

import pytest
from rdflib import RDF, Literal, XSD
import numpy as np

from bdd_dsl.models.urirefs import URI_EXEC_PRED_PATH
from rdf_utils.constraints import check_shacl_constraints
from rdf_utils.models.distribution import (
    DistributionModel,
    distrib_from_sampled_quantity,
    sample_from_distrib,
)
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_TYPE_DISTRIB,
    URI_DISTRIB_TYPE_NORMAL,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DYN_TYPE_MASS_SCALAR,
    URI_GEOM_TYPE_RIGID_BODY,
    URI_KC_PRED_JOINTS,
    URI_KC_TYPE_REVOLUTE_JOINT,
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
from scene_dsl.classes.geom import DirectionCosineOrientationSpec, PoseSpec
from scene_dsl.classes.ktree import RigidBodyInertia
from scene_dsl.langs import scene_metamodel, scenex_metamodel
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
    URI_GEOM_TYPE_KTREE,
    URI_ACT_PRED_COMMAND_INTERFACE,
    URI_ACT_PRED_JOINT,
    URI_KC_EXT_PRED_DEPENDENT_JOINT,
    URI_KC_EXT_PRED_MULTIPLIER,
    URI_KC_EXT_PRED_OFFSET,
    URI_QUDT_TYPE_QUANTITY,
    URI_KC_EXT_PRED_INDEPENDENT_JOINT,
    URI_KC_EXT_TYPE_JOINT_COUPLING,
    URI_KC_EXT_TYPE_SCLERONOMIC,
    URI_ACT_PRED_STATE_INTERFACE,
    URI_ACT_TYPE_ACTUATION,
    ACTUATION_INTERFACE_TYPES,
)
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import URI_EXEC_PRED_LINKS_BODY, create_scenex_model_graph
from scene_dsl.rdf.sensors import URI_EXEC_PRED_HAS_KINEMATICS
from scene_dsl.rdf.sensors import (
    CAMERA_TYPES,
    OBSERVED_QUANTITIES,
    URI_QUDT_PRED_VALUE,
    URI_SENS_PRED_CAMERA_KIND,
    URI_SENS_PRED_FIELD_OF_VIEW,
    URI_SENS_PRED_FRAME,
    URI_SENS_PRED_UPDATE_RATE,
    URI_SENS_TYPE_CAMERA,
    URI_SENS_TYPE_FORCE_TORQUE_SENSOR,
    URI_SENS_TYPE_IMU,
    URI_SOSA_PRED_HOSTS,
    URI_SOSA_PRED_OBSERVES,
    URI_SOSA_TYPE_PLATFORM,
    URI_SOSA_TYPE_SENSOR,
)

MODELS_DIR = Path(__file__).parents[1] / "examples" / "models"

ZERO_POSE = (
    "pose default_pose { xyz: (0.0, 0.0, 0.0) m orientation: euler { angles: (0.0, 0.0, 0.0) } }"
)


def test_scene_parses_and_generates_rdf():
    model = scene_metamodel().model_from_file(MODELS_DIR / "lab.scene")

    assert model.scene_models
    assert model.sim_obj_sets
    assert len(create_scene_model_graph(model)) > 0


def test_scenex_references_scene_and_generates_rdf():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_inst = model.scene_insts[0]
    table_obj = next(obj for obj in scene_inst.modelled_objs if obj.obj.name == "dining_table")
    table_body = table_obj.body

    graph = create_scenex_model_graph(model)
    assert (table_body.uri, RDF.type, URI_GEOM_TYPE_RIGID_BODY) in graph
    assert (table_body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MASS_SCALAR) in graph
    assert (table_obj.modelled_uri, URI_EXEC_PRED_LINKS_BODY, table_body.uri) in graph


def test_shared_workspace_composition_is_rejected(tmp_path):
    model_path = tmp_path / "shared_workspace.scene"
    model_path.write_text(
        """ns n = "https://example.test/"

obj set (ns=n) objs { object cup }
ws set (ns=n) wss { workspace root, workspace a, workspace b, workspace shared }
agn set (ns=n) agns { agent robot }

comp (ns=n) shared_comp of ws <wss.shared> {
    obj <objs.cup>
}
comp (ns=n) a_comp of ws <wss.a> {
    ws comp <shared_comp>
}
comp (ns=n) b_comp of ws <wss.b> {
    ws comp <shared_comp>
}
comp (ns=n) root_comp of ws <wss.root> {
    ws comp <a_comp>
    ws comp <b_comp>
}
scene (ns=n) dag_scene {
    obj set <objs>
    ws set <wss>
    ws comp <root_comp>
    agn set <agns>
}
"""
    )

    model = scene_metamodel().model_from_file(model_path)
    with pytest.raises(RuntimeError, match="Shared or cyclic workspace compositions"):
        create_scene_model_graph(model)


def _write_collision_scene(tmp_path: Path) -> None:
    (tmp_path / "collision.scene").write_text(
        """ns n = "https://example.test/"
obj set (ns=n) objs { object cup, object bowl }
ws set (ns=n) wss { workspace table }
agn set (ns=n) agns { agent robot }
scene (ns=n) s {
    obj set <objs>
    ws set <wss>
    agn set <agns>
}
"""
    )


def test_duplicate_model_uri_is_rejected(tmp_path):
    _write_collision_scene(tmp_path)
    model_path = tmp_path / "duplicate_model.scenex"
    model_path.write_text(
        """import "collision.scene"
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
    _write_collision_scene(tmp_path)
    model_path = tmp_path / "units.scenex"
    model_path.write_text(
        f"""import "collision.scene"
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
    _write_collision_scene(tmp_path)
    model_path = tmp_path / "mass_quantity.scenex"
    model_path.write_text(
        f"""import "collision.scene"
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


def _write_robot_scene(tmp_path: Path) -> None:
    (tmp_path / "robot.scene").write_text(
        """ns n = "https://example.test/"
obj set (ns=n) objs { object cup }
ws set (ns=n) wss { workspace table }
agn set (ns=n) agns { agent robot }
scene (ns=n) s {
    obj set <objs>
    ws set <wss>
    agn set <agns>
}
"""
    )


def test_lab_scenex_agent_tree_link_and_sensors_emit_rdf():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_inst = model.scene_insts[0]
    panda = next(agent for agent in scene_inst.modelled_agns if agent.agn.name == "panda")
    joint = next(joint for joint in panda.ktree.joints_spec.joints if joint.name == "panda_joint1")
    sensor = next(sensor for sensor in panda.sensors if sensor.name == "wrist_cam")
    ft_sensor = next(sensor for sensor in panda.sensors if sensor.name == "wrist_ft")
    imu_sensor = next(sensor for sensor in panda.sensors if sensor.name == "wrist_imu")

    assert panda.ktree in scene_inst.ktree.trees
    orientation = panda.ktree.bodies[0].frames[0].poses[0].orientation
    assert np.allclose(orientation.rotation_matrix, np.eye(3))

    graph = create_scenex_model_graph(model)

    assert (scene_inst.ktree.uri, RDF.type, URI_GEOM_TYPE_KTREE) in graph
    assert (panda.ktree.uri, RDF.type, URI_GEOM_TYPE_KTREE) in graph
    assert (joint.uri, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT) in graph
    assert (panda.ktree.uri, URI_KC_PRED_JOINTS, joint.uri) in graph
    assert (panda.modelled_uri, RDF.type, URI_SOSA_TYPE_PLATFORM) in graph
    for platform_sensor in (sensor, ft_sensor, imu_sensor):
        assert (platform_sensor.uri, RDF.type, URI_SOSA_TYPE_SENSOR) in graph
        assert (panda.modelled_uri, URI_SOSA_PRED_HOSTS, platform_sensor.uri) in graph
        assert (
            platform_sensor.uri,
            URI_EXEC_PRED_HAS_KINEMATICS,
            platform_sensor.frame.uri,
        ) in graph
        assert (platform_sensor.uri, URI_SENS_PRED_FRAME, platform_sensor.frame.uri) in graph

        update_rate = graph.value(platform_sensor.uri, URI_SENS_PRED_UPDATE_RATE)
        assert (update_rate, RDF.type, URI_QUDT_TYPE_QUANTITY) in graph
        assert (
            update_rate,
            URI_QUDT_PRED_VALUE,
            Literal(platform_sensor.update_rate, datatype=XSD.double),
        ) in graph

    assert (sensor.uri, RDF.type, URI_SENS_TYPE_CAMERA) in graph
    assert (sensor.uri, URI_SENS_PRED_CAMERA_KIND, CAMERA_TYPES[sensor.cam_type]) in graph
    assert (
        graph.value(sensor.uri, URI_SENS_PRED_FIELD_OF_VIEW),
        RDF.type,
        URI_QUDT_TYPE_QUANTITY,
    ) in graph

    assert (ft_sensor.uri, RDF.type, URI_SENS_TYPE_FORCE_TORQUE_SENSOR) in graph
    assert (imu_sensor.uri, RDF.type, URI_SENS_TYPE_IMU) in graph
    for platform_sensor in (ft_sensor, imu_sensor):
        for observed in platform_sensor.observes:
            assert (
                platform_sensor.uri,
                URI_SOSA_PRED_OBSERVES,
                OBSERVED_QUANTITIES[observed],
            ) in graph

    assert (
        panda.model.uri,
        URI_EXEC_PRED_PATH,
        Literal("../robot-models/franka_emika_panda/mjx_panda.xml"),
    ) in graph


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
        URI_KC_EXT_PRED_MULTIPLIER,
        Literal(2.0, datatype=XSD.double),
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
    _write_robot_scene(tmp_path)
    model_path = tmp_path / "bad_robot.scenex"
    model_path.write_text(
        """import "robot.scene"
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


def test_shared_distributions_generate_sampled_quantity_links():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "distributions.scenex")
    graph = create_scenex_model_graph(model)
    distributions = {distribution.name: distribution for distribution in model.distributions}
    uniform_xyz = distributions["uniform-xyz"]
    rotation = distributions["rot"]
    normal_xyz = distributions["normal-xyz"]
    normal_scalar = distributions["normal-scalar"]

    assert (uniform_xyz.uri, RDF.type, URI_DISTRIB_TYPE_DISTRIB) in graph
    assert (normal_xyz.uri, RDF.type, URI_DISTRIB_TYPE_NORMAL) in graph
    poses = model.scene_insts[0].ktree.bodies[1].frames[0].poses
    uniform_pose, normal_pose = poses
    assert isinstance(uniform_pose, PoseSpec)
    assert (uniform_pose.position_coord_uri, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY) in graph
    assert (
        uniform_pose.position_coord_uri,
        URI_DISTRIB_PRED_FROM_DISTRIB,
        uniform_xyz.uri,
    ) in graph
    assert (
        uniform_pose.orientation_coord_uri,
        URI_DISTRIB_PRED_FROM_DISTRIB,
        rotation.uri,
    ) in graph
    assert (normal_pose.position_coord_uri, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY) in graph
    assert (normal_pose.position_coord_uri, URI_DISTRIB_PRED_FROM_DISTRIB, normal_xyz.uri) in graph

    uniform_sample = sample_from_distrib(
        distrib_from_sampled_quantity(uniform_pose.position_coord_uri, graph), size=(4, 3)
    )
    assert uniform_sample.shape == (4, 3)
    assert np.all(uniform_sample >= np.asarray(uniform_xyz.spec.lower.values))
    assert np.all(uniform_sample <= np.asarray(uniform_xyz.spec.upper.values))

    normal_model = DistributionModel(distrib_id=normal_xyz.uri, graph=graph)
    normal_sample = sample_from_distrib(
        distrib_from_sampled_quantity(normal_pose.position_coord_uri, graph), size=20
    )
    assert normal_sample.shape == (20, 3)
    assert np.isfinite(normal_sample).all()
    assert normal_model.distrib_type == URI_DISTRIB_TYPE_NORMAL

    scalar_model = DistributionModel(distrib_id=normal_scalar.uri, graph=graph)
    scalar_sample = sample_from_distrib(distrib=scalar_model, size=8)
    assert scalar_sample.shape == (8,)
    assert np.isfinite(scalar_sample).all()

    pytest.importorskip("scipy")
    rotation_sample = sample_from_distrib(
        distrib_from_sampled_quantity(uniform_pose.orientation_coord_uri, graph)
    )
    assert rotation_sample.as_matrix().shape == (3, 3)


def test_non_three_dimensional_normal_is_rejected_for_xyz_sampling():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "distributions.scenex")
    next(
        distribution for distribution in model.distributions if distribution.name == "normal-xyz"
    ).spec.dimension = 2
    with pytest.raises(ValueError, match="dimension 3"):
        create_scenex_model_graph(model)
