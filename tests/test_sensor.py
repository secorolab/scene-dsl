import numpy as np
from rdflib import RDF, Literal, URIRef, XSD

from rdf_utils.models.vocab import (
    URI_KC_PRED_JOINTS,
    URI_KC_TYPE_REVOLUTE_JOINT,
    URI_QUDT_TYPE_QUANTITY,
)
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf.ktree import URI_GEOM_TYPE_KTREE
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
from .test_common import MODELS_DIR


def test_lab_scenex_agent_tree_link_and_sensors_emit_rdf():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_inst = model.scene_insts[0]
    panda = next(agent for agent in scene_inst.modelled_agns if agent.agn.name == "panda")
    joint = next(joint for joint in panda.ktree.joints_spec.joints if joint.name == "panda_joint1")
    sensor = next(sensor for sensor in panda.sensors if sensor.name == "wrist_cam")
    ft_sensor = next(sensor for sensor in panda.sensors if sensor.name == "wrist_ft")
    imu_sensor = next(sensor for sensor in panda.sensors if sensor.name == "wrist_imu")

    assert panda.ktree in scene_inst.kgraph.trees[0].trees
    orientation = panda.ktree.bodies[0].frames[0].poses[0].orientation
    assert np.allclose(orientation.rotation_matrix, np.eye(3))

    graph = create_scenex_model_graph(model)

    assert (scene_inst.kgraph.trees[0].uri, RDF.type, URI_GEOM_TYPE_KTREE) in graph
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


def test_two_agents_may_each_carry_a_wrist_camera(tmp_path):
    """A sensor is scoped by its agent, so one name serves every robot that has one."""
    (tmp_path / "two.scene").write_text(
        """ns n = "https://example.test/"
obj set (ns=n) objs { object cup }
ws set (ns=n) wss { workspace table }
agn set (ns=n) agns { agent robot, agent helper }
scene (ns=n) s { obj set <objs> ws set <wss> agn set <agns> }
"""
    )
    path = tmp_path / "two.scenex"
    path.write_text(
        """import "two.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    agn <agns.robot> {
        model r-mjc as mjcf { sys path = "r.xml" }
        ktree (ns=n) r_tree { body r_body { frame r_wrist { } } joints { } }
        camera wrist { frame: <r_tree.r_body.r_wrist> type: rgb resolution: (640, 480)
            fov: 1.2 rad update-rate: 30.0 Hz }
    }
    agn <agns.helper> {
        model h-mjc as mjcf { sys path = "h.xml" }
        ktree (ns=n) h_tree { body h_body { frame h_wrist { } } joints { } }
        camera wrist { frame: <h_tree.h_body.h_wrist> type: rgb resolution: (640, 480)
            fov: 1.2 rad update-rate: 30.0 Hz }
    }
}
"""
    )
    model = scenex_metamodel().model_from_file(path)
    graph = create_scenex_model_graph(model)
    cams = {a.agn.name: a.sensors[0].uri for a in model.scene_insts[0].modelled_agns}
    assert cams["robot"] == URIRef("https://example.test/sx/robot/wrist")
    assert cams["helper"] == URIRef("https://example.test/sx/helper/wrist")
    for uri in cams.values():
        assert (uri, URI_SENS_PRED_FRAME, None) in graph
