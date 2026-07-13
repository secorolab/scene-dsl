# SPDX-License-Identifier: MPL-2.0
from rdflib import Namespace, RDF, Graph, Literal, URIRef, XSD

from bdd_dsl.models.namespace import NS_MM_EXEC
from rdf_utils.models.vocab import (
    URI_QUDT_PRED_QUANTITY_KIND,
    URI_QUDT_PRED_UNIT,
    URI_QUDT_PRED_VALUE,
    URI_QUDT_QK_ANG_VEL,
    URI_QUDT_QK_ANGLE,
    URI_QUDT_QK_FORCE,
    URI_QUDT_QK_FREQ,
    URI_QUDT_QK_LIN_ACCEL,
    URI_QUDT_QK_TORQUE,
    URI_QUDT_TYPE_QUANTITY,
)
from rdf_utils.namespace import NS_MM_QUDT_UNIT, URL_SECORO_MM

from scene_dsl.classes.sensors import CameraSensorSpec, ForceTorqueSensorSpec, ImuSensorSpec
from scene_dsl.rdf.geom import ANGLE_UNITS

NS_MM_SENS = Namespace(f"{URL_SECORO_MM}/robot/sensors#")
NS_SOSA = Namespace("http://www.w3.org/ns/sosa/")

URI_EXEC_PRED_HAS_KINEMATICS = NS_MM_EXEC["has-kinematics"]
URI_SENS_PRED_CAMERA_KIND = NS_MM_SENS["camera-kind"]
URI_SENS_PRED_FIELD_OF_VIEW = NS_MM_SENS["field-of-view"]
URI_SENS_PRED_FRAME = NS_MM_SENS["frame"]
URI_SENS_PRED_RESOLUTION_HEIGHT = NS_MM_SENS["resolution-height"]
URI_SENS_PRED_RESOLUTION_WIDTH = NS_MM_SENS["resolution-width"]
URI_SENS_PRED_UPDATE_RATE = NS_MM_SENS["update-rate"]
URI_SENS_TYPE_CAMERA = NS_MM_SENS["Camera"]
URI_SENS_TYPE_FORCE_TORQUE_SENSOR = NS_MM_SENS["ForceTorqueSensor"]
URI_SENS_TYPE_IMU = NS_MM_SENS["IMU"]
URI_SOSA_PRED_HOSTS = NS_SOSA["hosts"]
URI_SOSA_PRED_OBSERVES = NS_SOSA["observes"]
URI_SOSA_TYPE_PLATFORM = NS_SOSA["Platform"]
URI_SOSA_TYPE_SENSOR = NS_SOSA["Sensor"]

CAMERA_TYPES = {"rgb": NS_MM_SENS["rgb"], "depth": NS_MM_SENS["depth"], "rgbd": NS_MM_SENS["rgbd"]}
OBSERVED_QUANTITIES = {
    "force": URI_QUDT_QK_FORCE,
    "torque": URI_QUDT_QK_TORQUE,
    "angular-velocity": URI_QUDT_QK_ANG_VEL,
    "linear-acceleration": URI_QUDT_QK_LIN_ACCEL,
    "orientation": NS_MM_SENS["orientation"],
}


def _add_quantity(
    graph: Graph, uri: URIRef, value: float, unit: URIRef, quantity_kind: URIRef
) -> None:
    graph.add((uri, RDF.type, URI_QUDT_TYPE_QUANTITY))
    graph.add((uri, URI_QUDT_PRED_VALUE, Literal(value, datatype=XSD.double)))
    graph.add((uri, URI_QUDT_PRED_UNIT, unit))
    graph.add((uri, URI_QUDT_PRED_QUANTITY_KIND, quantity_kind))


def add_sensors(graph: Graph, agn_model) -> None:
    graph.add((agn_model.modelled_uri, RDF.type, URI_SOSA_TYPE_PLATFORM))
    for sensor in agn_model.sensors:
        update_rate_uri = URIRef(f"{sensor.uri}/update-rate")
        graph.add((sensor.uri, RDF.type, URI_SOSA_TYPE_SENSOR))
        graph.add((agn_model.modelled_uri, URI_SOSA_PRED_HOSTS, sensor.uri))
        graph.add((sensor.uri, URI_EXEC_PRED_HAS_KINEMATICS, sensor.frame.uri))
        graph.add((sensor.uri, URI_SENS_PRED_FRAME, sensor.frame.uri))
        graph.add((sensor.uri, URI_SENS_PRED_UPDATE_RATE, update_rate_uri))
        _add_quantity(
            graph=graph,
            uri=update_rate_uri,
            value=sensor.update_rate,
            unit=NS_MM_QUDT_UNIT["HZ"],
            quantity_kind=URI_QUDT_QK_FREQ,
        )

        if isinstance(sensor, CameraSensorSpec):
            fov_uri = URIRef(f"{sensor.uri}/field-of-view")
            width, height = sensor.resolution.as_width_height()
            graph.add((sensor.uri, RDF.type, URI_SENS_TYPE_CAMERA))
            graph.add((sensor.uri, URI_SENS_PRED_CAMERA_KIND, CAMERA_TYPES[sensor.cam_type]))
            graph.add(
                (sensor.uri, URI_SENS_PRED_RESOLUTION_WIDTH, Literal(width, datatype=XSD.integer))
            )
            graph.add(
                (sensor.uri, URI_SENS_PRED_RESOLUTION_HEIGHT, Literal(height, datatype=XSD.integer))
            )
            graph.add((sensor.uri, URI_SENS_PRED_FIELD_OF_VIEW, fov_uri))
            _add_quantity(
                graph=graph,
                uri=fov_uri,
                value=sensor.fov,
                unit=ANGLE_UNITS[sensor.fov_unit],
                quantity_kind=URI_QUDT_QK_ANGLE,
            )
            continue

        if isinstance(sensor, ForceTorqueSensorSpec):
            graph.add((sensor.uri, RDF.type, URI_SENS_TYPE_FORCE_TORQUE_SENSOR))
        elif isinstance(sensor, ImuSensorSpec):
            graph.add((sensor.uri, RDF.type, URI_SENS_TYPE_IMU))
        else:
            raise ValueError(f"Unsupported sensor type: {sensor}")

        for observed in sensor.observes:
            graph.add((sensor.uri, URI_SOSA_PRED_OBSERVES, OBSERVED_QUANTITIES[observed]))
