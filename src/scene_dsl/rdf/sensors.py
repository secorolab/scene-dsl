# SPDX-License-Identifier: MPL-2.0
from rdflib import RDF, Graph, Literal

from bdd_dsl.models.namespace import NS_MM_EXEC
from rdf_utils.namespace import NS_MM_AGN

URI_EXEC_PRED_HAS_KINEMATICS = NS_MM_EXEC["has-kinematics"]
URI_AGN_PRED_HAS_SENSOR = NS_MM_AGN["has-sensor"]
URI_AGN_PRED_HEIGHT = NS_MM_AGN["height"]
URI_AGN_PRED_OBSERVES = NS_MM_AGN["observes"]
URI_AGN_PRED_UPDATE_RATE = NS_MM_AGN["update-rate"]
URI_AGN_PRED_WIDTH = NS_MM_AGN["width"]


def add_sensors(graph: Graph, agn_model) -> None:
    for sensor in agn_model.sensors:
        graph.add((sensor.uri, RDF.type, NS_MM_AGN[sensor.__class__.__name__]))
        graph.add((agn_model.modelled_uri, URI_AGN_PRED_HAS_SENSOR, sensor.uri))
        graph.add((sensor.uri, URI_EXEC_PRED_HAS_KINEMATICS, sensor.body.uri))
        graph.add((sensor.uri, URI_AGN_PRED_UPDATE_RATE, Literal(sensor.update_rate)))
        if hasattr(sensor, "width"):
            graph.add((sensor.uri, URI_AGN_PRED_WIDTH, Literal(sensor.width)))
            graph.add((sensor.uri, URI_AGN_PRED_HEIGHT, Literal(sensor.height)))
        if hasattr(sensor, "observes"):
            for observed in sensor.observes:
                graph.add((sensor.uri, URI_AGN_PRED_OBSERVES, Literal(observed)))
