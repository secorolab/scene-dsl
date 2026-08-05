# SPDX-License-Identifier: MPL-2.0
from math import isfinite

from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import URI_QUDT_PRED_VALUE
from rdflib import Graph, Literal, URIRef

from scene_dsl.rdf.sensors import URI_SENS_PRED_UPDATE_RATE


def get_update_rate(graph: Graph, model: ModelBase) -> float:
    value = graph.value(model.id, URI_SENS_PRED_UPDATE_RATE, any=False)
    if isinstance(value, URIRef):
        value = graph.value(value, URI_QUDT_PRED_VALUE, any=False)
    rate = value.toPython() if isinstance(value, Literal) else None
    if (
        isinstance(rate, bool)
        or not isinstance(rate, (int, float))
        or not isfinite(rate)
        or rate <= 0
    ):
        raise ValueError(f"model '{model.id}' has invalid update-rate: {rate}")
    return float(rate)
