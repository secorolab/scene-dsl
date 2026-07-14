# SPDX-License-Identifier: MPL-2.0
from collections.abc import Iterable

from rdflib import RDF, Graph, Literal, URIRef, XSD

from rdf_utils.collection import add_literal_list_pred
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_DIM,
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_PRED_LOWER,
    URI_DISTRIB_PRED_UPPER,
    URI_DISTRIB_TYPE_DISTRIB,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DISTRIB_TYPE_UNIFORM,
    URI_DISTRIB_TYPE_UNIFORM_ROT,
)
from scene_dsl.classes.distrib import (
    Distribution,
    DistributionRef,
    UniformDistribution,
    UniformRotationDistribution,
)


def add_distribution(graph: Graph, distribution: Distribution) -> None:
    graph.add((distribution.uri, RDF.type, URI_DISTRIB_TYPE_DISTRIB))
    if isinstance(distribution.spec, UniformDistribution):
        graph.add((distribution.uri, RDF.type, URI_DISTRIB_TYPE_UNIFORM))
        graph.add(
            (distribution.uri, URI_DISTRIB_PRED_DIM, Literal(3, datatype=XSD.positiveInteger))
        )
        add_literal_list_pred(
            graph=graph,
            subject_uri=distribution.uri,
            pred_uri=URI_DISTRIB_PRED_LOWER,
            values=distribution.spec.lower,
        )
        add_literal_list_pred(
            graph=graph,
            subject_uri=distribution.uri,
            pred_uri=URI_DISTRIB_PRED_UPPER,
            values=distribution.spec.upper,
        )
    elif isinstance(distribution.spec, UniformRotationDistribution):
        graph.add((distribution.uri, RDF.type, URI_DISTRIB_TYPE_UNIFORM_ROT))
    else:
        raise ValueError(f"Unsupported distribution specification: {distribution.spec}")


def add_distributions(graph: Graph, distributions: Iterable[Distribution]) -> None:
    for distribution in distributions:
        add_distribution(graph=graph, distribution=distribution)


def add_sampled_quantity(graph: Graph, quantity_uri: URIRef, distrib_ref: DistributionRef) -> None:
    graph.add((quantity_uri, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY))
    graph.add((quantity_uri, URI_DISTRIB_PRED_FROM_DISTRIB, distrib_ref.distribution.uri))
