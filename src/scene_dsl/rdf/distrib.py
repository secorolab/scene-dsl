# SPDX-License-Identifier: MPL-2.0
from collections.abc import Iterable

from rdflib import BNode, RDF, Graph, Literal, URIRef, XSD
from rdflib.collection import Collection

from rdf_utils.collection import add_literal_list_pred, add_node_list_pred
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_DIM,
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_PRED_LOWER,
    URI_DISTRIB_PRED_MEAN,
    URI_DISTRIB_PRED_STD,
    URI_DISTRIB_PRED_COV,
    URI_DISTRIB_PRED_UPPER,
    URI_DISTRIB_TYPE_DISTRIB,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DISTRIB_TYPE_NORMAL,
    URI_DISTRIB_TYPE_UNIFORM,
    URI_DISTRIB_TYPE_UNIFORM_ROT,
)
from scene_dsl.classes.distrib import (
    Distribution,
    DistributionRef,
    NormalDistribution,
    UniformDistribution,
    UniformRotationDistribution,
)


def add_distribution(graph: Graph, distribution: Distribution) -> None:
    graph.add((distribution.uri, RDF.type, URI_DISTRIB_TYPE_DISTRIB))
    if isinstance(distribution.spec, UniformDistribution):
        graph.add((distribution.uri, RDF.type, URI_DISTRIB_TYPE_UNIFORM))
        graph.add(
            (
                distribution.uri,
                URI_DISTRIB_PRED_DIM,
                Literal(distribution.spec.dimension, datatype=XSD.positiveInteger),
            )
        )
        add_literal_list_pred(
            graph=graph,
            subject_uri=distribution.uri,
            pred_uri=URI_DISTRIB_PRED_LOWER,
            values=distribution.spec.lower.values,
        )
        add_literal_list_pred(
            graph=graph,
            subject_uri=distribution.uri,
            pred_uri=URI_DISTRIB_PRED_UPPER,
            values=distribution.spec.upper.values,
        )
    elif isinstance(distribution.spec, NormalDistribution):
        graph.add((distribution.uri, RDF.type, URI_DISTRIB_TYPE_NORMAL))
        graph.add(
            (
                distribution.uri,
                URI_DISTRIB_PRED_DIM,
                Literal(distribution.spec.dimension, datatype=XSD.positiveInteger),
            )
        )
        if distribution.spec.mean_vector is not None:
            add_literal_list_pred(
                graph=graph,
                subject_uri=distribution.uri,
                pred_uri=URI_DISTRIB_PRED_MEAN,
                values=distribution.spec.mean_vector.values,
            )
        elif distribution.spec.mean_scalar is not None:
            graph.add(
                triple=(
                    distribution.uri,
                    URI_DISTRIB_PRED_MEAN,
                    Literal(distribution.spec.mean_scalar),
                )
            )
        else:
            raise ValueError(
                f"Neither mean_scalar or mean_vector specified for NormalDistribution {distribution.name}"
            )

        if distribution.spec.std_dev is not None:
            graph.add((distribution.uri, URI_DISTRIB_PRED_STD, Literal(distribution.spec.std_dev)))
        elif distribution.spec.covariance is not None:
            row_nodes = []
            for row in distribution.spec.covariance.values:
                row_nodes.append(Collection(graph, BNode(), [Literal(value) for value in row]).uri)
            add_node_list_pred(
                graph=graph,
                subject_uri=distribution.uri,
                pred_uri=URI_DISTRIB_PRED_COV,
                nodes=row_nodes,
            )
        else:
            raise ValueError("NormalDistribution requires std-dev or covariance")

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
