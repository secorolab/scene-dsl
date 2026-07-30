# SPDX-License-Identifier: MPL-2.0
import json

from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import URI_EXEC_PRED_HAS_CONFIG
from rdflib import RDF, Graph, Literal, Node, URIRef


def load_attr_has_config(graph: Graph, model: ModelBase) -> None:
    serialized = graph.value(model.id, URI_EXEC_PRED_HAS_CONFIG, any=False)
    model.set_attr(
        URI_EXEC_PRED_HAS_CONFIG,
        {} if serialized is None else json.loads(str(serialized)),
    )


def _ensure_one_typed_obj_node(
    graph: Graph, subject: URIRef, predicate: URIRef, obj_type: type[Node]
) -> Node | None:
    values = list(graph.objects(subject, predicate))
    subj_rep = subject.n3(graph.namespace_manager)
    pred_rep = predicate.n3(graph.namespace_manager)
    if not values:
        return None

    if len(values) > 1 or not isinstance(values[0], obj_type):
        raise ValueError(
            f"Subject '{subj_rep}' must have exactly one '{obj_type.__name__}'"
            f" obj for predicate '{pred_rep}', found {values}"
        )

    return values[0]


def ensure_one_obj_uri(graph: Graph, subject: URIRef, predicate: URIRef) -> URIRef | None:
    uri = _ensure_one_typed_obj_node(
        graph=graph, subject=subject, predicate=predicate, obj_type=URIRef
    )
    if uri is None:
        return uri

    assert isinstance(uri, URIRef)
    return uri


def ensure_one_obj_literal(graph: Graph, subject: URIRef, predicate: URIRef) -> Literal | None:
    literal = _ensure_one_typed_obj_node(
        graph=graph, subject=subject, predicate=predicate, obj_type=Literal
    )
    if literal is None:
        return literal

    assert isinstance(literal, Literal)
    return literal


def ensure_one_typed_subject_uri(
    graph: Graph, obj: URIRef, predicate: URIRef, subject_type: URIRef
) -> URIRef | None:
    subjects = []
    for subj in graph.subjects(predicate=predicate, object=obj):
        if not isinstance(subj, URIRef):
            raise TypeError(
                f"Subject of obj '{obj}' via predicate '{predicate}' is not an URI: {subj}"
            )

        if (subj, RDF.type, subject_type) not in graph:
            # allow subjects of other types
            continue

        subjects.append(subj)

    if not subjects:
        return None

    if len(subjects) > 1:
        raise ValueError(
            f"Found multiple subjects for obj '{obj}' via predicate '{predicate}': {subjects}"
        )

    return subjects[0]
