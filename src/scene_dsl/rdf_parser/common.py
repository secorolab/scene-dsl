# SPDX-License-Identifier: MPL-2.0
import json
from dataclasses import dataclass

from rdf_utils.models.common import ModelBase, get_node_types
from rdf_utils.models.vocab import (
    URI_EXEC_PRED_HAS_CONFIG,
    URI_EXEC_PRED_HAS_MAPPING,
    URI_EXEC_PRED_MAPS,
    URI_EXEC_PRED_MODEL_ENTITY,
    URI_GEOM_TYPE_KTREE,
    URI_GEOM_TYPE_RIGID_BODY,
)
from rdflib import RDF, Graph, Literal, Node, URIRef


def load_attr_has_config(graph: Graph, model: ModelBase, **kwargs) -> None:
    serialized = graph.value(model.id, URI_EXEC_PRED_HAS_CONFIG, any=False)
    if serialized is not None:
        model.set_attr(URI_EXEC_PRED_HAS_CONFIG, json.loads(str(serialized)))


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


# What a model may map: a scene names a body or a whole tree, nothing else.
MAPPABLE_TYPES = {URI_GEOM_TYPE_RIGID_BODY, URI_GEOM_TYPE_KTREE}


@dataclass
class KinematicMapping:
    target_id: URIRef
    target_type: URIRef
    entity: str | None = None


def get_kinematic_mapping(mapping_id: URIRef, graph: Graph) -> KinematicMapping:
    target_uri = ensure_one_obj_uri(graph=graph, subject=mapping_id, predicate=URI_EXEC_PRED_MAPS)
    if target_uri is None:
        raise ValueError(f"mapping '{mapping_id}' does not map to an URI target")

    target_types = get_node_types(graph=graph, node_id=target_uri) & MAPPABLE_TYPES
    if len(target_types) != 1:
        raise ValueError(f"mapping '{mapping_id}' target '{target_uri}' must be one body or tree")

    entity_literal = ensure_one_obj_literal(
        graph=graph, subject=mapping_id, predicate=URI_EXEC_PRED_MODEL_ENTITY
    )

    entity = None
    if entity_literal is not None:
        entity = entity_literal.toPython()

        if not isinstance(entity, str):
            raise TypeError(f"mapping '{mapping_id}' entity must be a string")

    return KinematicMapping(target_uri, target_types.pop(), entity=entity)


def load_attr_kinematic_mappings(graph: Graph, model: ModelBase, **kwargs: object) -> None:
    mappings = []
    targets = set()
    for mapping_id in graph.objects(model.id, URI_EXEC_PRED_HAS_MAPPING):
        if not isinstance(mapping_id, URIRef):
            raise TypeError(f"model '{model}' has non-URI mapping: {mapping_id}")
        mapping = get_kinematic_mapping(mapping_id, graph)
        if mapping.target_id in targets:
            raise ValueError(f"multiple mappings found for target {mapping.target_id}")
        targets.add(mapping.target_id)
        mappings.append(mapping)
    model.set_attr(URI_EXEC_PRED_HAS_MAPPING, tuple(mappings))


def get_kinematic_mappings(
    model: ModelBase, target_type: URIRef | None = None
) -> tuple[KinematicMapping, ...]:
    mappings = model.get_attr(URI_EXEC_PRED_HAS_MAPPING)
    if not isinstance(mappings, tuple) or not all(
        isinstance(mapping, KinematicMapping) for mapping in mappings
    ):
        raise TypeError(f"model '{model}' has no loaded mappings")
    if target_type is None:
        return mappings
    return tuple(mapping for mapping in mappings if mapping.target_type == target_type)
