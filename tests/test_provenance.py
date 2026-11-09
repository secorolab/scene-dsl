# SPDX-License-Identifier: MPL-2.0
"""Tests for the generation steps and the provenance they add to a bundle."""

from importlib.resources import files
from pathlib import Path

from rdf_utils.constraints import check_shacl_constraints
from rdf_utils.models.prov import resolve_location
from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_TRANSFORMATION
from rdf_utils.namespace import URL_MM_PROV_EXT_SHACL, URL_MM_PROV_SHACL
from rdf_utils.resolver import install_resolver
from rdflib import Dataset, URIRef
from rdflib.namespace import DCTERMS, PROV, RDF, SDO, TIME

from scene_dsl.gens import kdl_header, scenex_graph_gen, scenex_kdl_gen
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.provenance import (
    KDL_TEMPLATE,
    build,
    create_scene_dsl_bundle,
    render,
    source_paths,
)
from scene_dsl.rdf.scenex import create_scenex_model_graph

from .test_common import MODELS_DIR

MODEL = MODELS_DIR / "lab.scenex"
SHAPES = {URL_MM_PROV_SHACL: "turtle", URL_MM_PROV_EXT_SHACL: "turtle"}


def test_build_records_the_graph_generated_from_the_model_and_its_imports():
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    dataset = Dataset()
    bundle_id, agent_id = create_scene_dsl_bundle(dataset)
    graph_id = build(model, dataset, bundle_id, agent_id, create_scenex_model_graph)

    bundle = dataset.graph(bundle_id)
    (activity,) = bundle.subjects(RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION)
    (entity,) = bundle.subjects(PROV.wasGeneratedBy, activity)
    paths = source_paths(model)
    assert entity == graph_id
    assert len(dataset.graph(graph_id)) > 0
    assert len(paths) > 1
    used = set(bundle.objects(activity, PROV.used))
    assert {resolve_location(bundle, source) for source in used} == {
        URIRef(path.as_uri()) for path in paths
    }
    assert all(bundle.value(source, DCTERMS.identifier) is not None for source in used)
    assert (activity, PROV.wasAssociatedWith, agent_id) in bundle
    assert str(bundle.value(agent_id, SDO.name)) == "scene_dsl"
    assert bundle.value(activity, TIME.hasBeginning) is not None
    assert bundle.value(activity, TIME.hasEnd) is not None
    check_shacl_constraints(bundle, SHAPES)


def test_render_uses_the_graph_and_the_template(tmp_path):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    dataset = Dataset()
    graph_id = build(model, dataset, *create_scene_dsl_bundle(dataset), create_scenex_model_graph)
    bundle_id, agent_id = create_scene_dsl_bundle(dataset)
    output_file = tmp_path.resolve() / "lab.kdl.hpp"
    render(
        dataset,
        graph_id,
        bundle_id,
        agent_id,
        lambda graph: kdl_header(graph, "lab", "lab.scenex"),
        output_file,
    )

    bundle = dataset.graph(bundle_id)
    template = Path(str(files("scene_dsl").joinpath(KDL_TEMPLATE)))
    (activity,) = bundle.subjects(RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION)
    (entity,) = bundle.subjects(PROV.wasGeneratedBy, activity)
    (template_id,) = set(bundle.objects(activity, PROV.used)) - {graph_id}
    assert bundle.value(entity, DCTERMS.identifier) is not None
    assert bundle.value(template_id, DCTERMS.identifier) is not None
    assert resolve_location(bundle, entity) == URIRef(output_file.as_uri())
    assert resolve_location(bundle, template_id) == URIRef(template.as_uri())
    check_shacl_constraints(bundle, SHAPES)


def test_generators_write_only_their_output_by_default(tmp_path):
    model = scenex_metamodel().model_from_file(MODEL)
    scenex_kdl_gen(None, model, str(tmp_path), False, False)
    scenex_graph_gen(None, model, str(tmp_path), False, False)
    assert {path.name for path in tmp_path.iterdir()} == {"lab.kdl.hpp", "lab.scenex.ld.json"}
    saved = Dataset().parse(tmp_path / "lab.scenex.ld.json", format="json-ld")
    assert not any((None, RDF.type, PROV.Bundle) in graph for graph in saved.graphs())


def test_graph_generator_saves_the_bundle_beside_the_graph_when_asked(tmp_path):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    scenex_graph_gen(None, model, str(tmp_path), False, False, provenance=True)

    assert {path.name for path in tmp_path.iterdir()} == {"lab.scenex.ld.json"}
    saved = Dataset().parse(tmp_path / "lab.scenex.ld.json", format="json-ld")
    (bundle,) = [g for g in saved.graphs() if (None, RDF.type, PROV.Bundle) in g]
    (activity,) = bundle.subjects(RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION)
    (graph_id,) = bundle.subjects(PROV.wasGeneratedBy, activity)
    assert len(saved.graph(graph_id)) > 0


def test_kdl_generator_records_both_steps_in_one_bundle_when_asked(tmp_path):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    scenex_kdl_gen(None, model, str(tmp_path), False, False, provenance=True)

    assert {path.name for path in tmp_path.iterdir()} == {"lab.kdl.hpp", "lab.kdl.hpp.ld.json"}
    record = Dataset().parse(tmp_path / "lab.kdl.hpp.ld.json", format="json-ld")
    (bundle,) = [g for g in record.graphs() if (None, RDF.type, PROV.Bundle) in g]
    generated = set(bundle.subjects(PROV.wasGeneratedBy))
    used = set(bundle.objects(predicate=PROV.used))
    (graph_id,) = generated & used
    assert len(record.graph(graph_id)) > 0
