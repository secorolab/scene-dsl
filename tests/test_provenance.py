# SPDX-License-Identifier: MPL-2.0
"""Tests for the generation steps and the provenance they add to a bundle."""

from importlib.resources import files
from pathlib import Path

import pytest
from rdf_utils.constraints import check_shacl_constraints
from rdf_utils.models.prov import create_bundle, get_pkg_info, load_pkg_prov, resolve_location
from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_TRANSFORMATION
from rdf_utils.namespace import URL_MM_PROV_EXT_SHACL, URL_MM_PROV_SHACL
from rdf_utils.resolver import install_resolver
from rdflib import Dataset, URIRef
from rdflib.namespace import DCTERMS, PROV, RDF, SDO, TIME

from scene_dsl.gens import kdl_header, scenex_graph_gen, scenex_kdl_gen
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.provenance import KDL_TEMPLATE, build, render, source_paths
from scene_dsl.rdf.scenex import create_scenex_model_graph

from .test_common import TEST_MODELS_DIR

MODEL = TEST_MODELS_DIR / "lab.scenex"
SHAPES = {URL_MM_PROV_SHACL: "turtle", URL_MM_PROV_EXT_SHACL: "turtle"}


@pytest.fixture
def bundle():
    agent, version, revision, repository, dirty = get_pkg_info("scene_dsl")
    graph = create_bundle(agent)
    load_pkg_prov(graph, agent, "scene_dsl", version, revision, repository, dirty=dirty)
    return graph


def test_build_records_the_graph_generated_from_the_model_and_its_imports(bundle):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    graph = build(model, bundle, create_scenex_model_graph)

    agent = bundle.value(bundle.identifier, PROV.wasAttributedTo)
    (activity,) = bundle.subjects(RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION)
    (entity,) = bundle.subjects(PROV.wasGeneratedBy, activity)
    paths = source_paths(model)
    assert entity == graph.identifier
    assert len(paths) > 1
    used = set(bundle.objects(activity, PROV.used))
    assert {resolve_location(bundle, source) for source in used} == {
        URIRef(path.as_uri()) for path in paths
    }
    assert all(bundle.value(source, DCTERMS.identifier) is not None for source in used)
    assert (activity, PROV.wasAssociatedWith, agent) in bundle
    assert str(bundle.value(agent, SDO.name)) == "scene_dsl"
    assert bundle.value(activity, TIME.hasBeginning) is not None
    assert bundle.value(activity, TIME.hasEnd) is not None
    check_shacl_constraints(bundle, SHAPES)


def test_build_records_a_saved_file_as_derived_from_the_graph(tmp_path, bundle):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    graph_file = tmp_path.resolve() / "lab.scenex.ld.json"
    graph = build(model, bundle, create_scenex_model_graph, graph_file)

    (activity,) = bundle.subjects(RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION)
    (file_id,) = set(bundle.subjects(PROV.wasGeneratedBy, activity)) - {graph.identifier}
    assert bundle.value(file_id, DCTERMS.identifier) is not None
    assert (file_id, PROV.wasDerivedFrom, graph.identifier) in bundle
    assert resolve_location(bundle, file_id) == URIRef(graph_file.as_uri())
    saved = Dataset().parse(graph_file, format="json-ld")
    assert graph.identifier in {g.identifier for g in saved.graphs()}
    check_shacl_constraints(bundle, SHAPES)


def test_render_uses_the_graph_and_the_template(tmp_path, bundle):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    agent = bundle.value(bundle.identifier, PROV.wasAttributedTo)
    graph = build(model, create_bundle(agent), create_scenex_model_graph)
    output_file = render(
        graph,
        bundle,
        lambda g: kdl_header(g, "lab", "lab.scenex"),
        tmp_path.resolve() / "lab.kdl.hpp",
    )

    template = Path(str(files("scene_dsl").joinpath(KDL_TEMPLATE)))
    (activity,) = bundle.subjects(RDF.type, URI_PROV_EXT_TYPE_TRANSFORMATION)
    (entity,) = bundle.subjects(PROV.wasGeneratedBy, activity)
    (template_id,) = set(bundle.objects(activity, PROV.used)) - {graph.identifier}
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


def test_kdl_generator_records_both_steps_in_one_bundle_when_asked(tmp_path):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    scenex_kdl_gen(None, model, str(tmp_path), False, False, provenance=True)

    assert {path.name for path in tmp_path.iterdir()} == {
        "lab.kdl.hpp",
        "lab.kdl.hpp.prov.ld.json",
    }
    record = Dataset().parse(tmp_path / "lab.kdl.hpp.prov.ld.json", format="json-ld")
    (bundle,) = [g for g in record.graphs() if (None, RDF.type, PROV.Bundle) in g]
    generated = set(bundle.subjects(PROV.wasGeneratedBy))
    used = set(bundle.objects(predicate=PROV.used))
    assert len(generated & used) == 1
