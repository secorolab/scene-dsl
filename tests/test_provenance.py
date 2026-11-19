# SPDX-License-Identifier: MPL-2.0
"""Tests for the generation steps and the provenance they add to a bundle."""

from importlib.resources import files
from pathlib import Path

from rdf_utils.models.prov import resolve_location
from rdf_utils.models.vocab import URI_PROV_EXT_TYPE_TRANSFORMATION
from rdf_utils.resolver import install_resolver
from rdflib import Dataset, URIRef
from rdflib.namespace import PROV, RDF

from scene_dsl.gens import KDL_TEMPLATE, scenex_graph_gen, scenex_kdl_gen
from scene_dsl.langs import scenex_metamodel

from .test_common import MODELS_DIR

MODEL = MODELS_DIR / "lab.scenex"


def test_generators_save_the_bundle_only_when_asked(tmp_path):
    install_resolver()
    model = scenex_metamodel().model_from_file(MODEL)
    default_dir, prov_dir = tmp_path / "default", tmp_path / "provenance"
    default_dir.mkdir()
    prov_dir.mkdir()
    scenex_kdl_gen(None, model, str(default_dir), False, False)
    scenex_graph_gen(None, model, str(default_dir), False, False)
    scenex_graph_gen(None, model, str(prov_dir), False, False, provenance=True)

    assert {path.name for path in default_dir.iterdir()} == {"lab.kdl.hpp", "lab.scenex.ld.json"}
    saved = Dataset().parse(default_dir / "lab.scenex.ld.json", format="json-ld")
    assert not any((None, RDF.type, PROV.Bundle) in graph for graph in saved.graphs())

    assert {path.name for path in prov_dir.iterdir()} == {"lab.scenex.ld.json"}
    saved = Dataset().parse(prov_dir / "lab.scenex.ld.json", format="json-ld")
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
    template = Path(str(files("scene_dsl").joinpath(KDL_TEMPLATE)))
    assert URIRef(template.as_uri()) in {resolve_location(bundle, source) for source in used}
