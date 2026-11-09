# SPDX-License-Identifier: MPL-2.0
"""The generation steps of scene-dsl, each recording itself in a bundle in the dataset it is given.

``build`` turns a model into a graph, and ``render`` turns that graph and a template into a KDL
header. Both work in one ``Dataset``: the scene graph and the ``prov:Bundle`` are named graphs in
it, and each step adds itself to the bundle as a ``prov-ext:Transformation`` associated with the
package, which the caller states in the bundle first. Files are named by their location, with the
hash of their bytes as ``dcterms:identifier``. Neither step writes the dataset.

The package, named by its purl, is the agent, though PROV-DM 2.2.1.1 defines a software agent as
running software.
"""

from collections.abc import Callable
from datetime import UTC, datetime
from importlib import resources
from pathlib import Path
from typing import Any
from uuid import uuid4

from rdf_utils.models.prov import (
    add_directory_location,
    add_entity,
    add_file_entity,
    add_file_hash,
    create_bundle,
    get_git_info,
    get_pkg_info,
    load_pkg_prov,
    load_transformation_prov,
)
from rdf_utils.models.time import add_wall_instant
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import SDO
from textx import get_location
from textx.scoping import get_included_models

from scene_dsl.langs import package_import

KDL_TEMPLATE = "templates/kdl.hpp.jinja2"


def create_scene_dsl_bundle(dataset: Dataset) -> tuple[URIRef, URIRef]:
    """Create a bundle for scene_dsl in a dataset, stating the package as its agent.

    Returns:
        The IDs of the bundle and of the package, to pass to `build` and `render`
    """
    purl, version, revision, repository, dirty = get_pkg_info("scene_dsl")
    bundle_id = create_bundle(dataset, purl)
    load_pkg_prov(
        dataset.graph(bundle_id), purl, "scene_dsl", version, revision, repository, dirty=dirty
    )
    return bundle_id, purl


def source_paths(model) -> dict[Path, str | None]:
    """The file of a directly loaded model and its imports, with the package shipping each."""
    models = get_included_models(model)
    packages = {
        imp.importURI.partition(":")[0]
        for item in models
        for imp in getattr(item, "imports", [])
        if package_import(imp.importURI) != imp.importURI
    }
    package_dirs = {package: Path(str(resources.files(package))).resolve() for package in packages}
    return {
        path: next(
            (package for package, root in package_dirs.items() if path.is_relative_to(root)), None
        )
        for path in (Path(get_location(item)["filename"]).resolve() for item in models)
    }


def _add_transformation(
    bundle: Graph,
    agent: URIRef,
    activity: URIRef,
    sources: list[URIRef],
    targets: list[URIRef],
    started_at: datetime,
    ended_at: datetime,
) -> None:
    """Add a transformation by an agent to a bundle, with its wall-clock instants."""
    beginning, end = URIRef(uuid4().urn), URIRef(uuid4().urn)
    add_wall_instant(bundle, beginning, started_at)
    add_wall_instant(bundle, end, ended_at)
    load_transformation_prov(bundle, activity, sources, targets, agent, beginning, end)


def build(
    model,
    dataset: Dataset,
    bundle_id: URIRef,
    agent_id: URIRef,
    create_graph: Callable[[Any, Graph], Graph],
    graph_id: URIRef | None = None,
) -> URIRef:
    """Build the graph of a model in a dataset and add how it was made to a bundle.

    Parameters:
        model: the textX model, loaded from a file; that file and its imports are the sources
        dataset: the dataset holding the scene graph and the bundle
        bundle_id: the `prov:Bundle` in the dataset to record in
        agent_id: the scene_dsl package, stated in the bundle with `load_pkg_prov`
        create_graph: the language's graph builder, called with the model and the graph to fill
        graph_id: the graph to build into; a new `urn:uuid:` when None

    Returns:
        The IRI of the scene graph
    """
    graph_id = URIRef(uuid4().urn) if graph_id is None else graph_id
    bundle = dataset.graph(bundle_id)
    started_at = datetime.now(UTC)
    create_graph(model, dataset.graph(graph_id))
    ended_at = datetime.now(UTC)

    model_dir = Path(get_location(model)["filename"]).resolve().parent
    root = add_directory_location(bundle, model_dir)
    sources = []
    for path, package in source_paths(model).items():
        if package is None:
            source = add_file_entity(bundle, root, path.relative_to(model_dir, walk_up=True))
            commit, repository, _, dirty = get_git_info(path)
            if commit is not None:
                revision = f"{commit}-dirty" if dirty else commit
                bundle.add((source, SDO.identifier, Literal(revision)))
            if repository is not None:
                bundle.add((source, SDO.codeRepository, URIRef(repository)))
        else:
            purl, version, revision, repository, dirty = get_pkg_info(package)
            load_pkg_prov(bundle, purl, package, version, revision, repository, dirty=dirty)
            package_dir = Path(str(resources.files(package))).resolve()
            source = add_file_entity(bundle, purl, path.relative_to(package_dir))
        add_file_hash(bundle, source)
        sources.append(source)
    activity = URIRef(uuid4().urn)
    _add_transformation(bundle, agent_id, activity, sources, [graph_id], started_at, ended_at)
    return graph_id


def render(
    dataset: Dataset,
    graph_id: URIRef,
    bundle_id: URIRef,
    agent_id: URIRef,
    create_header: Callable[[Graph], str],
    output_file: Path | str,
) -> None:
    """Render a scene graph in a dataset to a KDL header and add how it was made to a bundle.

    Parameters:
        dataset: the dataset holding the scene graph and the bundle
        graph_id: the scene graph to render, e.g. from `build`
        bundle_id: the `prov:Bundle` in the dataset to record in
        agent_id: the scene_dsl package, stated in the bundle with `load_pkg_prov`; its
            template is found under it
        create_header: renders the template for a graph
        output_file: the file to write the header to

    Raises:
        FileNotFoundError: when the template is missing; nothing is written
    """
    bundle = dataset.graph(bundle_id)
    started_at = datetime.now(UTC)
    output_file = Path(output_file).resolve()
    template_id = add_file_entity(bundle, agent_id, KDL_TEMPLATE)
    # Only the template: create_header also reads model files, whose absence is a different error.
    try:
        add_file_hash(bundle, template_id)
    except FileNotFoundError as error:
        raise FileNotFoundError(
            f"KDL template not found -- scene_dsl install is incomplete: {error}"
        ) from error
    output_file.write_text(create_header(dataset.graph(graph_id)))
    ended_at = datetime.now(UTC)

    add_entity(bundle, graph_id)
    activity = URIRef(uuid4().urn)
    output_id = add_file_entity(
        bundle,
        add_directory_location(bundle, output_file.parent),
        output_file.name,
        generated_by=activity,
    )
    add_file_hash(bundle, output_id)
    _add_transformation(
        bundle, agent_id, activity, [graph_id, template_id], [output_id], started_at, ended_at
    )
