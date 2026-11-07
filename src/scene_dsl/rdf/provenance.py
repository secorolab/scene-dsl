# SPDX-License-Identifier: MPL-2.0
"""The generation steps of scene-dsl, each recording itself in the bundle it is given.

``build`` turns a model into a graph, saved when a file is given, and ``render`` turns that graph
and a template into a KDL header. Each adds itself as a ``prov-ext:Transformation`` to the
``prov:Bundle`` the caller gives it, attributed to the package and stating it. The graph is named
by its ``urn:uuid:`` identifier and files by their location, with the hash of their bytes as
``dcterms:identifier``.

The package, named by its purl, is the agent, though PROV-DM 2.2.1.1 defines a software agent as
running software.
"""

from collections.abc import Callable
from datetime import UTC, datetime
from importlib import resources
from pathlib import Path
from uuid import uuid4

from rdf_utils import DatasetFormat
from rdf_utils.models.prov import (
    add_directory_location,
    add_entity,
    add_file_entity,
    add_file_hash,
    get_git_info,
    get_pkg_info,
    load_pkg_prov,
    load_transformation_prov,
)
from rdf_utils.models.time import add_wall_instant
from rdflib import Dataset, Graph, Literal, URIRef
from rdflib.namespace import PROV, SDO
from textx.scoping import get_included_models

from scene_dsl.langs import package_import

DOCUMENT_SUFFIX = f".prov.{DatasetFormat.JSON_LD.extension}"
KDL_TEMPLATE = "templates/kdl.hpp.jinja2"


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
        for path in (Path(item._tx_filename).resolve() for item in models)
    }


def _add_transformation(
    bundle: Graph,
    activity: URIRef,
    sources: list[URIRef],
    targets: list[URIRef],
    started_at: datetime,
    ended_at: datetime,
) -> None:
    """Add a transformation by the package to a bundle, with its wall-clock instants."""
    agent = bundle.value(bundle.identifier, PROV.wasAttributedTo)
    assert isinstance(agent, URIRef)
    beginning, end = URIRef(uuid4().urn), URIRef(uuid4().urn)
    add_wall_instant(bundle, beginning, started_at)
    add_wall_instant(bundle, end, ended_at)
    load_transformation_prov(bundle, activity, sources, targets, agent, beginning, end)


def build(
    model,
    bundle: Graph,
    create_graph: Callable[..., Graph],
    graph_file: Path | str | None = None,
    dataset_format: DatasetFormat = DatasetFormat.JSON_LD,
    auto_compact: bool = True,
) -> Graph:
    """Build the graph of a model and add how it was made to a bundle.

    Parameters:
        model: the textX model, loaded from a file; that file and its imports are the sources
        bundle: the `prov:Bundle` graph to add the transformation to, attributed to the package
        create_graph: the language's graph builder, called with `model=model`
        graph_file: the file to save the graph to; the graph is not saved when None
        dataset_format: the format to save it in
        auto_compact: compact JSON-LD with the graph's namespaces

    Returns:
        The graph
    """
    started_at = datetime.now(UTC)
    scene_graph = create_graph(model=model)
    if graph_file is not None:
        graph_file = Path(graph_file).resolve()
        dataset = Dataset()
        dataset.add_graph(scene_graph)
        graph_file.write_text(dataset.serialize(format=dataset_format, auto_compact=auto_compact))
    ended_at = datetime.now(UTC)

    model_dir = Path(model._tx_filename).resolve().parent
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
    assert isinstance(scene_graph.identifier, URIRef)
    targets = [scene_graph.identifier]
    if graph_file is not None:
        file_id = add_file_entity(
            bundle,
            add_directory_location(bundle, graph_file.parent),
            graph_file.name,
            generated_by=activity,
        )
        add_file_hash(bundle, file_id)
        bundle.add((file_id, PROV.wasDerivedFrom, scene_graph.identifier))
        targets.append(file_id)
    _add_transformation(bundle, activity, sources, targets, started_at, ended_at)
    return scene_graph


def render(
    scene_graph: Graph,
    bundle: Graph,
    create_header: Callable[[Graph], str],
    output_file: Path | str,
) -> Path:
    """Render a scene graph to a KDL header and add how it was made to a bundle.

    Parameters:
        scene_graph: the graph `build` returned
        bundle: the `prov:Bundle` graph to add the transformation to, attributed to the package,
            with `load_pkg_prov` stating it so that the template can be found
        create_header: renders the template for a graph
        output_file: the file to write the header to

    Returns:
        The written file

    Raises:
        FileNotFoundError: when the template is missing; nothing is written
    """
    started_at = datetime.now(UTC)
    output_file = Path(output_file).resolve()
    agent = bundle.value(bundle.identifier, PROV.wasAttributedTo)
    assert isinstance(agent, URIRef)
    template_id = add_file_entity(bundle, agent, KDL_TEMPLATE)
    # Only the template: create_header also reads model files, whose absence is a different error.
    try:
        add_file_hash(bundle, template_id)
    except FileNotFoundError as error:
        raise FileNotFoundError(
            f"KDL template not found -- scene_dsl install is incomplete: {error}"
        ) from error
    output_file.write_text(create_header(scene_graph))
    ended_at = datetime.now(UTC)

    assert isinstance(scene_graph.identifier, URIRef)
    add_entity(bundle, scene_graph.identifier)
    activity = URIRef(uuid4().urn)
    output_id = add_file_entity(
        bundle,
        add_directory_location(bundle, output_file.parent),
        output_file.name,
        generated_by=activity,
    )
    add_file_hash(bundle, output_id)
    _add_transformation(
        bundle, activity, [scene_graph.identifier, template_id], [output_id], started_at, ended_at
    )
    return output_file


def write_bundle(bundle: Graph, document: Path | str) -> None:
    """Write a bundle as its own JSON-LD document, replacing any earlier one."""
    dataset = Dataset()
    dataset.add_graph(bundle)
    Path(document).write_text(dataset.serialize(format=DatasetFormat.JSON_LD, indent=2))
