# SPDX-License-Identifier: MPL-2.0
from collections.abc import Callable
from os.path import basename, exists, join, splitext
from pathlib import Path
from typing import Any

from jinja2 import Environment, FileSystemLoader
from rdf_utils import DatasetFormat
from rdf_utils.models.prov import create_bundle, get_pkg_info, load_pkg_prov
from rdf_utils.naming import get_valid_var_name
from rdflib import Dataset, Graph

from scene_dsl.dot import create_dot
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.rdf.provenance import DOCUMENT_SUFFIX, build, render, write_bundle
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import create_scenex_model_graph


def rdf_full_output_path(
    model, output_path: str, dataset_format: DatasetFormat, model_type: str, **kwargs: Any
):
    filename = kwargs.get("filename", basename(model._tx_filename)) or "graph"
    full_filename = f"{splitext(filename)[0]}.{model_type}.{dataset_format.extension}"
    return join("" if output_path is None else output_path, full_filename)


def graph_gen_console(graph: Graph, **kwargs):
    dataset = Dataset()
    dataset.add_graph(graph)
    print(
        dataset.serialize(
            format=DatasetFormat(kwargs.get("format", DatasetFormat.JSON_LD)),
            auto_compact="nocompact" not in kwargs,
        )
    )


def graph_gen(
    create_graph: Callable[..., Graph], model, output_path, overwrite, model_type: str, **kwargs
):
    dataset_format = DatasetFormat(kwargs.get("format", DatasetFormat.JSON_LD))
    full_output_path = rdf_full_output_path(
        model=model,
        output_path=output_path,
        dataset_format=dataset_format,
        model_type=model_type,
        **kwargs,
    )
    if exists(full_output_path) and not overwrite:
        print(f"not overwriting existing file '{full_output_path}'")
        return

    auto_compact = "nocompact" not in kwargs
    if "provenance" in kwargs:
        agent, version, revision, repository, dirty = get_pkg_info("scene_dsl")
        bundle = create_bundle(agent)
        load_pkg_prov(bundle, agent, "scene_dsl", version, revision, repository, dirty=dirty)
        build(model, bundle, create_graph, full_output_path, dataset_format, auto_compact)
        stem = splitext(kwargs.get("filename", basename(model._tx_filename)) or "graph")[0]
        write_bundle(
            bundle, Path(full_output_path).with_name(f"{stem}.{model_type}{DOCUMENT_SUFFIX}")
        )
    else:
        dataset = Dataset()
        dataset.add_graph(create_graph(model=model))
        with open(full_output_path, "w") as outfile:
            outfile.write(dataset.serialize(format=dataset_format, auto_compact=auto_compact))
    print(f"... wrote {full_output_path}")


def scene_graph_gen_console(metamodel, model, output_path, overwrite, debug, **kwargs):
    graph_gen_console(create_scene_model_graph(model=model), **kwargs)


def scene_graph_gen(metamodel, model, output_path, overwrite, debug, **kwargs):
    graph_gen(
        create_scene_model_graph,
        model,
        output_path,
        overwrite,
        "scene",
        **kwargs,
    )


def scenex_graph_gen_console(metamodel, model, output_path, overwrite, debug, **kwargs):
    graph_gen_console(create_scenex_model_graph(model=model), **kwargs)


def scenex_graph_gen(metamodel, model, output_path, overwrite, debug, **kwargs):
    graph_gen(
        create_scenex_model_graph,
        model,
        output_path,
        overwrite,
        "scenex",
        **kwargs,
    )


_DOT_RENDER_FORMATS = ("png", "svg", "pdf")


def _render(dot_source: str, path: str, img_format: str) -> None:
    """Hand the graph to graphviz, which is what turns it into a picture."""
    from shutil import which
    from subprocess import run

    if which("dot") is None:
        raise ValueError(f"graphviz is needed to write '{img_format}': no 'dot' on PATH")
    run(["dot", f"-T{img_format}", "-o", path], input=dot_source, text=True, check=True)


def scenex_dot_gen_console(metamodel, model, output_path, overwrite, debug, **kwargs):
    print(create_dot(create_scenex_model_graph(model=model)), end="")


def scenex_dot_gen(metamodel, model, output_path, overwrite, debug, **kwargs):
    img_format = kwargs.get("format", "dot")
    if img_format not in ("dot",) + _DOT_RENDER_FORMATS:
        raise ValueError(
            f"unhandled format '{img_format}' for kinematics graph, "
            f"try {['dot', *_DOT_RENDER_FORMATS]}"
        )
    filename = kwargs.get("filename", basename(model._tx_filename)) or "kinematics"
    full_output_path = join(
        "" if output_path is None else output_path, f"{splitext(filename)[0]}.{img_format}"
    )
    if exists(full_output_path) and not overwrite:
        print(f"not overwriting existing file '{full_output_path}'")
        return

    dot_source = create_dot(create_scenex_model_graph(model=model))
    if img_format == "dot":
        with open(full_output_path, "w") as outfile:
            outfile.write(dot_source)
    else:
        _render(dot_source, full_output_path, img_format)
    print(f"... wrote {full_output_path}")


def kdl_header_names(scene_file: str | Path) -> tuple[str, str]:
    """The KDL header's file name and its C++ namespace, both named after SCENE_FILE."""
    stem = Path(scene_file).stem
    namespace = get_valid_var_name(stem)
    # A C++ identifier cannot start with a digit, as a scene file like `2f85.scenex` would.
    if namespace[0].isdigit():
        namespace = f"scene_{namespace}"
    return f"{stem}.kdl.hpp", namespace


def kdl_header(graph: Graph, name: str, source: str) -> str:
    """The C++ header building the graph's KDL trees, in namespace NAME, naming SOURCE."""
    trees = build_kdl_trees(graph)

    env = Environment(
        loader=FileSystemLoader(Path(__file__).parent / "templates"), keep_trailing_newline=True
    )
    template = env.get_template("kdl.hpp.jinja2")
    return template.render({"data": {"name": name, "source": source, "trees": trees}})


def scenex_kdl_gen(metamodel, model, output_path, overwrite, debug, **kwargs):
    header, name = kdl_header_names(kwargs.get("filename") or model._tx_filename or "kinematics")
    full_output_path = join("" if output_path is None else output_path, header)
    if exists(full_output_path) and not overwrite:
        print(f"not overwriting existing file '{full_output_path}'")
        return

    source = basename(model._tx_filename) or "a scene model"
    if "provenance" in kwargs:
        agent, version, revision, repository, dirty = get_pkg_info("scene_dsl")
        bundle = create_bundle(agent)
        load_pkg_prov(bundle, agent, "scene_dsl", version, revision, repository, dirty=dirty)
        output_file = render(
            build(model, bundle, create_scenex_model_graph),
            bundle,
            lambda g: kdl_header(g, name, source),
            full_output_path,
        )
        write_bundle(bundle, f"{output_file}{DOCUMENT_SUFFIX}")
    else:
        with open(full_output_path, "w") as outfile:
            outfile.write(kdl_header(create_scenex_model_graph(model=model), name, source))
    print(f"... wrote {full_output_path}")
