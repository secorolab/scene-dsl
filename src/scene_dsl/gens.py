# SPDX-License-Identifier: MPL-2.0
from collections.abc import Callable
from os.path import basename, exists, join, splitext
from pathlib import Path
from typing import Any
from uuid import uuid4

from jinja2 import Environment, FileSystemLoader
from rdf_utils import DatasetFormat
from rdf_utils.naming import get_valid_var_name
from rdflib import Dataset, Graph, URIRef
from textx import get_location

from scene_dsl.dot import create_dot
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.rdf.provenance import build, create_scene_dsl_bundle, render
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import create_scenex_model_graph


def rdf_full_output_path(
    model, output_path: str, dataset_format: DatasetFormat, model_type: str, **kwargs: Any
):
    filename = kwargs.get("filename", basename(get_location(model)["filename"])) or "graph"
    full_filename = f"{splitext(filename)[0]}.{model_type}.{dataset_format.extension}"
    return join("" if output_path is None else output_path, full_filename)


def graph_gen_console(create_graph: Callable[[Any, Graph], Graph], model, **kwargs):
    dataset = Dataset()
    create_graph(model, dataset.graph(URIRef(uuid4().urn)))
    print(
        dataset.serialize(
            format=DatasetFormat(kwargs.get("format", DatasetFormat.JSON_LD)),
            indent=2,
            auto_compact="nocompact" not in kwargs,
        )
    )


def graph_gen(
    create_graph: Callable[[Any, Graph], Graph],
    model,
    output_path,
    overwrite,
    model_type: str,
    **kwargs,
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

    dataset = Dataset()
    if "provenance" in kwargs:
        build(model, dataset, *create_scene_dsl_bundle(dataset), create_graph)
    else:
        create_graph(model, dataset.graph(URIRef(uuid4().urn)))
    with open(full_output_path, "w") as outfile:
        outfile.write(
            dataset.serialize(
                format=dataset_format, indent=2, auto_compact="nocompact" not in kwargs
            )
        )
    print(f"... wrote {full_output_path}")


def scene_graph_gen_console(metamodel, model, output_path, overwrite, debug, **kwargs):
    graph_gen_console(create_scene_model_graph, model, **kwargs)


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
    graph_gen_console(create_scenex_model_graph, model, **kwargs)


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
    filename = kwargs.get("filename", basename(get_location(model)["filename"])) or "kinematics"
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


def kdl_header(graph: Graph, name: str, source: str) -> str:
    """The C++ header building the graph's KDL trees, in namespace NAME, naming SOURCE."""
    trees = build_kdl_trees(graph)

    env = Environment(
        loader=FileSystemLoader(Path(__file__).parent / "templates"), keep_trailing_newline=True
    )
    template = env.get_template("kdl.hpp.jinja2")
    return template.render({"data": {"name": name, "source": source, "trees": trees}})


def scenex_kdl_gen(metamodel, model, output_path, overwrite, debug, **kwargs):
    filename = kwargs.get("filename", basename(get_location(model)["filename"])) or "kinematics"
    full_output_path = join(
        "" if output_path is None else output_path, f"{splitext(filename)[0]}.kdl.hpp"
    )
    if exists(full_output_path) and not overwrite:
        print(f"not overwriting existing file '{full_output_path}'")
        return

    source = basename(get_location(model)["filename"]) or "a scene model"
    # The header holds one scene's kinematics, so that scene names its namespace -- as a
    # C++ identifier, a file called `my.lab-v2.scenex` or `2f85.scenex` naming none.
    name = get_valid_var_name(splitext(source)[0] or "scene")
    name = f"scene_{name}" if name[0].isdigit() else name
    if "provenance" in kwargs:
        dataset_format = DatasetFormat(kwargs.get("format", DatasetFormat.JSON_LD))
        dataset = Dataset()
        bundle_id, agent_id = create_scene_dsl_bundle(dataset)
        graph_id = build(model, dataset, bundle_id, agent_id, create_scenex_model_graph)
        render(
            dataset,
            graph_id,
            bundle_id,
            agent_id,
            lambda graph: kdl_header(graph, name, source),
            full_output_path,
        )
        Path(f"{full_output_path}.{dataset_format.extension}").write_text(
            dataset.serialize(
                format=dataset_format, indent=2, auto_compact="nocompact" not in kwargs
            )
        )
    else:
        with open(full_output_path, "w") as outfile:
            outfile.write(kdl_header(create_scenex_model_graph(model=model), name, source))
    print(f"... wrote {full_output_path}")
