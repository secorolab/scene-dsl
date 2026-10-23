# SPDX-License-Identifier: MPL-2.0
from datetime import UTC, datetime
from os.path import basename, dirname, exists, join, splitext
from pathlib import Path
from typing import Any

from jinja2 import Environment, FileSystemLoader
from rdf_utils import DatasetFormat
from rdf_utils.naming import get_valid_var_name
from rdflib import Dataset, Graph

from scene_dsl.dot import create_dot
from scene_dsl.kdl_tree import build_kdl_trees
from scene_dsl.rdf.provenance import record
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


def graph_gen(graph: Graph, model, output_path, overwrite, model_type: str, **kwargs):
    started = datetime.now(UTC)
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
    dataset.add_graph(graph)
    with open(full_output_path, "w") as outfile:
        outfile.write(
            dataset.serialize(format=dataset_format, auto_compact="nocompact" not in kwargs)
        )
    record(model, full_output_path, started, **kwargs)
    print(f"... wrote {full_output_path}")


def scene_graph_gen_console(metamodel, model, output_path, overwrite, debug, **kwargs):
    graph_gen_console(create_scene_model_graph(model=model), **kwargs)


def scene_graph_gen(metamodel, model, output_path, overwrite, debug, **kwargs):
    graph_gen(
        create_scene_model_graph(model=model),
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
        create_scenex_model_graph(model=model),
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
    started = datetime.now(UTC)
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
    record(model, full_output_path, started, **kwargs)
    print(f"... wrote {full_output_path}")


def kdl_header_names(scene_file: str | Path) -> tuple[str, str]:
    """The KDL header's file name and its C++ namespace, both named after SCENE_FILE."""
    stem = Path(scene_file).stem
    namespace = get_valid_var_name(stem)
    # A C++ identifier cannot start with a digit, as a scene file like `2f85.scenex` would.
    if namespace[0].isdigit():
        namespace = f"scene_{namespace}"
    return f"{stem}.kdl.hpp", namespace


def scenex_kdl_gen(metamodel, model, output_path, overwrite, debug, **kwargs):
    started = datetime.now(UTC)
    header, name = kdl_header_names(kwargs.get("filename") or model._tx_filename or "kinematics")
    full_output_path = join("" if output_path is None else output_path, header)
    if exists(full_output_path) and not overwrite:
        print(f"not overwriting existing file '{full_output_path}'")
        return

    # Model file paths are written relative to the model declaring them.
    base_dir = dirname(model._tx_filename)
    trees = build_kdl_trees(
        create_scenex_model_graph(model=model), Path(base_dir) if base_dir else None
    )

    env = Environment(
        loader=FileSystemLoader(Path(__file__).parent / "templates"), keep_trailing_newline=True
    )
    template = env.get_template("kdl.hpp.jinja2")
    source = basename(model._tx_filename) or "a scene model"
    with open(full_output_path, "w") as outfile:
        outfile.write(template.render({"data": {"name": name, "source": source, "trees": trees}}))
    record(model, full_output_path, started, **kwargs)
    print(f"... wrote {full_output_path}")
