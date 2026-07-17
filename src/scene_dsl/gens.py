# SPDX-License-Identifier: MPL-2.0
from os.path import basename, exists, join, splitext
from typing import Any

from rdflib import Graph
from rdflib.plugin import PluginException

from scene_dsl.dot import create_dot
from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import create_scenex_model_graph

_GRAPH_FORMAT_EXT = {"json-ld": "json", "ttl": "ttl", "xml": "xml"}


def parse_rdflib_serial_args(**kwargs):
    g_format = kwargs.get("format", "json-ld")
    if g_format not in _GRAPH_FORMAT_EXT:
        raise ValueError(
            f"file extension not handled for graph format '{g_format}', "
            f"try {list(_GRAPH_FORMAT_EXT.keys())}"
        )
    ser_kwargs = {"format": g_format}
    if g_format == "json-ld":
        ser_kwargs["auto_compact"] = "nocompact" not in kwargs
    return ser_kwargs


def rdf_full_output_path(model, output_path: str, g_format: str, model_type: str, **kwargs: Any):
    filename = kwargs.get("filename", basename(model._tx_filename)) or "graph"
    full_filename = f"{splitext(filename)[0]}.{model_type}.{_GRAPH_FORMAT_EXT[g_format]}"
    return join("" if output_path is None else output_path, full_filename)


def graph_gen_console(graph: Graph, **kwargs):
    ser_kwargs = parse_rdflib_serial_args(**kwargs)
    try:
        print(graph.serialize(**ser_kwargs))
    except PluginException as e:
        raise ValueError(
            f"serialization format '{ser_kwargs['format']}' not supported by rdflib, "
            f"try {list(_GRAPH_FORMAT_EXT.keys())}: {e.msg}"
        )


def graph_gen(graph: Graph, model, output_path, overwrite, model_type: str, **kwargs):
    ser_kwargs = parse_rdflib_serial_args(**kwargs)
    full_output_path = rdf_full_output_path(
        model=model,
        output_path=output_path,
        g_format=ser_kwargs["format"],
        model_type=model_type,
        **kwargs,
    )
    if exists(full_output_path) and not overwrite:
        print(f"not overwriting existing file '{full_output_path}'")
        return

    with open(full_output_path, "w") as outfile:
        outfile.write(graph.serialize(**ser_kwargs))
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
    print(create_dot(model=model), end="")


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

    dot_source = create_dot(model=model)
    if img_format == "dot":
        with open(full_output_path, "w") as outfile:
            outfile.write(dot_source)
    else:
        _render(dot_source, full_output_path, img_format)
    print(f"... wrote {full_output_path}")
