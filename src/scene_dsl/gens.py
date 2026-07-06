# SPDX-License-Identifier: MPL-2.0
from os.path import basename, exists, join, splitext
from typing import Any

from rdflib import Graph
from rdflib.plugin import PluginException

from scene_dsl.rdf.scene import create_scene_model_graph
from scene_dsl.rdf.scenex import create_scenex_model_graph

_GRAPH_FORMAT_EXT = {"json-ld": "json", "ttl": "ttl", "xml": "xml"}


def parse_rdflib_serial_args(**kwargs):
    g_format = kwargs.get("format", "json-ld")
    assert g_format in _GRAPH_FORMAT_EXT, (
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
