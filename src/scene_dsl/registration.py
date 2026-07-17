# SPDX-License-Identifier: MPL-2.0
from textx import GeneratorDesc, LanguageDesc

from scene_dsl.gens import (
    scenex_dot_gen,
    scenex_dot_gen_console,
    scenex_graph_gen,
    scenex_graph_gen_console,
    scene_graph_gen,
    scene_graph_gen_console,
)
from scene_dsl.langs import scene_metamodel, scenex_metamodel

scene_lang = LanguageDesc(
    "scene",
    pattern="*.scene",
    description="Language for describing abstract robotic scenes",
    metamodel=scene_metamodel,
)
scenex_lang = LanguageDesc(
    "scenex",
    pattern="*.scenex",
    description="Language for describing executable robotic scene instances",
    metamodel=scenex_metamodel,
)

scene_console_gen = GeneratorDesc(
    language="scene",
    target="console",
    description="Print scene model RDF to the console, default format is JSON-LD",
    generator=scene_graph_gen_console,
)
scene_graph_gen_desc = GeneratorDesc(
    language="scene",
    target="graph",
    description="Generate scene model RDF, default format is JSON-LD",
    generator=scene_graph_gen,
)
scenex_console_gen = GeneratorDesc(
    language="scenex",
    target="console",
    description="Print executable scene RDF to the console, default format is JSON-LD",
    generator=scenex_graph_gen_console,
)
scenex_graph_gen_desc = GeneratorDesc(
    language="scenex",
    target="graph",
    description="Generate executable scene RDF, default format is JSON-LD",
    generator=scenex_graph_gen,
)

scenex_dot_console_gen = GeneratorDesc(
    language="scenex",
    target="dot-console",
    description="Print the scene's kinematics as a graphviz graph",
    generator=scenex_dot_gen_console,
)
scenex_dot_gen_desc = GeneratorDesc(
    language="scenex",
    target="dot",
    description="Draw the scene's kinematics: bodies joined by joints, chains picked out."
    " Formats: dot (default), png, svg, pdf",
    generator=scenex_dot_gen,
)
