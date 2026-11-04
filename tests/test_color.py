import re

import pytest
from rdflib import RDF, Graph, Literal
from textx.exceptions import TextXSyntaxError

from scene_dsl.classes.color import ColorScheme
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.color import COL
from scene_dsl.rdf.ktree import add_shape


def _shape_model(color: str):
    return scenex_metamodel().model_from_str(
        f"""ns n = "https://example.test/"
ktree (ns=n) arm {{ root: <base.f>
    body base {{
        frame f {{ }}
        shape UnitCube box {{ frame: <f> unit: m color: {color} }}
    }}
    joints {{ }}
}}
"""
    )


def _shape(color: str):
    [shape] = _shape_model(color).ktrees[0].bodies[0].shapes
    return shape


@pytest.mark.parametrize(
    ("authored", "scheme", "literal"),
    [
        # 0-255 channels go out as hex: exact, where floats in [0, 1] would round.
        ("rgb(230, 51, 0)", ColorScheme.RGB, "#E63300"),
        ("rgba(230, 51, 51, 255)", ColorScheme.RGBA, "#E63333FF"),
        ("#33b34d", ColorScheme.RGB, "#33B34D"),
        ("#33B34D80", ColorScheme.RGBA, "#33B34D80"),
        ("hsv(210, 0.6, 0.9)", ColorScheme.HSV, "210 0.6 0.9"),
        ("hsv(359.5, 1, 0)", ColorScheme.HSV, "359.5 1 0"),
        # No '-0' or exponent, which the metamodel's pattern refuses.
        ("hsv(0.0, -0.0, 1e-1)", ColorScheme.HSV, "0.0 0.0 0.1"),
    ],
)
def test_color_exports_its_scheme_and_value(authored, scheme, literal):
    shape = _shape(authored)
    graph = Graph()
    add_shape(graph, shape)

    assert shape.color.scheme == scheme
    assert (shape.uri, COL["has-color"], shape.color_uri) in graph
    assert (shape.color_uri, RDF.type, COL.Color) in graph
    assert (shape.color_uri, RDF.type, COL[scheme.value]) in graph
    assert list(graph.objects(shape.color_uri, COL.value)) == [Literal(literal)]


def test_shape_without_color_exports_none():
    model = scenex_metamodel().model_from_str(
        """ns n = "https://example.test/"
ktree (ns=n) arm { root: <base.f>
    body base {
        frame f { }
        shape UnitCube box { frame: <f> unit: m }
    }
    joints { }
}
"""
    )
    [shape] = model.ktrees[0].bodies[0].shapes
    graph = Graph()
    add_shape(graph, shape)

    assert shape.color is None
    assert not list(graph.objects(shape.uri, COL["has-color"]))


@pytest.mark.parametrize(
    ("authored", "message"),
    [
        ("rgb(256, 0, 0)", "red is 256, expected an integer in [0, 255]"),
        ("rgba(0, 0, 0, -1)", "alpha is -1, expected an integer in [0, 255]"),
        ("hsv(360, 0.5, 0.5)", "hsv hue is 360, expected degrees in [0, 360)"),
        ("hsv(10, 1.5, 0.5)", "hsv saturation is 1.5, expected a number in [0, 1]"),
        ("hsv(10, 0.5, -0.1)", "hsv value is -0.1, expected a number in [0, 1]"),
    ],
)
def test_out_of_range_color_is_rejected(authored, message):
    with pytest.raises(ValueError, match=re.escape(message)):
        _shape(authored)


@pytest.mark.parametrize(
    "authored",
    [
        "#12345",  # neither six nor eight digits
        "rgb(1.5, 0, 0)",  # RGB channels are integers
        "(0.9, 0.2, 0.2, 1.0)",  # a bare vector names no scheme
    ],
)
def test_malformed_color_is_a_syntax_error(authored):
    with pytest.raises(TextXSyntaxError):
        _shape(authored)
