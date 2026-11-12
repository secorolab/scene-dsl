import colorsys
import re

import pytest
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_COLOR_PRED_HAS_COLOR,
    URI_COLOR_PRED_VALUE,
    URI_COLOR_TYPE_COLOR,
)
from rdf_utils.namespace import NS_MM_COLOR
from rdflib import RDF, Graph, Literal, URIRef
from textx.exceptions import TextXSyntaxError

from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.shapes import add_shape
from scene_dsl.rdf_parser.color import ColorScheme, get_color


def _shape_model(color: str):
    return scenex_metamodel().model_from_str(
        f"""ns n = "https://example.test/"
ktree (ns=n) arm {{ root: <base.f>
    body base {{
        frame f {{ }}
        shape cube {{ frame: <f> unit: m color: {color} }}
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
        ("hsv(0.0, -0.0, 1e-5)", ColorScheme.HSV, "0.0 0.0 0.00001"),
    ],
)
def test_color_exports_its_scheme_and_value(authored, scheme, literal):
    shape = _shape(authored)
    graph = Graph()
    add_shape(graph, shape)

    assert shape.color.scheme == scheme
    assert (shape.uri, URI_COLOR_PRED_HAS_COLOR, shape.color_uri) in graph
    assert (shape.color_uri, RDF.type, URI_COLOR_TYPE_COLOR) in graph
    assert (shape.color_uri, RDF.type, NS_MM_COLOR[scheme.value]) in graph
    assert list(graph.objects(shape.color_uri, URI_COLOR_PRED_VALUE)) == [Literal(literal)]


def test_shape_without_color_exports_none():
    model = scenex_metamodel().model_from_str(
        """ns n = "https://example.test/"
ktree (ns=n) arm { root: <base.f>
    body base {
        frame f { }
        shape cube { frame: <f> unit: m }
    }
    joints { }
}
"""
    )
    [shape] = model.ktrees[0].bodies[0].shapes
    graph = Graph()
    add_shape(graph, shape)

    assert shape.color is None
    assert not list(graph.objects(shape.uri, URI_COLOR_PRED_HAS_COLOR))
    assert get_color(graph, shape.uri) is None


@pytest.mark.parametrize(
    ("authored", "scheme", "value", "rgba"),
    [
        ("rgb(230, 51, 0)", ColorScheme.RGB, "#E63300", (230 / 255, 51 / 255, 0.0, 1.0)),
        (
            "#33B34D80",
            ColorScheme.RGBA,
            "#33B34D80",
            (0x33 / 255, 0xB3 / 255, 0x4D / 255, 0x80 / 255),
        ),
        # HSV reads back as HSV; only the properties convert it.
        (
            "hsv(210, 0.6, 0.9)",
            ColorScheme.HSV,
            "210 0.6 0.9",
            (*colorsys.hsv_to_rgb(210 / 360, 0.6, 0.9), 1.0),
        ),
    ],
)
def test_color_reads_back_as_written(authored, scheme, value, rgba):
    shape = _shape(authored)
    graph = Graph()
    add_shape(graph, shape)

    color = get_color(graph, shape.uri)
    assert color.scheme is scheme
    assert color.value == value
    assert color.rgba == pytest.approx(rgba)


OWNER = URIRef("https://example.test/owner")


def _color_graph(scheme: ColorScheme, value: str, colors: int = 1) -> Graph:
    """OWNER drawn in COLORS colours, each in SCHEME with VALUE, as the metamodel states them."""
    graph = Graph()
    for index in range(colors):
        color = URIRef(f"https://example.test/color-{index}")
        graph.add((OWNER, URI_COLOR_PRED_HAS_COLOR, color))
        graph.add((color, RDF.type, URI_COLOR_TYPE_COLOR))
        graph.add((color, RDF.type, NS_MM_COLOR[scheme.value]))
        graph.add((color, URI_COLOR_PRED_VALUE, Literal(value)))
    return graph


def test_float_channels_read_as_written():
    # The metamodel takes floats as well as hex; scene-dsl writes hex, others need not.
    color = get_color(_color_graph(ColorScheme.RGBA, "0.9 0.2 0.2 0.5"), OWNER)
    assert color.value == "0.9 0.2 0.2 0.5"
    assert color.rgba == pytest.approx((0.9, 0.2, 0.2, 0.5))


@pytest.mark.parametrize(
    ("scheme", "value", "rgb", "alpha", "hsv"),
    [
        (ColorScheme.RGB, "#FF0000", (1.0, 0.0, 0.0), 1.0, (0.0, 1.0, 1.0)),
        (ColorScheme.RGBA, "#0000FF80", (0.0, 0.0, 1.0), 0x80 / 255, (240.0, 1.0, 1.0)),
        (ColorScheme.HSV, "120 1 0.5", (0.0, 0.5, 0.0), 1.0, (120.0, 1.0, 0.5)),
    ],
)
def test_color_converts_between_schemes(scheme, value, rgb, alpha, hsv):
    color = get_color(_color_graph(scheme, value), OWNER)
    assert color.value == value
    assert color.rgb == pytest.approx(rgb)
    assert color.rgba == pytest.approx((*rgb, alpha))
    assert color.hsv == pytest.approx(hsv)


@pytest.mark.parametrize(
    ("scheme", "value", "colors", "message"),
    [
        (ColorScheme.RGB, "#E63300", 2, "has 2 colors, expected one"),
        (ColorScheme.RGB, "#12345", 1, "invalid RGB value '#12345': expected 6 hex digits"),
        (ColorScheme.RGBA, "0.9 0.2 0.2", 1, "needs 4 finite RGBA channels"),
        (ColorScheme.RGB, "1.5 0 0", 1, "needs RGB channels in [0, 1]"),
        (ColorScheme.HSV, "360 0.5 0.5", 1, "needs HSV hue in [0, 360)"),
        (ColorScheme.HSV, "10 1.5 0.5", 1, "saturation and value in [0, 1]"),
    ],
)
def test_malformed_color_in_graph_is_reported(scheme, value, colors, message):
    graph = _color_graph(scheme, value, colors)
    with pytest.raises(ConstraintViolation, match=re.escape(message)):
        get_color(graph, OWNER)


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
