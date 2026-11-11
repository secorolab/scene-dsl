# SPDX-License-Identifier: MPL-2.0
"""A thing's colour, as the color metamodel states it, read as RGBA channels in [0, 1]."""

import colorsys
import math

from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import URI_COLOR_PRED_HAS_COLOR, URI_COLOR_PRED_VALUE
from rdf_utils.namespace import NS_MM_COLOR
from rdflib import RDF, Graph, URIRef

from scene_dsl.classes.color import ColorScheme
from scene_dsl.rdf_parser.common import ensure_one_obj_literal

CHANNEL_COUNTS = {ColorScheme.RGB: 3, ColorScheme.RGBA: 4, ColorScheme.HSV: 3}


def _channels(scheme: ColorScheme, value: str) -> list[float]:
    """The channels a `col:value` literal writes: hex digit pairs, or space-separated floats."""
    count = CHANNEL_COUNTS[scheme]
    if value.startswith("#") and scheme is not ColorScheme.HSV:
        if len(value) != 1 + 2 * count:
            raise ValueError(f"expected {2 * count} hex digits")
        return [int(value[i : i + 2], 16) / 255 for i in range(1, len(value), 2)]
    return [float(channel) for channel in value.split()]


def get_rgba(graph: Graph, owner: URIRef) -> tuple[float, float, float, float] | None:
    """The colour OWNER is drawn in, as red, green, blue and alpha in [0, 1].

    RGB is opaque and HSV is converted, which is how MJCF, URDF and SDF take a colour. None
    when the graph states no colour, which leaves the consumer's default.
    """
    colors = list(graph.objects(owner, URI_COLOR_PRED_HAS_COLOR))
    if not colors:
        return None
    if len(colors) > 1:
        raise ConstraintViolation("color", f"'{owner}' has {len(colors)} colors, expected one")
    [color] = colors

    schemes = [s for s in ColorScheme if (color, RDF.type, NS_MM_COLOR[s.value]) in graph]
    if len(schemes) != 1:
        raise ConstraintViolation(
            "color", f"color of '{owner}' needs one scheme of RGB, RGBA or HSV, found {schemes}"
        )
    [scheme] = schemes
    literal = ensure_one_obj_literal(graph=graph, subject=color, predicate=URI_COLOR_PRED_VALUE)
    if literal is None:
        raise ConstraintViolation("color", f"color of '{owner}' states no value")
    value = str(literal)
    try:
        channels = _channels(scheme, value)
    except ValueError as error:
        raise ConstraintViolation(
            "color", f"color of '{owner}' has an invalid {scheme.name} value '{value}': {error}"
        ) from error
    count = CHANNEL_COUNTS[scheme]
    if len(channels) != count or not all(math.isfinite(c) for c in channels):
        raise ConstraintViolation(
            "color", f"color of '{owner}' needs {count} finite {scheme.name} channels: '{value}'"
        )

    if scheme is ColorScheme.HSV:
        hue, saturation, brightness = channels
        if not 0 <= hue < 360 or not (0 <= saturation <= 1 and 0 <= brightness <= 1):
            raise ConstraintViolation(
                "color",
                f"color of '{owner}' needs HSV hue in [0, 360), saturation and value in [0, 1]: "
                f"'{value}'",
            )
        channels = list(colorsys.hsv_to_rgb(hue / 360, saturation, brightness))
    elif any(not 0 <= c <= 1 for c in channels):
        raise ConstraintViolation(
            "color", f"color of '{owner}' needs {scheme.name} channels in [0, 1]: '{value}'"
        )
    red, green, blue, *alpha = channels
    return red, green, blue, alpha[0] if alpha else 1.0
