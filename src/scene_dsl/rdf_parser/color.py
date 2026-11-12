# SPDX-License-Identifier: MPL-2.0
"""A thing's colour, read as the color metamodel states it: its scheme and its channels.

The reader converts nothing. Hex is only another way to write RGB(A) channels, so it is
decoded, but HSV stays HSV. A consumer that takes RGBA, as MJCF, URDF and SDF do, asks for
`ColorModel.rgba`.
"""

import colorsys
import math

from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import URI_COLOR_PRED_HAS_COLOR, URI_COLOR_PRED_VALUE
from rdf_utils.namespace import NS_MM_COLOR
from rdflib import Graph, URIRef

from scene_dsl.classes.color import ColorScheme
from scene_dsl.rdf_parser.common import ensure_one_obj_literal

CHANNEL_COUNTS = {ColorScheme.RGB: 3, ColorScheme.RGBA: 4, ColorScheme.HSV: 3}
SCHEME_TYPES = {NS_MM_COLOR[scheme.value]: scheme for scheme in ColorScheme}


def _channels(scheme: ColorScheme, value: str) -> list[float]:
    """The channels a `col:value` literal writes: hex digit pairs, or space-separated floats."""
    count = CHANNEL_COUNTS[scheme]
    if value.startswith("#") and scheme is not ColorScheme.HSV:
        if len(value) != 1 + 2 * count:
            raise ValueError(f"expected {2 * count} hex digits")
        return [int(value[i : i + 2], 16) / 255 for i in range(1, len(value), 2)]
    return [float(channel) for channel in value.split()]


class ColorModel(ModelBase):
    """One colour, as the graph states it.

    Attributes:
        scheme: the scheme the colour is written in
        channels: as written -- red, green, blue (and alpha) in [0, 1], hex decoded; or hue in
            degrees [0, 360), then saturation and value in [0, 1]
    """

    scheme: ColorScheme
    channels: tuple[float, ...]

    def __init__(self, color_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=color_id, graph=graph)
        schemes = [SCHEME_TYPES[t] for t in self.types if t in SCHEME_TYPES]
        if len(schemes) != 1:
            raise ConstraintViolation(
                "color", f"color '{self.id}' needs one scheme of RGB, RGBA or HSV, found {schemes}"
            )
        [scheme] = schemes
        self.scheme = scheme

        literal = ensure_one_obj_literal(graph=graph, subject=self.id, predicate=URI_COLOR_PRED_VALUE)
        if literal is None:
            raise ConstraintViolation("color", f"color '{self.id}' states no value")
        value = str(literal)
        try:
            channels = _channels(scheme, value)
        except ValueError as error:
            raise ConstraintViolation(
                "color", f"color '{self.id}' has an invalid {scheme.name} value '{value}': {error}"
            ) from error
        count = CHANNEL_COUNTS[scheme]
        if len(channels) != count or not all(math.isfinite(c) for c in channels):
            raise ConstraintViolation(
                "color", f"color '{self.id}' needs {count} finite {scheme.name} channels: '{value}'"
            )

        if scheme is ColorScheme.HSV:
            hue, saturation, brightness = channels
            if not 0 <= hue < 360 or not (0 <= saturation <= 1 and 0 <= brightness <= 1):
                raise ConstraintViolation(
                    "color",
                    f"color '{self.id}' needs HSV hue in [0, 360), saturation and value in "
                    f"[0, 1]: '{value}'",
                )
        elif any(not 0 <= c <= 1 for c in channels):
            raise ConstraintViolation(
                "color", f"color '{self.id}' needs {scheme.name} channels in [0, 1]: '{value}'"
            )
        self.channels = tuple(channels)

    @property
    def rgba(self) -> tuple[float, float, float, float]:
        """Red, green, blue and alpha in [0, 1], for a consumer that takes RGBA.

        HSV is converted, and RGB is opaque.
        """
        channels = self.channels
        if self.scheme is ColorScheme.HSV:
            hue, saturation, brightness = channels
            channels = colorsys.hsv_to_rgb(hue / 360, saturation, brightness)
        red, green, blue, *alpha = channels
        return red, green, blue, alpha[0] if alpha else 1.0


def get_color(graph: Graph, owner: URIRef) -> ColorModel | None:
    """The colour OWNER is drawn in; None when the graph states none, leaving a default."""
    colors = list(graph.objects(owner, URI_COLOR_PRED_HAS_COLOR))
    if not colors:
        return None
    if len(colors) > 1:
        raise ConstraintViolation("color", f"'{owner}' has {len(colors)} colors, expected one")
    [color] = colors
    return ColorModel(color, graph)
