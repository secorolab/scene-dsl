# SPDX-License-Identifier: MPL-2.0
"""A thing's colour, read as the color metamodel states it: its scheme and its `col:value`.

The reader keeps the value as written. It parses it once, in the colour's own scheme, to check
it; hex is only another way to write RGB(A) channels, so it is decoded. `rgb`, `rgba` and `hsv`
convert to the other schemes on first use. A consumer that takes RGBA, as MJCF, URDF and SDF
do, asks for `ColorModel.rgba`.
"""

import colorsys
import math
from enum import StrEnum

from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.common import ModelBase
from rdf_utils.models.vocab import URI_COLOR_PRED_HAS_COLOR, URI_COLOR_PRED_VALUE
from rdf_utils.namespace import NS_MM_COLOR
from rdflib import Graph, URIRef

from scene_dsl.rdf_parser.common import ensure_one_obj_literal


class ColorScheme(StrEnum):
    """The metamodel class each colour is exported as."""

    RGB = "RGBScheme"
    RGBA = "RGBAScheme"
    HSV = "HSVScheme"


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
        value: the `col:value` literal, as written
    """

    scheme: ColorScheme
    value: str
    # Filled at read time for the colour's own scheme; the others on first use.
    _rgb: tuple[float, float, float] | None
    _rgba: tuple[float, float, float, float] | None
    _hsv: tuple[float, float, float] | None

    def __init__(self, color_id: URIRef, graph: Graph) -> None:
        super().__init__(node_id=color_id, graph=graph)
        schemes = [SCHEME_TYPES[t] for t in self.types if t in SCHEME_TYPES]
        if len(schemes) != 1:
            raise ConstraintViolation(
                "color", f"color '{self.id}' needs one scheme of RGB, RGBA or HSV, found {schemes}"
            )
        [scheme] = schemes
        self.scheme = scheme

        literal = ensure_one_obj_literal(
            graph=graph, subject=self.id, predicate=URI_COLOR_PRED_VALUE
        )
        if literal is None:
            raise ConstraintViolation("color", f"color '{self.id}' states no value")
        self.value = value = str(literal)
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

        self._rgb = self._rgba = self._hsv = None
        if scheme is ColorScheme.HSV:
            hue, saturation, brightness = channels
            if not 0 <= hue < 360 or not (0 <= saturation <= 1 and 0 <= brightness <= 1):
                raise ConstraintViolation(
                    "color",
                    f"color '{self.id}' needs HSV hue in [0, 360), saturation and value in "
                    f"[0, 1]: '{value}'",
                )
            self._hsv = (hue, saturation, brightness)
            return
        if any(not 0 <= c <= 1 for c in channels):
            raise ConstraintViolation(
                "color", f"color '{self.id}' needs {scheme.name} channels in [0, 1]: '{value}'"
            )
        if scheme is ColorScheme.RGBA:
            red, green, blue, alpha = channels
            self._rgba = (red, green, blue, alpha)
        else:
            red, green, blue = channels
            self._rgb = (red, green, blue)

    @property
    def rgb(self) -> tuple[float, float, float]:
        """Red, green and blue in [0, 1]. HSV is converted, and RGBA's alpha dropped."""
        if self._rgb is None:
            if self.scheme is ColorScheme.HSV:
                hue, saturation, brightness = self.hsv
                self._rgb = colorsys.hsv_to_rgb(hue / 360, saturation, brightness)
            else:
                red, green, blue, _ = self.rgba
                self._rgb = (red, green, blue)
        return self._rgb

    @property
    def rgba(self) -> tuple[float, float, float, float]:
        """Red, green, blue and alpha in [0, 1]. RGB and HSV are opaque."""
        if self._rgba is None:
            self._rgba = (*self.rgb, 1.0)
        return self._rgba

    @property
    def hsv(self) -> tuple[float, float, float]:
        """Hue in degrees [0, 360), then saturation and value in [0, 1]. RGBA's alpha dropped."""
        if self._hsv is None:
            hue, saturation, brightness = colorsys.rgb_to_hsv(*self.rgb)
            self._hsv = (hue * 360, saturation, brightness)
        return self._hsv


def get_color(graph: Graph, owner: URIRef) -> ColorModel | None:
    """The colour OWNER is drawn in; None when the graph states none, leaving a default."""
    colors = list(graph.objects(owner, URI_COLOR_PRED_HAS_COLOR))
    if not colors:
        return None
    if len(colors) > 1:
        raise ConstraintViolation("color", f"'{owner}' has {len(colors)} colors, expected one")
    [color] = colors
    return ColorModel(color, graph)
