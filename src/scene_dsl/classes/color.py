# SPDX-License-Identifier: MPL-2.0
"""Colours as the color metamodel has them: one scheme each, and one `col:value` string."""

from __future__ import annotations

from decimal import Decimal
from enum import StrEnum
from math import isfinite

from scene_dsl.classes.common import IHasParent


class ColorScheme(StrEnum):
    """The metamodel class each colour is exported as."""

    RGB = "RGBScheme"
    RGBA = "RGBAScheme"
    HSV = "HSVScheme"


class Color(IHasParent):
    scheme: ColorScheme

    @property
    def literal(self) -> str:
        """The `col:value` literal, in a form the metamodel's pattern for the scheme takes."""
        raise NotImplementedError(f"'literal' not implemented for '{self.__class__.__name__}'")


class RGBColor(Color):
    scheme = ColorScheme.RGB

    def __init__(self, parent, red: int, green: int, blue: int) -> None:
        super().__init__(parent=parent)
        for name, channel in (("red", red), ("green", green), ("blue", blue)):
            if not 0 <= channel <= 255:
                raise ValueError(f"{name} is {channel}, expected an integer in [0, 255]")
        self.red, self.green, self.blue = red, green, blue

    @property
    def literal(self) -> str:
        # Hex: 0-255 integers carry over exactly, where floats in [0, 1] would round.
        return f"#{self.red:02X}{self.green:02X}{self.blue:02X}"


class RGBAColor(RGBColor):
    scheme = ColorScheme.RGBA

    def __init__(self, parent, red: int, green: int, blue: int, alpha: int) -> None:
        super().__init__(parent=parent, red=red, green=green, blue=blue)
        if not 0 <= alpha <= 255:
            raise ValueError(f"alpha is {alpha}, expected an integer in [0, 255]")
        self.alpha = alpha

    @property
    def literal(self) -> str:
        return super().literal + f"{self.alpha:02X}"


class HexColor(Color):
    def __init__(self, parent, code: str) -> None:
        super().__init__(parent=parent)
        self.code = code.upper()
        # The grammar's HexCode regex admits only '#RRGGBB' or '#RRGGBBAA' (7 or 9
        # characters), so the length picks the scheme.
        self.scheme = ColorScheme.RGBA if len(self.code) == 9 else ColorScheme.RGB

    @property
    def literal(self) -> str:
        return self.code


def _plain(number: float) -> str:
    """No exponent and no '-0', which the metamodel's patterns would refuse."""
    if isinstance(number, int):
        return str(number)
    return format(Decimal(repr(number + 0.0)), "f")


class HSVColor(Color):
    scheme = ColorScheme.HSV

    def __init__(self, parent, hue: float, saturation: float, value: float) -> None:
        super().__init__(parent=parent)
        if not isfinite(hue) or not 0 <= hue < 360:
            raise ValueError(f"hsv hue is {hue}, expected degrees in [0, 360)")
        for name, channel in (("saturation", saturation), ("value", value)):
            if not isfinite(channel) or not 0 <= channel <= 1:
                raise ValueError(f"hsv {name} is {channel}, expected a number in [0, 1]")
        self.hue, self.saturation, self.value = hue, saturation, value

    @property
    def literal(self) -> str:
        return " ".join(_plain(n) for n in (self.hue, self.saturation, self.value))
