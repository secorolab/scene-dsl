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

    def range_error(self) -> str | None:
        """What is out of range, if anything.

        Checked by a model processor rather than here: during construction the colour's
        parents are not yet linked up, so textX cannot place an error at its line.
        """
        return None

    @property
    def literal(self) -> str:
        """The `col:value` literal, in a form the metamodel's pattern for the scheme takes."""
        raise NotImplementedError(f"'literal' not implemented for '{self.__class__.__name__}'")


def _byte_range_error(form: str, channels: dict[str, int]) -> str | None:
    for name, channel in channels.items():
        if not 0 <= channel <= 255:
            return f"{form} {name} is {channel}, expected an integer in [0, 255]"
    return None


class RGBColor(Color):
    scheme = ColorScheme.RGB

    def __init__(self, parent, red: int, green: int, blue: int) -> None:
        super().__init__(parent=parent)
        self.channels = {"red": red, "green": green, "blue": blue}

    def range_error(self) -> str | None:
        return _byte_range_error("rgb", self.channels)

    @property
    def literal(self) -> str:
        # Hex: 0-255 integers carry over exactly, where floats in [0, 1] would round.
        return "#" + "".join(f"{channel:02X}" for channel in self.channels.values())


class RGBAColor(RGBColor):
    scheme = ColorScheme.RGBA

    def __init__(self, parent, red: int, green: int, blue: int, alpha: int) -> None:
        super().__init__(parent, red, green, blue)
        self.channels["alpha"] = alpha

    def range_error(self) -> str | None:
        return _byte_range_error("rgba", self.channels)


class HexColor(Color):
    """'#RRGGBB' or '#RRGGBBAA': the grammar admits only these, so no range to check."""

    def __init__(self, parent, code: str) -> None:
        super().__init__(parent=parent)
        self.code = code.upper()
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
        self.hue, self.saturation, self.value = hue, saturation, value

    def range_error(self) -> str | None:
        if not isfinite(self.hue) or not 0 <= self.hue < 360:
            return f"hsv hue is {self.hue}, expected degrees in [0, 360)"
        for name, channel in (("saturation", self.saturation), ("value", self.value)):
            if not isfinite(channel) or not 0 <= channel <= 1:
                return f"hsv {name} is {channel}, expected a number in [0, 1]"
        return None

    @property
    def literal(self) -> str:
        return " ".join(_plain(n) for n in (self.hue, self.saturation, self.value))
