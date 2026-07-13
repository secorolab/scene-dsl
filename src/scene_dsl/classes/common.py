# SPDX-License-Identifier: MPL-2.0
from typing import Any
from uuid import UUID, uuid4

from rdflib import Namespace, URIRef


class IHasParent:
    def __init__(self, **kwargs) -> None:
        self.parent = kwargs.get("parent", None)
        if self.parent is None:
            raise ValueError(f"'parent' not handled for type '{self.__class__.__name__}'")


class IHasNamespace(IHasParent):
    @property
    def namespace(self) -> Namespace:
        raise NotImplementedError(
            f"'namespace' property not implemented for '{self.__class__.__name__}'"
        )


class IHasNamespaceDeclare(IHasNamespace):
    uri: URIRef
    ns_prefix: str
    _ns_obj: Namespace

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.ns = kwargs.get("ns", None)
        if self.ns is None:
            raise ValueError("Namespace declaration requires 'ns'")
        self.ns_prefix = self.ns.name

        self.name = kwargs.get("name", None)
        if self.name is None:
            raise ValueError("Namespace declaration requires 'name'")

        self._ns_obj = Namespace(self.ns.uri)
        self.uri = self._ns_obj[self.name]

    @property
    def namespace(self) -> Namespace:
        return self._ns_obj

    def __str__(self) -> str:
        return f"<({self.__class__.__name__}) {self.ns_prefix}:{self.name}>"


class IHasUUID(IHasParent):
    uuid: UUID

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.uuid = uuid4()

    @property
    def uri(self) -> URIRef:
        raise NotImplementedError(f"'uri' property not implemented for '{self.__class__.__name__}'")


class SetBase(IHasParent):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)


def _expect_size(values: list, size: int, context: str) -> list:
    if len(values) != size:
        raise ValueError(f"{context} requires {size} values, got {len(values)}")
    return values


class FloatVector:
    values: list[float]

    def __init__(self, parent, values) -> None:
        self.parent = parent
        self.values = values

    def as_xyz(self, context: str = "FloatVector.as_xyz") -> tuple[float, float, float]:
        return tuple(_expect_size(values=self.values, size=3, context=context))

    def as_low_high(self, context: str = "FloatVector.as_low_high") -> tuple[float, float]:
        return tuple(_expect_size(values=self.values, size=2, context=context))


class IntVector:
    values: list[int]

    def __init__(self, parent, values) -> None:
        self.parent = parent
        self.values = values

    def as_width_height(self, context: str = "IntVector.as_width_height") -> tuple[int, int]:
        return tuple(_expect_size(values=self.values, size=2, context=context))


def parse_py_module_attr(model: Any) -> tuple[str, str]:
    modules = getattr(model, "modules", None)
    if not isinstance(modules, list):
        raise ValueError(
            f"PyModuleAttr {model} doesn't have 'modules' field of list type: {modules}"
        )
    attr_name = getattr(model, "attr_name", None)
    if attr_name is None:
        raise ValueError(f"PyModuleAttr {model} doesn't have valid 'attr_name' field")
    return ".".join(modules), attr_name
