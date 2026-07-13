# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from rdflib import Namespace, URIRef

from scene_dsl.classes.common import IHasNamespace, IntVector


class SensorBase(IHasNamespace):
    def __init__(self, parent, name, body, update_rate, rate_unit) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.body = body
        self.update_rate = update_rate
        self.rate_unit = rate_unit
        self._uri = None

    @property
    def namespace(self) -> Namespace:
        agent = self.parent
        if not isinstance(agent, IHasNamespace):
            raise TypeError(f"sensor parent has no namespace: {agent}")
        return agent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri


class CameraSensorSpec(SensorBase):
    resolution: IntVector

    def __init__(
        self,
        parent,
        name,
        body,
        type,
        resolution: IntVector,
        fov,
        fov_unit,
        update_rate,
        rate_unit,
    ) -> None:
        super().__init__(parent, name, body, update_rate, rate_unit)
        self.type = type
        self.resolution = resolution
        self.fov = fov
        self.fov_unit = fov_unit


class ForceTorqueSensorSpec(SensorBase):
    def __init__(self, parent, name, body, observes, update_rate, rate_unit) -> None:
        super().__init__(parent, name, body, update_rate, rate_unit)
        self.observes = observes


class ImuSensorSpec(SensorBase):
    def __init__(self, parent, name, body, observes, update_rate, rate_unit) -> None:
        super().__init__(parent, name, body, update_rate, rate_unit)
        self.observes = observes
