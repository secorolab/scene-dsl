# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations
from typing import Optional

from rdflib import Namespace, URIRef

from scene_dsl.classes.common import IHasNamespace, IntVector
from scene_dsl.classes.geom import Frame


class SensorBase(IHasNamespace):
    frame: Frame
    update_rate: float
    rate_unit: str
    _uri: Optional[URIRef]

    def __init__(self, parent, name, frame, update_rate, rate_unit) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.frame = frame
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
            # Scoped by the agent carrying it: two robots may each have a 'wrist' camera.
            self._uri = self.namespace[self.scoped()]
        return self._uri


class CameraSensorSpec(SensorBase):
    cam_type: str
    resolution: IntVector
    fov: float
    fov_unit: str

    def __init__(
        self,
        parent,
        name,
        frame,
        cam_type,
        resolution: IntVector,
        fov,
        fov_unit,
        update_rate,
        rate_unit,
    ) -> None:
        super().__init__(
            parent=parent, name=name, frame=frame, update_rate=update_rate, rate_unit=rate_unit
        )
        self.cam_type = cam_type
        self.resolution = resolution
        self.fov = fov
        self.fov_unit = fov_unit


class ForceTorqueSensorSpec(SensorBase):
    observes: list[str]

    def __init__(self, parent, name, frame, observes, update_rate, rate_unit) -> None:
        super().__init__(
            parent=parent, name=name, frame=frame, update_rate=update_rate, rate_unit=rate_unit
        )
        self.observes = observes


class ImuSensorSpec(SensorBase):
    observes: list[str]

    def __init__(self, parent, name, frame, observes, update_rate, rate_unit) -> None:
        super().__init__(
            parent=parent, name=name, frame=frame, update_rate=update_rate, rate_unit=rate_unit
        )
        self.observes = observes
