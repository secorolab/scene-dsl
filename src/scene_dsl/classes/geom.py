# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from typing import Optional

from rdflib import Namespace, URIRef

from scene_dsl.classes.common import FloatVector, IHasNamespace


class OrientationSpec:
    pass


class EulerOrientationSpec(OrientationSpec):
    axes: str
    extrinsic: bool
    angles: FloatVector
    unit: str

    alpha: float
    beta: float
    gamma: float

    def __init__(self, parent, axes, extrinsic, angles, unit) -> None:
        self.parent = parent
        self.extrinsic = extrinsic
        self.alpha, self.beta, self.gamma = angles.as_xyz("EulerOrientationSpec.angles")
        self.axes = axes or "zyx"
        self.unit = unit or "rad"


class DirectionCosineOrientationSpec(OrientationSpec):
    x_axis: tuple[float, float, float]
    y_axis: tuple[float, float, float]
    z_axis: tuple[float, float, float]

    def __init__(
        self, parent, x_axis: FloatVector, y_axis: FloatVector, z_axis: FloatVector
    ) -> None:
        self.parent = parent
        self.x_axis = x_axis.as_xyz("DirectionCosineOrientationSpec.x")
        self.y_axis = y_axis.as_xyz("DirectionCosineOrientationSpec.y")
        self.z_axis = z_axis.as_xyz("DirectionCosineOrientationSpec.z")


class PoseSpec(IHasNamespace):
    name: str
    xyz: FloatVector
    length_unit: str
    orientation: OrientationSpec

    _uri: Optional[URIRef]
    _uri_coord: Optional[URIRef]
    _position_uri: Optional[URIRef]
    _position_coord_uri: Optional[URIRef]
    _wrt: Optional[Frame]

    def __init__(self, parent, name, wrt, xyz, length_unit, orientation) -> None:
        super().__init__(parent=parent)
        self.name = name
        self._wrt = wrt
        self.xyz = xyz
        self.length_unit = length_unit
        self.orientation = orientation
        self._uri = None
        self._uri_coord = None
        self._position_uri = None

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, Frame):
            raise TypeError(f"parent of PoseSpec is not a Frame: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri

    @property
    def uri_coord(self) -> URIRef:
        if self._uri_coord is None:
            self._uri_coord = self.namespace[f"{self.name}-coord"]
        return self._uri_coord

    @property
    def position_uri(self) -> URIRef:
        if self._position_uri is None:
            self._position_uri = self.namespace[f"{self.name}-position"]
        return self._position_uri

    @property
    def wrt(self) -> Frame:
        if self._wrt is not None:
            return self._wrt

        if not isinstance(self.parent, IDefaultFrame):
            raise TypeError(
                f"PoseSpec({self.name}).wrt: no wrt specified and parent not a IDefaultFrame: {self.parent}"
            )

        return self.parent.default_frame

    @property
    def of_frame(self) -> Frame:
        if not isinstance(self.parent, Frame):
            raise TypeError(f"PoseSpec({self.name}).of_frame: parent is not a Frame: {self.parent}")
        return self.parent


class IDefaultFrame:
    @property
    def default_frame(self) -> Frame:
        raise NotImplementedError(f"IDefaultFrame.default_frame not implemented for {self}")


class Frame(IHasNamespace, IDefaultFrame):
    name: str
    poses: list[PoseSpec]

    _uri: Optional[URIRef]
    _origin_uri: Optional[URIRef]
    _axis_vector_uris: dict[str, URIRef]

    def __init__(self, parent, name, poses) -> None:
        super().__init__(parent=parent)
        self.name = name
        self.poses = poses
        self._uri = None
        self._origin_uri = None
        self._axis_vector_uris = {}

    @property
    def namespace(self) -> Namespace:
        if not isinstance(self.parent, IHasNamespace):
            raise TypeError(f"parent of frame has no namespace: {self.parent}")
        return self.parent.namespace

    @property
    def uri(self) -> URIRef:
        if self._uri is None:
            self._uri = self.namespace[self.name]
        return self._uri

    @property
    def origin_uri(self) -> URIRef:
        if self._origin_uri is None:
            self._origin_uri = URIRef(f"{self.uri}-origin")
        return self._origin_uri

    @property
    def default_frame(self) -> Frame:
        if not isinstance(self.parent, IDefaultFrame):
            raise ValueError(
                f"Frame({self.name}).default_frame: parent not a IDefaultFrame: {self.parent}"
            )

        return self.parent.default_frame

    def axis_vector_uri(self, axis: str) -> URIRef:
        if axis in self._axis_vector_uris:
            return self._axis_vector_uris[axis]

        if axis != "x" and axis != "y" and axis != "z":
            raise ValueError(f"Frame.axis_vector_uri: invalid axis for frame '{self.name}': {axis}")

        uri = self.namespace[f"{self.name}-vector-{axis}"]
        self._axis_vector_uris[axis] = uri
        return uri


class FrameAxis:
    frame: Frame
    axis: str

    def __init__(self, parent, frame, axis) -> None:
        self.parent = parent
        self.frame = frame
        self.axis = axis
