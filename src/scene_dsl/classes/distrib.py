# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations
from typing import Optional

from scene_dsl.classes.common import FloatVector, IHasNamespaceDeclare


class FloatMatrix:
    rows: list[FloatVector]

    def __init__(self, parent, rows) -> None:
        self.parent = parent
        self.rows = rows

    @property
    def values(self) -> tuple[tuple[float, ...], ...]:
        return tuple(tuple(row.values) for row in self.rows)


class UniformDistribution:
    dimension: int
    lower: FloatVector
    upper: FloatVector

    def __init__(self, parent, dimension, lower, upper) -> None:
        self.parent = parent
        self.dimension = dimension
        if self.dimension < 1:
            raise ValueError(
                f"UniformDistribution ({self.parent}): dimension not positive {self.dimension}"
            )
        self.lower = lower
        self.upper = upper
        len_lower = len(self.lower.values)
        len_upper = len(self.upper.values)
        if len_lower != self.dimension or len_upper != self.dimension:
            raise ValueError(
                f"UniformDistribution ({self.parent}): dimension ({self.dimension}) doesn't match"
                f" number of lower ({len_lower}) or upper ({len_upper}) values"
            )
        if any(low >= high for low, high in zip(self.lower.values, self.upper.values)):
            raise ValueError(
                f"({self.parent})UniformDistribution.lower must be strictly"
                " less than upper for every component"
            )


class NormalDistribution:
    dimension: int
    mean_scalar: Optional[float]
    mean_vector: Optional[FloatVector]
    std_dev: Optional[float]
    covariance: Optional[FloatMatrix]

    def __init__(
        self,
        parent,
        dimension,
        mean_scalar,
        mean_vector,
        std_dev,
        covariance,
    ) -> None:
        self.parent = parent
        self.dimension = dimension
        if self.dimension < 1:
            raise ValueError(
                f"NormalDistribution ({self.parent}): dimension not positive {self.dimension}"
            )
        self.mean_vector = mean_vector
        if self.mean_vector is None:
            self.mean_scalar = mean_scalar
        else:
            self.mean_scalar = None
        self.std_dev = std_dev if covariance is None else None
        self.covariance = covariance


class UniformRotationDistribution:
    kind: str

    def __init__(self, parent, kind) -> None:
        self.parent = parent
        self.kind = kind


class Distribution(IHasNamespaceDeclare):
    spec: UniformDistribution | NormalDistribution | UniformRotationDistribution

    def __init__(self, parent, ns, name, spec) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.spec = spec


class DistributionRef:
    distribution: Distribution

    def __init__(self, parent, distribution: Distribution) -> None:
        self.parent = parent
        self.distribution = distribution
