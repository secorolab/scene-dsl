# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations


from scene_dsl.classes.common import FloatVector, IHasNamespaceDeclare


class UniformDistribution:
    lower: tuple[float, float, float]
    upper: tuple[float, float, float]

    def __init__(self, parent, lower: FloatVector, upper: FloatVector) -> None:
        self.parent = parent
        self.lower = lower.as_xyz("UniformDistribution.lower")
        self.upper = upper.as_xyz("UniformDistribution.upper")
        if any(low >= high for low, high in zip(self.lower, self.upper)):
            raise ValueError(
                "UniformDistribution.lower must be strictly less than upper for every component"
            )


class UniformRotationDistribution:
    def __init__(self, parent, kind) -> None:
        self.parent = parent
        self.kind = kind


class Distribution(IHasNamespaceDeclare):
    spec: UniformDistribution | UniformRotationDistribution

    def __init__(self, parent, ns, name, spec) -> None:
        super().__init__(parent=parent, ns=ns, name=name)
        self.spec = spec


class DistributionRef:
    distribution: Distribution

    def __init__(self, parent, distribution: Distribution) -> None:
        self.parent = parent
        self.distribution = distribution
