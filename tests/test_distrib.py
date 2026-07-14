import numpy as np
import pytest
from rdflib import RDF

from rdf_utils.models.distribution import (
    DistributionModel,
    distrib_from_sampled_quantity,
    sample_from_distrib,
)
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_TYPE_DISTRIB,
    URI_DISTRIB_TYPE_NORMAL,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
)
from scene_dsl.classes.geom import PoseSpec
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from .test_common import MODELS_DIR


def test_shared_distributions_generate_sampled_quantity_links():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "distributions.scenex")
    graph = create_scenex_model_graph(model)
    distributions = {distribution.name: distribution for distribution in model.distributions}
    uniform_xyz = distributions["uniform-xyz"]
    rotation = distributions["rot"]
    normal_xyz = distributions["normal-xyz"]
    normal_scalar = distributions["normal-scalar"]

    assert (uniform_xyz.uri, RDF.type, URI_DISTRIB_TYPE_DISTRIB) in graph
    assert (normal_xyz.uri, RDF.type, URI_DISTRIB_TYPE_NORMAL) in graph
    poses = model.scene_insts[0].ktree.bodies[1].frames[0].poses
    uniform_pose, normal_pose = poses
    assert isinstance(uniform_pose, PoseSpec)
    assert (uniform_pose.position_coord_uri, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY) in graph
    assert (
        uniform_pose.position_coord_uri,
        URI_DISTRIB_PRED_FROM_DISTRIB,
        uniform_xyz.uri,
    ) in graph
    assert (
        uniform_pose.orientation_coord_uri,
        URI_DISTRIB_PRED_FROM_DISTRIB,
        rotation.uri,
    ) in graph
    assert (normal_pose.position_coord_uri, RDF.type, URI_DISTRIB_TYPE_SAMPLED_QUANTITY) in graph
    assert (normal_pose.position_coord_uri, URI_DISTRIB_PRED_FROM_DISTRIB, normal_xyz.uri) in graph

    uniform_sample = sample_from_distrib(
        distrib_from_sampled_quantity(uniform_pose.position_coord_uri, graph), size=(4, 3)
    )
    assert uniform_sample.shape == (4, 3)
    assert np.all(uniform_sample >= np.asarray(uniform_xyz.spec.lower.values))
    assert np.all(uniform_sample <= np.asarray(uniform_xyz.spec.upper.values))

    normal_model = DistributionModel(distrib_id=normal_xyz.uri, graph=graph)
    normal_sample = sample_from_distrib(
        distrib_from_sampled_quantity(normal_pose.position_coord_uri, graph), size=20
    )
    assert normal_sample.shape == (20, 3)
    assert np.isfinite(normal_sample).all()
    assert normal_model.distrib_type == URI_DISTRIB_TYPE_NORMAL

    scalar_model = DistributionModel(distrib_id=normal_scalar.uri, graph=graph)
    scalar_sample = sample_from_distrib(distrib=scalar_model, size=8)
    assert scalar_sample.shape == (8,)
    assert np.isfinite(scalar_sample).all()

    pytest.importorskip("scipy")
    rotation_sample = sample_from_distrib(
        distrib_from_sampled_quantity(uniform_pose.orientation_coord_uri, graph)
    )
    assert rotation_sample.as_matrix().shape == (3, 3)


def test_non_three_dimensional_normal_is_rejected_for_xyz_sampling():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "distributions.scenex")
    next(
        distribution for distribution in model.distributions if distribution.name == "normal-xyz"
    ).spec.dimension = 2
    with pytest.raises(ValueError, match="dimension 3"):
        create_scenex_model_graph(model)
