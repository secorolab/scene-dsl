import numpy as np
import pytest
from rdf_utils.models.distribution import (
    DistributionModel,
    distrib_from_sampled_quantity,
    sample_from_distrib,
)
from rdf_utils.models.geom_coord import (
    get_rotation_between_frames,
    get_transform_between_frames,
    get_translation_between_points,
)
from rdf_utils.models.vocab import (
    URI_DISTRIB_PRED_FROM_DISTRIB,
    URI_DISTRIB_PRED_LOWER,
    URI_DISTRIB_PRED_UPPER,
    URI_DISTRIB_TYPE_DISTRIB,
    URI_DISTRIB_TYPE_NORMAL,
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_DISTRIB_TYPE_UNIFORM_ROT,
    URI_DYN_PRED_ABOUT,
)
from rdflib import RDF, Graph

from scene_dsl.classes.distrib import DistributionRef
from scene_dsl.classes.geom import PoseSpec
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.geom import add_pose
from scene_dsl.rdf.scenex import create_scenex_model_graph

from .test_common import MODELS_DIR


def test_shared_distributions_generate_sampled_quantity_links():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "sampled-poses.scenex")
    graph = create_scenex_model_graph(model)
    distributions = {distribution.name: distribution for distribution in model.distributions}
    uniform_xyz = distributions["uniform-xyz"]
    rotation = distributions["rot"]
    normal_xyz = distributions["normal-xyz"]
    normal_scalar = distributions["normal-scalar"]

    assert (uniform_xyz.uri, RDF.type, URI_DISTRIB_TYPE_DISTRIB) in graph
    assert (normal_xyz.uri, RDF.type, URI_DISTRIB_TYPE_NORMAL) in graph
    bodies = {body.name: body for body in model.scene_insts[0].kgraph.bodies}
    table = bodies["table"]
    assert table.inertia.frame is table.default_frame
    assert (table.inertia_uri, URI_DYN_PRED_ABOUT, table.default_frame.origin_uri) in graph
    uniform_pose = bodies["uniform_object"].frames[0].poses[0]
    normal_pose = bodies["normal_object"].frames[0].poses[0]
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
    assert (
        normal_pose.orientation_coord_uri,
        URI_DISTRIB_PRED_FROM_DISTRIB,
        rotation.uri,
    ) in graph

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


def test_sampled_pose_adds_each_referenced_distribution_once():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "sampled-poses.scenex")
    pose = next(
        pose
        for body in model.scene_insts[0].kgraph.bodies
        for frame in body.frames
        for pose in frame.poses
        if isinstance(pose.position_spec, DistributionRef)
    )
    assert isinstance(pose.position_spec, DistributionRef)
    assert isinstance(pose.orientation.spec, DistributionRef)
    position_distribution = pose.position_spec.distribution
    orientation_distribution = pose.orientation.spec.distribution
    graph = Graph()

    add_pose(graph, pose)
    add_pose(graph, pose)

    assert (position_distribution.uri, RDF.type, URI_DISTRIB_TYPE_DISTRIB) in graph
    assert (orientation_distribution.uri, RDF.type, URI_DISTRIB_TYPE_UNIFORM_ROT) in graph
    assert len(list(graph.objects(position_distribution.uri, URI_DISTRIB_PRED_LOWER))) == 1
    assert len(list(graph.objects(position_distribution.uri, URI_DISTRIB_PRED_UPPER))) == 1


def test_non_three_dimensional_normal_is_rejected_for_xyz_sampling():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "sampled-poses.scenex")
    next(
        distribution for distribution in model.distributions if distribution.name == "normal-xyz"
    ).spec.dimension = 2
    with pytest.raises(ValueError, match="dimension 3"):
        create_scenex_model_graph(model)


def test_pose_paths_resolve_concrete_and_sampled_coordinates():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "sampled-poses.scenex")
    graph = create_scenex_model_graph(model)
    bodies = {body.name: body for body in model.scene_insts[0].kgraph.bodies}
    world = bodies["world"].default_frame
    table = bodies["table"].default_frame

    assert get_translation_between_points(table.origin_uri, world.origin_uri, graph) == (
        1.0,
        2.0,
        0.75,
    )
    assert np.allclose(
        get_rotation_between_frames(table.uri, world.uri, graph).as_matrix(), np.eye(3)
    )
    assert np.allclose(
        get_transform_between_frames(table.uri, world.uri, graph).as_matrix(),
        np.array(
            (
                (1.0, 0.0, 0.0, 1.0),
                (0.0, 1.0, 0.0, 2.0),
                (0.0, 0.0, 1.0, 0.75),
                (0.0, 0.0, 0.0, 1.0),
            )
        ),
    )

    for body_name in ("uniform_object", "normal_object"):
        sampled = bodies[body_name].default_frame
        sampled_pose = sampled.poses[0]

        seed = 42
        expected_translation = sample_from_distrib(
            distrib_from_sampled_quantity(sampled_pose.position_coord_uri, graph),
            rng=np.random.default_rng(seed),
        ) + np.array((1.0, 2.0, 0.75))
        assert np.allclose(
            get_translation_between_points(
                sampled.origin_uri,
                world.origin_uri,
                graph,
                rng=np.random.default_rng(seed),
            ),
            expected_translation,
        )

        expected_rotation = sample_from_distrib(
            distrib_from_sampled_quantity(sampled_pose.orientation_coord_uri, graph),
            rng=np.random.default_rng(seed),
        )
        assert np.allclose(
            get_rotation_between_frames(
                sampled.uri,
                world.uri,
                graph,
                rng=np.random.default_rng(seed),
            ).as_matrix(),
            expected_rotation.as_matrix(),
        )

        expected_rng = np.random.default_rng(seed)
        object_transform = np.eye(4)
        object_transform[:3, 3] = sample_from_distrib(
            distrib_from_sampled_quantity(sampled_pose.position_coord_uri, graph),
            rng=expected_rng,
        )
        object_transform[:3, :3] = sample_from_distrib(
            distrib_from_sampled_quantity(sampled_pose.orientation_coord_uri, graph),
            rng=expected_rng,
        ).as_matrix()
        table_transform = get_transform_between_frames(table.uri, world.uri, graph)
        assert np.allclose(
            get_transform_between_frames(
                sampled.uri, world.uri, graph, rng=np.random.default_rng(seed)
            ).as_matrix(),
            table_transform.as_matrix() @ object_transform,
        )
