import pytest
from rdflib import RDF, Literal, URIRef
from textx.exceptions import TextXSemanticError, TextXSyntaxError

from rdf_utils.constraints import check_shacl_constraints
from rdf_utils.models.vocab import (
    URI_QUDT_QK_ANGLE,
    URI_QUDT_PRED_QUANTITY_KIND,
    URI_QUDT_PRED_UNIT,
)
from rdf_utils.models.geometry import (
    OrientCoordModel,
    PoseCoordModel,
    PositionCoordModel,
    get_coord_vectorxyz,
    get_euler_angles_abg,
)

from rdf_utils.namespace import URL_MM_GEOM_SHACL_EXTS, URL_MM_GEOM_SHACL_REL, URL_SECORO_MM
from rdf_utils.resolver import install_resolver

from scene_dsl.classes.common import FloatVector
from scene_dsl.classes.geom import DirectionCosineOrientationSpec
from scene_dsl.classes.ktree import KinematicTreeInstance, KinematicTreeTemplate, RigidBodyInertia
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.geom import ANGLE_UNITS, URI_GEOM_TYPE_FRAME
from scene_dsl.rdf.ktree import (
    INERTIA_UNITS,
    URI_GEOM_TYPE_KGRAPH,
    URI_GEOM_TYPE_RIGID_BODY,
    NS_MM_KC_EXT,
    URI_GEOM_TYPE_KTREE,
    URI_KC_TYPE_KC,
    URI_KC_TYPE_SERIAL,
    URI_KC_PRED_BETWEEN_ATTACHMENTS,
    URI_KC_TYPE_REVOLUTE_JOINT,
    MASS_UNITS,
    URI_DYN_PRED_ABOUT,
    URI_DYN_PRED_IXX,
    URI_DYN_PRED_IXY,
    URI_DYN_PRED_IXZ,
    URI_DYN_PRED_IYY,
    URI_DYN_PRED_IYZ,
    URI_DYN_PRED_IZZ,
    URI_DYN_TYPE_MOMENT_OF_INERTIA_XYZ,
    URI_DYN_TYPE_PRODUCT_OF_INERTIA_XYZ,
    URI_ACT_PRED_COMMAND_INTERFACE,
    URI_ACT_PRED_JOINT,
    URI_KC_EXT_PRED_DEPENDENT_JOINT,
    URI_KC_EXT_PRED_OFFSET,
    URI_QUDT_TYPE_QUANTITY,
    URI_KC_EXT_PRED_INDEPENDENT_JOINT,
    URI_KC_EXT_TYPE_JOINT_COUPLING,
    URI_KC_EXT_TYPE_SCLERONOMIC,
    URI_ACT_PRED_STATE_INTERFACE,
    URI_ACT_TYPE_ACTUATION,
    ACTUATION_INTERFACE_TYPES,
)
from scene_dsl.rdf.scenex import (
    URI_EXEC_PRED_HAS_KTREE,
    URI_EXEC_PRED_LINKS_BODY,
    create_scenex_model_graph,
)
from .test_common import MODELS_DIR, write_example_scene

ZERO_POSE = (
    "pose default_pose { xyz: (0.0, 0.0, 0.0) m orientation: euler { angles: (0.0, 0.0, 0.0) } }"
)


def test_top_level_template_is_a_kinematic_tree_spec():
    model = scenex_metamodel().model_from_str(
        """ktree arm {
    body base { frame root { } }
    joints { }
}
"""
    )

    [tree] = model.ktrees
    assert isinstance(tree, KinematicTreeTemplate)


def test_top_level_tree_instance_is_a_distinct_kinematic_tree_spec():
    model = scenex_metamodel().model_from_str(
        """ns n = "https://example.test/"
ktree arm {
    body base { frame root { } }
    joints { }
}
ktree inst (ns=n) robot_arm of <arm>
"""
    )

    assert isinstance(model.ktrees[1], KinematicTreeInstance)


def test_duplicate_model_uri_is_rejected(tmp_path):
    write_example_scene(tmp_path)
    model_path = tmp_path / "duplicate_model.scenex"
    model_path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    obj <objs.cup> {
        model dup as urdf { sys path = "cup.urdf" }
    }
    obj <objs.bowl> {
        model dup as urdf { sys path = "bowl.urdf" }
    }
}
"""
    )

    with pytest.raises(TextXSemanticError, match="minted by both"):
        scenex_metamodel().model_from_file(model_path)


def test_joint_iri_is_scoped_by_tree(tmp_path):
    """A frame may be named after the joint it carries: the joint is scoped by its tree."""
    write_example_scene(tmp_path)
    model_path = tmp_path / "scoped.scenex"
    model_path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    agn <agns.robot> {
        model m as urdf { sys path = "robot.urdf" }
        ktree (ns=n) t {
            body b1 { frame j1 { } }
            body b2 { frame f2 { } }
            joints {
                revolute j1 {
                    parent: <b1.j1>.z
                    child: <b2.f2>.z
                    polarity: PositivePolarity
                }
            }
        }
    }
}
"""
    )

    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(model_path))
    frame_uri = URIRef("https://example.test/t/b1/j1")
    joint_uri = URIRef("https://example.test/t/j1")
    assert (frame_uri, RDF.type, URI_GEOM_TYPE_FRAME) in graph
    assert (joint_uri, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT) in graph
    assert (frame_uri, RDF.type, URI_KC_TYPE_REVOLUTE_JOINT) not in graph


def test_duplicate_iri_across_element_kinds_is_rejected(tmp_path):
    """A body and a joint are different elements, and both are scoped by their tree alone."""
    write_example_scene(tmp_path)
    model_path = tmp_path / "duplicate_joint.scenex"
    model_path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    agn <agns.robot> {
        model m as urdf { sys path = "robot.urdf" }
        ktree (ns=n) t {
            body wrist { frame f1 { } }
            body b2 { frame f2 { } }
            joints {
                revolute wrist {
                    parent: <wrist.f1>.z
                    child: <b2.f2>.z
                    polarity: PositivePolarity
                }
            }
        }
    }
}
"""
    )

    with pytest.raises(TextXSemanticError, match="minted by both"):
        scenex_metamodel().model_from_file(model_path)


@pytest.mark.parametrize(
    ("length_unit", "mass_unit", "inertia_unit"),
    [("m", "kg", "kg*m^2"), ("cm", "g", "kg*m^2"), ("mm", "kg", "kg*m^2")],
)
def test_scenex_length_and_mass_units(tmp_path, length_unit, mass_unit, inertia_unit):
    write_example_scene(tmp_path)
    model_path = tmp_path / "units.scenex"
    model_path.write_text(
        f"""import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {{
    scene: <s>
    kgraph (ns=n) g {{
    body world_body {{ frame world {{ }} }}
    body cup_body {{
        frame cup_root {{
            pose cup_in_world {{
                wrt: <world_body.world>
                xyz: (1.0, 2.0, 3.0) {length_unit}
                orientation: euler {{ angles: (0.0, 0.0, 0.0) }}
            }}
        }}
        inertia {{
            frame: cup_root
            mass: 10.0 {mass_unit}
            inertia-matrix: (
                (1.0, 0.1, 0.2),
                (0.1, 2.0, 0.3),
                (0.2, 0.3, 3.0)
            ) {inertia_unit}
        }}
    }}
    }}
    obj <objs.cup> {{
        model cup_model as urdf {{ sys path = "cup.urdf" }}
        body: <g.cup_body>
    }}
}}
"""
    )

    model = scenex_metamodel().model_from_file(model_path)
    cup_body = model.scene_insts[0].modelled_objs[0].body
    pose = cup_body.frames[0].poses[0]
    graph = create_scenex_model_graph(model)
    position_coord = PositionCoordModel(pose.position_coord_uri, graph)
    orient_coord = OrientCoordModel(pose.orientation_coord_uri, graph)
    pose_coord = PoseCoordModel(pose.uri_coord, graph)

    assert position_coord.position == pose.position_uri
    assert position_coord.of == pose.of_frame.origin_uri
    assert position_coord.wrt == pose.wrt.origin_uri
    assert position_coord.as_seen_by == pose.wrt.uri
    assert pose_coord.pose == pose.uri
    assert pose_coord.of.id == pose.of_frame.uri
    assert pose_coord.wrt.id == pose.wrt.uri
    assert pose_coord.as_seen_by == pose.wrt.uri

    assert get_coord_vectorxyz(position_coord, graph) == tuple(pose.position_spec.values)

    axes, is_intrinsic, unit_uri, angles = get_euler_angles_abg(orient_coord, graph)
    assert axes == pose.orientation.axes
    assert is_intrinsic != pose.orientation.extrinsic
    assert unit_uri == ANGLE_UNITS[pose.orientation.unit]
    assert angles == tuple(pose.orientation.angles.values)

    assert (cup_body.inertia_coord_uri, URI_QUDT_PRED_UNIT, MASS_UNITS[mass_unit]) in graph
    assert (cup_body.inertia_coord_uri, URI_QUDT_PRED_UNIT, INERTIA_UNITS[inertia_unit]) in graph
    assert (cup_body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_MOMENT_OF_INERTIA_XYZ) in graph
    assert (cup_body.inertia_coord_uri, RDF.type, URI_DYN_TYPE_PRODUCT_OF_INERTIA_XYZ) in graph

    matrix = cup_body.inertia.matrix
    for predicate, value in (
        (URI_DYN_PRED_IXX, matrix[0][0]),
        (URI_DYN_PRED_IXY, matrix[0][1]),
        (URI_DYN_PRED_IXZ, matrix[0][2]),
        (URI_DYN_PRED_IYY, matrix[1][1]),
        (URI_DYN_PRED_IYZ, matrix[1][2]),
        (URI_DYN_PRED_IZZ, matrix[2][2]),
    ):
        assert (cup_body.inertia_coord_uri, predicate, Literal(value)) in graph


def test_scenex_mass_quantity_validation(tmp_path):
    write_example_scene(tmp_path)
    model_path = tmp_path / "mass_quantity.scenex"
    model_path.write_text(
        f"""import "example.scene"
ns n = "https://example.test/"
ktree (ns=n) world_tree {{
    body cup_body {{
        frame cup_root {{ {ZERO_POSE} }}
        inertia {{
            frame: cup_root
            mass: 0.0 kg
            inertia-matrix: (
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0)
            ) kg*m^2
        }}
    }}
    joints {{ }}
}}
scene inst (ns=n) sx {{
    scene: <s>
    kgraph (ns=n) world_tree_graph {{
        tree <world_tree>
    }}
}}
"""
    )

    cup_body = scenex_metamodel().model_from_file(model_path).scene_insts[0].kgraph.trees[0].bodies[0]
    assert cup_body.inertia.mass == 0.0

    with pytest.raises(ValueError, match="RigidBodyInertia.mass must be"):
        RigidBodyInertia(
            parent=None,
            frame=None,
            mass=-1.0,
            mass_unit="kg",
            row1=FloatVector(None, [1.0, 0.0, 0.0]),
            row2=FloatVector(None, [0.0, 1.0, 0.0]),
            row3=FloatVector(None, [0.0, 0.0, 1.0]),
            inertia_unit="kg*m^2",
        )


def test_rigid_body_inertia_rejects_non_symmetric_matrix():
    with pytest.raises(ValueError, match="RigidBodyInertia.matrix must be symmetric"):
        RigidBodyInertia(
            parent=None,
            frame=None,
            mass=0.0,
            mass_unit="kg",
            row1=FloatVector(None, [1.0, 0.1, 0.0]),
            row2=FloatVector(None, [0.0, 1.0, 0.0]),
            row3=FloatVector(None, [0.0, 0.0, 1.0]),
            inertia_unit="kg*m^2",
        )


def test_direction_cosine_orientation_rejects_non_orthogonal_matrix():
    with pytest.raises(ValueError, match="rotation_matrix must be orthogonal"):
        DirectionCosineOrientationSpec(
            parent=None,
            x_axis=FloatVector(None, [1.0, 0.0, 0.0]),
            y_axis=FloatVector(None, [1.0, 0.0, 0.0]),
            z_axis=FloatVector(None, [0.0, 0.0, 1.0]),
        )


def write_device_ktree(tmp_path) -> None:
    (tmp_path / "device.ktree").write_text(
        """ktree arm_tree {
    body arm_base { frame arm_root { } }
    joints { }
}
"""
    )


def _scene_using_device(name: str) -> str:
    return f"""import "example.scene"
import "device.ktree"
ns {name} = "https://example.test/{name}/"
ktree inst (ns={name}) arm of <arm_tree>
ktree (ns={name}) world_tree {{
    tree <arm>
    body table_body {{ frame table_root {{ }} }}
    joints {{
        fixed arm_on_table {{
            parent: <table_body.table_root>
            child: <arm.arm_base.arm_root>
        }}
    }}
}}
scene inst (ns={name}) {name} {{
    scene: <s>
    kgraph (ns={name}) world_tree_graph {{
        tree <world_tree>
    }}
    obj <objs.cup> {{
        model cup_model as urdf {{ sys path = "cup.urdf" }}
        body: <world_tree.table_body>
    }}
}}
"""


def test_ktree_file_is_reused_across_scenes(tmp_path):
    """The same .ktree imported by two scenes is one tree, in its own namespace."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    uris = []
    for name in ("scene_a", "scene_b"):
        path = tmp_path / f"{name}.scenex"
        path.write_text(_scene_using_device(name))
        model = scenex_metamodel().model_from_file(path)
        tree = model.scene_insts[0].kgraph.trees[0].trees[0]
        uris.append(tree.uri)

        graph = create_scenex_model_graph(model)
        assert (tree.uri, RDF.type, URI_GEOM_TYPE_KTREE) in graph
        assert (
            URIRef(f"https://example.test/{name}/arm/arm_base/arm_root"),
            RDF.type,
            URI_GEOM_TYPE_FRAME,
        ) in graph

    assert uris[0] != uris[1]


def test_ktree_import_does_not_break_local_refs(tmp_path):
    """A '.ktree' import must be parsed by the importing metamodel.

    Registering a separate metamodel for '*.ktree' would re-register the shared user
    classes mid-parse and wipe textX's in-flight attribute store, silently breaking
    unrelated local FQN lookups.
    """
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "scene_a.scenex"
    path.write_text(_scene_using_device("scene_a"))

    model = scenex_metamodel().model_from_file(path)
    # A ref into the *local* tree, resolved while an imported .ktree is in the repository.
    assert model.scene_insts[0].modelled_objs[0].body.name == "table_body"


def test_modelled_object_can_reference_instanced_body(tmp_path):
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "scene_a.scenex"
    path.write_text(
        _scene_using_device("scene_a").replace(
            "body: <world_tree.table_body>", "body: <arm.arm_base>"
        )
    )

    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(path))
    assert URIRef("https://example.test/scene_a/arm/arm_base") in graph.objects(
        predicate=URI_EXEC_PRED_LINKS_BODY
    )


def test_instances_of_one_template_share_a_namespace(tmp_path):
    """The instance name scopes the copy, so one namespace holds any number of devices."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "dual.scenex"
    path.write_text(_dual_arm_scene())

    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(path))
    for arm in ("arm1", "arm2"):
        assert (URIRef(f"https://example.test/lab/{arm}"), RDF.type, URI_GEOM_TYPE_KTREE) in graph
        assert (
            URIRef(f"https://example.test/lab/{arm}/arm_base/arm_root"),
            RDF.type,
            URI_GEOM_TYPE_FRAME,
        ) in graph


def _dual_arm_scene() -> str:
    return """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"

ktree inst (ns=lab) arm1 of <arm_tree>
ktree inst (ns=lab) arm2 of <arm_tree>

ktree (ns=lab) world_tree {
    tree <arm1>
    tree <arm2>
    body table_body { frame table_root { } }
    joints {
        fixed arm1_on_table {
            parent: <table_body.table_root>
            child: <arm1.arm_base.arm_root>
        }
        fixed arm2_on_table {
            parent: <table_body.table_root>
            child: <arm2.arm_base.arm_root>
        }
    }
}
scene inst (ns=lab) dual {
    scene: <s>
    kgraph (ns=lab) world_tree_graph {
        tree <world_tree>
    }
}
"""


def test_tree_instances_are_distinct_but_identical(tmp_path):
    """Two instances of one device share no IRI, and are otherwise the same tree."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "dual.scenex"
    path.write_text(_dual_arm_scene())

    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(path))
    prefix_a, prefix_b = "https://example.test/lab/arm1", "https://example.test/lab/arm2"

    def normalise(triple, prefix):
        out = []
        for term in triple:
            text = str(term)
            if isinstance(term, URIRef) and text.startswith(prefix):
                local = text[len(prefix) :]
                out.append(URIRef("x:tree" if local == "" else f"x:{local}"))
            else:
                out.append(term)
        return tuple(out)

    arm1 = {t for t in graph if str(t[0]).startswith(prefix_a)}
    arm2 = {t for t in graph if str(t[0]).startswith(prefix_b)}
    assert arm1 and arm2

    assert not {str(s) for s, _, _ in arm1} & {str(s) for s, _, _ in arm2}
    assert {normalise(t, prefix_a) for t in arm1} == {normalise(t, prefix_b) for t in arm2}

    # The template's own namespace is not emitted: only instances of it are.
    assert not [t for t in graph if str(t[0]).startswith("https://example.test/device/")]


def test_instanced_frame_ref_resolves_to_its_own_instance(tmp_path):
    """'<arm1.arm_base.arm_root>' and '<arm2...>' are the same template frame.

    Only the reference records which instance it was written through, so this is what
    keeps two attachments of one device model apart.
    """
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "dual.scenex"
    path.write_text(_dual_arm_scene())

    model = scenex_metamodel().model_from_file(path)
    graph = create_scenex_model_graph(model)
    joints = {j.name: j for j in model.scene_insts[0].kgraph.trees[0].joints_spec.joints}

    for arm in ("arm1", "arm2"):
        attached = set(
            graph.objects(joints[f"{arm}_on_table"].uri, URI_KC_PRED_BETWEEN_ATTACHMENTS)
        )
        assert URIRef(f"https://example.test/lab/{arm}/arm_base/arm_root") in attached
        assert URIRef("https://example.test/lab/world_tree/table_body/table_root") in attached


def test_composing_a_template_directly_is_rejected(tmp_path):
    """A template names no device, so a tree composes an instance of it, never it."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "direct.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree (ns=lab) world_tree {
    tree <arm_tree>
    body table_body { frame table_root { } }
    joints { }
}
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) g { tree <world_tree> }
}
"""
    )
    with pytest.raises(TextXSemanticError, match='"arm_tree" of class "KinematicTreeModel"'):
        scenex_metamodel().model_from_file(path)


def test_instancing_a_non_template_is_rejected(tmp_path):
    """A tree already names a particular device: only a template may be instanced."""
    write_example_scene(tmp_path)
    (tmp_path / "device.ktree").write_text(
        """ns dev = "https://example.test/dev/"
ktree (ns=dev) arm_tree {
    body arm_base { frame arm_root { } }
    joints { }
}
"""
    )
    path = tmp_path / "shadow.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm of <arm_tree>
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) g { tree <arm> }
}
"""
    )
    with pytest.raises(TextXSemanticError, match='"arm_tree" of class "KinematicTreeTemplate"'):
        scenex_metamodel().model_from_file(path)


def test_instancing_a_template_inline_is_rejected(tmp_path):
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "inline.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
scene inst (ns=lab) sx {
    scene: <s>
    agn <agns.robot> {
        model robot as urdf { sys path = "robot.urdf" }
        ktree inst (ns=lab) arm of <arm_tree>
    }
}
"""
    )
    with pytest.raises(TextXSemanticError, match="must be declared at model level"):
        scenex_metamodel().model_from_file(path)


def test_a_template_composing_a_tree_is_rejected(tmp_path):
    """A template is copied whole, so it composes nothing: there is no syntax for it."""
    write_example_scene(tmp_path)
    (tmp_path / "device.ktree").write_text(
        """ktree grip_tree {
    body g_base { frame g_root { } }
    joints { }
}
ktree arm_tree {
    tree <grip_tree>
    body arm_base { frame arm_root { } }
    joints { }
}
"""
    )
    path = tmp_path / "nested.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm of <arm_tree>
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) g { tree <arm> }
}
"""
    )
    with pytest.raises(TextXSyntaxError):
        scenex_metamodel().model_from_file(path)


def test_two_devices_sharing_a_name_is_rejected(tmp_path):
    """A template mints no IRI until bound, so instances must still be checked for collisions.

    Two devices may share a namespace -- the instance name scopes each copy. Sharing the
    name is what collides, and a template's elements are only reachable through it.
    """
    write_example_scene(tmp_path)
    (tmp_path / "device.ktree").write_text(
        """ktree arm_tree {
    body arm_base { frame arm_root { } }
    joints { }
}
ktree grip_tree {
    body arm_base { frame arm_root { } }
    joints { }
}
"""
    )
    path = tmp_path / "collide.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ns shared = "https://example.test/lab/shared/"
ktree inst (ns=shared) arm of <arm_tree>
ktree inst (ns=shared) arm of <grip_tree>
ktree (ns=lab) world_tree {
    body table_body { frame table_root { } }
    joints { }
}
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) world_tree_graph {
        tree <world_tree>
    }
}
"""
    )
    with pytest.raises(TextXSemanticError, match="must be unique per IRI"):
        scenex_metamodel().model_from_file(path)


def test_template_referencing_outside_itself_is_rejected(tmp_path):
    """Instancing copies the tree, so a reference leaving it has nothing to copy to."""
    write_example_scene(tmp_path)
    (tmp_path / "ext.ktree").write_text(
        """ns ext = "https://example.test/ext/"
ktree (ns=ext) ext_tree {
    body ext_body { frame ext_frame { } }
    joints { }
}
"""
    )
    (tmp_path / "device.ktree").write_text(
        """import "ext.ktree"
ktree arm_tree {
    body arm_base {
        frame arm_root { }
        frame arm_tip {
            pose p {
                wrt: <ext_tree.ext_body.ext_frame>
                xyz: (0, 0, 1) m
                orientation: euler { axes: xyz extrinsic angles: (0, 0, 0) unit: deg }
            }
        }
    }
    joints { }
}
"""
    )
    path = tmp_path / "leaky.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm of <arm_tree>
ktree (ns=lab) world_tree {
    tree <arm>
    body table_body { frame table_root { } }
    joints { }
}
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) world_tree_graph {
        tree <world_tree>
    }
}
"""
    )
    with pytest.raises(TextXSemanticError, match="must be self-contained"):
        scenex_metamodel().model_from_file(path)


def test_inertia_about_frame_follows_its_instance(tmp_path):
    """'about' names the same instanced frame as 'as-seen-by', not the template's."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "inertia.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm of <arm_tree>
ktree (ns=lab) world_tree {
    tree <arm>
    body table_body {
        frame table_root { }
        inertia {
            frame: arm.arm_base.arm_root
            mass: 1.0 kg
            inertia-matrix: ((1.0,0.0,0.0),(0.0,1.0,0.0),(0.0,0.0,1.0)) kg*m^2
        }
    }
    joints {
        fixed arm_on_table {
            parent: <table_body.table_root>
            child: <arm.arm_base.arm_root>
        }
    }
}
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) world_tree_graph {
        tree <world_tree>
    }
}
"""
    )
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(path))
    assert (
        URIRef("https://example.test/lab/world_tree/table_body-inertia"),
        URI_DYN_PRED_ABOUT,
        URIRef("https://example.test/lab/arm/arm_base/arm_root-origin"),
    ) in graph


def test_lab_scenex_composes_imported_device_trees():
    """lab.scenex mixes inline trees with imported ones and owns the assembly joints."""
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    trees = {tree.name: tree for tree in model.scene_insts[0].kgraph.trees[0].trees}

    scene_ns = "https://secorolab.github.io/models/acceptance-criteria/bdd/scenes/secorolab-mjc/"
    robots_ns = f"{scene_ns}robots/"
    trees.update({tree.name: tree for tree in trees["arm1_gripper"].trees})
    for instance in ("arm1", "arm2", "gripper"):
        assert str(trees[instance].uri) == f"{robots_ns}{instance}"
    assert str(trees["panda_tree"].uri).startswith(scene_ns)

    graph = create_scenex_model_graph(model)
    for tree in trees.values():
        assert (tree.uri, RDF.type, URI_GEOM_TYPE_KTREE) in graph

    joints = {j.name: j for j in model.scene_insts[0].kgraph.trees[0].joints_spec.joints}
    for arm in ("arm1", "arm2"):
        assert (
            joints[f"{arm}_on_table"].uri,
            URI_KC_PRED_BETWEEN_ATTACHMENTS,
            URIRef(f"{robots_ns}{arm}/base_link/base_link_origin"),
        ) in graph

    # The gripper hangs off arm1 only: one device model, but the arms stay apart.
    mount = {j.name: j for j in trees["arm1_gripper"].joints_spec.joints}["g_mount_on_pinch_site"]
    assert (
        mount.uri,
        URI_KC_PRED_BETWEEN_ATTACHMENTS,
        URIRef(f"{robots_ns}arm1/bracelet_link/pinch_site"),
    ) in graph


def test_agent_kinematics_may_be_defined_or_named():
    """An agent defines its kinematics inline, or names an instance it cannot define."""
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    scene_inst = model.scene_insts[0]
    agents = {agent.agn.name: agent for agent in scene_inst.modelled_agns}
    graph = create_scenex_model_graph(model)

    for name in ("panda", "ur10", "arm1_gripper", "kinova2"):
        assert agents[name].ktree is not None
        assert (agents[name].ktree.uri, RDF.type, URI_GEOM_TYPE_KTREE) in graph

    assert agents["panda"].ktree is agents["panda"].ktree_inline  # defined here
    assert agents["arm1_gripper"].ktree_inline is None  # named elsewhere

    # One device model, two robots: the arms are separate kinematics under separate agents.
    assert agents["arm1_gripper"].ktree is not agents["kinova2"].ktree
    assert agents["arm1_gripper"].ktree.uri != agents["kinova2"].ktree.uri

    # A named tree is the very tree the scene composes, not a second copy of it.
    assembled = {tree.name: tree for tree in scene_inst.kgraph.trees[0].trees}["arm1_gripper"]
    assert agents["arm1_gripper"].ktree is assembled


def test_assembled_agent_carries_a_model_file_per_device():
    """An agent spanning two devices names a file each; a bare device needs only one.

    Both shapes are agents: what differs is whether the kinematics are one device or
    several bolted together.
    """
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    agents = {agent.agn.name: agent for agent in model.scene_insts[0].modelled_agns}
    graph = create_scenex_model_graph(model)
    robots_ns = (
        "https://secorolab.github.io/models/acceptance-criteria/bdd/scenes/secorolab-mjc/robots/"
    )

    # Assembled: one file per device, each bound to the sub-tree it describes.
    assembled = {m.name: m for m in agents["arm1_gripper"].models}
    for model_name, device in (("arm1-mjc", "arm1"), ("gripper-mjc", "gripper")):
        tree_uri = URIRef(f"{robots_ns}{device}")
        assert assembled[model_name].tree.uri == tree_uri
        assert (assembled[model_name].uri, URI_EXEC_PRED_HAS_KTREE, tree_uri) in graph

    # Bare: one file describes the whole device, so it has nothing to disambiguate.
    (bare,) = agents["kinova2"].models
    assert bare.tree is None
    assert not list(graph.objects(bare.uri, URI_EXEC_PRED_HAS_KTREE))


def test_arm_and_gripper_compose_into_one_chain():
    """Two devices bolted together are one chain: the arm's base to the gripper's TCP."""
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    graph = create_scenex_model_graph(model)
    scene_ns = "https://secorolab.github.io/models/acceptance-criteria/bdd/scenes/secorolab-mjc/"
    assembled = {t.name: t for t in model.scene_insts[0].kgraph.trees[0].trees}["arm1_gripper"]

    assert (assembled.uri, RDF.type, URI_KC_TYPE_KC) in graph
    assert (assembled.uri, RDF.type, URI_KC_TYPE_SERIAL) in graph
    assert (
        assembled.uri,
        NS_MM_KC_EXT["root"],
        URIRef(f"{scene_ns}robots/arm1/base_link/base_link_origin"),
    ) in graph
    assert (
        assembled.uri,
        NS_MM_KC_EXT["tip"],
        URIRef(f"{scene_ns}robots/gripper/g_base/g_pinch"),
    ) in graph


def _agent_scene(agn_body: str) -> str:
    return f"""import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm1 of <arm_tree>
ktree inst (ns=lab) arm2 of <arm_tree>
scene inst (ns=lab) sx {{
    scene: <s>
    agn <agns.robot> {{
{agn_body}
    }}
}}
"""


def test_model_for_a_tree_the_agent_lacks_is_rejected(tmp_path):
    """'for' must name kinematics the agent has, or nothing says what the file describes."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "wrong_tree.scenex"
    path.write_text(
        _agent_scene(
            """        model a-mjc as mjcf { sys path = "a.xml" } for <arm1>
        model b-mjc as mjcf { sys path = "b.xml" } for <arm2>
        ktree <arm1>"""
        )
    )
    with pytest.raises(TextXSemanticError, match="but agent 'robot' has kinematics 'arm1'"):
        scenex_metamodel().model_from_file(path)


def test_model_for_a_tree_when_agent_has_no_kinematics_is_rejected(tmp_path):
    """An agent without kinematics has nothing for a model to describe."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "no_ktree.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm1 of <arm_tree>
scene inst (ns=lab) sx {
    scene: <s>
    agn <agns.robot> {
        model a-mjc as mjcf { sys path = "a.xml" } for <arm1>
    }
}
"""
    )
    with pytest.raises(TextXSemanticError, match="has no kinematics"):
        scenex_metamodel().model_from_file(path)


def test_two_models_describing_one_tree_is_rejected(tmp_path):
    """One device, one file: two files for a tree leave it undecided which drives it."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "dup_for.scenex"
    path.write_text(
        _agent_scene(
            """        model a-mjc as mjcf { sys path = "a.xml" } for <arm1>
        model b-mjc as mjcf { sys path = "b.xml" } for <arm1>
        ktree <arm1>"""
        )
    )
    with pytest.raises(TextXSemanticError, match="both describe tree 'arm1'"):
        scenex_metamodel().model_from_file(path)


def test_cyclic_tree_composition_names_the_cycle(tmp_path):
    """A tree composing itself is rejected where it is written, and the chain is named."""
    write_example_scene(tmp_path)
    path = tmp_path / "cycle.scenex"
    path.write_text(
        """import "example.scene"
ns lab = "https://example.test/lab/"
ktree (ns=lab) a { tree <b> body ab { frame af { } } joints { } }
ktree (ns=lab) b { tree <a> body bb { frame bf { } } joints { } }
scene inst (ns=lab) sx {
    scene: <s>
    agn <agns.robot> {
        model a-mjc as mjcf { sys path = "a.xml" } for <a>
        ktree <a>
    }
}
"""
    )
    with pytest.raises(TextXSemanticError, match=r"'a' composes itself: a -> b -> a"):
        scenex_metamodel().model_from_file(path)


def test_assembled_agent_model_without_for_is_rejected(tmp_path):
    """Several models and no 'for' leaves the files an unordered pile."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "no_for.scenex"
    path.write_text(
        _agent_scene(
            """        model a-mjc as mjcf { sys path = "a.xml" } for <arm1>
        model b-mjc as mjcf { sys path = "b.xml" }
        ktree <arm1>"""
        )
    )
    with pytest.raises(TextXSemanticError, match="each must say which kinematics"):
        scenex_metamodel().model_from_file(path)


def test_for_on_a_non_agent_model_is_rejected(tmp_path):
    """An object has no kinematics, so a model of one has nothing to describe."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "obj_for.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm1 of <arm_tree>
scene inst (ns=lab) sx {
    scene: <s>
    obj <objs.cup> {
        model cup_model as urdf { sys path = "cup.urdf" } for <arm1>
    }
}
"""
    )
    with pytest.raises(TextXSemanticError, match="only an agent has kinematics"):
        scenex_metamodel().model_from_file(path)


def test_lab_scenex_generated_geometry_validates_against_shacl():
    install_resolver()
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")

    graph = create_scenex_model_graph(model)
    check_shacl_constraints(
        graph=graph,
        shacl_dict={
            URL_MM_GEOM_SHACL_EXTS: "ttl",
            URL_MM_GEOM_SHACL_REL: "ttl",
            # Skipping because of DirectionCosine rule that would fail, potentially
            # because of https://github.com/RDFLib/rdflib/issues/2009
            # URL_MM_GEOM_SHACL_COORD: "ttl",
            f"{URL_SECORO_MM}/robot/actuation.shacl.ttl": "ttl",
            f"{URL_SECORO_MM}/robot/sensors.shacl.ttl": "ttl",
            f"{URL_SECORO_MM}/kinematic-chain/structural-entities-extension.shacl.ttl": "ttl",
        },
        quiet=False,
    )


def test_lab_scenex_generates_panda_joint_actuation():
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    panda_tree = next(
        tree for tree in model.scene_insts[0].kgraph.trees[0].trees if tree.name == "panda_tree"
    )
    joint = next(joint for joint in panda_tree.joints_spec.joints if joint.name == "panda_joint1")
    graph = create_scenex_model_graph(model)

    assert (joint.actuation_uri, RDF.type, URI_ACT_TYPE_ACTUATION) in graph
    assert (joint.actuation_uri, URI_ACT_PRED_JOINT, joint.uri) in graph
    for interface in joint.actuation.cmd_interfaces:
        assert (
            joint.actuation_uri,
            URI_ACT_PRED_COMMAND_INTERFACE,
            ACTUATION_INTERFACE_TYPES[interface],
        ) in graph
    for interface in joint.actuation.state_interfaces:
        assert (
            joint.actuation_uri,
            URI_ACT_PRED_STATE_INTERFACE,
            ACTUATION_INTERFACE_TYPES[interface],
        ) in graph

    mimic_joint = next(
        joint for joint in panda_tree.joints_spec.joints if joint.name == "panda_joint2"
    )
    assert (mimic_joint.mimic_uri, RDF.type, URI_KC_EXT_TYPE_JOINT_COUPLING) in graph
    assert (mimic_joint.mimic_uri, RDF.type, URI_KC_EXT_TYPE_SCLERONOMIC) in graph
    assert (
        mimic_joint.mimic_uri,
        URI_KC_EXT_PRED_DEPENDENT_JOINT,
        mimic_joint.uri,
    ) in graph
    assert (
        mimic_joint.mimic_uri,
        URI_KC_EXT_PRED_INDEPENDENT_JOINT,
        mimic_joint.mimic.joint.uri,
    ) in graph
    assert (
        mimic_joint.mimic_uri,
        URI_KC_EXT_PRED_OFFSET,
        mimic_joint.mimic_offset_uri,
    ) in graph
    assert (mimic_joint.mimic_offset_uri, RDF.type, URI_QUDT_TYPE_QUANTITY) in graph
    assert (
        mimic_joint.mimic_offset_uri,
        URI_QUDT_PRED_QUANTITY_KIND,
        URI_QUDT_QK_ANGLE,
    ) in graph


def test_scenex_embedded_kinematic_tree_rejects_bad_frame_ref(tmp_path):
    write_example_scene(tmp_path)
    model_path = tmp_path / "bad_robot.scenex"
    model_path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
ktree (ns=n) world_tree {
    body world_body { frame world { } }
    joints {
        fixed bad_mount {
            parent: <missing>
            child: <world_body.world>
        }
    }
}
scene inst (ns=n) sx {
    scene: <s>
    kgraph (ns=n) world_tree_graph {
        tree <world_tree>
    }
    agn <agns.robot> {
        model robot_mjcf as mjcf { sys path = "robot.xml" }
    }
}
"""
    )

    with pytest.raises(Exception, match="missing"):
        scenex_metamodel().model_from_file(model_path)


def test_direction_cosine_orientation_rejects_reflection():
    with pytest.raises(ValueError, match=r"determinant \+1"):
        DirectionCosineOrientationSpec(
            parent=None,
            x_axis=FloatVector(None, [1.0, 0.0, 0.0]),
            y_axis=FloatVector(None, [0.0, 1.0, 0.0]),
            z_axis=FloatVector(None, [0.0, 0.0, -1.0]),
        )


def _tree_scene(bodies: str, joints: str) -> str:
    return f"""import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {{
    scene: <s>
    agn <agns.robot> {{
        model m as urdf {{ sys path = "r.urdf" }}
        ktree (ns=n) t {{
{bodies}
            joints {{
{joints}
            }}
        }}
    }}
}}
"""


TWO_BODIES = """            body b1 { frame f1 { } frame f1b { } }
            body b2 { frame f2 { } frame f2b { } }"""


def test_joints_closing_a_loop_are_rejected(tmp_path):
    """A tree is a tree: bodies joined back around to each other close a loop."""
    write_example_scene(tmp_path)
    path = tmp_path / "loop.scenex"
    path.write_text(
        _tree_scene(
            TWO_BODIES,
            """                fixed j1 { parent: <b1.f1> child: <b2.f2> }
                fixed j2 { parent: <b2.f2b> child: <b1.f1b> }""",
        )
    )
    with pytest.raises(TextXSemanticError, match="close a loop"):
        scenex_metamodel().model_from_file(path)


def test_body_attached_by_two_joints_is_rejected(tmp_path):
    """Two joints onto one body close a loop just as surely, by merging two chains."""
    write_example_scene(tmp_path)
    path = tmp_path / "merge.scenex"
    path.write_text(
        _tree_scene(
            TWO_BODIES
            + """
            body b3 { frame f3 { } frame f3b { } }""",
            """                fixed j1 { parent: <b1.f1> child: <b3.f3> }
                fixed j2 { parent: <b2.f2> child: <b3.f3b> }""",
        )
    )
    with pytest.raises(TextXSemanticError, match="'b3' is attached by both joint"):
        scenex_metamodel().model_from_file(path)


def test_serial_chain_over_unjoined_bodies_is_rejected(tmp_path):
    """A graph may hold unjoined bodies, but a chain over them is still no chain."""
    write_example_scene(tmp_path)
    path = tmp_path / "phantom.scenex"
    path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    kgraph (ns=n) g {
        body b1 { frame f1 { } }
        body b2 { frame f2 { } }
        joints {
            serial { root: <b1.f1> tip: <b2.f2> }
        }
    }
}
"""
    )
    with pytest.raises(TextXSemanticError, match="is not below root") as fits:
        scenex_metamodel().model_from_file(path)
    assert "tip 'f2'" in str(fits.value)


def test_serial_chain_that_branches_is_rejected(tmp_path):
    """Siblings are not a chain: the path from one to the other runs up and back down."""
    write_example_scene(tmp_path)
    path = tmp_path / "branch.scenex"
    path.write_text(
        _tree_scene(
            """            body b1 { frame f1 { } frame f1b { } }
            body b2 { frame f2 { } }
            body b3 { frame f3 { } }""",
            """                fixed j1 { parent: <b1.f1> child: <b2.f2> }
                fixed j2 { parent: <b1.f1b> child: <b3.f3> }
                serial { root: <b2.f2> tip: <b3.f3> }""",
        )
    )
    with pytest.raises(TextXSemanticError, match="is not below root"):
        scenex_metamodel().model_from_file(path)


def test_serial_chain_across_two_devices_is_accepted():
    """arm1_gripper's chain spans two instanced devices, and is a chain."""
    model = scenex_metamodel().model_from_file(MODELS_DIR / "lab.scenex")
    assembled = {t.name: t for t in model.scene_insts[0].kgraph.trees[0].trees}["arm1_gripper"]
    # a branch off the path (the gripper's driver) does not stop it being serial
    assert len(assembled.all_joints) == 10


def test_kgraph_holds_a_body_attached_to_nothing(tmp_path):
    """A graph promises no shape, so a body hanging from nothing is at home in one."""
    write_example_scene(tmp_path)
    write_device_ktree(tmp_path)
    path = tmp_path / "free.scenex"
    path.write_text(
        """import "example.scene"
import "device.ktree"
ns lab = "https://example.test/lab/"
ktree inst (ns=lab) arm1 of <arm_tree>
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) lab_graph {
        tree <arm1>
        body cup_body { frame cup_root { } }
    }
}
"""
    )
    graph = create_scenex_model_graph(scenex_metamodel().model_from_file(path))
    # the graph is a graph, and the tree it holds is still a tree
    assert (URIRef("https://example.test/lab/lab_graph"), RDF.type, URI_GEOM_TYPE_KGRAPH) in graph
    assert (URIRef("https://example.test/lab/arm1"), RDF.type, URI_GEOM_TYPE_KTREE) in graph
    # the free body is emitted, scoped by the graph that holds it
    assert (
        URIRef("https://example.test/lab/lab_graph/cup_body"),
        RDF.type,
        URI_GEOM_TYPE_RIGID_BODY,
    ) in graph


def test_kgraph_allows_merges_and_cycles(tmp_path):
    """A graph has no tree-only parent or acyclicity constraint."""
    write_example_scene(tmp_path)
    path = tmp_path / "graph.scenex"
    path.write_text(
        """import "example.scene"
ns n = "https://example.test/"
scene inst (ns=n) sx {
    scene: <s>
    kgraph (ns=n) graph {
        body a { frame a_to_c { } frame from_c { } }
        body b { frame b_to_c { } }
        body c { frame from_a { } frame to_a { } }
        joints {
            fixed a_to_c { parent: <a.a_to_c> child: <c.from_a> }
            fixed b_to_c { parent: <b.b_to_c> child: <c.to_a> }
            fixed c_to_a { parent: <c.to_a> child: <a.from_c> }
            serial { root: <b.b_to_c> tip: <a.from_c> }
        }
    }
}
"""
    )

    graph = scenex_metamodel().model_from_file(path).scene_insts[0].kgraph
    assert {joint.name for joint in graph.joints_spec.joints} == {"a_to_c", "b_to_c", "c_to_a"}


def test_a_free_body_in_a_tree_is_rejected(tmp_path):
    """The same body in a tree is a second root, and a tree has one."""
    write_example_scene(tmp_path)
    path = tmp_path / "rooted.scenex"
    path.write_text(
        """import "example.scene"
ns lab = "https://example.test/lab/"
ktree (ns=lab) t {
    body b1 { frame f1 { } }
    body floating { frame f2 { } }
    joints { }
}
scene inst (ns=lab) sx {
    scene: <s>
    kgraph (ns=lab) g { tree <t> }
}
"""
    )
    with pytest.raises(TextXSemanticError, match="but a tree has one root"):
        scenex_metamodel().model_from_file(path)
