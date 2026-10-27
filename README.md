# scene-dsl

textX languages for abstract scene models (`scene`) and executable scene instances (`scenex`).

## Languages

- `scene` (`*.scene`) describes abstract robotic scene content: objects,
  workspaces, agents, workspace compositions, and scene models.
- `scenex` (`*.scenex`) describes executable scene instances: model files,
  geometry, frames, poses, kinematics, attachments, bodies, and inertial mass.
- `.ktree` files hold reusable device trees. They are parsed with the `scenex`
  metamodel, and imported like a `.scene` file.

`scenex` references the registered `scene` language, so executable models can
import a `.scene` file and link to `SceneModel` declarations.

A tree without a namespace is a template: it describes a device without being
any particular one, so every use is an instance that copies it under its own
name. An element's IRI is its namespace plus the path down to it, so two
instances of one device may share a namespace, and a sensor is scoped by the
agent carrying it.

## Shipped Robots

The package ships device trees for common robots in `scene_dsl/robots/`:

| File | Device |
|---|---|
| `kinova_gen3_7dof.ktree` | Kinova Gen3, 7 DoF arm |
| `robotiq_2f85.ktree` | Robotiq 2F-85 gripper |
| `ur10e.ktree` | Universal Robots UR10e arm |
| `eddie_base.ktree` | Eddie mobile base |

Import one with the `scene_dsl:` prefix, which names the package it comes from:

```text
import "scene_dsl:robots/kinova_gen3_7dof.ktree"
```

`<package>:<path>` resolves to `<path>` inside any installed top-level package,
through `importlib.resources`, so another package can ship its own trees the
same way. Every other import is relative to the importing file.

## Installation

`scene-dsl` is a Python package installable with `pip`. It depends on
[`rdf-utils`](https://github.com/minhnh/rdf-utils), `textx`, `rdflib`, and
`numpy`.

For local development with sibling checkouts:

```bash
pip install "../rdf-utils[all]"
pip install -e .
```

## RDF Generation

Example models live under `examples/models`.

Print abstract scene RDF as Turtle:

```bash
textx generate examples/models/lab.scene --target console --format ttl
```

Write abstract scene RDF to `examples/generated/lab.scene.ttl`:

```bash
textx generate examples/models/lab.scene --target graph -o examples/generated --format ttl
```

Print executable scenex RDF as Turtle:

```bash
textx generate examples/models/lab.scenex --target console --format ttl
```

Write executable scenex RDF to `examples/generated/lab.scenex.ttl`:

```bash
textx generate examples/models/lab.scenex --target graph -o examples/generated --format ttl
```

The RDF generators accept `format` (`json-ld`, `ttl`, or `xml`) and `filename`.
For JSON-LD, pass `--nocompact` to skip compacting IRIs.

## Body shapes

Declare one or more shapes inside a body in `.scenex` or `.ktree`:

```text
body ball {
    frame center { }
    shape sphere surface { frame: <center> scale: 0.10 }
}
```

Each shape references a frame on its body; pose that frame with `wrt` to place
the shape. Primitive local frames sit at the midpoint of their axis-aligned
bounds. The unit bases have full X/Y/Z extents of 1 m: a rectangular prism has
1 m sides; a sphere has 0.5 m radius; a cylinder and right circular cone have
0.5 m radius and 1 m height along local Z. The cone base is at local Z = -0.5 m
and its apex at +0.5 m. The regular tetrahedron and right-isosceles triangular
prism each have 1 m bounds along all axes. Their fixed `astro:length` denotes
that unit extent, not an edge length. The triangular prism's bounds midpoint
is not its volume centroid. The body frame can be elsewhere. Cube, sphere, and
cylinder marker origins are centered in
[ROS 2 RViz](https://github.com/ros2/ros2_documentation/blob/rolling/source/Developer-Tools/Visualization/RViz/Marker-Display-types/Marker-Display-types.rst).
The cone, tetrahedron, and triangular prism origins here follow the stated
bounds-midpoint convention.

Author the cone as `shape right-circular-cone`; RDF gives its unit base type
`shapes:RightCircularCone`, a subclass of `astro:circularCone`. Equal X/Y scale
keeps the rendered cone circular; different X/Y factors make an elliptical cone.

Give each primitive a required positive, dimensionless scale. A scalar
`scale: 0.05` keeps proportions; `scale: (0.2, 0.3, 0.01)` sets full X/Y/Z
extents in metres. Different sphere factors make an ellipsoid; different
cylinder X/Y factors make an elliptical cylinder; the same factors work for a
cone, tetrahedron, or triangular prism. STL meshes use `shape mesh` with a
`.stl` `path` and may use three component scale. The mesh keeps its authored
origin; STL coordinates are interpreted as metres. Any shape may have an
optional RGBA `color` in `[0, 1]`. Shapes do not supply mass or inertia.
The primitive type describes the unit-base geometry before scaling. Scale acts
before the attached frame pose. To migrate `shape cube`, use
`shape rectangular-prism` and replace `side` with a scalar `scale` equal to the
side length in metres. For older sphere and cylinder models, double each
radius in metres to obtain X/Y scale, and use the cylinder height in metres
for Z scale. Shear and reflection are not supported.

Primitive and mesh scale also accept `sample <distribution>`. Use a
one-component uniform distribution for proportional primitive scale, or three
components for independent X/Y/Z scale; mesh scale takes three components.
Bounds must be finite and positive. A value is drawn once per run and reused
wherever referenced. The robot example's optional
`[sampling].seed` makes runs repeatable, though the viewer and generated run
may draw different values for the same seed.

From this repository directory, view the [compound example](examples/models/compound.scenex)
for fixed and sampled shapes, local poses, and STL scale. With MuJoCo assets
installed, the [robot example](examples/models/lab-sampled.scenex) shows the
new primitives on a body alongside a Kinova arm:

```bash
motion-spec view examples/models/compound.scenex
motion-spec view examples/models/lab-sampled.scenex --config examples/models/lab-sampled.toml
motion-spec run examples/models/lab-sampled.robmot --headless --steps 5
```

Use `--headless` with `view` to check compilation without opening a window.
Generated runs use these shapes for visual and collision geometry. RDF export
links shapes to frames with `shapes:attached-to-frame` and dimensions with
`shapes:has-dimension`. Each primitive carries fixed unit-base dimensions.
A scaled shape has type `shapes:ScaledShape`. A fixed scalar uses
`shapes:uniform-scale`; fixed XYZ factors use `shapes:scale-x/y/z`; sampled
factors use `distrib:from-distribution` on the shape.
For an STL mesh, RDF links the shape to a `prov:Entity` for its source file.
That entity has a `prov-ext:RelativeLocation`: its `rel-path` is relative to
the authored `.scenex` directory. An absolute DSL path is represented by its
parent directory and filename. Generated runs copy the STL beside the
controller, so they no longer depend on the original checkout.
The companion metamodels repository validates
these nodes. MuJoCo keeps prisms as boxes and isotropically scaled spheres and
cylinders as native primitives. Non-uniform spheres and cylinders use triangle
meshes, so curved surfaces are approximated.

## Drawing the Kinematics

Draw the scene's bodies and the joints between them, with each device a cluster
and any declared serial chain picked out. Needs [graphviz](https://graphviz.org)
for anything but `dot`:

```bash
textx generate examples/models/lab.scenex --target dot -o examples/generated --format png
```

`format` takes `dot` (the default), `png`, `svg`, or `pdf`. To print the graph
instead of writing it:

```bash
textx generate examples/models/lab.scenex --target dot-console
```

## Development

```bash
pytest -q tests
ruff check src tests
```
