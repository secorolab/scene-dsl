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
    shape UnitSphere surface { frame: <center> scale: 0.10 }
}
```

Each shape references a frame on its body; pose that frame with `wrt` to place
the shape. The only primitives are `UnitCube`, `UnitCylinder`, and `UnitSphere`.
Their attached frames sit at their centroids. The cube has 1 m sides; the
cylinder has radius 1 m and height 1 m along local Z; the sphere has radius
1 m. The body frame can be elsewhere.

Give each primitive a required positive, dimensionless scale. A scalar
`scale: 0.05` keeps proportions. For `UnitCube`, the three factors set full
X/Y/Z extents in metres. For `UnitSphere`, they set X/Y/Z radii; for
`UnitCylinder`, X/Y radii and full Z height. Independent cube factors make a
rectangular prism; independent sphere factors make an ellipsoid; different
cylinder X/Y factors make an elliptical cylinder. The primitive type identifies
the unit base before scaling. Scale acts before the frame pose. Shear and
reflection are not supported.

File-based shapes use `shape file` with a `.stl` `path` and optional scalar or
three-component scale. The STL mesh keeps its authored origin; STL coordinates
are interpreted as metres. Any shape may have an optional RGBA `color` in
`[0, 1]`. Shapes do not supply mass or inertia.

Primitive and file shape scale also accept `sample <distribution>`. Use a
one-component uniform distribution for proportional scale, or three components
for independent X/Y/Z scale. Both work for file shapes.
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
links shapes to frames with `shapes:attached-to-frame`. Each primitive has its
corresponding `shapes:UnitCube`, `shapes:UnitCylinder`, or `shapes:UnitSphere`
type; its base dimensions are fixed by that type.
A scaled shape has type `shapes:ScaledShape`. Its `shapes:scale-x`,
`shapes:scale-y`, and `shapes:scale-z` links point to dimensionless
`qudt:Quantity` nodes. Fixed factors have `qudt:value`; sampled factors are
also `distrib:SampledQuantity` nodes linked to scalar distributions. A scalar
DSL scale shares one factor node across all three axes. A three-component
DSL distribution exports three scalar distributions, one per axis.
For an STL mesh, RDF links the shape to a `prov:Entity` for its source file.
That entity has a `prov-ext:RelativeLocation`: its `rel-path` is relative to
the authored `.scenex` directory. An absolute DSL path is represented by its
parent directory and filename. Generated runs copy the STL beside the
controller, so they no longer depend on the original checkout.
The companion metamodels repository validates
these nodes. MuJoCo keeps cubes as boxes and isotropically scaled spheres and
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
