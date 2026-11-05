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
textx generate examples/models/kinova_with_objects.scene --target console --format ttl
```

Write abstract scene RDF to `examples/generated/kinova_with_objects.scene.ttl`:

```bash
textx generate examples/models/kinova_with_objects.scene --target graph -o examples/generated --format ttl
```

Print executable scenex RDF as Turtle:

```bash
textx generate examples/models/kinova_with_objects.scenex --target console --format ttl
```

Write executable scenex RDF to `examples/generated/kinova_with_objects.scenex.ttl`:

```bash
textx generate examples/models/kinova_with_objects.scenex --target graph -o examples/generated --format ttl
```

The RDF generators accept `format` (`json-ld`, `ttl`, or `xml`) and `filename`.
For JSON-LD, pass `--nocompact` to skip compacting IRIs.

## Scene shapes

Declare one or more shapes inside a body in `.scenex` or `.ktree`:

```text
body ball {
    frame center { }
    shape UnitSphere surface { frame: <center> unit: m scale: 0.10 }
}
```

The body's inertia is optional, including when its shapes are used only for
collision. Only bodies hold shapes: for static geometry, put the shape on the
body the scene stands on, e.g. the one holding its `kgraph` anchor:

```text
kgraph (ns=demo) world {
    anchor: <ground.origin>
    body ground {
        frame origin { }
        shape UnitSphere marker { frame: <origin> unit: m scale: 0.10 }
    }
}
```

Each shape references a frame on its own body. Pose the frame with `wrt` to
place the shape. The only primitives are `UnitCube`,
`UnitCylinder`, and `UnitSphere`.
Their attached frames sit at their centroids. The cube has sides of length 1;
the cylinder has radius 1 and height 1 along local Z; the sphere has radius 1.
These dimensions use each shape's required `unit: m`, `unit: cm`, or `unit: mm`.
An owning body's frame can be elsewhere.

Scale is optional for primitives; omitting it keeps their unit dimensions.
A scalar `scale: 0.05` keeps proportions. For `UnitCube`, the three factors set full
X/Y/Z extents in the declared unit. For `UnitSphere`, they set X/Y/Z radii; for
`UnitCylinder`, X/Y radii and full Z height. Independent cube factors make a
rectangular prism; independent sphere factors make an ellipsoid; different
cylinder X/Y factors make an elliptical cylinder. The primitive type identifies
the unit base before scaling. Scale acts before the frame pose. Shear and
reflection are not supported.

File-based shapes use `shape file` with a `.stl` file and optional scalar or
three-component scale. Name the file as an element model names its file: on
this system, absolute or relative to the `.scenex` declaring the shape, or in a
ROS package:

```text
shape file marker { frame: <f> sys path = "../assets/tetra.stl" unit: cm }
shape file base { frame: <g> ros pkg = "kortex_description", path = "meshes/base_link.stl" unit: m }
```

Declare the unit of the STL coordinates with `unit:`.
The STL mesh keeps its authored origin. Shapes do not supply mass or inertia.

Any shape may have an optional `color`, in one of the schemes of the
[color metamodel](https://github.com/secorolab/metamodels/blob/main/color.shacl.ttl):

| Form | Scheme | Channels |
| --- | --- | --- |
| `rgb(230, 51, 51)` | RGB | integers in `[0, 255]` |
| `rgba(230, 51, 51, 255)` | RGBA | integers in `[0, 255]`, alpha included |
| `#E63333` / `#E63333FF` | RGB / RGBA | the same channels in hex |
| `hsv(210, 0.6, 0.9)` | HSV | hue in degrees `[0, 360)`, saturation and value in `[0, 1]` |

RGB and RGBA colours are exported as hex, which carries 0-255 channels exactly; HSV
is exported as written.

From this repository directory, view the
[Kinova example](examples/models/kinova_with_objects.scenex) to see primitive
and STL shapes on bodies alongside a Kinova arm. MuJoCo assets must be installed:

```bash
motion-spec view examples/models/kinova_with_objects.scenex
```

Use `--headless` with `view` to check compilation without opening a window.
Generated runs use these shapes for visual and collision geometry. RDF export
links shapes to frames with `shapes:attached-to-frame`. Each primitive has its
corresponding `shapes:UnitCube`, `shapes:UnitCylinder`, or `shapes:UnitSphere`
type; its base numerical dimensions are fixed by that type.
A shape's `qudt:unit` records the length unit for its base geometry or STL
coordinates. Scale factors stay dimensionless; the renderer converts the
resulting dimensions to metres.
A scaled shape has type `shapes:ScaledShape`. Its `shapes:scale-x`,
`shapes:scale-y`, and `shapes:scale-z` values are dimensionless `xsd:double`
literals. A scalar DSL scale writes the same value on all three axes; three
values write the corresponding factor on each axis.
For an STL mesh, RDF links the shape to a `prov:Entity` for its source file,
located with the [rdf-utils](https://github.com/minhnh/rdf-utils) PROV helpers.
A relative `sys path` is a `prov-ext:RelativeLocation` under the authored
`.scenex` directory; a `ros pkg` path is one under the package, a `ros:Package`
named by its `https://index.ros.org/p/<pkg>` IRI. An absolute `sys path` becomes
a `file:` IRI typed `prov:Location`. rdf-utils' `get_file_path` resolves any
of them to a local file. Generated runs copy the STL
beside the controller, so they no longer depend on the original checkout.
The companion metamodels repository validates
these nodes. MuJoCo keeps cubes as boxes and isotropically scaled spheres and
cylinders as native primitives. Non-uniform spheres and cylinders use triangle
meshes, so curved surfaces are approximated.

## Drawing the Kinematics

Draw the scene's bodies and the joints between them, with each device a cluster
and any declared serial chain picked out. Needs [graphviz](https://graphviz.org)
for anything but `dot`:

```bash
textx generate examples/models/kinova_with_objects.scenex --target dot -o examples/generated --format png
```

`format` takes `dot` (the default), `png`, `svg`, or `pdf`. To print the graph
instead of writing it:

```bash
textx generate examples/models/kinova_with_objects.scenex --target dot-console
```

## Development

```bash
pytest -q tests
ruff check src tests
```
