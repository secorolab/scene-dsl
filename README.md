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
    shape sphere surface { frame: <center> radius: 0.05 m }
}
```

Each shape references a frame on its body; pose that frame with `wrt` to place
the shape. Sphere, cube, rectangular prism, and cylinder primitives are centered
at that frame. Prism dimensions are full X/Y/Z lengths, and cylinder height
follows local Z. Lengths must be positive and use `m`, `cm`, or `mm`.

STL meshes use `shape mesh` with a `.stl` `path` and optional positive
`scale: (x, y, z)`. The mesh keeps its authored origin; STL coordinates are
interpreted as metres. Any shape may have an optional RGBA `color` in `[0, 1]`.
Shapes do not supply mass or inertia.

Sizes and mesh scale also accept `sample <distribution>`. Use a one-component
uniform distribution for scalar sizes or three components for prism dimensions
and mesh scale; size bounds must be finite and positive. A value is drawn once
per run and reused wherever referenced. The robot example's optional
`[sampling].seed` makes runs repeatable, though the viewer and generated run
may draw different values for the same seed.

From this repository directory, view the [compound example](examples/models/compound.scenex)
for fixed and sampled shapes, local poses, and STL scale. With MuJoCo assets
installed, the [robot example](examples/models/lab-sampled.scenex) shows a
sampled sphere on a body alongside a Kinova arm:

```bash
motion-spec view examples/models/compound.scenex
motion-spec view examples/models/lab-sampled.scenex --config examples/models/lab-sampled.toml
motion-spec run examples/models/lab-sampled.robmot --headless --steps 5
```

Use `--headless` with `view` to check compilation without opening a window.
Generated runs use these shapes for visual and collision geometry. RDF export
links shapes to frames with `shapes:attached-to-frame` and dimensions with
`shapes:has-dimension`. A rectangular prism has one `geom-coord:VectorXYZ`
dimension node with fixed `geom-coord:x/y/z` lengths or one sampled distribution.
A scaled mesh has type `shapes:Scale`; fixed factors
use `geom-coord:x/y/z` on that mesh, while sampled factors use
`distrib:from-distribution` on it. The companion metamodels repository validates
these nodes.

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
