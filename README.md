# scene-dsl

textX languages for abstract scene models (`scene`) and executable scene instances (`scenex`).

## Languages

- `scene` (`*.scene`) describes abstract robotic scene content: objects,
  workspaces, agents, workspace compositions, and scene models.
- `scenex` (`*.scenex`) describes executable scene instances: model files,
  geometry, frames, poses, kinematics, attachments, bodies, and inertial mass.

`scenex` references the registered `scene` language, so executable models can
import a `.scene` file and link to `SceneModel` declarations.

## Installation

`scene-dsl` is a Python package installable with `pip`. It depends on
[`rdf-utils`](https://github.com/minhnh/rdf-utils),
[`bdd-dsl`](https://github.com/minhnh/bdd-dsl), `textx`, and `rdflib`.

For local development with sibling checkouts:

```bash
pip install "../rdf-utils[all]"
pip install ../bdd-dsl
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

## Development

```bash
pytest -q tests
ruff check src tests
```
