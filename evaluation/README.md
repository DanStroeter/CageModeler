# Evaluation configs

`gen_eval_configs.py` generates the evaluation JSON files an evaluation run consumes. It
is a *generator*, not a runner: it never loads a mesh, computes coordinates or exports a
result. Only the standard library and numpy are imported, so it runs standalone without a
build of the viewer.

## Workflow

1. Edit the `CONFIGURATION` block at the top of `gen_eval_configs.py`.
2. Generate:

   ```sh
   python3 evaluation/gen_eval_configs.py
   ```

   It writes one config per (model entry x coordinate setup) pair into
   `evaluation/generated/`, plus a `manifest.json` listing all of them.

3. Run one config, or loop over the manifest:

   ```sh
   cageDeformationViewer --eval-config evaluation/generated/<name>.json
   # or
   CAGEMODELER_EVAL_CONFIG=evaluation/generated/<name>.json cageDeformationViewer
   ```

   Without an override the viewer falls back to `evaluation/projects.json`.

Use `--dry-run` to validate the configuration without writing anything.

## Configuration

**Model entries** are `(model, cage, deformed cage)` triples. A single entry lists several
`deformed_cages`, so a model is never repeated. The three color map exports are properties
of the *entry*, because their vertex indices address the undeformed cage that all of the
entry's deformed cages share:

| Map                   | Toggle                   | Vertex selection                     |
| --------------------- | ------------------------ | ------------------------------------ |
| Influence map         | `influence_map`          | `influence_vertices` — a list        |
| Euclidean distance map| `euclidean_distance_map` | `euclidean_vertex` — exactly one     |
| Interior distance map | `interior_distance_map`  | `interior_vertex` — exactly one      |

Indices are 0-based into the undeformed cage: index *n* is the (*n*+1)-th `v` line of the
cage OBJ, because the cage loader keeps the file order.

**Coordinate setups** are a separate list of variants and their parameters
(`coordinate_type`, `hit_count`, `alpha` / `beta` / `theta`, `use_interior_distance`,
`samples`, and the distance map isoline settings). The generator emits the full cartesian
product of model entries and coordinate setups.

Filenames and project names are `<entry>__<deformed cage stem>__<setup>`, iteration is
sorted, so regenerating an unchanged configuration reproduces byte-identical output.

## Validation

Nothing is written until the whole configuration validates. Every problem found is
reported at once, naming the offending entry:

- every referenced file exists (and is not an unfetched Git LFS pointer)
- vertex indices are within the cage's vertex count and non-negative
- no duplicate model entry names, coordinate setup names or generated filenames
- exactly one vertex where exactly one is required, at least one where a list is required
- coordinate types are known, and a setup does not ask for something the viewer overrides
  (`PMVCO` forces a single hit and no interior distance)
- coordinate types requiring a tetrahedral embedding have one

## Paths

`ASSET_ROOT` is where the meshes live on disk and is used only for validation.
`CONFIG_PATH_PREFIX` is what gets written into the JSON. They differ because the viewer
resolves config paths relative to the `evaluation` directory at run time, against the
build directory's `assets/meshes`, which CMake packages from `models/`.

Validation reads the cage files, so fetch them first:

```sh
git lfs pull
```

## The interior distance map

The interior distances are read back from the table the interior-distance PMVC variant
fills during the run; they are never recomputed by the export. A coordinate setup that is
not PMVC with `use_interior_distance=True` therefore has nothing to read. The generator
still emits the pair, but writes `interiorDistanceMap: false` and records the reason in
the manifest, so no config silently exports nothing.

## `evaluation.py`

`evaluation.py` is the older, unrelated evaluation path. It drives the `cageDeformation3D`
command line binary, which has no PMVC or PMVCO options, and it does not currently parse
(`IndentationError` at line 55). It is untouched by this generator.
