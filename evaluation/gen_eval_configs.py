#!/usr/bin/env python3
"""Generates the evaluation configuration files consumed by an evaluation run.

This is a *generator*, not a runner: it never loads a mesh, computes coordinates or
exports a result. It emits one ready-to-use evaluation JSON per (model entry x
coordinate setup) pair, plus a manifest that a batch run can iterate over.

Run it with no arguments to (re)generate everything:

    python3 evaluation/gen_eval_configs.py

The emitted files are consumed by the viewer, which resolves every path inside a
config relative to the ``evaluation`` directory. Point it at a single generated
config with either of:

    cageDeformationViewer --eval-config evaluation/generated/<name>.json
    CAGEMODELER_EVAL_CONFIG=evaluation/generated/<name>.json cageDeformationViewer

Only the standard library and numpy are imported, so the script runs standalone
without a build of the host application.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

# ===========================================================================
# Schema
#
# The dataclasses the configuration below is written in. Editing the
# configuration never requires touching anything past the CONFIGURATION block.
# ===========================================================================


@dataclass
class ModelEntry:
    """A model, its cage, and every deformed cage that should be evaluated with it.

    The three map exports are properties of the *entry*, not of an individual
    deformed cage: the vertex indices address the undeformed cage, which is
    shared by all of the entry's deformed cages.
    """

    #: Short identifier, used in the generated filenames. Must be unique.
    name: str

    #: Paths relative to ASSET_ROOT (for validation) and CONFIG_PATH_PREFIX (in the JSON).
    mesh: str
    cage: str

    #: One or more deformed cages evaluated against the (mesh, cage) pair above.
    deformed_cages: list = field(default_factory=list)

    #: Tetrahedral embedding, required by the LBC / Harmonic / BBW coordinate types.
    embedding: Optional[str] = None

    #: Influence map: any number of marked cage vertices.
    influence_map: bool = False
    influence_vertices: list = field(default_factory=list)

    #: Euclidean distance map: measured from exactly one cage vertex.
    euclidean_distance_map: bool = False
    euclidean_vertex: Optional[int] = None

    #: Interior distance map: measured from exactly one cage vertex.
    interior_distance_map: bool = False
    interior_vertex: Optional[int] = None

    def __post_init__(self) -> None:
        self.deformed_cages = list(self.deformed_cages)
        self.influence_vertices = list(self.influence_vertices)


@dataclass
class CoordinateSetup:
    """One coordinate variant and its parameters.

    The field names mirror the keys the evaluation config reader understands, so
    a setup maps one-to-one onto the emitted JSON.
    """

    #: Short identifier, used in the generated filenames. Must be unique.
    name: str

    #: One of KNOWN_COORDINATE_TYPES.
    coordinate_type: str

    #: PMVC depth-peeling layers. Three enables the alpha / beta / theta weights.
    hit_count: Optional[int] = None
    alpha: Optional[float] = None
    beta: Optional[float] = None
    theta: Optional[float] = None

    #: Weight PMVC with heat-method interior distances instead of the rasterized depth.
    #: This is what fills the table the interior distance map is read back from.
    use_interior_distance: bool = False

    #: Number of parameter samples.
    samples: Optional[int] = None

    #: Distance map rendering: isoline spacing in world units (0 draws none, None
    #: spaces them automatically), emphasis of every n-th isoline, and a fixed
    #: normalization maximum that keeps separate exports comparable.
    distance_interval: Optional[float] = None
    distance_emphasis: Optional[int] = None
    distance_max: Optional[float] = None


# ===========================================================================
# CONFIGURATION
#
# Everything below this banner and above the "Generator" banner is data. Edit
# freely; the generator validates it before writing anything.
# ===========================================================================

#: Directory the generated JSONs and the manifest are written to, relative to this script.
OUTPUT_DIR = "generated"

#: Where the mesh files actually live on disk. Used only to validate that every
#: referenced file exists and to read the cage vertex counts.
ASSET_ROOT = "../models"

#: Prefix prepended to every mesh path inside the generated JSON. The viewer
#: resolves config paths relative to the ``evaluation`` directory, and CMake
#: packages ``models/`` into the build directory's ``assets/meshes``.
CONFIG_PATH_PREFIX = "../assets/meshes"

#: Prefix for the project name and the timings file inside the generated JSON.
#: The viewer resolves both relative to the ``evaluation`` directory, so exports
#: land in ``evaluation/<RESULTS_PREFIX><project>/``.
RESULTS_PREFIX = "results/"


MODEL_ENTRIES = [
    ModelEntry(
        name="armadillo",
        mesh="armadilloman.obj",
        cage="armadilloman_cages_triangulated.obj",
        deformed_cages=[
            "armadilloman_cages_triangulated_deformed_1.obj",
            "armadilloman_cages_triangulated_deformed_5.obj",
            "armadilloman_cages_triangulated_deformed_8.obj",
        ],
        influence_map=True,
        influence_vertices=[0, 1, 2],
        euclidean_distance_map=True,
        euclidean_vertex=0,
        interior_distance_map=True,
        interior_vertex=0,
    ),
    ModelEntry(
        name="cactus",
        mesh="cactus.obj",
        cage="cactus_cages_triangulated.obj",
        deformed_cages=[
            "cactus_cages_triangulated_deformed.obj",
        ],
        embedding="cactus_cages_triangulated_embedding.msh",
        influence_map=True,
        influence_vertices=[0],
        euclidean_distance_map=True,
        euclidean_vertex=0,
        interior_distance_map=False,
        interior_vertex=None,
    ),
]


COORDINATE_SETUPS = [
    CoordinateSetup(
        name="mvc",
        coordinate_type="MVC",
    ),
    CoordinateSetup(
        name="green",
        coordinate_type="Green",
    ),
    CoordinateSetup(
        name="pmvc_1hit",
        coordinate_type="PMVC",
        hit_count=1,
    ),
    CoordinateSetup(
        name="pmvc_3hit",
        coordinate_type="PMVC",
        hit_count=3,
        alpha=1.0,
        beta=-1.0,
        theta=1.0,
    ),
    CoordinateSetup(
        name="pmvc_interior",
        coordinate_type="PMVC",
        hit_count=1,
        use_interior_distance=True,
        distance_interval=0.05,
        distance_emphasis=5,
    ),
    CoordinateSetup(
        name="pmvco",
        coordinate_type="PMVCO",
    ),
]


# ===========================================================================
# Generator
# ===========================================================================

#: The coordinate types the evaluation config reader accepts (matched case-insensitively).
KNOWN_COORDINATE_TYPES = (
    "MVC",
    "QMVC",
    "Harmonic",
    "BBW",
    "LBC",
    "MEC",
    "MLC",
    "Green",
    "QGC",
    "Somigliana",
    "PMVC",
    "PMVCO",
)

#: Coordinate types that cannot be computed without a tetrahedral embedding.
EMBEDDING_COORDINATE_TYPES = ("Harmonic", "BBW", "LBC")

#: The hit count that enables the three-hit PMVC weights, mirroring PMVCSettings.
THREE_HIT_COUNT = 3

#: Characters allowed in a name that becomes part of a filename and a directory.
NAME_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")

#: First line of an unfetched Git LFS pointer file.
LFS_POINTER_PREFIX = b"version https://git-lfs.github.com/spec/v1"


class ConfigError(Exception):
    """Raised with every validation problem found, so one run reports them all."""


class Problems:
    """Collects validation problems so the generator can fail once, loudly."""

    def __init__(self) -> None:
        self._errors: list = []
        self._warnings: list = []

    def error(self, where: str, message: str) -> None:
        self._errors.append("{}: {}".format(where, message))

    def warn(self, where: str, message: str) -> None:
        self._warnings.append("{}: {}".format(where, message))

    @property
    def warnings(self) -> list:
        return list(self._warnings)

    def raise_if_any(self) -> None:
        if not self._errors:
            return

        lines = [
            "Refusing to generate: {} configuration problem(s) found.".format(len(self._errors))
        ]
        lines.extend("  [{}] {}".format(i + 1, e) for i, e in enumerate(self._errors))

        raise ConfigError("\n".join(lines))


# ---------------------------------------------------------------------------
# Mesh file inspection
# ---------------------------------------------------------------------------


def _is_lfs_pointer(path: Path) -> bool:
    """A pointer file stands in for content that ``git lfs pull`` has not fetched."""
    try:
        with path.open("rb") as handle:
            return handle.read(len(LFS_POINTER_PREFIX)) == LFS_POINTER_PREFIX
    except OSError:
        return False


def _count_obj_vertices(path: Path) -> int:
    """Counts the ``v`` lines of an OBJ.

    The cage loader reads the file with igl and keeps the file order, so the
    n-th ``v`` line is cage vertex index n - 1.
    """
    count = 0
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.startswith("v") and line[1:2].isspace():
                count += 1

    return count


def _count_off_vertices(path: Path) -> int:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        tokens = []
        for line in handle:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            if not tokens and line.upper().startswith("OFF"):
                line = line[3:].strip()
                if not line:
                    continue
            tokens.extend(line.split())
            if len(tokens) >= 3:
                break

    if not tokens:
        raise ValueError("no OFF header found")

    return int(tokens[0])


def _count_msh_vertices(path: Path) -> int:
    """Reads the node count out of a gmsh file (both the 2.x and 4.x layouts)."""
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.strip() != "$Nodes":
                continue

            counts = handle.readline().split()
            if not counts:
                raise ValueError("empty $Nodes header")

            # Legacy (2.x) stores just the node count, 4.x stores
            # "numEntityBlocks numNodes minTag maxTag".
            return int(counts[1]) if len(counts) >= 4 else int(counts[0])

    raise ValueError("no $Nodes section found")


def read_vertex_count(path: Path) -> int:
    """Returns the number of vertices stored in a mesh file."""
    suffix = path.suffix.lower()
    if suffix == ".obj":
        return _count_obj_vertices(path)
    if suffix == ".off":
        return _count_off_vertices(path)
    if suffix == ".msh":
        return _count_msh_vertices(path)

    raise ValueError("unsupported mesh format '{}'".format(suffix))


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


def _check_indices(problems, where, label, indices, cage_vertex_count):
    """Validates a set of cage vertex indices, naming every offending value."""
    array = np.asarray(indices)
    if array.size == 0:
        return

    if not np.issubdtype(array.dtype, np.integer):
        problems.error(
            where,
            "{} must be whole numbers, got {!r}.".format(label, list(indices)),
        )
        return

    if cage_vertex_count is None:
        # The cage could not be read; the file-level error already says why.
        return

    out_of_range = array[(array < 0) | (array >= cage_vertex_count)]
    if out_of_range.size:
        problems.error(
            where,
            "{} out of range: {} (the cage has {} vertices, so valid indices are 0..{}).".format(
                label,
                sorted(set(int(v) for v in out_of_range)),
                cage_vertex_count,
                cage_vertex_count - 1,
            ),
        )

    unique, counts = np.unique(array, return_counts=True)
    duplicates = sorted(int(v) for v in unique[counts > 1])
    if duplicates:
        problems.warn(where, "{} lists duplicate indices {}.".format(label, duplicates))


def _resolve_asset(problems, where, label, relative_path, asset_root):
    """Checks that a referenced mesh file exists and has real content behind it."""
    if not relative_path:
        problems.error(where, "{} is empty.".format(label))
        return None

    resolved = asset_root / relative_path
    if not resolved.is_file():
        problems.error(
            where,
            "{} '{}' does not exist (looked in '{}').".format(label, relative_path, resolved),
        )
        return None

    if _is_lfs_pointer(resolved):
        problems.error(
            where,
            "{} '{}' is an unfetched Git LFS pointer, not a mesh. Run 'git lfs pull' "
            "before generating, otherwise vertex indices cannot be validated.".format(
                label, relative_path
            ),
        )
        return None

    return resolved


def _validate_names(problems, items, kind):
    seen = {}
    for item in items:
        where = "{} '{}'".format(kind, item.name)
        if not NAME_PATTERN.match(item.name or ""):
            problems.error(
                where,
                "name must start with a letter or digit and use only letters, digits, "
                "'.', '_' and '-', because it becomes part of a filename.",
            )
        if item.name in seen:
            problems.error(where, "duplicate {} name.".format(kind))
        seen[item.name] = item


def _validate_setup(problems, setup):
    where = "coordinate setup '{}'".format(setup.name)

    lowered = {t.lower(): t for t in KNOWN_COORDINATE_TYPES}
    canonical = lowered.get((setup.coordinate_type or "").lower())
    if canonical is None:
        problems.error(
            where,
            "unknown coordinate_type '{}'. Known types: {}.".format(
                setup.coordinate_type, ", ".join(KNOWN_COORDINATE_TYPES)
            ),
        )
        return None

    if setup.hit_count is not None and setup.hit_count <= 0:
        problems.error(where, "hit_count must be greater than zero, got {}.".format(setup.hit_count))

    if not is_pmvc(canonical):
        if setup.hit_count is not None:
            problems.warn(where, "hit_count is only used by the PMVC coordinate types.")
        if setup.use_interior_distance:
            problems.error(
                where,
                "use_interior_distance requires the PMVC coordinate type, not '{}'.".format(canonical),
            )

    # The viewer derives the offset variant's settings from the coordinate type
    # itself, so a setup that disagrees would not evaluate what it claims to.
    if canonical == "PMVCO":
        if setup.hit_count not in (None, 1):
            problems.error(
                where,
                "PMVCO always runs with a single hit, so hit_count {} would be overridden. "
                "Drop it or use the PMVC coordinate type.".format(setup.hit_count),
            )
        if setup.use_interior_distance:
            problems.error(
                where,
                "PMVCO has no distance term, so use_interior_distance would be overridden. "
                "Use the PMVC coordinate type for the interior distance variant.",
            )

    if canonical == "PMVC" and setup.hit_count != THREE_HIT_COUNT:
        if setup.alpha is not None or setup.beta is not None or setup.theta is not None:
            problems.warn(
                where,
                "alpha / beta / theta only apply to the three-hit variant "
                "(hit_count={}), they are ignored here.".format(THREE_HIT_COUNT),
            )

    if setup.distance_emphasis is not None and setup.distance_emphasis < 0:
        problems.error(
            where, "distance_emphasis must not be negative, got {}.".format(setup.distance_emphasis)
        )

    if setup.distance_interval is not None and setup.distance_interval < 0:
        problems.error(
            where, "distance_interval must not be negative, got {}.".format(setup.distance_interval)
        )

    return canonical


def _validate_entry(problems, entry, asset_root):
    """Validates one model entry and returns its cage vertex count, if readable."""
    where = "model entry '{}'".format(entry.name)

    _resolve_asset(problems, where, "mesh", entry.mesh, asset_root)
    cage_path = _resolve_asset(problems, where, "cage", entry.cage, asset_root)

    if entry.embedding:
        _resolve_asset(problems, where, "embedding", entry.embedding, asset_root)

    if not entry.deformed_cages:
        problems.error(where, "deformed_cages is empty, there is nothing to evaluate.")

    seen_tags = {}
    for deformed_cage in entry.deformed_cages:
        _resolve_asset(problems, where, "deformed cage", deformed_cage, asset_root)

        tag = Path(deformed_cage).stem if deformed_cage else ""
        if tag in seen_tags:
            problems.error(
                where,
                "deformed cages '{}' and '{}' share the filename stem '{}', which would "
                "produce colliding output names.".format(seen_tags[tag], deformed_cage, tag),
            )
        seen_tags[tag] = deformed_cage

        if tag and not NAME_PATTERN.match(tag):
            problems.error(
                where,
                "deformed cage '{}' has a filename stem that cannot be used in an output "
                "name. Rename the file or the entry.".format(deformed_cage),
            )

    cage_vertex_count = None
    if cage_path is not None:
        try:
            cage_vertex_count = read_vertex_count(cage_path)
        except (OSError, ValueError) as exc:
            problems.error(where, "cannot read the vertex count of cage '{}': {}.".format(entry.cage, exc))
        else:
            if cage_vertex_count == 0:
                problems.error(where, "cage '{}' contains no vertices.".format(entry.cage))
                cage_vertex_count = None

    # Influence map: any number of marked cage vertices, but at least one.
    if entry.influence_map:
        if not entry.influence_vertices:
            problems.error(
                where, "influence_map is enabled but influence_vertices is empty."
            )
        else:
            _check_indices(
                problems, where, "influence_vertices", entry.influence_vertices, cage_vertex_count
            )
    elif entry.influence_vertices:
        problems.warn(where, "influence_vertices is set but influence_map is disabled.")

    # Both distance maps are measured from exactly one cage vertex.
    for enabled, vertex, toggle_name, vertex_name in (
        (entry.euclidean_distance_map, entry.euclidean_vertex, "euclidean_distance_map", "euclidean_vertex"),
        (entry.interior_distance_map, entry.interior_vertex, "interior_distance_map", "interior_vertex"),
    ):
        if enabled:
            if vertex is None:
                problems.error(
                    where,
                    "{} is enabled but {} is not set; exactly one cage vertex is "
                    "required.".format(toggle_name, vertex_name),
                )
            elif isinstance(vertex, bool) or not isinstance(vertex, (int, np.integer)):
                problems.error(
                    where,
                    "{} must be a single vertex index, got {!r}.".format(vertex_name, vertex),
                )
            else:
                _check_indices(problems, where, vertex_name, [int(vertex)], cage_vertex_count)
        elif vertex is not None:
            problems.warn(where, "{} is set but {} is disabled.".format(vertex_name, toggle_name))

    return cage_vertex_count


def is_pmvc(coordinate_type: str) -> bool:
    return coordinate_type in ("PMVC", "PMVCO")


def produces_interior_distances(setup: CoordinateSetup, coordinate_type: str) -> bool:
    """Whether a run of this setup fills the table the interior distance map reads back.

    Only the interior-distance PMVC variant computes it; every other setup would
    export nothing at all.
    """
    return coordinate_type == "PMVC" and bool(setup.use_interior_distance)


# ---------------------------------------------------------------------------
# Emission
# ---------------------------------------------------------------------------


def build_project(entry, deformed_cage, setup, coordinate_type, project_name, path_prefix):
    """Builds the project object of one evaluation config, and its manifest notes."""
    notes = []

    def asset(relative_path):
        return "{}/{}".format(path_prefix.rstrip("/"), relative_path) if path_prefix else relative_path

    project = {
        "name": project_name,
        "coordinateType": coordinate_type,
        "mesh": asset(entry.mesh),
        "cage": asset(entry.cage),
        "deformedCage": asset(deformed_cage),
    }

    if entry.embedding:
        project["embedding"] = asset(entry.embedding)

    if setup.samples is not None:
        project["samples"] = setup.samples

    if is_pmvc(coordinate_type):
        if setup.hit_count is not None:
            project["hitCount"] = setup.hit_count
        if setup.alpha is not None:
            project["alpha"] = setup.alpha
        if setup.beta is not None:
            project["beta"] = setup.beta
        if setup.theta is not None:
            project["theta"] = setup.theta
        project["useInteriorDistance"] = bool(setup.use_interior_distance)

    project["influenceMap"] = bool(entry.influence_map)
    if entry.influence_map:
        project["influenceVertices"] = [int(v) for v in entry.influence_vertices]

    project["euclideanDistanceMap"] = bool(entry.euclidean_distance_map)
    if entry.euclidean_distance_map:
        project["euclideanDistanceVertex"] = int(entry.euclidean_vertex)

    # The interior distance map is read back from the table the interior-distance
    # PMVC variant fills. Any other coordinate setup has nothing to read, so the
    # toggle is dropped instead of emitting a config that exports nothing.
    interior_enabled = bool(entry.interior_distance_map)
    if interior_enabled and not produces_interior_distances(setup, coordinate_type):
        interior_enabled = False
        notes.append(
            "interior distance map disabled: coordinate setup '{}' ({}) does not compute "
            "interior distances".format(setup.name, coordinate_type)
        )

    project["interiorDistanceMap"] = interior_enabled
    if interior_enabled:
        project["interiorDistanceVertex"] = int(entry.interior_vertex)

    if entry.euclidean_distance_map or interior_enabled:
        if setup.distance_interval is not None:
            project["distanceFieldInterval"] = setup.distance_interval
        if setup.distance_emphasis is not None:
            project["distanceFieldEmphasis"] = setup.distance_emphasis
        if setup.distance_max is not None:
            project["distanceFieldMax"] = setup.distance_max

    return project, notes


def generate(entries, setups, output_dir, asset_root, path_prefix, results_prefix, dry_run=False):
    """Validates the configuration, then writes one config per pair plus a manifest."""
    problems = Problems()

    _validate_names(problems, entries, "model entry")
    _validate_names(problems, setups, "coordinate setup")

    for entry in entries:
        _validate_entry(problems, entry, asset_root)

    canonical_types = {}
    for setup in setups:
        canonical = _validate_setup(problems, setup)
        if canonical is not None:
            canonical_types[setup.name] = canonical

    # Nothing is built from data that failed validation, so a bad entry is always
    # reported by name instead of surfacing as an error deeper in the generator.
    problems.raise_if_any()

    # Sorted iteration so a rerun of an unchanged configuration is byte identical.
    sorted_entries = sorted(entries, key=lambda e: e.name)
    sorted_setups = sorted(setups, key=lambda s: s.name)

    configs = []
    seen_stems = {}
    for entry in sorted_entries:
        for deformed_cage in sorted(entry.deformed_cages, key=lambda c: Path(c).stem):
            for setup in sorted_setups:
                coordinate_type = canonical_types.get(setup.name)
                if coordinate_type is None:
                    continue

                if coordinate_type in EMBEDDING_COORDINATE_TYPES and not entry.embedding:
                    problems.error(
                        "model entry '{}' x coordinate setup '{}'".format(entry.name, setup.name),
                        "{} requires a tetrahedral embedding, but the model entry has no "
                        "'embedding'.".format(coordinate_type),
                    )
                    continue

                stem = "{}__{}__{}".format(entry.name, Path(deformed_cage).stem, setup.name)
                if stem in seen_stems:
                    problems.error(
                        "model entry '{}' x coordinate setup '{}'".format(entry.name, setup.name),
                        "generated name '{}' collides with the one of {}.".format(
                            stem, seen_stems[stem]
                        ),
                    )
                    continue
                seen_stems[stem] = "model entry '{}' x coordinate setup '{}'".format(
                    entry.name, setup.name
                )

                project_name = "{}{}".format(results_prefix, stem)
                timings_file = "{}timings_{}.json".format(results_prefix, stem)

                project, notes = build_project(
                    entry, deformed_cage, setup, coordinate_type, project_name, path_prefix
                )

                configs.append(
                    {
                        "filename": "{}.json".format(stem),
                        "document": {"timingsFile": timings_file, "projects": [project]},
                        "manifest": {
                            "config": "{}.json".format(stem),
                            "projectName": project_name,
                            "timingsFile": timings_file,
                            "modelEntry": entry.name,
                            "mesh": project["mesh"],
                            "cage": project["cage"],
                            "deformedCage": project["deformedCage"],
                            "embedding": project.get("embedding"),
                            "coordinateSetup": setup.name,
                            "coordinateType": coordinate_type,
                            "maps": {
                                "influence": project["influenceMap"],
                                "euclideanDistance": project["euclideanDistanceMap"],
                                "interiorDistance": project["interiorDistanceMap"],
                            },
                            "notes": notes,
                        },
                    }
                )

    problems.raise_if_any()

    for warning in problems.warnings:
        print("warning: {}".format(warning), file=sys.stderr)

    manifest = {
        "generator": Path(__file__).name,
        "version": 1,
        "assetRoot": str(asset_root),
        "configPathPrefix": path_prefix,
        "resultsPrefix": results_prefix,
        "modelEntryCount": len(sorted_entries),
        "coordinateSetupCount": len(sorted_setups),
        "configCount": len(configs),
        "configs": [c["manifest"] for c in configs],
    }

    if dry_run:
        print(
            "Validated {} model entries x {} coordinate setups -> {} configs (nothing written).".format(
                len(sorted_entries), len(sorted_setups), len(configs)
            )
        )
        return manifest

    output_dir.mkdir(parents=True, exist_ok=True)

    written = set()
    for config in configs:
        _write_json(output_dir / config["filename"], config["document"])
        written.add(config["filename"])

    _remove_stale(output_dir, written)
    _write_json(output_dir / "manifest.json", manifest)

    print(
        "Wrote {} configs and a manifest to '{}'.".format(len(configs), output_dir)
    )

    return manifest


def _write_json(path: Path, document) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(document, handle, indent=2, sort_keys=False)
        handle.write("\n")


def _remove_stale(output_dir: Path, written) -> None:
    """Drops configs a previous run wrote that this configuration no longer produces."""
    manifest_path = output_dir / "manifest.json"
    if not manifest_path.is_file():
        return

    try:
        with manifest_path.open("r", encoding="utf-8") as handle:
            previous = json.load(handle)
    except (OSError, ValueError):
        return

    for entry in previous.get("configs", []):
        filename = entry.get("config")
        if not filename or filename in written:
            continue

        stale = output_dir / filename
        if stale.is_file():
            stale.unlink()
            print("Removed stale config '{}'.".format(stale))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Override OUTPUT_DIR (relative paths are resolved against this script).",
    )
    parser.add_argument(
        "--asset-root",
        default=None,
        help="Override ASSET_ROOT (relative paths are resolved against this script).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate the configuration without writing anything.",
    )
    args = parser.parse_args(argv)

    script_dir = Path(__file__).resolve().parent
    output_dir = (script_dir / (args.output_dir or OUTPUT_DIR)).resolve()
    asset_root = (script_dir / (args.asset_root or ASSET_ROOT)).resolve()

    if not asset_root.is_dir():
        print(
            "error: the asset root '{}' does not exist. Point ASSET_ROOT at the directory "
            "holding the mesh files.".format(asset_root),
            file=sys.stderr,
        )
        return 2

    try:
        generate(
            MODEL_ENTRIES,
            COORDINATE_SETUPS,
            output_dir,
            asset_root,
            CONFIG_PATH_PREFIX,
            RESULTS_PREFIX,
            dry_run=args.dry_run,
        )
    except ConfigError as exc:
        print("error: {}".format(exc), file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
