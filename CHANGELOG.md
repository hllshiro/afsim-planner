# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] — 2026-07-17

### Added

- **3D deterministic A\* path search** on sparse implicit grid with 26-directional neighbors
- **Kinematic pruning** — maximum turn angle and climb angle constraints applied during neighbor expansion, ensuring AFSIM Mover executability
- **Fast collision detection** — analytic line-sphere intersection (radar threats) and line-polygon prism intersection (no-fly zones)
- **Segmented route planning** — ordered control waypoints split route into independent segments, solved sequentially with heading propagation
- **Stdin/stdout JSON pipeline** — zero disk I/O, fully streaming interface for integration with frontend mapping tools
- **Auto seed generation** — supports explicit `seed`, `seed: null`, and missing `seed` field; always returns `seed_used` for deterministic reproducibility
- **Progress-stagnation weight decay** — detects A\* greedy traps (U-shaped obstacles) by monitoring h-score improvement over consecutive expansions; auto-relaxes heuristic weight from 1.5 to 1.0 to escape local minima without expensive tree rewinding
- **Per-segment waypoint pruning** — greedy double-pointer path shortcutter that collapses hundreds of collinear grid artifacts into minimal critical-maneuver waypoints, while preserving user-specified control waypoints

### Infrastructure

- **GitHub Actions CI/CD** — `Build & Release` workflow: CI (build + test) on push/PR to `master`; release (Linux musl + Windows MSVC cross-compile) on tag `v*`
- Rust 2021 edition, dependencies: `serde` 1, `serde_json` 1, `rand` 0.8

### Known limitations

- A\* search may exceed time budget on long-range routes (>20 km) with dense obstacles at current grid resolution
- `TargetZone.radius` not yet used for early termination (currently terminates within one grid cell of target center)

[0.1.0]: https://github.com/hllshiro/afsim-planner/releases/tag/v0.1.0
