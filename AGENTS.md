# AGENTS.md — afsim-planner

## Build, test, run

```bash
cargo build --release          # build
cargo test --release           # test (inline #[cfg(test)], no separate test crate)
```

Pipeline smoke-test with the fixture:

```bash
cat tests/fixtures/sample_input.json | cargo run --release
```

There is no `cargo fmt`, `cargo clippy`, or `cargo check` configured — just `build` and `test`.

## Architecture (6-file single crate)

| File | Role |
|------|------|
| `src/main.rs` | stdin → parse → validate → seed mgmt → segment loop → prune → stdout. All I/O. |
| `src/config.rs` | Serde `Deserialize`/`Serialize` structs — the data contract. |
| `src/error.rs` | Error variants + failure output struct. |
| `src/geometry.rs` | Segment-sphere (radar), segment-polygon-prism (NFZ) collision, safe corridor. |
| `src/solver.rs` | 3D weighted A\* (26-neighbor), kinematic pruning, stagnation detector, waypoint pruner. |
| `src/macro_router.rs` | Macroscopic topology router: obstacle AABB → visibility-graph A\* → intermediate control points for long-range segmentation. |

The binary is a deterministic pipeline: **stdin JSON → stdout JSON, no disk I/O, no args**.

## Non-obvious behavior

- `seed` field tri-state: explicit `u64` vs `null` vs **missing key** — missing key triggers auto-generation via `thread_rng()`, and so does `null` (both deserialize to `Option::None`, both hit `unwrap_or_else`). All three return `seed_used` in the output. The seed value drives FNV-1a noise perturbation (3%) on the heuristic, creating seed-dependent anisotropic "resistance fields" that break symmetry around obstacles, and enables deterministic tie-breaking in the `BinaryHeap` when f-scores are equal.
- `emit_json` calls `process::exit(0)` — successful and handled-failure paths both terminate the process; this is intentional (not a leak or early-return bug).
- Grid resolution = `max(min_turn_radius / 2, 100)` meters. This is computed in `main.rs`, not solver config.
- Stagnation detection in A\*: after **120 consecutive expansions without h-score improvement**, heuristic weight decays from 1.5 → 1.0 (standard A\*). No back-off — it stays at 1.0 until a new best-h resets it.
- Waypoint pruning (`prune_waypoints`) is a **greedy forward scanner**: it tries to shortcut to the furthest reachable raw-grid point and breaks at the first collision or kinematic violation. It does not backtrack to find a longer shortcut.
- `TargetZone.radius` in the input is **not used for early termination** — the search terminates when within one `grid_resolution` of the target center.
- Input validation is minimal: negative altitude on control waypoints and `max_turn_angle_deg ∉ (0, 180]`. Everything else is trusted.

## Project conventions

- **Conventional Commits**: format `<type>(<scope>): <description>`. English type/scope, Chinese description.
- **CHANGELOG.md must be updated** with every commit that changes user-visible behavior, and the changelog is committed together with the code — same commit.
- Release process (tag-driven): push a `v*` tag → CI builds Linux musl + Windows MSVC static binaries → attaches to GitHub Release.
- The `.mimocode` directory holds the MiMoCode CLI plugin config, not application source.
- Chinese inline comments are the norm (this is a Chinese-authored project).
- `src/config.rs` has `#![allow(dead_code)]` because some output structs are only serialized (never deserialized back).

## Dependencies

| crate | version | why |
|-------|---------|-----|
| `serde` + `serde_json` | 1 | stdin/stdout JSON |
| `rand` | 0.8 | seed-based RNG for deterministic search |
