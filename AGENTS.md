# AGENTS.md — rnav (AFSIM Planner)

## Project structure

```
cli/        Rust crate: lib + bin — deterministic 3D A* route planner (stdin JSON → stdout JSON)
demo/       Demo stack: Axum HTTP server + React/Three.js 3D visualization (not core; see below)
```

Cargo workspace (`Cargo.toml` at root): `members = ["cli", "demo/server"]`.
`demo/web/` is a separate pnpm project, not part of the workspace.

## Build, test, run

```bash
cargo build --release              # build entire workspace
cargo build --release -p cli       # build CLI only
cargo test --release               # test entire workspace (inline #[cfg(test)], no separate test crate)
```

There is no `cargo fmt`, `cargo clippy`, or `cargo check` configured — just `build` and `test`.

Tests are inline `#[cfg(test)]` modules inside source files. The `test/` and `tests/` directories are **gitignored** — do not create standalone test files there.

**⚠️ `.cargo/config.toml` is gitignored.** The repo ships without it; the in-tree copy contains machine-local GCC 11 paths for this development machine. On a different machine, the default system toolchain is used.

### Demo (optional)

```bash
./demo/start.sh                    # builds CLI + server, then starts server :3001, web dev :5173
```

## CLI: architecture & non-obvious behavior

| File | Role |
|------|------|
| `cli/src/lib.rs` | Crate root — re-exports modules so `demo/server` can depend on `cli` as a library (build-ordering only; server spawns the binary, not the library). |
| `cli/src/main.rs` | stdin → parse → validate → seed mgmt → segment loop → prune → stdout. All I/O. |
| `cli/src/config.rs` | Serde `Deserialize`/`Serialize` structs — the data contract. Has `#![allow(dead_code)]` because some output structs are only serialized. |
| `cli/src/error.rs` | Error variants + failure output struct. |
| `cli/src/geometry.rs` | Segment-sphere (radar), segment-polygon-prism (NFZ) collision, safe corridor. |
| `cli/src/solver.rs` | 3D weighted A\* (26-neighbor), kinematic pruning, stagnation detector, waypoint pruner. |
| `cli/src/macro_router.rs` | Macroscopic topology router: obstacle AABB → visibility-graph A\* → intermediate control points for long-range segmentation. Auto-triggered when start-target distance > 10km and user provides no control waypoints. |

The CLI binary is a deterministic pipeline: **stdin JSON → stdout JSON, no disk I/O, no args**.

### CLI quirks

- `seed` field tri-state: explicit `u64` vs `null` vs **missing key** — missing key triggers auto-generation via `thread_rng()`, `null` does the same (both deserialize to `None`). All three return `seed_used` in output. Seed drives FNV-1a noise perturbation (3%) on the heuristic, creating seed-dependent anisotropic "resistance fields" that break symmetry around obstacles, and enables deterministic tie-breaking in `BinaryHeap`.
- `emit_json` calls `process::exit(0)` — intentional (success and handled-failure paths both terminate).
- Grid resolution = `max(min_turn_radius / 2, 100)` meters. Computed in `main.rs`, not solver config.
- Stagnation detection: after **120 consecutive expansions without h-score improvement**, heuristic weight decays from 1.5 → 1.0 (standard A\*). No back-off — stays at 1.0 until a new best-h resets it.
- Waypoint pruning (`prune_waypoints`) is a **greedy forward scanner**: tries to shortcut to the furthest reachable raw-grid point, breaks at first collision or kinematic violation. No backtracking.
- `TargetZone.radius` is **not used for early termination** — search terminates when within one `grid_resolution` of target center.
- Input validation is minimal: negative altitude on control waypoints, `max_turn_angle_deg ∉ (0, 180]`. All else trusted.

## Server quirks

- Server spawns the CLI **binary** (`./target/release/cli`) as a subprocess — must be built first.
- Only one endpoint: `POST /api/plan`, accepts full `InputConfig` JSON, returns `PlanResult` JSON.
- No CORS middleware configured — the Vite dev server's `/api` proxy side-steps this.

## Web frontend

- `pnpm` is the package manager (not npm). Has `.npmrc` with `onlyBuiltDependencies: [esbuild]`.
- `pnpm build` runs `tsc && vite build` (typecheck then bundle).
- Zero-config proxy: Vite proxies `/api` requests to `http://localhost:3001`.
- Uses `@react-three/fiber` + `@react-three/drei` for 3D rendering.

## Workflow commands (mimocode)

`mimocode.json` defines `commit`, `push`, `sync-changes`, and `release` commands. Use these when asked to commit or release:

- **`commit`**: analyzes changes, writes Conventional Commits message, updates CHANGELOG.md, commits code + changelog together.
- **`release`**: reads [Unreleased] in CHANGELOG, recommends version bump (MAJOR/MINOR/PATCH), updates CHANGELOG + git tag, pushes.

## Project conventions

- **Conventional Commits**: `<type>(<scope>): <description>`. English type/scope, Chinese description.
- **CHANGELOG.md must be updated** with every commit that changes user-visible behavior, committed together with the code.
- Chinese inline comments are the norm.
- Release process (tag-driven): push a `v*` tag → CI builds Linux musl + Windows MSVC static binaries → attaches to GitHub Release. Only the CLI binary is released (not the server).
- `.mimocode/` and `mimocode.json` are MiMoCode CLI plugin config — not application source.

## CI (GitHub Actions)

Workflow at `.github/workflows/release.yml`:
- **On push/PR to `master`**: `cargo build --release` + `cargo test --release` (ubuntu-latest only).
- **On tag `v*`**: cross-compile CLI binary for `x86_64-unknown-linux-musl` and `x86_64-pc-windows-msvc`, strip + upload to GitHub Release.
