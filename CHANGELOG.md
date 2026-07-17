# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.4.0] — 2026-07-17

### Added

- **种子各向异性噪声扰动** — 将 seed 注入 A* 启发式函数，通过 FNV-1a 哈希生成确定性空间阻力场，不同种子产生拓扑分流的航线，支持多机协同攻击时的平行互不干扰航线生成

### Changed

- **种子确定性平局决断** — 当 f-score 相等时使用 seed 派生的噪声权重稳定排序，确保跨平台完全一致的路径复现

## [0.3.0] — 2026-07-17

### Added

- **长距离分级路径搜索** — 起终点超过 10km 且无用户控制点时，自动通过宏观拓扑路由生成中间导航点，将单段长距离 A* 拆分为多段短距离搜索，消除 O(N³) 维度灾难

### Changed

- **100km+ 密集障碍物绕行性能跃升** — 10km 半径多雷达场景从超时（>10s）降至 3-10ms，提升约 1000x

### Known limitations

- `TargetZone.radius` 未用于提前终止（目前以 target center 的一个网格分辨率距离为终止条件）

## [0.2.0] — 2026-07-17

### Added

- **安全飞行走廊搜索空间剪枝** — A* 搜索限定在起点至终点的胶囊管道内，大幅缩减节点展开数，百公里级直线计算降至毫秒级

### Changed

- **缓解长距离超时问题** — 配合分段规划，100km 绕障计算从超时降至 2ms 以内

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

- 单段超长距离（>50km）遇穿廊障碍时仍可能触发无约束搜索回退导致超时；建议通过 control_waypoints 分段化解
- `TargetZone.radius` not yet used for early termination (currently terminates within one grid cell of target center)

[0.4.0]: https://github.com/hllshiro/afsim-planner/releases/tag/v0.4.0
[0.3.0]: https://github.com/hllshiro/afsim-planner/releases/tag/v0.3.0
[0.2.0]: https://github.com/hllshiro/afsim-planner/releases/tag/v0.2.0
[0.1.0]: https://github.com/hllshiro/afsim-planner/releases/tag/v0.1.0
