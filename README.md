# afsim-planner

**AFSIM 战前部署航路规划 CLI 工具** — 一款专为 AFSIM（Advanced Framework for Simulation, Integration, and Modeling）战前静态筹划场景设计的轻量级、确定性 3D 航路规划引擎。

## 特性

- **确定性 3D A\* 算法** — 基于三维稀疏网格的启发式搜索，100% 可复现
- **运动学剪枝** — 最大转弯角 + 爬升角约束，确保航线可被 AFSIM Mover 执行
- **极速碰撞检测** — 线段-球体（雷达威胁）、线段-多边形棱柱（禁飞区）的解析几何判断
- **标准 I/O 管道** — 完全基于 stdin/stdout 的 JSON 交互，零磁盘 I/O
- **智能种子管理** — 支持显式种子 / `null` / 缺省三种模式，自动生成高熵种子并回传

## 快速开始

```bash
# 编译
cargo build --release

# 管道调用
cat task_request.json | ./target/release/afsim-planner > path_result.json
```

## 输入格式

```json
{
  "session": { "seed": 42, "max_calculation_time_ms": 5000 },
  "vehicle": {
    "min_turn_radius": 350.0,
    "max_climb_angle": 25.0,
    "max_turn_angle_deg": 60.0
  },
  "route_definition": {
    "start_state": { "position": [0, 0, 500], "heading_deg": 45 },
    "control_waypoints": [[5000, 4000, 600]],
    "target": { "center": [20000, 20000, 1000], "radius": 500 }
  },
  "environment": {
    "radars": [
      { "id": "SAM_S300", "center": [8000, 8000, 0], "radius": 5000 }
    ],
    "no_fly_zones": [
      { "id": "BORDER_A", "boundary_points": [[2000,3000],[4000,3000],[4000,5000],[2000,5000]], "alt_min": 0, "alt_max": 99999 }
    ]
  }
}
```

## 输出格式

成功时返回航路点序列；失败时返回结构化错误信息，均包含 `seed_used` 以保证幂等可复现。

## 架构

```
src/
├── main.rs      # stdin/stdout 管道 + 种子管理 + 分段编排
├── error.rs     # 错误类型定义
├── config.rs    # 输入/输出数据契约
├── geometry.rs  # 空间碰撞检测（线-球、线-多边形棱柱）
└── solver.rs    # 3D A* 规划引擎（运动学剪枝）
```

## License

MIT
