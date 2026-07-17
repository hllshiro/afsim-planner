# afsim-planner

[![CI](https://github.com/hllshiro/afsim-planner/actions/workflows/release.yml/badge.svg)](https://github.com/hllshiro/afsim-planner/actions/workflows/release.yml)

**AFSIM 战前部署 3D 航路规划 CLI 引擎** — 专为 AFSIM（Advanced Framework for Simulation, Integration, and Modeling）战前静态筹划场景设计的轻量级、确定性三维避障航路规划工具。安全飞行走廊 + 加权 A* 双层架构，百公里级直航毫秒级响应。通过标准输入输出 JSON 管道与前端标绘系统集成。

## 特性

- **安全飞行走廊搜索剪枝** — 将 A* 搜索空间约束在起点至终点的胶囊管道内，百公里级搜索节点数精确控制在 ～n 个（n = 网格格长），消除 O(N³) 维度灾难
- **确定性 3D 加权 A\* 搜索** — 26 向隐式稀疏网格 + 欧氏距离启发式，相同种子 100% 可复现
- **进度停滞-权重动态衰减** — 实时监控 h-score 改善率，检测到 U 型口袋阵（贪婪陷阱）时自动将启发式权重从 1.5 熔断至 1.0，实现无痛绕行
- **运动学剪枝** — 最大转弯角 + 爬升角约束在节点扩展时即时过滤，确保输出航线可被 AFSIM Mover 执行
- **极速解析几何碰撞检测** — 线段-球体（雷达威胁）、线段-多边形棱柱（禁飞区）
- **航点抽稀简化** — 贪心双指针后处理，从数百个冗余网格点压缩至 3-5 个关键机动拐点，同时保留用户指定的控制导航点
- **智能种子管理** — 支持显式种子 / `null` / 缺失三种模式，自动生成高熵种子并始终回传 `seed_used`
- **标准流管道** — 完全基于 stdin/stdout 的 JSON 交互，零磁盘 I/O

## 安装

### 从源码编译

```bash
git clone https://github.com/hllshiro/afsim-planner.git
cd afsim-planner
cargo build --release
```

二进制文件位于 `target/release/afsim-planner`。

### 预编译二进制

前往 [Releases](https://github.com/hllshiro/afsim-planner/releases) 页面下载 Linux musl / Windows MSVC 静态链接版本。

## 使用

```bash
# 管道调用
cat task_request.json | ./target/release/afsim-planner > path_result.json

# 零障碍直航
echo '{
  "session": {"seed": 42},
  "vehicle": {"min_turn_radius": 500, "max_climb_angle": 30, "max_turn_angle_deg": 45},
  "route_definition": {
    "start_state": {"position": [0, 0, 500], "heading_deg": 0},
    "target": {"center": [5000, 0, 500], "radius": 100}
  },
  "environment": {"radars": [], "no_fly_zones": []}
}' | ./target/release/afsim-planner
```

## 输入格式

```jsonc
{
  "session": {
    "seed": 42,                    // 可选：显式种子，null 或缺失则自动生成
    "max_calculation_time_ms": 5000 // 可选：搜索超时毫秒数
  },
  "vehicle": {
    "min_turn_radius": 350.0,
    "max_climb_angle": 25.0,       // 最大爬升角（度）
    "max_turn_angle_deg": 60.0     // 最大转弯角（度）
  },
  "route_definition": {
    "start_state": {
      "position": [0, 0, 500],
      "heading_deg": 45
    },
    "control_waypoints": [         // 可选：有序必经控制点
      [5000, 4000, 600]
    ],
    "target": {
      "center": [20000, 20000, 1000],
      "radius": 500
    }
  },
  "environment": {
    "radars": [
      {
        "id": "SAM_S300",
        "center": [8000, 8000, 0],
        "radius": 5000
      }
    ],
    "no_fly_zones": [
      {
        "id": "BORDER_A",
        "boundary_points": [[2000, 3000], [4000, 3000], [4000, 5000], [2000, 5000]],
        "alt_min": 0,
        "alt_max": 99999
      }
    ]
  }
}
```

## 输出格式

### 成功

```json
{
  "status": "SUCCESS",
  "diagnostics": {
    "calculation_time_ms": 0.06,
    "nodes_explored": 20,
    "seed_used": 42
  },
  "summary": {
    "total_length_m": 5000.0,
    "max_climb_angle_utilized": 0.0
  },
  "waypoints": [
    { "index": 0, "position": [0.0, 0.0, 500.0], "type": "START" },
    { "index": 1, "position": [2000.0, 0.0, 600.0], "type": "WAYPOINT" },
    { "index": 2, "position": [5000.0, 0.0, 500.0], "type": "TARGET" }
  ]
}
```

### 失败

```json
{
  "status": "FAILED",
  "error": {
    "code": "MAX_CALCULATION_TIME_EXCEEDED",
    "message": "Segment 0: ROUTE_GENERATION_FAILED: Max calculation time exceeded.",
    "location": [5000.0, 5000.0, 500.0],
    "seed_used": 42
  }
}
```

## 架构

```
src/
├── main.rs      # stdin/stdout 管道 + 种子管理 + 分段编排 + 抽稀
├── error.rs     # 错误码定义
├── config.rs    # 输入/输出 JSON 数据契约 + 3D 向量基元
├── geometry.rs  # 空间碰撞检测 + 安全飞行走廊（AABB/Capsule 管道）
└── solver.rs    # 3D 加权 A* 引擎（走廊剪枝 + 运动学剪枝 + 停滞检测 + 航点抽稀）
```

## 性能

| 场景 | 耗时 | 节点数 |
|------|------|--------|
| 100km 直线，零威胁 | 3.0 ms | 572 |
| 100km + 高墙绕障，分段路点导引 | 1.9 ms | 574 |
| 20km + 密集障碍物（典型战术场景） | <40 ms | ~4,000 |

## 已知限制

- 单段超长距离（>50km）遇穿廊障碍物时，走廊内无可行路径会回退到无约束搜索，建议通过 `control_waypoints` 分段化解
- `TargetZone.radius` 未用于提前终止（目前以 target center 的一个网格分辨率距离为终止条件）

## License

MIT
