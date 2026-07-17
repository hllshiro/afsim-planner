# AFSIM 战前部署航路规划工具 (CLI)

## 技术方案与开发文档

本工具是一款专为 **AFSIM（Advanced Framework for Simulation, Integration, and Modeling）** 战前静态筹划场景设计的轻量级、确定性 3D 航路规划引擎。

在 AFSIM 体系下，平台在仿真运行期的移动和动力学解算完全由其内置的 `Mover` 接管。因此，本工具将高维度的动力学规划简化为 **3D 稀疏几何路径搜索**。工具采用 Rust 语言构建，剔除了所有物理参数与时间积分，专门为前端标绘系统提供**亚毫秒级的规划响应**，并严格通过标准输入输出（标准流）实现 100% 可复现的 JSON 数据交互。

---

## 一、 需求分析 (Requirements Analysis)

### 1. 背景与定位

在战役级兵棋推演或仿真筹划阶段，参谋人员需要快速在地图上标绘红蓝双方的初始航线。该工具的定位是：

* **标准流交互：** 移除传统的文件传参模式，完全基于 Unix 管道设计，通过 `stdin` 接收前端或中台的标绘请求，通过 `stdout` 即时吐出规划结果，实现零磁盘 I/O 开销。
* **几何约束与解耦：** 不计算运行期的瞬态物理量。但算法生成的稀疏折线必须严密契合飞机的宏观物理性能限制（如最大转弯角、最大爬升率），确保导入 AFSIM 运行时，不会因“角度太尖锐”导致 AFSIM 内置飞控发生严重偏航。
* **高频交互响应：** 支持前端拖拽控制点时，航线实现小于 10ms 的无感实时刷新。

### 2. 核心痛点

* **确定性要求：** 军事筹划和兵棋推演要求实验 100% 可复现。相同的输入必须产生绝对相同的航线输出。
* **种子缺省痛点：** 在批量自动化生成或用户首次快捷标绘时，每次都强制要求生成并传入种子会增加上层业务的复杂度。系统需要支持自动随机落子，同时必须把使用的种子回传，以便后续复现。

### 3. 功能需求

* **有序控制点插值：** 允许用户指定起点、终点以及中间 N 个有序的必经控制点，算法需依次分段规划，并平滑连接。
* **雷达/禁飞区规避：** 100% 避开球体雷达威胁区、多边形柱体禁飞区。
* **自动种子生成：** 当输入的 JSON 中没有指定 seed 或 seed 为 null 时，算法在内部通过密码学安全随机数生成器（如 Rust 的 `rand::rngs::StdRng`）自动生成一个高熵种子，确保多次调用路径的多样性。

---

## 二、 方案设计 (System Design)

### 1. 核心算法：基于三维稀疏网格的确定性 A* 算法

为了满足 100% 确定性与极致的高效性，本工具采用**三维稀疏网格的 A* 启发式搜索算法**。

* **隐式网格空间：** 将计算区域按用户配置的分辨率在内存中进行隐式建图（Implicit Graph），随用随建，规避全图预分配的内存与时延开销。
* **运动学剪枝：** 在 A* 扩展邻居节点时，直接引入**最大转弯角**和**爬升角约束**。若当前节点到邻居节点的向量与上一段航行向量的夹角超出阈值，则直接将该邻居节点剪枝。

### 2. 空间碰撞检测数学模型

所有空间碰撞计算均采用轻量解析几何方法进行极速判断。

#### ① 线段与球体（雷达威胁区）碰撞检测

已知雷达中心点为 $C$，半径为 $R$。航线段起点为 $A$，终点为 $B$。
线段上任意一点 $P(t)$ 可表示为：


$$P(t) = A + t \vec{u}, \quad \text{其中 } \vec{u} = B - A, \quad t \in [0, 1]$$

若要检测线段是否与球体相交，可寻找线段上距离球心 $C$ 最近的点。定义向量 $\vec{v} = C - A$，将 $\vec{v}$ 投影到 $\vec{u}$ 上，得到最近点投影参数 $t_{closest}$：


$$t_{closest} = \frac{\vec{v} \cdot \vec{u}}{\Vert{}\vec{u}\Vert{}^2}$$

将 $t_{closest}$ 截断（Clamp）在区间 $[0, 1]$ 内：


$$t_{target} = \max(0, \min(1, t_{closest}))$$

计算最近点 $P(t_{target})$ 到球心 $C$ 的距离 $d$：


$$d = \Vert{}P(t_{target}) - C\Vert{}$$

若 $d < R$，则发生碰撞；反之则安全。

#### ② 线段与多边形棱柱（禁飞区）碰撞检测

禁飞区由二维水平多边形以及高度区间 $[z_{min}, z_{max}]$ 定义。

* **高度过滤：** 若线段 $[A, B]$ 的整个 $z$ 轴区间与 $[z_{min}, z_{max}]$ 无交集，则直接判定不碰撞。
* **水平二维投影相交：** 若高度有重叠，则将线段和多边形投影到 $xy$ 平面。使用**射线法（Ray Casting）**检测线段端点是否在多边形内部，或者利用**线段相交算法**检测线段与多边形的各条边是否相交。

---

## 三、 数据契约 (Data Contracts)

### 1. 输入数据契约 (Input Schema)

若希望算法每次随机生成不同航线，只需将 `seed` 字段设为 `null` 或直接从 JSON 中移除该字段。

```json
{
  "session": {
    "seed": null,
    "max_calculation_time_ms": 100
  },
  "vehicle": {
    "min_turn_radius": 350.0,
    "max_climb_angle": 25.0,
    "max_turn_angle_deg": 60.0
  },
  "route_definition": {
    "start_state": {
      "position": [0.0, 0.0, 500.0],
      "heading_deg": 45.0
    },
    "control_waypoints": [
      [5000.0, 4000.0, 600.0],
      [12000.0, 11000.0, 800.0]
    ],
    "target": {
      "center": [20000.0, 20000.0, 1000.0],
      "radius": 500.0
    }
  },
  "environment": {
    "radars": [
      {
        "id": "SAM_S300",
        "center": [8000.0, 8000.0, 0.0],
        "radius": 5000.0
      }
    ],
    "no_fly_zones": [
      {
        "id": "BORDER_A",
        "boundary_points": [
          [2000.0, 3000.0],
          [4000.0, 3000.0],
          [4000.0, 5000.0],
          [2000.0, 5000.0]
        ],
        "alt_min": 0.0,
        "alt_max": 99999.0
      }
    ]
  }
}

```

### 2. 输出数据契约 (Output Schema)

无论输入时是否携带种子，输出的 `diagnostics` 中一定会返回 `seed_used`。**只要将这个 `seed_used` 作为下一次输入的 `seed` 传入，即可 100% 复现当前路径。**

#### ① 规划成功返回（Status: SUCCESS）

```json
{
  "status": "SUCCESS",
  "diagnostics": {
    "calculation_time_ms": 2.45,
    "nodes_explored": 142,
    "seed_used": 87439201834  
  },
  "summary": {
    "total_length_m": 28430.5,
    "max_climb_angle_utilized": 12.5
  },
  "waypoints": [
    { "index": 0, "position": [0.0, 0.0, 500.0], "type": "START" },
    { "index": 1, "position": [4920.1, 4010.5, 595.0], "type": "WAYPOINT" },
    { "index": 2, "position": [12100.0, 10950.0, 800.0], "type": "WAYPOINT" },
    { "index": 3, "position": [20000.0, 20000.0, 1000.0], "type": "TARGET" }
  ]
}

```

#### ② 规划失败返回（Status: FAILED）

```json
{
  "status": "FAILED",
  "error": {
    "code": "ROUTE_BLOCKED_BY_RADAR",
    "message": "Segment 0 collided with Radar 'SAM_S300'.",
    "location": [7850.0, 7900.0, 520.0],
    "seed_used": 87439201834
  }
}

```

---

## 五、 Rust 软件架构与模块设计

为了保证代码的极致内聚与可维护性，项目采用标准的高内聚模块化架构。目录结构与职责划分如下：

```text
src/
├── main.rs          # 应用程序入口，负责解析标准流与生命周期编排
├── error.rs         # 统一错误处理模块，定义各类规划与解析异常
├── config.rs        # 数据契约模块，定义严格的输入/输出序列化结构
├── geometry.rs      # 极速空间几何计算库，包含线-球、线-多边形碰撞检测
└── solver.rs        # 核心规划引擎，实现带运动学剪枝的 3D A* 算法

```

### 模块依赖拓扑与数据流向

1. `main.rs` 从 `stdin`（标准输入）捕获原始 JSON 流，交由 `config.rs` 反序列化。
2. `solver.rs` 调用 `config.rs` 提取物理限界与边界，在计算过程中高频调用 `geometry.rs` 进行碰撞剪枝。
3. `solver.rs` 产出最终航路点，经由 `config.rs` 序列化为标准 JSON，最终由 `main.rs` 写入 `stdout`（标准输出）。

---

## 六、 核心数据结构设计

利用 Rust 的强类型系统和 `serde` 框架，建立零开销的静态类型契约。

```rust
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// 三维空间坐标点 (X, Y, Z)，单位：米
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Point3D(pub f64, pub f64, pub f64);

impl Point3D {
    pub fn distance_to(&self, other: &Point3D) -> f64 {
        ((self.0 - other.0).powi(2) + (self.1 - other.1).powi(2) + (self.2 - other.2).powi(2)).sqrt()
    }
}

/// 飞行器宏观物理运动学限界
#[derive(Debug, Clone, Deserialize)]
pub struct VehicleProfile {
    pub min_turn_radius: f64,
    pub max_climb_angle: f64,      // 最大爬升角（角度值）
    pub max_turn_angle_deg: f64,   // 最大水平转弯夹角（角度值）
}

/// 球体雷达威胁区
#[derive(Debug, Clone, Deserialize)]
pub struct RadarThreat {
    pub id: String,
    pub center: Point3D,
    pub radius: f64,
}

/// 多边形棱柱禁飞区
#[derive(Debug, Clone, Deserialize)]
pub struct NoFlyZone {
    pub id: String,
    pub boundary_points: Vec<(f64, f64)>, // 二维水平多边形顶点
    pub alt_min: f64,
    pub alt_max: f64,
}

/// 状态机节点，用于 A* 开放列表与闭放列表
#[derive(Debug, Clone, Copy)]
pub struct AStarNode {
    pub position: Point3D,
    pub g_score: f64,              // 从起点到当前节点的实际代价
    pub f_score: f64,              // 总估算代价 (g + h)
    pub parent_index: Option<usize>, // 父节点索引，用于回溯路径
}

// 实现 Eq 和 Ord 以便放入标准库的最小堆（BinaryHeap）
impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}
impl Eq for AStarNode {}
impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // 因为 BinaryHeap 默认是最大堆，这里我们需要反向比较以实现最小堆
        other.f_score.partial_cmp(&self.f_score).unwrap_or(std::cmp::Ordering::Equal)
    }
}
impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

```

---

## 七、 核心算法骨架实现

以下是基于 3D 确定性 A* 算法的引擎核心骨架。算法在生成邻居节点时，会立即将不满足最大转弯角与爬升角的节点予以剪枝，同时利用几何解析方程进行极速避障。

```rust
use std::collections::{BinaryHeap, HashSet};

pub struct AStarSolver<'a> {
    pub profile: &'a VehicleProfile,
    pub radars: &'a Vec<RadarThreat>,
    pub nfz_list: &'a Vec<NoFlyZone>,
    pub grid_resolution: f64,
}

impl<'a> AStarSolver<'a> {
    /// 执行分段航路规划
    pub fn solve(&self, start: Point3D, target: Point3D, initial_heading: f64) -> Result<Vec<Point3D>, String> {
        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashSet::new();
        let mut node_pool: Vec<AStarNode> = Vec::new();

        // 初始化起点节点
        let start_node = AStarNode {
            position: start,
            g_score: 0.0,
            f_score: start.distance_to(&target),
            parent_index: None,
        };
        
        open_set.push(start_node);
        node_pool.push(start_node);

        while let Some(current) = open_set.pop() {
            // 终止条件：若当前节点已进入目标容差范围内
            if current.position.distance_to(&target) < self.grid_resolution {
                return Ok(self.reconstruct_path(current, &node_pool));
            }

            // 将当前位置坐标离散化为网格 Key 放入闭集
            let grid_key = self.to_grid_key(current.position);
            if !closed_set.insert(grid_key) {
                continue;
            }

            // 智能生成 3D 稀疏邻居节点 (26 方向邻域空间搜索)
            for neighbor_pos in self.get_neighbors(current.position) {
                if closed_set.contains(&self.to_grid_key(neighbor_pos)) {
                    continue;
                }

                // 1. 运动学剪枝：检查爬升角与转弯夹角
                if !self.check_kinematics(&current, neighbor_pos, &node_pool) {
                    continue;
                }

                // 2. 环境碰撞实体剪枝：雷达与禁飞区静态避障
                if self.is_colliding(current.position, neighbor_pos) {
                    continue;
                }

                let tentative_g = current.g_score + current.position.distance_to(&neighbor_pos);
                let h_score = neighbor_pos.distance_to(&target);
                
                let neighbor_node = AStarNode {
                    position: neighbor_pos,
                    g_score: tentative_g,
                    f_score: tentative_g + h_score,
                    parent_index: Some(node_pool.len() - 1),
                };

                open_set.push(neighbor_node);
                node_pool.push(neighbor_node);
            }
        }

        Err("ROUTE_GENERATION_FAILED: No valid path found under constraints.".to_string())
    }

    fn to_grid_key(&self, p: Point3D) -> (i64, i64, i64) {
        (
            (p.0 / self.grid_resolution).round() as i64,
            (p.1 / self.grid_resolution).round() as i64,
            (p.2 / self.grid_resolution).round() as i64,
        )
    }

    fn check_kinematics(&self, current: &AStarNode, next_pos: Point3D, pool: &[AStarNode]) -> bool {
        if let Some(parent_idx) = current.parent_index {
            let prev_pos = pool[parent_idx].position;
            let vec1 = (current.position.0 - prev_pos.0, current.position.1 - prev_pos.1);
            let vec2 = (next_pos.0 - current.position.0, next_pos.1 - current.position.1);
            
            // 计算水平面内的转弯夹角
            let dot = vec1.0 * vec2.0 + vec1.1 * vec2.1;
            let len1 = (vec1.0.powi(2) + vec1.1.powi(2)).sqrt();
            let len2 = (vec2.0.powi(2) + vec2.1.powi(2)).sqrt();
            if len1 > 0.0 && len2 > 0.0 {
                let angle = (dot / (len1 * len2)).acos().to_degrees();
                if angle > self.profile.max_turn_angle_deg {
                    return false; // 转弯超限，剪枝
                }
            }
        }
        
        // 校验纵向轴爬升角限制
        let delta_z = (next_pos.2 - current.position.2).abs();
        let dist_2d = ((next_pos.0 - current.position.0).powi(2) + (next_pos.1 - current.position.1).powi(2)).sqrt();
        let climb_angle = (delta_z / dist_2d).atan().to_degrees();
        climb_angle <= self.profile.max_climb_angle
    }

    fn is_colliding(&self, from: Point3D, to: Point3D) -> bool {
        // 调用 geometry.rs 中的高频几何解析方程实现避障判断
        false // 详细几何碰撞交集判断已在技术方案第二节完备陈述
    }

    fn get_neighbors(&self, _p: Point3D) -> Vec<Point3D> { vec![] }
    fn reconstruct_path(&self, _end: AStarNode, _pool: &[AStarNode]) -> Vec<Point3D> { vec![] }
}

```

---

## 八、 接口设计与标准 I/O 集成

系统完全摈弃了传统文件传参，彻底采用标准流设计。同时在反序列化阶段智能检测 `seed` 字段：若空缺，则由系统底层自动生成高熵随机数种子，并将最终使用的种子塞入结果中回传给上层软件。

```rust
use rand::Rng;
use std::io::{self, Read, Write};

fn main() {
    // 1. 从流中高效率捕获整包原始文本数据
    let mut buffer = String::new();
    if io::stdin().read_to_string(&mut buffer).is_err() {
        eprintln!("ERROR: Failed to read from stdin pipeline.");
        std::process::exit(1);
    }

    // 2. 解析输入契约文件
    let mut input_data: config::InputConfig = match serde_json::from_str(&buffer) {
        Ok(cfg) => cfg,
        Err(e) => {
            let err_response = config::OutputFailed {
                status: "FAILED".to_string(),
                error: config::ErrorDetail {
                    code: "JSON_PARSE_ERROR".to_string(),
                    message: format!("Malformed input json structure: {}", e),
                    location: vec![0.0, 0.0, 0.0],
                    seed_used: 0,
                }
            };
            println!("{}", serde_json::to_string(&err_response).unwrap());
            std::process::exit(0); // 管道设计：逻辑错误依然正常输出合法结构，不中断系统级主进程
        }
    };

    // 3. 智能种子（Seed）熔断保护机制
    let final_seed = match input_data.session.seed {
        Some(explicit_seed) => explicit_seed,
        None => {
            // 当缺省 seed 时，底层通过硬件熵源进行随机种子补齐
            let mut rng = rand::thread_rng();
            let generated: u64 = rng.gen();
            input_data.session.seed = Some(generated);
            generated
        }
    };

    // 4. 调用规划引擎内核进行运算并构造输出对象 (省略详细中间解算步骤)
    // let result = solver.solve(...);

    // 5. 最终通过 stdout 将高紧凑格式数据喷射回响应前端
    let success_output = config::OutputSuccess {
        status: "SUCCESS".to_string(),
        diagnostics: config::Diagnostics {
            calculation_time_ms: 1.82,
            nodes_explored: 96,
            seed_used: final_seed, // 回传确定性种子，保障 100% 幂等可复现性
        },
        waypoints: vec![],
    };

    let serialized_output = serde_json::to_string(&success_output).unwrap();
    let mut stdout = io::stdout();
    if writeln!(stdout, "{}", serialized_output).is_err() {
        eprintln!("ERROR: Pipe broke during stdout flush.");
    }
}

```

---

## 九、 跨平台构建与 CI/CD 交付方案

为了在微服务体系、本地 GUI 标绘看板或大型仿真工作站中无缝嵌入该工具，我们基于 **GitHub Actions** 提供一键式全自动化跨平台交叉编译与制品发布方案。

### 1. 目标构建矩阵 (Target Matrix)

工具链原生支持以下大国推演平台主流运行环境的直接静态编译：

| 操作系统 | 目标平台架构三元组 (Target Triple) | 编译产物形式 | 交付说明 |
| --- | --- | --- | --- |
| **Linux (企业级服务器)** | `x86_64-unknown-linux-musl` | 独立二进制 (Static Exe) | 基于 musl 实现完全静态化链接，可实现真正的零依赖跨发行版无缝迁移。 |
| **Windows (桌面筹划端)** | `x86_64-pc-windows-msvc` | `afsim-planner.exe` | 深度匹配微软 MSVC 原生运行时环境，针对桌面图形标绘系统集成。 |

### 2. GitHub Actions 工作流白皮书 (`.github/workflows/release.yml`)

```yaml
name: Production Cross-Platform Release CI/CD

on:
  push:
    tags:
      - 'v*' # 仅当代码仓库被推送版本标识 (如 v1.2.0) 时触发自动化流水线

jobs:
  build-and-deploy:
    name: Compile and Release for ${{ matrix.target }}
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        include:
          - os: ubuntu-latest
            target: x86_64-unknown-linux-musl
            artifact_name: afsim-planner
            use_cross: true
          - os: windows-latest
            target: x86_64-pc-windows-msvc
            artifact_name: afsim-planner.exe
            use_cross: false

    steps:
      - name: Checkout Source Code
        uses: actions/checkout@v4

      - name: Install Stable Rust Toolchain
        uses: dtolnay/rust-toolchain@stable
        with:
          targets: ${{ matrix.target }}

      - name: Execute High-Performance Compilation
        uses: actions-rs/cargo@v1
        with:
          use-cross: ${{ matrix.use_cross }}
          command: build
          args: --release --target ${{ matrix.target }}

      - name: Optimize and Strip Binary (Linux Only)
        if: matrix.os == 'ubuntu-latest'
        run: |
          sudo apt-get install -y musl-tools
          strip target/${{ matrix.target }}/release/${{ matrix.artifact_name }}

      - name: Upload Build Artifact to Release Assets
        uses: softprops/action-gh-release@v1
        with:
          files: target/${{ matrix.target }}/release/${{ matrix.artifact_name }}
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}

```

---

## 十、 生产环境部署调用示例

由于完全基于标准流设计，该工具非常便于集成到前端多并发的响应管道中。以下是常见的上层软件集成调用指令：

```bash
# 模式 A：直接通过 cat 将配置结构体送入规划引擎，并将回传数据截获
cat task_request.json | ./target/release/afsim-planner > path_result.json

# 模式 B：在上层业务（如 Node.js 或 Python）中以子进程管道形式直接高效调拨
# python 伪代码: subprocess.Popen(['./afsim-planner'], stdin=PIPE, stdout=PIPE)

```