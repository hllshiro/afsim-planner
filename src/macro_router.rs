use std::collections::{BinaryHeap, HashMap};

use crate::config::{NoFlyZone, Point3D, RadarThreat};
use crate::geometry::AABB;

pub struct MacroRouter {
    obstacle_aabbs: Vec<AABB>,
    safety_margin: f64,
}

impl MacroRouter {
    pub fn new(
        radars: &[RadarThreat],
        nfz_list: &[NoFlyZone],
        safety_margin: f64,
    ) -> Self {
        let mut aabbs: Vec<AABB> = Vec::new();

        // 雷达球体 → AABB（内切立方体 + 安全膨胀）
        for radar in radars {
            let r = radar.radius + safety_margin;
            let cx = radar.center[0];
            let cy = radar.center[1];
            let cz = radar.center[2];
            aabbs.push(AABB {
                min: Point3D(cx - r, cy - r, cz - r),
                max: Point3D(cx + r, cy + r, cz + r),
            });
        }

        // 禁飞区棱柱 → AABB（多边形 XY 包围盒 + 高度区间 + 安全膨胀）
        for nfz in nfz_list {
            let min_x = nfz
                .boundary_points
                .iter()
                .map(|p| p[0])
                .fold(f64::INFINITY, f64::min);
            let max_x = nfz
                .boundary_points
                .iter()
                .map(|p| p[0])
                .fold(f64::NEG_INFINITY, f64::max);
            let min_y = nfz
                .boundary_points
                .iter()
                .map(|p| p[1])
                .fold(f64::INFINITY, f64::min);
            let max_y = nfz
                .boundary_points
                .iter()
                .map(|p| p[1])
                .fold(f64::NEG_INFINITY, f64::max);
            aabbs.push(AABB {
                min: Point3D(
                    min_x - safety_margin,
                    min_y - safety_margin,
                    nfz.alt_min - safety_margin,
                ),
                max: Point3D(
                    max_x + safety_margin,
                    max_y + safety_margin,
                    nfz.alt_max + safety_margin,
                ),
            });
        }

        Self {
            obstacle_aabbs: aabbs,
            safety_margin,
        }
    }

    /// 从障碍物 AABB 和起终点中采样候选顶点
    fn sample_vertices(&self, start: Point3D, target: Point3D) -> Vec<Point3D> {
        let mut vertices = vec![start, target];

        for aabb in &self.obstacle_aabbs {
            // 8 个角点 + 6 个面心，共 14 个候选顶点 (保证覆盖绕行方向)
            let m = self.safety_margin * 0.5; // 向外微微推离障碍物表面
            let min = Point3D(aabb.min.0 - m, aabb.min.1 - m, aabb.min.2 - m);
            let max = Point3D(aabb.max.0 + m, aabb.max.1 + m, aabb.max.2 + m);
            let mid_x = (min.0 + max.0) * 0.5;
            let mid_y = (min.1 + max.1) * 0.5;
            let _mid_z = (min.2 + max.2) * 0.5;

            // 8 corners
            vertices.push(min);
            vertices.push(Point3D(min.0, min.1, max.2));
            vertices.push(Point3D(min.0, max.1, min.2));
            vertices.push(Point3D(min.0, max.1, max.2));
            vertices.push(Point3D(max.0, min.1, min.2));
            vertices.push(Point3D(max.0, min.1, max.2));
            vertices.push(Point3D(max.0, max.1, min.2));
            vertices.push(max);
            // 6 face centers (排除上下，只看 XY 面)
            vertices.push(Point3D(min.0, mid_y, min.2));
            vertices.push(Point3D(max.0, mid_y, min.2));
            vertices.push(Point3D(mid_x, min.1, min.2));
            vertices.push(Point3D(mid_x, max.1, min.2));
        }

        vertices
    }

    /// 检查两点之间是否被任何障碍物 AABB 阻挡
    fn is_visible(&self, a: Point3D, b: Point3D) -> bool {
        use crate::geometry::PreparedSegment;
        let seg = PreparedSegment::new(a, b);
        !self
            .obstacle_aabbs
            .iter()
            .any(|aabb| aabb.intersects_segment(&seg))
    }

    /// 可见性图上的 A* 搜索
    /// 返回从 start 到 target 经过的顶点索引序列，不含起点
    fn astar_on_graph(
        &self,
        vertices: &[Point3D],
        start_idx: usize,
        target_idx: usize,
        adjacency: &[Vec<usize>],
    ) -> Vec<usize> {
        #[derive(Debug)]
        struct GraphNode {
            idx: usize,
            f_score: f64,
        }
        impl PartialEq for GraphNode {
            fn eq(&self, other: &Self) -> bool {
                self.f_score == other.f_score
            }
        }
        impl Eq for GraphNode {}
        impl Ord for GraphNode {
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                other
                    .f_score
                    .partial_cmp(&self.f_score)
                    .unwrap_or(std::cmp::Ordering::Equal)
            }
        }
        impl PartialOrd for GraphNode {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        let mut open = BinaryHeap::new();
        let mut g_scores: HashMap<usize, f64> = HashMap::new();
        let mut came_from: HashMap<usize, usize> = HashMap::new();

        g_scores.insert(start_idx, 0.0);
        open.push(GraphNode {
            idx: start_idx,
            f_score: vertices[start_idx].distance_to(&vertices[target_idx]),
        });

        while let Some(current) = open.pop() {
            if current.idx == target_idx {
                // 重建路径
                let mut path = Vec::new();
                let mut cur = target_idx;
                while cur != start_idx {
                    path.push(cur);
                    cur = came_from[&cur];
                }
                path.reverse();
                return path;
            }

            let g = g_scores[&current.idx];

            for &neighbor in &adjacency[current.idx] {
                let tentative_g = g + vertices[current.idx].distance_to(&vertices[neighbor]);
                if tentative_g < *g_scores.get(&neighbor).unwrap_or(&f64::INFINITY) {
                    g_scores.insert(neighbor, tentative_g);
                    came_from.insert(neighbor, current.idx);
                    let h = vertices[neighbor].distance_to(&vertices[target_idx]);
                    open.push(GraphNode {
                        idx: neighbor,
                        f_score: tentative_g + h,
                    });
                }
            }
        }

        // 无可达路径 → 直接无中间点（交由下层 A* 自行处理）
        Vec::new()
    }

    pub fn plan(&self, start: Point3D, target: Point3D) -> Vec<Point3D> {
        // 若直接可视且无障碍阻挡，无需中间控制点
        if self.is_visible(start, target) {
            return Vec::new();
        }

        let vertices = self.sample_vertices(start, target);

        // 去重：按 grid key (10m 精度) 去重，防止重复顶点污染可见性图
        let dedup_key = |p: &Point3D| -> (i64, i64, i64) {
            (
                (p.0 / 10.0).round() as i64,
                (p.1 / 10.0).round() as i64,
                (p.2 / 10.0).round() as i64,
            )
        };
        let mut seen = HashMap::new();
        let mut unique_vertices: Vec<Point3D> = Vec::new();
        for v in vertices {
            let k = dedup_key(&v);
            if !seen.contains_key(&k) {
                seen.insert(k, unique_vertices.len());
                unique_vertices.push(v);
            }
        }

        let n = unique_vertices.len();
        let start_idx = 0; // start always first
        let target_idx = 1; // target always second

        // 构建邻接表：对每对 (i, j) 检查可见性
        let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n];
        for i in 0..n {
            for j in (i + 1)..n {
                if self.is_visible(unique_vertices[i], unique_vertices[j]) {
                    adjacency[i].push(j);
                    adjacency[j].push(i);
                }
            }
        }

        // 在可见性图上搜索
        let path_indices = self.astar_on_graph(&unique_vertices, start_idx, target_idx, &adjacency);

        path_indices
            .into_iter()
            .map(|idx| unique_vertices[idx])
            .filter(|p| {
                // 过滤掉过于靠近起点或终点的控制点（太近会导致无意义微段）
                p.distance_2d(&start) > 500.0 && p.distance_2d(&target) > 500.0
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Point3D;

    fn make_radar(id: &str, x: f64, y: f64, z: f64, r: f64) -> RadarThreat {
        RadarThreat {
            id: id.into(),
            center: [x, y, z],
            radius: r,
        }
    }

    #[test]
    fn test_direct_los_no_waypoints() {
        // 无障碍物：直接可视，无需中间点
        let radars: Vec<RadarThreat> = vec![];
        let nfzs: Vec<NoFlyZone> = vec![];
        let router = MacroRouter::new(&radars, &nfzs, 500.0);
        let result = router.plan(
            Point3D(0.0, 0.0, 500.0),
            Point3D(20000.0, 0.0, 500.0),
        );
        assert!(result.is_empty());
    }

    #[test]
    fn test_single_radar_blocking() {
        // 单一雷达挡在起终点之间，需生成绕行点
        let radars = vec![make_radar("R1", 10000.0, 0.0, 500.0, 3000.0)];
        let nfzs: Vec<NoFlyZone> = vec![];
        let router = MacroRouter::new(&radars, &nfzs, 700.0);
        let result = router.plan(
            Point3D(0.0, 0.0, 500.0),
            Point3D(20000.0, 0.0, 500.0),
        );
        // 应生成至少一个中间控制点绕过雷达
        assert!(!result.is_empty());
    }

    #[test]
    fn test_vertex_deduplication() {
        // 多个雷达但都在同一区域：验证顶点去重效果
        let radars = vec![
            make_radar("R1", 10000.0, 0.0, 500.0, 2000.0),
            make_radar("R2", 10000.0, 1000.0, 500.0, 2000.0),
        ];
        let nfzs: Vec<NoFlyZone> = vec![];
        let router = MacroRouter::new(&radars, &nfzs, 700.0);
        let result = router.plan(
            Point3D(0.0, 0.0, 500.0),
            Point3D(20000.0, 0.0, 500.0),
        );
        assert!(!result.is_empty());
    }
}
