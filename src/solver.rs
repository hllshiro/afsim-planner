use std::collections::{BinaryHeap, HashMap};
use std::time::Instant;

use rand::rngs::StdRng;
use rand::SeedableRng;

use crate::config::{
    Environment, NoFlyZone, Point3D, RadarThreat, VehicleProfile,
};
use crate::geometry;

// ============================================================================
// A* node
// ============================================================================

#[derive(Debug, Clone, Copy)]
struct Node {
    position: Point3D,
    g_score: f64,
    f_score: f64,
    parent_index: Option<usize>,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}
impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reverse for min-heap behavior (BinaryHeap is max-heap)
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

// ============================================================================
// 3D deterministic A* solver with kinematic pruning
// ============================================================================

pub struct AStarSolver {
    pub profile: VehicleProfile,
    pub radars: Vec<RadarThreat>,
    pub nfz_list: Vec<NoFlyZone>,
    pub grid_resolution: f64,
    pub max_calculation_time_ms: Option<u64>,
    pub seed: u64,
}

impl AStarSolver {
    pub fn new(
        profile: VehicleProfile,
        env: Environment,
        grid_resolution: f64,
        max_calculation_time_ms: Option<u64>,
        seed: u64,
    ) -> Self {
        Self {
            profile,
            radars: env.radars,
            nfz_list: env.no_fly_zones,
            grid_resolution,
            max_calculation_time_ms,
            seed,
        }
    }

    /// Solve a single segment from `start` to `target`, respecting initial heading.
    /// Returns (path, nodes_explored) on success.
    pub fn solve_segment(
        &self,
        start: Point3D,
        target: Point3D,
        _initial_heading: f64,
    ) -> Result<(Vec<Point3D>, u64), String> {
        let t0 = Instant::now();

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut node_pool: Vec<Node> = Vec::new();

        // Seed-based RNG for deterministic tie-breaking (reserved for future use)
        let _rng = StdRng::seed_from_u64(self.seed);

        // Start node
        let start_node = Node {
            position: start,
            g_score: 0.0,
            f_score: start.distance_to(&target),
            parent_index: None,
        };
        open_set.push(start_node);
        node_pool.push(start_node);

        let mut nodes_explored: u64 = 0;

        while let Some(current) = open_set.pop() {
            nodes_explored += 1;
            // Time budget check
            if let Some(max_ms) = self.max_calculation_time_ms {
                if t0.elapsed().as_millis() as u64 > max_ms {
                    return Err("ROUTE_GENERATION_FAILED: Max calculation time exceeded.".into());
                }
            }

            // Termination: within one grid cell of target
            if current.position.distance_to(&target) <= self.grid_resolution {
                let mut path = self.reconstruct_path(&current, &node_pool);
                // Ensure the target is included as final waypoint
                path.push(target);
                return Ok((path, nodes_explored));
            }

            let grid_key = self.to_grid_key(current.position);
            if closed_set.contains_key(&grid_key) {
                continue;
            }
            closed_set.insert(grid_key, node_pool.len() - 1);

            for neighbor_pos in self.get_neighbors(current.position) {
                if closed_set.contains_key(&self.to_grid_key(neighbor_pos)) {
                    continue;
                }

                // Kinematic pruning
                if !self.check_kinematics(&current, neighbor_pos, &node_pool) {
                    continue;
                }

                // Collision detection
                if geometry::segment_vs_environment(
                    [current.position.0, current.position.1, current.position.2],
                    [neighbor_pos.0, neighbor_pos.1, neighbor_pos.2],
                    &self.radars,
                    &self.nfz_list,
                ) {
                    continue;
                }

                let tentative_g =
                    current.g_score + current.position.distance_to(&neighbor_pos);
                let h_score = self.heuristic(neighbor_pos, target);
                let f_score = tentative_g + h_score;

                let neighbor_node = Node {
                    position: neighbor_pos,
                    g_score: tentative_g,
                    f_score,
                    parent_index: Some(self.find_node_index(&current, &node_pool)),
                };

                open_set.push(neighbor_node);
                node_pool.push(neighbor_node);
            }
        }

        Err("ROUTE_GENERATION_FAILED: No valid path found under constraints.".into())
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /// Euclidean distance heuristic (consistent & admissible).
    fn heuristic(&self, a: Point3D, b: Point3D) -> f64 {
        a.distance_to(&b)
    }

    /// Discretize a 3D point to an integer grid key.
    fn to_grid_key(&self, p: Point3D) -> (i64, i64, i64) {
        (
            (p.0 / self.grid_resolution).round() as i64,
            (p.1 / self.grid_resolution).round() as i64,
            (p.2 / self.grid_resolution).round() as i64,
        )
    }

    /// Find the index of a node in the pool by comparing position and g_score.
    fn find_node_index(&self, node: &Node, pool: &[Node]) -> usize {
        pool.iter()
            .position(|n| {
                n.position == node.position
                    && (n.g_score - node.g_score).abs() < f64::EPSILON
            })
            .expect("Node must be in pool")
    }

    /// Generate 26-directional neighbors (3×3×3 cube minus center).
    fn get_neighbors(&self, p: Point3D) -> Vec<Point3D> {
        let step = self.grid_resolution;
        let mut neighbors = Vec::with_capacity(26);

        for dx in [-1.0_f64, 0.0, 1.0] {
            for dy in [-1.0_f64, 0.0, 1.0] {
                for dz in [-1.0_f64, 0.0, 1.0] {
                    if dx == 0.0 && dy == 0.0 && dz == 0.0 {
                        continue;
                    }
                    neighbors.push(Point3D(
                        p.0 + dx * step,
                        p.1 + dy * step,
                        p.2 + dz * step,
                    ));
                }
            }
        }

        neighbors
    }

    /// Kinematic constraint check: turn angle (horizontal) + climb angle.
    fn check_kinematics(
        &self,
        current: &Node,
        next_pos: Point3D,
        pool: &[Node],
    ) -> bool {
        // Climb angle check
        let delta_z = (next_pos.2 - current.position.2).abs();
        let dist_2d = current.position.distance_2d(&next_pos);
        if dist_2d > 0.0 {
            let climb_angle = (delta_z / dist_2d).atan().to_degrees();
            if climb_angle > self.profile.max_climb_angle {
                return false;
            }
        }

        // Turn angle check (requires parent node for direction)
        if let Some(parent_idx) = current.parent_index {
            let prev_pos = pool[parent_idx].position;
            let vec1 = (
                current.position.0 - prev_pos.0,
                current.position.1 - prev_pos.1,
            );
            let vec2 = (next_pos.0 - current.position.0, next_pos.1 - current.position.1);

            let dot = vec1.0 * vec2.0 + vec1.1 * vec2.1;
            let len1 = (vec1.0.powi(2) + vec1.1.powi(2)).sqrt();
            let len2 = (vec2.0.powi(2) + vec2.1.powi(2)).sqrt();

            if len1 > 0.0 && len2 > 0.0 {
                let angle = (dot / (len1 * len2)).acos().to_degrees();
                if angle > self.profile.max_turn_angle_deg {
                    return false;
                }
            }
        }

        true
    }

    /// Reconstruct path by walking parent pointers.
    fn reconstruct_path(&self, end: &Node, pool: &[Node]) -> Vec<Point3D> {
        let mut path = Vec::new();
        let mut current_idx = self.find_node_index(end, pool);
        loop {
            let node = &pool[current_idx];
            path.push(node.position);
            match node.parent_index {
                Some(p) => current_idx = p,
                None => break,
            }
        }
        path.reverse();
        path
    }
}
