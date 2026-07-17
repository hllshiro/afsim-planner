use std::collections::{BinaryHeap, HashMap};
use std::time::Instant;

use crate::config::{
    Environment, NoFlyZone, Point3D, RadarThreat, VehicleProfile,
};
use crate::geometry::{self, BoundingCorridor};

// ============================================================================
// Seed-based noise perturbation (FNV-1a hash)
// ============================================================================

/// FNV-1a 64-bit hash mixing grid index with seed → deterministic [0.0, 1.0).
/// Produces a spatially-smooth anisotropic noise field that is consistent
/// across platforms and completely determined by (ix, iy, iz, seed).
#[inline(always)]
fn coordinate_noise(ix: i64, iy: i64, iz: i64, seed: u64) -> f64 {
    const FNV_PRIME: u64 = 0x00000100000001B3;
    const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;

    let mut hash = FNV_OFFSET_BASIS;
    hash ^= seed;
    hash = hash.wrapping_mul(FNV_PRIME);
    for &val in &[ix, iy, iz] {
        hash ^= val as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }

    // Map high 53 bits into [0.0, 1.0) without expensive division
    let bits = (hash >> 11) | 0x3FF0000000000000;
    f64::from_bits(bits) - 1.0
}

// ============================================================================
// A* node
// ============================================================================

#[derive(Debug, Clone, Copy)]
struct Node {
    position: Point3D,
    g_score: f64,
    f_score: f64,
    parent_index: Option<usize>,
    /// Cached noise for deterministic tie-breaking when f_scores are equal
    noise_weight: f64,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score && self.noise_weight == other.noise_weight
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
            .then_with(|| {
                // Deterministic tie-break via seed-derived noise weight
                other
                    .noise_weight
                    .partial_cmp(&self.noise_weight)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
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
    /// Perturbation strength applied to heuristic (0.02–0.05).
    /// Different seeds produce different anisotropic "resistance fields",
    /// yielding topologically distinct but equally optimal paths.
    pub perturbation_strength: f64,
}

impl AStarSolver {
    pub fn new(
        profile: VehicleProfile,
        env: Environment,
        grid_resolution: f64,
        max_calculation_time_ms: Option<u64>,
        seed: u64,
        perturbation_strength: f64,
    ) -> Self {
        Self {
            profile,
            radars: env.radars,
            nfz_list: env.no_fly_zones,
            grid_resolution,
            max_calculation_time_ms,
            seed,
            perturbation_strength,
        }
    }

    /// Core A* search. `corridor_radius` constrains search space; `None` means unconstrained.
    fn search_impl(
        &self,
        start: Point3D,
        target: Point3D,
        corridor_radius: Option<f64>,
    ) -> Result<(Vec<Point3D>, u64), String> {
        let t0 = Instant::now();

        let mut open_set = BinaryHeap::new();
        let mut closed_set: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut node_pool: Vec<Node> = Vec::new();

        let start_node = Node {
            position: start,
            g_score: 0.0,
            f_score: start.distance_to(&target),
            parent_index: None,
            noise_weight: {
                let (sx, sy, sz) = self.to_grid_key(start);
                coordinate_noise(sx, sy, sz, self.seed)
            },
        };
        open_set.push(start_node);
        node_pool.push(start_node);

        let mut nodes_explored: u64 = 0;
        let mut min_h = start.distance_to(&target);
        let mut stagnation_count: usize = 0;
        let mut active_weight: f64 = 1.5;

        let corridor = corridor_radius
            .map(|r| BoundingCorridor::from_segment(start, target, r));

        while let Some(current) = open_set.pop() {
            nodes_explored += 1;
            if let Some(max_ms) = self.max_calculation_time_ms {
                if t0.elapsed().as_millis() as u64 > max_ms {
                    return Err("ROUTE_GENERATION_FAILED: Max calculation time exceeded.".into());
                }
            }

            // Stagnation monitor
            let current_h = current.position.distance_to(&target);
            if current_h < min_h {
                min_h = current_h;
                stagnation_count = 0;
                active_weight = 1.5;
            } else {
                stagnation_count += 1;
            }
            if stagnation_count > 120 {
                active_weight = 1.0;
            }

            // Termination
            if current.position.distance_to(&target) <= self.grid_resolution {
                let mut path = self.reconstruct_path(&current, &node_pool);
                path.push(target);
                return Ok((path, nodes_explored));
            }

            let grid_key = self.to_grid_key(current.position);
            if closed_set.contains_key(&grid_key) {
                continue;
            }
            closed_set.insert(grid_key, node_pool.len() - 1);

            for neighbor_pos in self.get_neighbors(current.position) {
                let grid_key = self.to_grid_key(neighbor_pos);
                if closed_set.contains_key(&grid_key) {
                    continue;
                }

                // Corridor prune — first filter (cheapest check)
                if let Some(ref cor) = corridor {
                    if !cor.is_inside(&neighbor_pos) {
                        continue;
                    }
                }

                if !self.check_kinematics(&current, neighbor_pos, &node_pool) {
                    continue;
                }

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
                // Perturbed heuristic: seed-driven anisotropic noise breaks symmetry
                let raw_h = neighbor_pos.distance_to(&target);
                let noise = coordinate_noise(grid_key.0, grid_key.1, grid_key.2, self.seed);
                let perturbed_h = raw_h * (1.0 + self.perturbation_strength * noise);
                let f_score = tentative_g + active_weight * perturbed_h;

                let neighbor_node = Node {
                    position: neighbor_pos,
                    g_score: tentative_g,
                    f_score,
                    parent_index: Some(self.find_node_index(&current, &node_pool)),
                    noise_weight: noise,
                };

                open_set.push(neighbor_node);
                node_pool.push(neighbor_node);
            }
        }

        Err("ROUTE_GENERATION_FAILED: No valid path found under constraints.".into())
    }

    /// Solve a single segment from `start` to `target`, respecting initial heading.
    /// Returns (path, nodes_explored) on success.
    ///
    /// Two-stage strategy: first try with 5000m corridor constraint; if no path is
    /// found, fall back to unconstrained search.
    pub fn solve_segment(
        &self,
        start: Point3D,
        target: Point3D,
        _initial_heading: f64,
    ) -> Result<(Vec<Point3D>, u64), String> {
        // Stage 1: constrained search within 5000m corridor
        if let Ok(result) = self.search_impl(start, target, Some(5000.0)) {
            return Ok(result);
        }
        // Stage 2: fallback — unconstrained search
        self.search_impl(start, target, None)
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

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

    // ------------------------------------------------------------------
    // Waypoint pruning (path shortcutter)
    // ------------------------------------------------------------------

    /// Check if a straight-line segment collides with any environmental obstacle.
    pub fn is_colliding(&self, from: Point3D, to: Point3D) -> bool {
        geometry::segment_vs_environment(
            [from.0, from.1, from.2],
            [to.0, to.1, to.2],
            &self.radars,
            &self.nfz_list,
        )
    }

    /// Kinematic constraint check for a straight-line segment between two waypoints.
    /// `incoming` is the waypoint preceding `from` (used for turn-angle check).
    fn check_segment_kinematics(
        &self,
        incoming: Option<Point3D>,
        from: Point3D,
        to: Point3D,
    ) -> bool {
        // Climb angle
        let delta_z = (to.2 - from.2).abs();
        let dist_2d = from.distance_2d(&to);
        if dist_2d > 0.0 {
            let climb_angle = (delta_z / dist_2d).atan().to_degrees();
            if climb_angle > self.profile.max_climb_angle {
                return false;
            }
        }

        // Turn angle: check the bend at `from` between incoming and outgoing direction
        if let Some(prev) = incoming {
            let vec1 = (from.0 - prev.0, from.1 - prev.1);
            let vec2 = (to.0 - from.0, to.1 - from.1);

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

    /// Greedy double-pointer waypoint pruning.
    ///
    /// Eliminates redundant collinear grid points from an A* path by trying to
    /// "shortcut" from each waypoint to the furthest reachable waypoint that
    /// is collision-free and kinematically valid. Reduces hundreds of grid
    /// points down to a handful of critical maneuver waypoints.
    pub fn prune_waypoints(&self, raw_path: &[Point3D]) -> Vec<Point3D> {
        if raw_path.len() <= 2 {
            return raw_path.to_vec();
        }

        let mut pruned: Vec<Point3D> = Vec::new();
        pruned.push(raw_path[0]);

        let mut start_idx = 0;
        while start_idx < raw_path.len() - 1 {
            let mut best_target_idx = start_idx + 1;

            for test_idx in (start_idx + 2)..raw_path.len() {
                let from = raw_path[start_idx];
                let to = raw_path[test_idx];

                let incoming = if pruned.len() >= 2 {
                    Some(pruned[pruned.len() - 2])
                } else {
                    None
                };

                if !self.is_colliding(from, to)
                    && self.check_segment_kinematics(incoming, from, to)
                {
                    best_target_idx = test_idx;
                } else {
                    break;
                }
            }

            pruned.push(raw_path[best_target_idx]);
            start_idx = best_target_idx;
        }

        pruned
    }
}
