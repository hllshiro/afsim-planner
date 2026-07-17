/// Fast spatial collision detection for 3D route planning.
///
/// Provides two collision primitives:
/// 1. Line segment vs sphere   (radar threats)
/// 2. Line segment vs polygon prism (no-fly zones)

use crate::config::{NoFlyZone, Point3D, RadarThreat};

// ============================================================================
// Line segment ↔ Sphere collision (radar threat)
// ============================================================================

/// Returns `true` if the line segment [from, to] intersects the radar sphere.
///
/// Mathematically: find the closest point on the segment to the sphere center,
/// then check if distance < radius.
pub fn segment_vs_sphere(from: [f64; 3], to: [f64; 3], radar: &RadarThreat) -> bool {
    let c = radar.center;
    let r = radar.radius;

    let ux = to[0] - from[0];
    let uy = to[1] - from[1];
    let uz = to[2] - from[2];
    let u_sq = ux * ux + uy * uy + uz * uz;

    if u_sq == 0.0 {
        // Degenerate segment: single point
        return point_vs_sphere(from, c, r);
    }

    let vx = c[0] - from[0];
    let vy = c[1] - from[1];
    let vz = c[2] - from[2];

    // Project v onto u:  t = (v·u) / |u|²
    let t = (vx * ux + vy * uy + vz * uz) / u_sq;

    // Clamp to [0, 1]
    let t_clamped = t.clamp(0.0, 1.0);

    let px = from[0] + t_clamped * ux;
    let py = from[1] + t_clamped * uy;
    let pz = from[2] + t_clamped * uz;

    let dx = px - c[0];
    let dy = py - c[1];
    let dz = pz - c[2];
    let dist_sq = dx * dx + dy * dy + dz * dz;

    dist_sq < r * r
}

fn point_vs_sphere(p: [f64; 3], c: [f64; 3], r: f64) -> bool {
    (p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2) < r * r
}

// ============================================================================
// Line segment ↔ polygon prism collision (no-fly zone)
// ============================================================================

/// Returns `true` if the line segment [from, to] intersects the no-fly zone prism.
///
/// Two-stage check:
/// 1. Height filter — if the segment's Z-range has no overlap with the prism, reject.
/// 2. 2D projection — check if either endpoint lies inside the polygon,
///    or if any edge of the polygon intersects the projected segment.
pub fn segment_vs_no_fly_zone(from: [f64; 3], to: [f64; 3], nfz: &NoFlyZone) -> bool {
    // Height filter
    let z_min = from[2].min(to[2]);
    let z_max = from[2].max(to[2]);
    if z_max < nfz.alt_min || z_min > nfz.alt_max {
        return false;
    }

    // 2D projection
    let seg_start = [from[0], from[1]];
    let seg_end = [to[0], to[1]];

    // If either endpoint is inside the polygon → collision
    if point_in_polygon(seg_start, &nfz.boundary_points)
        || point_in_polygon(seg_end, &nfz.boundary_points)
    {
        return true;
    }

    // Check segment vs every polygon edge
    let n = nfz.boundary_points.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let edge_start = nfz.boundary_points[i];
        let edge_end = nfz.boundary_points[j];
        if segments_intersect_2d(seg_start, seg_end, edge_start, edge_end) {
            return true;
        }
    }

    false
}

// ---------------------------------------------------------------------------
// 2D geometry helpers
// ---------------------------------------------------------------------------

/// Ray-casting point-in-polygon test (even-odd rule).
fn point_in_polygon(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let n = polygon.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (polygon[i][0], polygon[i][1]);
        let (xj, yj) = (polygon[j][0], polygon[j][1]);
        if ((yi > point[1]) != (yj > point[1]))
            && (point[0] < (xj - xi) * (point[1] - yi) / (yj - yi) + xi)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Checks if two 2D line segments intersect (excluding collinear overlap).
fn segments_intersect_2d(
    a: [f64; 2],
    b: [f64; 2],
    c: [f64; 2],
    d: [f64; 2],
) -> bool {
    let cross = |u: [f64; 2], v: [f64; 2]| u[0] * v[1] - u[1] * v[0];

    let ab = [b[0] - a[0], b[1] - a[1]];
    let cd = [d[0] - c[0], d[1] - c[1]];
    let ac = [c[0] - a[0], c[1] - a[1]];

    let cross_ab_cd = cross(ab, cd);

    // Parallel (or collinear) — treat as no intersection for simplicity
    if cross_ab_cd.abs() < 1e-12 {
        return false;
    }

    let t = cross(ac, cd) / cross_ab_cd;
    let u = cross(ac, ab) / cross_ab_cd;

    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

// ============================================================================
// Bulk collision checks
// ============================================================================

/// Check if a segment collides with any radar.
pub fn segment_vs_any_radar(from: [f64; 3], to: [f64; 3], radars: &[RadarThreat]) -> bool {
    radars.iter().any(|radar| segment_vs_sphere(from, to, radar))
}

/// Check if a segment collides with any no-fly zone.
pub fn segment_vs_any_nfz(from: [f64; 3], to: [f64; 3], nfzs: &[NoFlyZone]) -> bool {
    nfzs.iter().any(|nfz| segment_vs_no_fly_zone(from, to, nfz))
}

/// Check if a segment collides with any environmental obstacle.
pub fn segment_vs_environment(
    from: [f64; 3],
    to: [f64; 3],
    radars: &[RadarThreat],
    nfzs: &[NoFlyZone],
) -> bool {
    segment_vs_any_radar(from, to, radars) || segment_vs_any_nfz(from, to, nfzs)
}

// ============================================================================
// Bounding Corridor — safe flight corridor for search-space pruning
// ============================================================================

/// Axis-aligned bounding box for fast coarse rejection.
#[derive(Debug, Clone)]
pub struct AABB {
    pub min: Point3D,
    pub max: Point3D,
}

impl AABB {
    pub fn from_segment(a: &Point3D, b: &Point3D, radius: f64) -> Self {
        Self {
            min: Point3D(
                a.0.min(b.0) - radius,
                a.1.min(b.1) - radius,
                a.2.min(b.2) - radius,
            ),
            max: Point3D(
                a.0.max(b.0) + radius,
                a.1.max(b.1) + radius,
                a.2.max(b.2) + radius,
            ),
        }
    }

    #[inline(always)]
    pub fn contains(&self, p: &Point3D) -> bool {
        p.0 >= self.min.0
            && p.0 <= self.max.0
            && p.1 >= self.min.1
            && p.1 <= self.max.1
            && p.2 >= self.min.2
            && p.2 <= self.max.2
    }

    /// Williams 变体 Fast Ray-Box 求交 (线段约束版)
    /// 返回 true 当射线 [origin, origin+dir] 与 AABB 有交集
    #[inline]
    pub fn intersects_segment(&self, ray: &PreparedSegment) -> bool {
        let tx1 = (self.min.0 - ray.origin.0) * ray.inv_dir.0;
        let tx2 = (self.max.0 - ray.origin.0) * ray.inv_dir.0;
        let mut tmin = tx1.min(tx2);
        let mut tmax = tx1.max(tx2);

        let ty1 = (self.min.1 - ray.origin.1) * ray.inv_dir.1;
        let ty2 = (self.max.1 - ray.origin.1) * ray.inv_dir.1;
        tmin = tmin.max(ty1.min(ty2));
        tmax = tmax.min(ty1.max(ty2));

        let tz1 = (self.min.2 - ray.origin.2) * ray.inv_dir.2;
        let tz2 = (self.max.2 - ray.origin.2) * ray.inv_dir.2;
        tmin = tmin.max(tz1.min(tz2));
        tmax = tmax.min(tz1.max(tz2));

        tmax >= tmin && tmin <= 1.0 && tmax >= 0.0
    }
}

/// Capsule-shaped corridor segment: cylinder + spherical caps around [start, end].
#[derive(Debug, Clone)]
pub struct CorridorSegment {
    pub start: Point3D,
    pub end: Point3D,
    pub radius: f64,
    aabb: AABB,
}

impl CorridorSegment {
    pub fn new(start: Point3D, end: Point3D, radius: f64) -> Self {
        let aabb = AABB::from_segment(&start, &end, radius);
        Self {
            start,
            end,
            radius,
            aabb,
        }
    }

    #[inline]
    pub fn contains(&self, p: &Point3D) -> bool {
        // Level 1: AABB coarse rejection
        if !self.aabb.contains(p) {
            return false;
        }

        // Level 2: projected squared-distance-to-segment (no sqrt)
        let ab = self.end.sub(&self.start);
        let ap = p.sub(&self.start);
        let ab_len_sq = ab.dot(&ab);

        if ab_len_sq < 1e-9 {
            return ap.dot(&ap) <= self.radius * self.radius;
        }

        let t = (ap.dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
        let closest = self.start.add(&ab.mul(t));
        let diff = p.sub(&closest);
        diff.dot(&diff) <= self.radius * self.radius
    }
}

/// Bounding corridor for single-segment search-space constraint.
pub struct BoundingCorridor {
    segments: Vec<CorridorSegment>,
}

impl BoundingCorridor {
    pub fn from_segment(start: Point3D, end: Point3D, radius: f64) -> Self {
        Self {
            segments: vec![CorridorSegment::new(start, end, radius)],
        }
    }

    #[inline]
    pub fn is_inside(&self, p: &Point3D) -> bool {
        self.segments.iter().any(|seg| seg.contains(p))
    }
}

// ============================================================================
// PreparedSegment — Williams 倒数法射线 (macro routing 加速原语)
// ============================================================================

/// 预计算线段：保存起点和方向倒数，加速 AABB 求交
pub struct PreparedSegment {
    pub origin: Point3D,
    pub inv_dir: Point3D,
}

impl PreparedSegment {
    pub fn new(start: Point3D, end: Point3D) -> Self {
        let mut dx = end.0 - start.0;
        let mut dy = end.1 - start.1;
        let mut dz = end.2 - start.2;

        // 防止与坐标轴平行时产生 NaN (0.0 / 0.0)
        let epsilon = 1e-12_f64;
        if dx.abs() < epsilon {
            dx = epsilon.copysign(dx);
        }
        if dy.abs() < epsilon {
            dy = epsilon.copysign(dy);
        }
        if dz.abs() < epsilon {
            dz = epsilon.copysign(dz);
        }

        Self {
            origin: start,
            inv_dir: Point3D(1.0 / dx, 1.0 / dy, 1.0 / dz),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Point3D;
    use crate::config::RadarThreat;

    #[test]
    fn test_segment_vs_sphere_hit() {
        let radar = RadarThreat {
            id: "R1".into(),
            center: [5.0, 5.0, 5.0],
            radius: 3.0,
        };
        // Segment passes directly through sphere center
        assert!(segment_vs_sphere([0.0, 5.0, 5.0], [10.0, 5.0, 5.0], &radar));
    }

    #[test]
    fn test_segment_vs_sphere_miss() {
        let radar = RadarThreat {
            id: "R1".into(),
            center: [5.0, 5.0, 5.0],
            radius: 1.0,
        };
        // Segment far away
        assert!(!segment_vs_sphere([0.0, 0.0, 0.0], [0.0, 10.0, 0.0], &radar));
    }

    #[test]
    fn test_segment_vs_sphere_tangent() {
        let radar = RadarThreat {
            id: "R1".into(),
            center: [0.0, 0.0, 0.0],
            radius: 3.0,
        };
        // Segment passes exactly 3 units away — should miss (distance == radius, not <)
        assert!(!segment_vs_sphere([-10.0, 3.0, 0.0], [10.0, 3.0, 0.0], &radar));
    }

    #[test]
    fn test_aabb_contains() {
        let aabb = AABB {
            min: Point3D(0.0, 0.0, 0.0),
            max: Point3D(10.0, 10.0, 10.0),
        };
        assert!(aabb.contains(&Point3D(5.0, 5.0, 5.0)));
        assert!(!aabb.contains(&Point3D(15.0, 5.0, 5.0)));
        assert!(!aabb.contains(&Point3D(-1.0, 5.0, 5.0)));
    }

    #[test]
    fn test_aabb_from_segment() {
        let a = Point3D(0.0, 0.0, 0.0);
        let b = Point3D(100.0, 200.0, 300.0);
        let aabb = AABB::from_segment(&a, &b, 50.0);
        assert_eq!(aabb.min.0, -50.0);
        assert_eq!(aabb.max.0, 150.0);
        assert_eq!(aabb.min.1, -50.0);
        assert_eq!(aabb.max.1, 250.0);
        assert_eq!(aabb.min.2, -50.0);
        assert_eq!(aabb.max.2, 350.0);
    }

    #[test]
    fn test_corridor_contains_centerline() {
        let seg = CorridorSegment::new(Point3D(0.0, 0.0, 0.0), Point3D(100.0, 0.0, 0.0), 50.0);
        assert!(seg.contains(&Point3D(50.0, 0.0, 0.0)));
        assert!(seg.contains(&Point3D(50.0, 30.0, 0.0)));
    }

    #[test]
    fn test_corridor_contains_outside() {
        let seg = CorridorSegment::new(Point3D(0.0, 0.0, 0.0), Point3D(100.0, 0.0, 0.0), 50.0);
        assert!(!seg.contains(&Point3D(50.0, 100.0, 0.0)));
        assert!(!seg.contains(&Point3D(200.0, 0.0, 0.0)));
    }

    #[test]
    fn test_corridor_contains_endpoint_cap() {
        let seg = CorridorSegment::new(Point3D(0.0, 0.0, 0.0), Point3D(100.0, 0.0, 0.0), 50.0);
        assert!(seg.contains(&Point3D(100.0, 30.0, 0.0)));
    }

    #[test]
    fn test_bounding_corridor_is_inside() {
        let bc = BoundingCorridor::from_segment(
            Point3D(0.0, 0.0, 0.0),
            Point3D(100.0, 0.0, 0.0),
            50.0,
        );
        assert!(bc.is_inside(&Point3D(50.0, 10.0, 0.0)));
        assert!(!bc.is_inside(&Point3D(50.0, 200.0, 0.0)));
    }

    // ====================================================================
    // PreparedSegment / AABB ray intersection tests
    // ====================================================================

    #[test]
    fn test_prepared_segment_hit_full_intersection() {
        let aabb = AABB {
            min: Point3D(0.0, 0.0, 0.0),
            max: Point3D(10.0, 10.0, 10.0),
        };
        let ray = PreparedSegment::new(Point3D(-5.0, 5.0, 5.0), Point3D(15.0, 5.0, 5.0));
        assert!(aabb.intersects_segment(&ray));
    }

    #[test]
    fn test_prepared_segment_hit_axis_aligned_x() {
        let aabb = AABB {
            min: Point3D(0.0, 0.0, 0.0),
            max: Point3D(10.0, 10.0, 10.0),
        };
        let ray = PreparedSegment::new(Point3D(-2.0, 5.0, 5.0), Point3D(12.0, 5.0, 5.0));
        assert!(aabb.intersects_segment(&ray));
    }

    #[test]
    fn test_prepared_segment_miss_outside() {
        let aabb = AABB {
            min: Point3D(0.0, 0.0, 0.0),
            max: Point3D(10.0, 10.0, 10.0),
        };
        let ray = PreparedSegment::new(Point3D(5.0, 20.0, 5.0), Point3D(5.0, 30.0, 5.0));
        assert!(!aabb.intersects_segment(&ray));
    }

    #[test]
    fn test_prepared_segment_miss_short_segment_nearby() {
        let aabb = AABB {
            min: Point3D(0.0, 0.0, 0.0),
            max: Point3D(10.0, 10.0, 10.0),
        };
        // 线段完全在 AABB 外部且不穿过
        let ray =
            PreparedSegment::new(Point3D(15.0, 15.0, 15.0), Point3D(16.0, 16.0, 16.0));
        assert!(!aabb.intersects_segment(&ray));
    }

    #[test]
    fn test_prepared_segment_hit_segment_inside_aabb() {
        let aabb = AABB {
            min: Point3D(0.0, 0.0, 0.0),
            max: Point3D(10.0, 10.0, 10.0),
        };
        let ray = PreparedSegment::new(Point3D(2.0, 2.0, 2.0), Point3D(8.0, 8.0, 8.0));
        assert!(aabb.intersects_segment(&ray));
    }

    #[test]
    fn test_prepared_segment_hit_endpoint_on_surface() {
        let aabb = AABB {
            min: Point3D(0.0, 0.0, 0.0),
            max: Point3D(10.0, 10.0, 10.0),
        };
        let ray =
            PreparedSegment::new(Point3D(-5.0, 5.0, 5.0), Point3D(0.0, 5.0, 5.0));
        assert!(aabb.intersects_segment(&ray));
    }
}
