/// Fast spatial collision detection for 3D route planning.
///
/// Provides two collision primitives:
/// 1. Line segment vs sphere   (radar threats)
/// 2. Line segment vs polygon prism (no-fly zones)

use crate::config::{NoFlyZone, RadarThreat};

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

#[cfg(test)]
mod tests {
    use super::*;
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
}
