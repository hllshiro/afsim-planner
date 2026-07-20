#![allow(dead_code)]

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Geometric primitives
// ---------------------------------------------------------------------------

/// 3D point in meters (X, Y, Z)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Point3D(pub f64, pub f64, pub f64);

impl Point3D {
    pub fn distance_to(&self, other: &Point3D) -> f64 {
        ((self.0 - other.0).powi(2) + (self.1 - other.1).powi(2) + (self.2 - other.2).powi(2))
            .sqrt()
    }

    pub fn distance_2d(&self, other: &Point3D) -> f64 {
        ((self.0 - other.0).powi(2) + (self.1 - other.1).powi(2)).sqrt()
    }

    pub fn add(&self, other: &Point3D) -> Point3D {
        Point3D(self.0 + other.0, self.1 + other.1, self.2 + other.2)
    }

    pub fn sub(&self, other: &Point3D) -> Point3D {
        Point3D(self.0 - other.0, self.1 - other.1, self.2 - other.2)
    }

    pub fn mul(&self, scalar: f64) -> Point3D {
        Point3D(self.0 * scalar, self.1 * scalar, self.2 * scalar)
    }

    pub fn dot(&self, other: &Point3D) -> f64 {
        self.0 * other.0 + self.1 * other.1 + self.2 * other.2
    }
}

/// 2D point used for polygon boundary definition
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Point2D(pub f64, pub f64);

// ---------------------------------------------------------------------------
// Input data structures
// ---------------------------------------------------------------------------

/// Session-level configuration
#[derive(Debug, Clone, Deserialize)]
pub struct SessionConfig {
    pub seed: Option<u64>,
    pub max_calculation_time_ms: Option<u64>,
}

/// Vehicle kinematic limits
#[derive(Debug, Clone, Deserialize)]
pub struct VehicleProfile {
    pub min_turn_radius: f64,
    pub max_climb_angle: f64,
    pub max_turn_angle_deg: f64,
}

/// Starting state of the vehicle
#[derive(Debug, Clone, Deserialize)]
pub struct StartState {
    pub position: [f64; 3],
    pub heading_deg: f64,
}

/// Target zone definition
#[derive(Debug, Clone, Deserialize)]
pub struct TargetZone {
    pub center: [f64; 3],
    pub radius: f64,
}

/// Route definition containing ordered waypoints
#[derive(Debug, Clone, Deserialize)]
pub struct RouteDefinition {
    pub start_state: StartState,
    #[serde(default)]
    pub control_waypoints: Vec<[f64; 3]>,
    pub target: TargetZone,
}

/// Spherical radar threat zone
#[derive(Debug, Clone, Deserialize)]
pub struct RadarThreat {
    pub id: String,
    pub center: [f64; 3],
    pub radius: f64,
}

/// Polygonal prism no-fly zone
#[derive(Debug, Clone, Deserialize)]
pub struct NoFlyZone {
    pub id: String,
    pub boundary_points: Vec<[f64; 2]>,
    pub alt_min: f64,
    pub alt_max: f64,
}

/// Environment threats and obstacles
#[derive(Debug, Clone, Deserialize, Default)]
pub struct Environment {
    #[serde(default)]
    pub radars: Vec<RadarThreat>,
    #[serde(default)]
    pub no_fly_zones: Vec<NoFlyZone>,
}

/// Complete input contract received via stdin
#[derive(Debug, Clone, Deserialize)]
pub struct InputConfig {
    pub session: SessionConfig,
    pub vehicle: VehicleProfile,
    pub route_definition: RouteDefinition,
    #[serde(default)]
    pub environment: Environment,
}

// ---------------------------------------------------------------------------
// Output data structures
// ---------------------------------------------------------------------------

/// Diagnostics returned in successful output
#[derive(Debug, Clone, Serialize)]
pub struct Diagnostics {
    pub calculation_time_ms: f64,
    pub nodes_explored: u64,
    pub seed_used: u64,
}

/// Path summary
#[derive(Debug, Clone, Serialize)]
pub struct Summary {
    pub total_length_m: f64,
    pub max_climb_angle_utilized: f64,
}

/// A single waypoint in the output path
#[derive(Debug, Clone, Serialize)]
pub struct Waypoint {
    pub index: usize,
    pub position: [f64; 3],
    #[serde(rename = "type")]
    pub wp_type: WaypointType,
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum WaypointType {
    Start,
    Waypoint,
    Target,
}

/// Successful output payload
#[derive(Debug, Clone, Serialize)]
pub struct OutputSuccess {
    pub status: String,
    pub diagnostics: Diagnostics,
    pub summary: Summary,
    pub waypoints: Vec<Waypoint>,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod point_tests {
    use super::Point3D;

    #[test]
    fn test_point_add() {
        let a = Point3D(1.0, 2.0, 3.0);
        let b = Point3D(4.0, 5.0, 6.0);
        let r = a.add(&b);
        assert_eq!(r.0, 5.0);
        assert_eq!(r.1, 7.0);
        assert_eq!(r.2, 9.0);
    }

    #[test]
    fn test_point_sub() {
        let a = Point3D(5.0, 7.0, 9.0);
        let b = Point3D(1.0, 2.0, 3.0);
        let r = a.sub(&b);
        assert_eq!(r.0, 4.0);
        assert_eq!(r.1, 5.0);
        assert_eq!(r.2, 6.0);
    }

    #[test]
    fn test_point_mul() {
        let a = Point3D(1.0, 2.0, 3.0);
        let r = a.mul(3.0);
        assert_eq!(r.0, 3.0);
        assert_eq!(r.1, 6.0);
        assert_eq!(r.2, 9.0);
    }

    #[test]
    fn test_point_dot() {
        let a = Point3D(1.0, 2.0, 3.0);
        let b = Point3D(4.0, 5.0, 6.0);
        let r = a.dot(&b);
        assert_eq!(r, 32.0);
    }
}
