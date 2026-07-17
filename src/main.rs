use std::io::{self, Read, Write};
use std::time::Instant;

use rand::Rng;

mod config;
mod error;
mod geometry;
mod macro_router;
mod solver;

use config::{
    InputConfig, OutputSuccess, Point3D, Diagnostics, Summary, Waypoint, WaypointType,
};
use error::{ErrorCode, ErrorDetail, OutputFailed};
use solver::AStarSolver;

fn main() {
    // 1. Capture full JSON payload from stdin
    let mut buffer = String::new();
    if io::stdin().read_to_string(&mut buffer).is_err() {
        emit_fatal("Failed to read from stdin");
    }

    // 2. Parse input contract
    let input: InputConfig = match serde_json::from_str(&buffer) {
        Ok(cfg) => cfg,
        Err(e) => {
            let resp = OutputFailed {
                status: "FAILED".into(),
                error: ErrorDetail {
                    code: ErrorCode::JsonParseError,
                    message: format!("Malformed input JSON: {}", e),
                    location: [0.0, 0.0, 0.0],
                    seed_used: 0,
                },
            };
            emit_json(&resp);
        }
    };

    // 3. Seed resolution (provided or auto-generated)
    let final_seed = input.session.seed.unwrap_or_else(|| {
        let mut rng = rand::thread_rng();
        rng.gen()
    });

    // 4. Validate input basics
    if let Err(msg) = validate_input(&input) {
        let resp = OutputFailed {
            status: "FAILED".into(),
            error: ErrorDetail {
                code: ErrorCode::InvalidInputData,
                message: msg,
                location: [0.0, 0.0, 0.0],
                seed_used: final_seed,
            },
        };
        emit_json(&resp);
    }

    // 5. Build solver
    let grid_resolution = compute_grid_resolution(&input);
    let solver = AStarSolver::new(
        input.vehicle.clone(),
        input.environment.clone(),
        grid_resolution,
        input.session.max_calculation_time_ms,
        final_seed,
    );

    // 6. Build ordered waypoint list for segmented planning
    let start = Point3D(
        input.route_definition.start_state.position[0],
        input.route_definition.start_state.position[1],
        input.route_definition.start_state.position[2],
    );
    let target_center = Point3D(
        input.route_definition.target.center[0],
        input.route_definition.target.center[1],
        input.route_definition.target.center[2],
    );
    let mut headings = input.route_definition.start_state.heading_deg;

    let mut full_path: Vec<Point3D> = Vec::new();
    let mut total_nodes_explored: u64 = 0;
    let mut max_climb_utilized = 0.0;

    let mut segment_endpoints: Vec<(Point3D, Point3D)> = Vec::new();
    {
        let wpts = &input.route_definition.control_waypoints;
        if wpts.is_empty() {
            // Direct: start -> target
            segment_endpoints.push((start, target_center));
        } else {
            // start -> wpt[0]
            segment_endpoints.push((start, Point3D(wpts[0][0], wpts[0][1], wpts[0][2])));
            // intermediate wpts
            for i in 1..wpts.len() {
                let prev = Point3D(wpts[i - 1][0], wpts[i - 1][1], wpts[i - 1][2]);
                let next = Point3D(wpts[i][0], wpts[i][1], wpts[i][2]);
                segment_endpoints.push((prev, next));
            }
            // last wpt -> target
            let last_wpt = Point3D(
                wpts[wpts.len() - 1][0],
                wpts[wpts.len() - 1][1],
                wpts[wpts.len() - 1][2],
            );
            segment_endpoints.push((last_wpt, target_center));
        }
    }

    let t0 = Instant::now();

    for (seg_idx, &(seg_start, seg_end)) in segment_endpoints.iter().enumerate() {
        let seg_result = solver.solve_segment(seg_start, seg_end, headings);

        let (seg_path, seg_nodes_explored) = match seg_result {
            Ok(result) => result,
            Err(err_msg) => {
                let code = classify_error(&err_msg);
                let location = [
                    seg_start.0 + (seg_end.0 - seg_start.0) * 0.5,
                    seg_start.1 + (seg_end.1 - seg_start.1) * 0.5,
                    seg_start.2 + (seg_end.2 - seg_start.2) * 0.5,
                ];
                let resp = OutputFailed {
                    status: "FAILED".into(),
                    error: ErrorDetail {
                        code,
                        message: format!("Segment {}: {}", seg_idx, err_msg),
                        location,
                        seed_used: final_seed,
                    },
                };
                emit_json(&resp);
            }
        };

        // Prune A* grid artifacts from this segment (preserves segment boundaries)
        let seg_path = solver.prune_waypoints(&seg_path);

        // Update heading from the last two points of the segment
        if seg_path.len() >= 2 {
            let last = seg_path[seg_path.len() - 1];
            let prev = seg_path[seg_path.len() - 2];
            let dx = last.0 - prev.0;
            let dy = last.1 - prev.1;
            headings = dy.atan2(dx).to_degrees();
        }

        // Merge path: omit first point except for the first segment
        let start_offset = if full_path.is_empty() { 0 } else { 1 };
        for pt in &seg_path[start_offset..] {
            full_path.push(*pt);
        }

        total_nodes_explored += seg_nodes_explored;
    }

    // Compute total length from the (per-segment-pruned) path
    let mut total_length = 0.0_f64;
    for i in 1..full_path.len() {
        total_length += full_path[i].distance_to(&full_path[i - 1]);
    }

    let calc_time = t0.elapsed().as_secs_f64() * 1000.0;

    // Compute max climb angle utilized
    for i in 1..full_path.len() {
        let dz = (full_path[i].2 - full_path[i - 1].2).abs();
        let d2d = full_path[i].distance_2d(&full_path[i - 1]);
        if d2d > 0.0 {
            let angle = (dz / d2d).atan().to_degrees();
            if angle > max_climb_utilized {
                max_climb_utilized = angle;
            }
        }
    }

    // 7. Assemble output
    // Compute per-segment clash info for diagnostic location info
    // (handled in error branch above)

    let waypoints: Vec<Waypoint> = full_path
        .iter()
        .enumerate()
        .map(|(i, pt)| {
            let wp_type = if i == 0 {
                WaypointType::Start
            } else if i == full_path.len() - 1 {
                WaypointType::Target
            } else {
                WaypointType::Waypoint
            };
            Waypoint {
                index: i,
                position: [pt.0, pt.1, pt.2],
                wp_type,
            }
        })
        .collect();

    let output = OutputSuccess {
        status: "SUCCESS".into(),
        diagnostics: Diagnostics {
            calculation_time_ms: calc_time,
            nodes_explored: total_nodes_explored,
            seed_used: final_seed,
        },
        summary: Summary {
            total_length_m: total_length,
            max_climb_angle_utilized: max_climb_utilized,
        },
        waypoints,
    };

    emit_json(&output);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn validate_input(input: &InputConfig) -> Result<(), String> {
    let r = &input.route_definition;
    if !r.control_waypoints.is_empty() {
        // Verify Z-altitude is reasonable (non-negative)
        for (i, wpt) in r.control_waypoints.iter().enumerate() {
            if wpt[2] < 0.0 {
                return Err(format!("control_waypoints[{}] has negative altitude", i));
            }
        }
    }
    if input.vehicle.max_turn_angle_deg <= 0.0 || input.vehicle.max_turn_angle_deg > 180.0 {
        return Err("max_turn_angle_deg must be in (0, 180]".into());
    }
    Ok(())
}

/// Compute adaptive grid resolution based on turn radius.
fn compute_grid_resolution(input: &InputConfig) -> f64 {
    // Grid resolution defaults to half the min turn radius, minimum 100m
    (input.vehicle.min_turn_radius / 2.0).max(100.0)
}

fn classify_error(msg: &str) -> ErrorCode {
    if msg.contains("Radar") {
        ErrorCode::RouteBlockedByRadar
    } else if msg.contains("NoFlyZone") || msg.contains("no_fly") {
        ErrorCode::RouteBlockedByNoFlyZone
    } else if msg.contains("time") {
        ErrorCode::MaxCalculationTimeExceeded
    } else {
        ErrorCode::RouteGenerationFailed
    }
}

fn emit_json<T: serde::Serialize>(payload: &T) -> ! {
    let serialized = serde_json::to_string(payload).unwrap_or_else(|_| {
        r#"{"status":"FAILED","error":{"code":"JSON_PARSE_ERROR","message":"Failed to serialize output","location":[0,0,0],"seed_used":0}}"#.into()
    });
    let mut stdout = io::stdout();
    let _ = writeln!(stdout, "{}", serialized);
    std::process::exit(0);
}

fn emit_fatal(msg: &str) -> ! {
    eprintln!("FATAL: {}", msg);
    std::process::exit(1);
}
