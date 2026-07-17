// === Coordinate type alias ===
export type Vec3 = [number, number, number];
export type Vec2 = [number, number];

// === Input types (matching CLI JSON schema) ===

export interface SessionConfig {
  seed?: number | null;
  max_calculation_time_ms?: number | null;
}

export interface VehicleProfile {
  min_turn_radius: number;
  max_climb_angle: number;
  max_turn_angle_deg: number;
}

export interface StartState {
  position: Vec3;
  heading_deg: number;
}

export interface TargetZone {
  center: Vec3;
  radius: number;
}

export interface RouteDefinition {
  start_state: StartState;
  control_waypoints: Vec3[];
  target: TargetZone;
}

export interface RadarThreat {
  id: string;
  center: Vec3;
  radius: number;
}

export interface NoFlyZone {
  id: string;
  boundary_points: Vec2[];
  alt_min: number;
  alt_max: number;
}

export interface Environment {
  radars: RadarThreat[];
  no_fly_zones: NoFlyZone[];
}

export interface InputConfig {
  session: SessionConfig;
  vehicle: VehicleProfile;
  route_definition: RouteDefinition;
  environment: Environment;
}

// === Output types (matching CLI JSON schema) ===

export interface Diagnostics {
  calculation_time_ms: number;
  nodes_explored: number;
  seed_used: number;
}

export interface Summary {
  total_length_m: number;
  max_climb_angle_utilized: number;
}

export interface Waypoint {
  index: number;
  position: Vec3;
  type: 'START' | 'WAYPOINT' | 'TARGET';
}

export interface PlanSuccess {
  status: 'SUCCESS';
  diagnostics: Diagnostics;
  summary: Summary;
  waypoints: Waypoint[];
}

export interface ErrorDetail {
  code: string;
  message: string;
  location: Vec3;
  seed_used: number;
}

export interface PlanFailed {
  status: 'FAILED';
  error: ErrorDetail;
}

export type PlanResult = PlanSuccess | PlanFailed;

// === Default config ===

export function defaultInputConfig(): InputConfig {
  return {
    session: { seed: null, max_calculation_time_ms: 5000 },
    vehicle: { min_turn_radius: 350, max_climb_angle: 25, max_turn_angle_deg: 60 },
    route_definition: {
      start_state: { position: [0, 0, 500], heading_deg: 45 },
      control_waypoints: [],
      target: { center: [20000, 20000, 1000], radius: 500 },
    },
    environment: { radars: [], no_fly_zones: [] },
  };
}
