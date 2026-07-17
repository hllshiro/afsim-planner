use serde::Serialize;

/// Unified error codes for route planning failures
#[derive(Debug, Clone, Serialize)]
pub enum ErrorCode {
    #[serde(rename = "JSON_PARSE_ERROR")]
    JsonParseError,
    #[serde(rename = "ROUTE_BLOCKED_BY_RADAR")]
    RouteBlockedByRadar,
    #[serde(rename = "ROUTE_BLOCKED_BY_NOFLYZONE")]
    RouteBlockedByNoFlyZone,
    #[serde(rename = "ROUTE_GENERATION_FAILED")]
    RouteGenerationFailed,
    #[serde(rename = "MAX_CALCULATION_TIME_EXCEEDED")]
    MaxCalculationTimeExceeded,
    #[serde(rename = "INVALID_INPUT_DATA")]
    InvalidInputData,
}

/// Structured error detail returned in failure output
#[derive(Debug, Clone, Serialize)]
pub struct ErrorDetail {
    pub code: ErrorCode,
    pub message: String,
    pub location: [f64; 3],
    pub seed_used: u64,
}

/// Failure output payload
#[derive(Debug, Clone, Serialize)]
pub struct OutputFailed {
    pub status: String,
    pub error: ErrorDetail,
}
