use std::io::Write;
use std::process::{Command, Stdio};

use axum::{Json, Router, routing::post};
use serde_json::Value;

/// POST /api/plan — accepts full InputConfig JSON, runs CLI binary, returns result
async fn plan_route(Json(payload): Json<Value>) -> Json<Value> {
    let input_json = serde_json::to_string(&payload).unwrap_or_else(|_| "{}".into());

    let result = run_cli(&input_json);

    match result {
        Ok(json) => Json(json),
        Err(msg) => Json(serde_json::json!({
            "status": "FAILED",
            "error": {
                "code": "INTERNAL_SERVER_ERROR",
                "message": msg,
                "location": [0.0, 0.0, 0.0],
                "seed_used": 0
            }
        })),
    }
}

fn run_cli(input_json: &str) -> Result<Value, String> {
    // Binary lives at <workspace_root>/target/release/cli
    // Server's cwd is workspace_root when launched via `cargo run -p cli-server`
    let mut child = Command::new("./target/release/cli")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("Failed to spawn CLI: {}", e))?;

    {
        let stdin = child
            .stdin
            .as_mut()
            .ok_or("Failed to open CLI stdin")?;
        stdin
            .write_all(input_json.as_bytes())
            .map_err(|e| format!("Failed to write to CLI stdin: {}", e))?;
    }

    let output = child
        .wait_with_output()
        .map_err(|e| format!("CLI process error: {}", e))?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("CLI exited with error: {}", stderr));
    }

    let stdout = String::from_utf8_lossy(&output.stdout).trim().to_string();
    if stdout.is_empty() {
        return Err("CLI produced empty output".into());
    }

    serde_json::from_str(&stdout).map_err(|e| format!("Failed to parse CLI output: {}", e))
}

#[tokio::main]
async fn main() {
    let app = Router::new().route("/api/plan", post(plan_route));

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3001")
        .await
        .expect("Failed to bind port 3001");

    println!("Server listening on http://0.0.0.0:3001");
    axum::serve(listener, app).await.expect("Server error");
}
