#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
BINARY="$ROOT/target/release/afsim-planner"

echo "==> Building afsim-planner CLI..."
cargo build --release -p afsim-planner

echo "==> Building server..."
cargo build --release -p afsim-planner-server

echo "==> Starting server on :3001..."
cargo run --release -p afsim-planner-server &
SERVER_PID=$!

# Give server a moment to bind
sleep 1

cleanup() {
    echo ""
    echo "==> Shutting down..."
    kill "$SERVER_PID" 2>/dev/null
    wait "$SERVER_PID" 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

echo "==> Starting frontend dev server on :5173..."
cd "$ROOT/web"
pnpm dev
