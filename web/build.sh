#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
bindgen=${WASM_BINDGEN:-wasm-bindgen}
required=$(awk '/^\[\[package\]\]$/ { found = 0 } /^name = "wasm-bindgen"$/ { found = 1; next } found && /^version = / { gsub(/"/, "", $3); print $3; exit }' Cargo.lock)
if [ -z "$required" ]; then
  echo "Could not find the wasm-bindgen version in Cargo.lock." >&2
  exit 1
fi
installed=$("$bindgen" --version 2>/dev/null || true)
if [ "$installed" != "wasm-bindgen $required" ]; then
  echo "Expected wasm-bindgen $required; found ${installed:-no executable at $bindgen}." >&2
  echo "Install it with: cargo install wasm-bindgen-cli --version $required --locked" >&2
  echo "Or set WASM_BINDGEN to a matching executable." >&2
  exit 1
fi
cargo build --release -p happywheels-web --target wasm32-unknown-unknown
"$bindgen" --target web --out-dir web/pkg target/wasm32-unknown-unknown/release/happywheels_web.wasm
