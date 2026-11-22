#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
cargo build --release -p happywheels-web --target wasm32-unknown-unknown
wasm-bindgen --target web --out-dir web/pkg target/wasm32-unknown-unknown/release/happywheels_web.wasm
