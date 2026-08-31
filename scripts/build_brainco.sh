#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NATIVE_ROOT="$(cd "${SCRIPT_DIR}/../dexfull/hand_drivers/brainco/native" && pwd)"
cmake -S "${NATIVE_ROOT}" -B "${NATIVE_ROOT}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${NATIVE_ROOT}/build" --parallel

