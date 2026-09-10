#!/bin/bash
# Run CSV diag under SCHED_FIFO 80 to keep DC Sync0 from dropping.
# Usage:
#   bash scripts/run_csv_rt.sh --fw 8.1.50 --rps 0.5 --seconds 20
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
BIN=./build/diag_csv_single

if [[ ! -x "$BIN" ]]; then
  make csv ETHERLAB_DIR="${ETHERLAB_DIR:-/usr/local}"
fi

chmod 666 /dev/EtherCAT0 2>/dev/null || true

if chrt -f 80 true 2>/dev/null; then
  exec chrt -f 80 "$BIN" "$@"
fi

echo "Need root for SCHED_FIFO 80 — prompting sudo..."
exec sudo chrt -f 80 "$BIN" "$@"
