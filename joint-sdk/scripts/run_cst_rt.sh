#!/bin/bash
# Run CST diag under SCHED_FIFO 80 + drive_model from --fw.
# Usage:
#   bash scripts/run_cst_rt.sh --fw 8.1.50 --nm 1 --seconds 10
#   bash scripts/run_cst_rt.sh --fw 8.1.44 --nm 0.5 --seconds 10
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
BIN=./build/diag_cst_single

if [[ ! -x "$BIN" ]]; then
  make cst ETHERLAB_DIR="${ETHERLAB_DIR:-/usr/local}"
fi

chmod 666 /dev/EtherCAT0 2>/dev/null || true

# Default args if none given
if [[ $# -eq 0 ]]; then
  set -- --fw 8.1.50 --nm 1 --vmax 40000 --seconds 10
fi

if chrt -f 80 true 2>/dev/null; then
  exec chrt -f 80 "$BIN" "$@"
fi

echo "Need root for SCHED_FIFO 80 — prompting sudo..."
exec sudo chrt -f 80 "$BIN" "$@"
