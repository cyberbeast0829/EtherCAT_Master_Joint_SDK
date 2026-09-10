#!/bin/bash
# CSV 联调（现场标定：act ≈ 20.5 * cmd）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
LOG_DIR="$ROOT/build/csv_logs"
mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)

chmod 666 /dev/EtherCAT0 2>/dev/null || true
ethercat slaves || { echo "no slaves"; exit 1; }
make csv ETHERLAB_DIR=/usr/local
BIN=./build/diag_csv_single
# Prefer RT to keep DC Sync0 from dropping (sudo chrt -f 80).
RUN=()
if chrt -f 80 true 2>/dev/null; then
  RUN=(chrt -f 80)
elif sudo -n chrt -f 80 true 2>/dev/null; then
  RUN=(sudo -n chrt -f 80)
fi

run_one() {
  local tag=$1; shift
  local log="$LOG_DIR/${STAMP}_${tag}.csv"
  echo; echo "======== $tag (${RUN[*]} $BIN $*) ========"
  if "${RUN[@]}" "$BIN" --seconds 10 --log "$log" "$@"; then
    echo "RESULT $tag: PASS ($log)"; return 0
  fi
  echo "RESULT $tag: FAIL ($log)"; return 1
}

run_one hold --hold || true

# --rps 用标定系数换算 0x60FF
if run_one rps1 --rps 1; then
  echo "OK: sudo ./build/diag_csv_single --rps 1"
  exit 0
fi
if run_one rps1_inv --rps 1 --invert; then
  echo "OK: need --invert"
  exit 0
fi
if run_one vel800 --vel 800; then
  echo "OK: --vel 800 ≈ 1 rps"
  exit 0
fi

echo "See $LOG_DIR"; exit 1
