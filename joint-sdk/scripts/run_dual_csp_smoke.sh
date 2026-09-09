#!/usr/bin/env bash
# Dual CSP smoke: FreeRun hold → DC(sync0=task) hold → DC delta.
# Run from joint-sdk:  sudo ./scripts/run_dual_csp_smoke.sh
set -uo pipefail
cd "$(dirname "$0")/.."
BIN=./build/diag_csp_dual
LOGDIR=./build/dual_smoke_logs
mkdir -p "$LOGDIR"
TS=$(date +%Y%m%d_%H%M%S)
FAIL=0

need_rebuild() {
  [[ ! -x "$BIN" ]] && return 0
  [[ examples/diag_csp_dual.c -nt "$BIN" ]] && return 0
  [[ src/transport.c -nt build/transport.o ]] && return 0
  return 1
}

if need_rebuild; then
  echo "=== rebuild dual ==="
  make -j"$(nproc)" dual
fi

echo "=== slaves before ==="
ethercat slaves || true

run_one() {
  local name=$1
  shift
  local out="$LOGDIR/${TS}_${name}.log"
  echo
  echo "=== $name: $* ==="
  set +e
  chrt -f 80 "$BIN" "$@" 2>&1 | tee "$out"
  local rc=${PIPESTATUS[0]}
  set -e
  echo "exit=$rc  log=$out"
  echo "--- dmesg tail ---"
  dmesg | grep -iE 'ethercat|SAFEOP|sync|working counter' | tail -20 || true
  ethercat slaves || true
  if [[ "$rc" -ne 0 ]]; then
    FAIL=1
  fi
  sleep 2
  return 0
}

# Invert is CLI: --no-invert0 --invert1 (same as make dual Prefer).
# 1) FreeRun: prove dual PDO / CiA402 without DC
run_one freerun_hold \
  --fw0 8.1.50 --fw1 8.1.60 --no-invert0 --invert1 --no-dc --hold --seconds 6

# 2) DC with Sync0==task (default for dual diag after fix)
run_one dc_hold \
  --fw0 8.1.50 --fw1 8.1.60 --no-invert0 --invert1 --hold --seconds 6

# 3) Motion
run_one dc_delta \
  --fw0 8.1.50 --fw1 8.1.60 --no-invert0 --invert1 \
  --delta 18192 --speed 4000 --seconds 20

echo
echo "=== all smoke steps finished (FAIL=$FAIL) ==="
ethercat slaves || true
exit "$FAIL"
