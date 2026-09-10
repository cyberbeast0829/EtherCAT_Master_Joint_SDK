#!/bin/bash
# Score DC Sync0 candidates by dropout rate (not just PASS/FAIL).
# Usage: bash scripts/dc_tune_score.sh
set -u
cd "$(dirname "$0")/.."
BIN=./build/diag_csv_single
FW="${FW:-8.1.50}"
RPS="${RPS:-0.5}"
SECS="${SECS:-12}"
LOGDIR=build/dc_tune_logs
mkdir -p "$LOGDIR"

if [[ ! -x "$BIN" ]]; then
  make csv ETHERLAB_DIR="${ETHERLAB_DIR:-/usr/local}"
fi

# SCHED_FIFO 80 cuts DC Sync0 dropouts (needs root or RT caps).
RUNNER=()
if command -v chrt >/dev/null 2>&1; then
  if chrt -f 80 true 2>/dev/null; then
    RUNNER=(chrt -f 80)
    echo "Using SCHED_FIFO 80"
  elif sudo -n chrt -f 80 true 2>/dev/null; then
    RUNNER=(sudo -n chrt -f 80)
    echo "Using sudo chrt -f 80"
  else
    echo "WARNING: no RT priority — expect more unknown/al=04 dropouts"
  fi
fi

score_log() {
  local f=$1
  local unknown ok al08 lines score
  unknown=$(grep -c '^unknown' "$f" 2>/dev/null) || unknown=0
  ok=$(grep -c ' YES$' "$f" 2>/dev/null) || ok=0
  al08=$(grep -E 'operation_enabled' "$f" 2>/dev/null | grep -c ' 08 ') || al08=0
  lines=$(grep -cE '^(operation_enabled|unknown|switch|ready|fault)' "$f" 2>/dev/null) || lines=0
  # strip newlines; force decimal
  unknown=${unknown%%$'\n'*}
  ok=${ok%%$'\n'*}
  al08=${al08%%$'\n'*}
  lines=${lines%%$'\n'*}
  score=$(( unknown * 100 + (lines - al08) * 10 - ok * 2 ))
  echo "$score $unknown $ok $al08 $lines"
}

echo "=== DC shift sweep FW=$FW secs=$SECS ==="
BEST_SHIFT=""
BEST_SCORE=999999
RESULTS=()

for SHIFT in 0 50000 75000 100000 125000 150000 175000 200000 225000 250000 300000 350000 400000; do
  LOG="$LOGDIR/shift_${SHIFT}.log"
  echo -n "shift=${SHIFT} ... "
  set +e
  "${RUNNER[@]}" "$BIN" --fw "$FW" --dc --sync0-shift "$SHIFT" \
      --rps "$RPS" --seconds "$SECS" >"$LOG" 2>&1
  RC=$?
  set -e
  read -r SCORE UNKNOWN OK AL08 LINES < <(score_log "$LOG")
  PASS="FAIL"
  grep -q 'PASS: velocity' "$LOG" && PASS="PASS"
  printf "rc=%d %s score=%s unknown=%s ok=%s al08=%s lines=%s\n" \
      "$RC" "$PASS" "$SCORE" "$UNKNOWN" "$OK" "$AL08" "$LINES"
  RESULTS+=("shift=$SHIFT score=$SCORE unknown=$UNKNOWN pass=$PASS")
  # Prefer PASS with lowest unknown; among PASS pick lowest score
  if [[ "$PASS" == "PASS" ]] && [[ "$SCORE" -lt "$BEST_SCORE" ]]; then
    BEST_SCORE=$SCORE
    BEST_SHIFT=$SHIFT
  fi
  # Brief settle between runs
  sleep 1
done

echo
echo "=== Ranking ==="
printf '%s\n' "${RESULTS[@]}" | sort -t= -k3 -n

if [[ -n "$BEST_SHIFT" ]]; then
  echo
  echo "BEST sync0_shift_ns=${BEST_SHIFT} (score=${BEST_SCORE})"
  echo "$BEST_SHIFT" > "$LOGDIR/best_shift.txt"
  exit 0
fi
echo "No PASS candidate"
exit 1
