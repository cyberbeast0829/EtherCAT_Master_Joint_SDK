#!/bin/bash
# DC Sync0 shift tune for ISVD90RC (esp. FW v8.1.50).
# Practice: FreeRun first (optional), then raise sync0_shift from 100000 ns.
set -e
cd "$(dirname "$0")/.."
BIN=./build/diag_csv_single
FW="${FW:-8.1.50}"
RPS="${RPS:-0.5}"
SECS="${SECS:-8}"

if [[ ! -x "$BIN" ]]; then
  make csv ETHERLAB_DIR="${ETHERLAB_DIR:-/usr/local}"
fi

RUN=()
if chrt -f 80 true 2>/dev/null; then
  RUN=(chrt -f 80)
elif sudo -n chrt -f 80 true 2>/dev/null; then
  RUN=(sudo -n chrt -f 80)
else
  echo "WARNING: run as: sudo chrt -f 80 bash $0"
fi

echo "=== 1) FreeRun sanity (PDO + CiA402) ==="
"${RUN[@]}" $BIN --fw "$FW" --no-dc --rps "$RPS" --seconds 5 || true

echo
echo "=== 2) DC Sync0 shift sweep (from 100000 ns) ==="
BEST=""
for SHIFT in 100000 150000 200000 250000 300000 50000 0; do
  echo "---- sync0_shift=${SHIFT} ----"
  if "${RUN[@]}" $BIN --fw "$FW" --dc --sync0-shift "$SHIFT" --rps "$RPS" --seconds "$SECS"; then
    echo "PASS at sync0_shift=${SHIFT}"
    BEST=$SHIFT
    break
  fi
  echo "FAIL at sync0_shift=${SHIFT}"
done

if [[ -n "$BEST" ]]; then
  echo
  echo "Calibrated: set sync0_shift_ns=${BEST} in src/drive_model.c for ISVD90RC-v${FW}"
else
  echo "No stable DC shift found; check dmesg for 0x001A / use sudo chrt -f 80"
  exit 1
fi
