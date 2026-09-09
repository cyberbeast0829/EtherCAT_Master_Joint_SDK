#!/bin/bash
# CST 联调：限速开启 + 小力矩
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
LOG_DIR="$ROOT/build/csv_logs"
mkdir -p "$LOG_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)

chmod 666 /dev/EtherCAT0 2>/dev/null || true
ethercat slaves || { echo "no slaves — try: sudo ethercatctl restart"; exit 1; }

make cst ETHERLAB_DIR=/usr/local
BIN=./build/diag_cst_single
FW="${FW:-8.1.50}"

RUN=()
if chrt -f 80 true 2>/dev/null; then
  RUN=(chrt -f 80)
elif sudo -n chrt -f 80 true 2>/dev/null; then
  RUN=(sudo -n chrt -f 80)
else
  echo "WARNING: prefer: sudo chrt -f 80 $BIN --fw $FW ..."
fi

run_one() {
  local tag=$1; shift
  local log="$LOG_DIR/${STAMP}_cst_${tag}.csv"
  echo; echo "======== CST $tag (${RUN[*]} $BIN --fw $FW $*) ========"
  if "${RUN[@]}" "$BIN" --fw "$FW" --seconds 8 --log "$log" "$@"; then
    echo "RESULT $tag: PASS ($log)"; return 0
  fi
  echo "RESULT $tag: FAIL ($log)"; return 1
}

# 清可能残留的 SAFEOP+E：重启应用前确保主站可用
run_one hold --hold || true

# 小力矩 0.3 N·m
if run_one nm03 --nm 0.3 --vmax 40000; then
  echo "OK: sudo chrt -f 80 ./build/diag_cst_single --fw $FW --nm 0.3 --vmax 40000"
  exit 0
fi

# 1 N·m
if run_one nm1 --nm 1 --vmax 40000; then
  echo "OK: --nm 1"
  exit 0
fi

# 更严限速
if run_one nm03_v20k --nm 0.3 --vmax 20000; then
  echo "OK: --nm 0.3 --vmax 20000"
  exit 0
fi

echo "See $LOG_DIR"; exit 1
