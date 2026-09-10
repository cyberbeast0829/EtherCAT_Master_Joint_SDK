#!/usr/bin/env bash
# Dual CSV DC sweep: find period / Sync0 / shift with fewest AL/WC holes.
#
# Constraint: master task period_ns is ONE global value.
# Per-joint may differ sync0_cycle_ns + sync0_shift_ns only.
#
# Usage (from joint-sdk):
#   sudo bash scripts/dc_dual_sweep.sh
#
# Scores PASS first, then min recoveries, then min wc_drops.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
SETUP=./build/dc_timing_setup
DIAG=./build/diag_csv_dual
RESULTS=/tmp/dc_dual_sweep_$$.tsv
BEST_CONF=config/dc_timing.conf

if [[ ! -x $SETUP || ! -x $DIAG ]]; then
  echo "build tools first: make all dc-setup" >&2
  exit 1
fi

USE_CHRT=0
if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
  USE_CHRT=1
elif [[ -w /dev/EtherCAT0 ]]; then
  echo "WARN: not root — running WITHOUT chrt (SCHED_FIFO may be unavailable)." >&2
  echo "WARN: ranking is approximate; re-confirm best with:" >&2
  echo "  sudo bash $0" >&2
else
  echo "need /dev/EtherCAT0 access + preferably root:" >&2
  echo "  sudo bash $0" >&2
  exit 1
fi

if [[ ! -e /dev/EtherCAT0 ]]; then
  MAJOR=$(awk '/EtherCAT/ {print $1}' /proc/devices)
  [[ -n "$MAJOR" ]] || { echo "ec_master not loaded" >&2; exit 1; }
  mknod -m 666 /dev/EtherCAT0 c "$MAJOR" 0
fi

echo -e "id\tperiod\ts50_sync\ts50_shift\ts44_sync\ts44_shift\texit\tpass\trecover\twc_drops\tsettle\tok0\tok1\tnote" \
  >"$RESULTS"

run_one() {
  local id="$1" period="$2" s50="$3" sh50="$4" s44="$5" sh44="$6" note="$7"
  local LOG=/tmp/dc_dual_${id}.log
  echo "=== $id  period=$period  50:sync=$s50/sh=$sh50  44:sync=$s44/sh=$sh44  ($note) ==="

  "$SETUP" --per-joint --period "$period" --fw 8.1.50 \
    --sync0-cycle "$s50" --sync0-shift "$sh50" --wait-ms 100 >/dev/null
  "$SETUP" --per-joint --period "$period" --fw 8.1.44 \
    --sync0-cycle "$s44" --sync0-shift "$sh44" --wait-ms 100 >/dev/null

  ethercat states -p0 INIT >/dev/null 2>&1 || true
  ethercat states -p1 INIT >/dev/null 2>&1 || true
  sleep 0.5

  set +e
  if [[ $USE_CHRT -eq 1 ]]; then
    chrt -f 80 "$DIAG" --fw0 8.1.50 --fw1 8.1.44 --rps 2 --seconds 14 \
      >"$LOG" 2>&1
  else
    "$DIAG" --fw0 8.1.50 --fw1 8.1.44 --rps 2 --seconds 14 \
      >"$LOG" 2>&1
  fi
  local EC=$?
  set -e

  local PASS=0
  grep -q '^PASS:' "$LOG" && PASS=1
  local REC
  REC=$(grep -c 'bus recovered' "$LOG" || true)
  local WC=999
  if grep -q 'wc_drops=' "$LOG"; then
    WC=$(grep 'wc_drops=' "$LOG" | tail -1 | sed -n 's/.*wc_drops=\([0-9]*\).*/\1/p')
  fi
  local SETTLE=0
  grep -q 'bus settle ok' "$LOG" && SETTLE=1
  local OK0=-1 OK1=-1
  if grep -q '^summary j0:' "$LOG"; then
    OK0=$(grep '^summary j0:' "$LOG" | sed -n 's/.*ok=\([0-9]*\)\/\([0-9]*\).*/\1\/\2/p')
    OK1=$(grep '^summary j1:' "$LOG" | sed -n 's/.*ok=\([0-9]*\)\/\([0-9]*\).*/\1\/\2/p')
  fi

  echo -e "${id}\t${period}\t${s50}\t${sh50}\t${s44}\t${sh44}\t${EC}\t${PASS}\t${REC}\t${WC}\t${SETTLE}\t${OK0}\t${OK1}\t${note}" \
    | tee -a "$RESULTS"
  grep -E 'PASS|FAIL|wc_drops=|summary j' "$LOG" | head -6 || true
  echo
}

# --- matrix ---------------------------------------------------------------
# User single-motor baselines:
#   8.1.50: task/sync0=2ms shift=300us
#   8.1.44: task/sync0=1ms shift=250us
# Global task must pick one; prefer 2ms so 8.1.50 keeps its Sync0=task.

run_one U0 2000000 2000000 300000 1000000 250000 "user_single_combo"
run_one U1 2000000 2000000 300000 2000000 250000 "2ms_both_shift_300_250"
run_one U2 2000000 2000000 300000 2000000 300000 "2ms_both_shift_300"
run_one U3 2000000 2000000 250000 1000000 250000 "2ms_50sh250_44_1ms250"
run_one U4 2000000 2000000 350000 1000000 250000 "2ms_50sh350_44_1ms250"
run_one U5 2000000 2000000 300000 1000000 300000 "2ms_44_1ms_sh300"
run_one U6 2000000 2000000 300000 1000000 200000 "2ms_44_1ms_sh200"
run_one U7 2000000 2000000 400000 1000000 250000 "2ms_50sh400_44_1ms250"
run_one U8 2000000 2000000 300000 2000000 200000 "2ms_both_44sh200"
run_one U9 2000000 2000000 250000 2000000 250000 "2ms_both_sh250"

run_one V0 1000000 1000000 300000 1000000 250000 "1ms_50sh300_44sh250"
run_one V1 1000000 1000000 250000 1000000 250000 "1ms_both_sh250"
run_one V2 1000000 1000000 300000 1000000 300000 "1ms_both_sh300"
run_one V3 1000000 2000000 300000 1000000 250000 "1ms_task_50_sync2ms"
run_one V4 1000000 1000000 350000 1000000 250000 "1ms_50sh350_44sh250"

run_one W0 4000000 4000000 500000 2000000 250000 "4ms_mixed"
run_one W1 2000000 2000000 500000 1000000 250000 "2ms_50sh500_44_1ms"

echo "===== RESULTS ($RESULTS) ====="
column -t -s $'\t' "$RESULTS" 2>/dev/null || cat "$RESULTS"

# Rank: pass desc, recover asc, wc_drops asc, settle desc
BEST=$(awk -F'\t' 'NR>1 {
  # pass, -recover, -wc, settle
  key = sprintf("%d %05d %05d %d", $8, 9999-$9, 9999-($10==""?999:$10), $11)
  print key "\t" $0
}' "$RESULTS" | sort -r | head -1 | cut -f2-)

if [[ -z "$BEST" ]]; then
  echo "no ranked row" >&2
  exit 1
fi

BID=$(echo "$BEST" | cut -f1)
BPERIOD=$(echo "$BEST" | cut -f2)
BS50=$(echo "$BEST" | cut -f3)
BSH50=$(echo "$BEST" | cut -f4)
BS44=$(echo "$BEST" | cut -f5)
BSH44=$(echo "$BEST" | cut -f6)
BNOTE=$(echo "$BEST" | cut -f14)

echo
echo "===== BEST: $BID ($BNOTE) ====="
echo "  period=$BPERIOD"
echo "  8.1.50 sync0=$BS50 shift=$BSH50"
echo "  8.1.44 sync0=$BS44 shift=$BSH44"

"$SETUP" --per-joint --period "$BPERIOD" --fw 8.1.50 \
  --sync0-cycle "$BS50" --sync0-shift "$BSH50" --wait-ms 100
"$SETUP" --per-joint --period "$BPERIOD" --fw 8.1.44 \
  --sync0-cycle "$BS44" --sync0-shift "$BSH44" --wait-ms 100
"$SETUP" --show
echo "locked → $BEST_CONF"
echo "confirm:"
echo "  sudo chrt -f 80 $DIAG --fw0 8.1.50 --fw1 8.1.44 --rps 2 --seconds 14"
