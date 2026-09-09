#!/usr/bin/env bash
# task = Sync0 = 2 ms; sweep only shift for 8.1.50 (pos0) and 8.1.44 (pos1).
set -euo pipefail
cd "$(cd "$(dirname "$0")/.." && pwd)"
SETUP=./build/dc_timing_setup
DIAG=./build/diag_csv_dual
TSV=/tmp/dc_shift_2ms.tsv
PERIOD=2000000
SYNC=2000000

echo -e "kind\tsh50\tsh44\texit\tpass\trecover\twc\tsettle\tok0\tok1" >"$TSV"

run() {
  local kind="$1" sh50="$2" sh44="$3"
  local log=/tmp/dc_sh_${kind}_${sh50}_${sh44}.log
  echo "=== $kind  8.1.50=$sh50  8.1.44=$sh44 ==="
  "$SETUP" --per-joint --period "$PERIOD" --fw 8.1.50 \
    --sync0-cycle "$SYNC" --sync0-shift "$sh50" --wait-ms 100 >/dev/null
  "$SETUP" --per-joint --period "$PERIOD" --fw 8.1.44 \
    --sync0-cycle "$SYNC" --sync0-shift "$sh44" --wait-ms 100 >/dev/null
  ethercat states -p0 INIT >/dev/null 2>&1 || true
  ethercat states -p1 INIT >/dev/null 2>&1 || true
  sleep 0.4
  set +e
  "$DIAG" --fw0 8.1.50 --fw1 8.1.44 --rps 2 --seconds 14 >"$log" 2>&1
  local ec=$?
  set -e
  local pass=0 rec=0 wc=999 settle=0 ok0=-1 ok1=-1
  grep -q '^PASS:' "$log" && pass=1
  rec=$(grep -c 'bus recovered' "$log" || true)
  grep -q 'bus settle ok' "$log" && settle=1
  if grep -q 'wc_drops=' "$log"; then
    wc=$(grep 'wc_drops=' "$log" | tail -1 | sed -n 's/.*wc_drops=\([0-9]*\).*/\1/p')
  fi
  if grep -q '^summary j0:' "$log"; then
    ok0=$(sed -n 's/^summary j0:.*ok=\([0-9]*\/[0-9]*\).*/\1/p' "$log")
    ok1=$(sed -n 's/^summary j1:.*ok=\([0-9]*\/[0-9]*\).*/\1/p' "$log")
  fi
  echo -e "${kind}\t${sh50}\t${sh44}\t${ec}\t${pass}\t${rec}\t${wc}\t${settle}\t${ok0}\t${ok1}" \
    | tee -a "$TSV"
  grep -E 'PASS|FAIL|wc_drops=' "$log" | head -3 || true
  echo
}

# Common window: same shift on both
for sh in 150000 200000 250000 300000 350000 400000 450000 500000; do
  run same "$sh" "$sh"
done

# 8.1.50 at 250us, sweep 8.1.44
for sh44 in 150000 200000 300000 350000 400000; do
  run a250 250000 "$sh44"
done

# 8.1.50 at 300us (single-motor), sweep 8.1.44
for sh44 in 150000 200000 250000 350000 400000; do
  run a300 300000 "$sh44"
done

echo "===== TSV ====="
cat "$TSV"
