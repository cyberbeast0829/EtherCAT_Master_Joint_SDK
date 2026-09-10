#!/usr/bin/env bash
# Dual CSP: soft-ramp @ --speed, high 0x6080 floor (motors must spin).
set -euo pipefail
cd "$(dirname "$0")/.."

SPEED="${SPEED:-4000}"
DELTA="${DELTA:-18192}"
SECS="${SECS:-15}"

echo "diag_csp_dual delta=$DELTA speed=$SPEED seconds=$SECS"
exec sudo chrt -f 80 ./build/diag_csp_dual \
  --fw0 8.1.50 --fw1 8.1.44 \
  --invert0 --no-invert1 \
  --delta "$DELTA" --speed "$SPEED" --seconds "$SECS"
