#!/bin/bash
# One-shot verification for joint-sdk CSP example.
# Run in an interactive terminal (needs sudo password once):
#   bash ~/下载/EtherCAT_Master/joint-sdk/scripts/run_verify.sh

set -euo pipefail
cd "$(dirname "$0")/.."

echo "==> Fix /dev/EtherCAT0 permissions (needs sudo)..."
sudo chmod 666 /dev/EtherCAT0
ls -la /dev/EtherCAT0

echo "==> Rebuild..."
make all ETHERLAB_DIR=/usr/local

echo "==> Run csp_single_sdk (Ctrl+C to stop)..."
exec ./build/csp_single_sdk
