#!/bin/bash
# Run with: sudo bash joint-sdk/scripts/ecat_diag.sh
set -euo pipefail

echo "=== dmesg (EtherCAT / SAFEOP) ==="
dmesg 2>&1 | grep -iE 'EtherCAT|SAFEOP|Timeout|AL status|application time|did not sync' | tail -80 || true

echo
echo "=== ethercat slaves -v ==="
ethercat slaves -v

echo
echo "=== AL Status 0x130 ==="
ethercat reg_read -p0 0x130 2

echo
echo "=== AL Status Code 0x134 ==="
ethercat reg_read -p0 0x134 2

echo
echo "=== DC AssignActivate 0x980 ==="
ethercat reg_read -p0 0x980 2

echo
echo "=== Sync0/1 cycle 0x9a0 ==="
ethercat reg_read -p0 0x9a0 8

echo
echo "=== SM2/SM3 0x810 ==="
ethercat reg_read -p0 0x810 16

echo
echo "=== master ==="
ethercat master

echo
echo "=== slaves ==="
ethercat slaves
