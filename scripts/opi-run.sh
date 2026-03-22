#!/usr/bin/env bash
# Start/restart flight_computer on Orange Pi 4A via systemd.
# Usage: ./scripts/opi-run.sh [IP] [USER] [PASS]
set -euo pipefail

IP="${1:-172.31.255.248}"
USER="${2:-orangepi}"
PASS="${3:-orangepi}"

SSH="sshpass -p $PASS ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no $USER@$IP"

echo "=== Ensuring UART permissions ==="
$SSH "echo '$PASS' | sudo -S chmod 666 /dev/ttyAS4 /dev/ttyAS2 2>/dev/null; true"

echo "=== Restarting flight_computer service ==="
$SSH "echo '$PASS' | sudo -S systemctl restart flight_computer"

sleep 2
$SSH "systemctl status flight_computer --no-pager -l 2>/dev/null | head -15"
echo ""
echo "=== flight_computer running. View logs: ssh $USER@$IP tail -f ~/fc.log ==="
