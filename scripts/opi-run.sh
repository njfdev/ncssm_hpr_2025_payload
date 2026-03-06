#!/usr/bin/env bash
# Start flight_computer on Orange Pi 4A over UART7.
# Usage: ./scripts/opi-run.sh [IP] [USER] [PASS] [UART_DEV]
set -euo pipefail

IP="${1:-172.31.255.248}"
USER="${2:-orangepi}"
PASS="${3:-orangepi}"
UART="${4:-/dev/ttyAS7}"
BAUD="${5:-115200}"

SSH="sshpass -p $PASS ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no $USER@$IP"

echo "=== Starting flight_computer on $UART @ $BAUD ==="
$SSH "killall flight_computer 2>/dev/null; true"
$SSH "echo '$PASS' | sudo -S chmod 666 $UART 2>/dev/null"
$SSH "nohup ~/flight_computer serial:${UART}:${BAUD} > ~/fc.log 2>&1 & disown"

sleep 2
$SSH 'pgrep -a flight_computer && echo "" && head -5 ~/fc.log'
echo ""
echo "=== flight_computer running. View logs: ssh $USER@$IP tail -f ~/fc.log ==="
