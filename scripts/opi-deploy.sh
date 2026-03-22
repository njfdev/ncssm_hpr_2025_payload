#!/usr/bin/env bash
# Cross-compile flight_computer and deploy to Orange Pi 4A.
# Usage: ./scripts/opi-deploy.sh [IP] [USER] [PASS]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
IP="${1:-172.31.255.248}"
USER="${2:-orangepi}"
PASS="${3:-orangepi}"

SCP="sshpass -p $PASS scp -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no"
SSH="sshpass -p $PASS ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no $USER@$IP"

command -v sshpass >/dev/null || { echo "Install sshpass: brew install sshpass"; exit 1; }
command -v cargo-zigbuild >/dev/null || { echo "Install cargo-zigbuild: cargo install cargo-zigbuild"; exit 1; }
command -v zig >/dev/null || { echo "Install zig: brew install zig"; exit 1; }

echo "=== Cross-compiling flight_computer for aarch64 ==="
cd "$PROJECT_DIR/flight_computer"
rustup target add aarch64-unknown-linux-gnu 2>/dev/null || true
cargo zigbuild --release --target aarch64-unknown-linux-gnu

BINARY="$PROJECT_DIR/flight_computer/target/aarch64-unknown-linux-gnu/release/flight_computer"
SERVICE="$SCRIPT_DIR/flight_computer.service"
echo ""
echo "=== Deploying to $USER@$IP ==="
$SSH "echo '$PASS' | sudo -S systemctl stop flight_computer 2>/dev/null; true"
$SSH 'killall flight_computer 2>/dev/null; true'
$SCP "$BINARY" "$USER@$IP:~/flight_computer"
$SSH 'chmod +x ~/flight_computer && echo "Deployed: $(ls -lh ~/flight_computer)"'

echo ""
echo "=== Installing systemd service ==="
$SCP "$SERVICE" "$USER@$IP:/tmp/flight_computer.service"
$SSH "echo '$PASS' | sudo -S bash -c '
    cp /tmp/flight_computer.service /etc/systemd/system/flight_computer.service
    chmod 666 /dev/ttyAS4 /dev/ttyAS2 2>/dev/null || true
    mkdir -p /home/orangepi/flight_data
    chown orangepi:orangepi /home/orangepi/flight_data
    systemctl daemon-reload
    systemctl enable flight_computer
    echo \"Service installed and enabled\"
'"

echo ""
echo "=== Deploy complete ==="
echo "  Start now:    ./scripts/opi-run.sh"
echo "  Auto-starts on boot via systemd"
