#!/usr/bin/env bash
# One-time setup for Orange Pi 4A: enable UART7, configure static IP, deploy binary.
# Usage: ./scripts/opi-setup.sh [IP] [USER] [PASS]
set -euo pipefail

IP="${1:-172.31.255.248}"
USER="${2:-orangepi}"
PASS="${3:-orangepi}"

SSH="sshpass -p $PASS ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no $USER@$IP"

command -v sshpass >/dev/null || { echo "Install sshpass: brew install sshpass"; exit 1; }

echo "=== Enabling UART7 overlay ==="
$SSH "echo '$PASS' | sudo -S bash -c '
  if grep -q uart7 /boot/orangepiEnv.txt 2>/dev/null; then
    echo \"UART7 already enabled\"
  else
    sed -i \"s/^overlays=\\(.*\\)/overlays=\\1 uart7/\" /boot/orangepiEnv.txt
    echo \"UART7 overlay added\"
  fi
  cat /boot/orangepiEnv.txt
'"

echo ""
echo "=== Configuring static IP ($IP) ==="
$SSH "echo '$PASS' | sudo -S bash -c '
  # Netplan config
  mkdir -p /etc/netplan
  cat > /etc/netplan/01-eth0-static.yaml <<NETPLAN
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      addresses:
        - $IP/24
NETPLAN

  # Systemd service to activate on boot
  cat > /etc/systemd/system/eth0-static-ip.service <<SVC
[Unit]
Description=Activate static IP on eth0
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
ExecStartPre=/bin/sleep 3
ExecStart=/usr/bin/nmcli con up netplan-eth0
RemainAfterExit=yes
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
SVC

  systemctl daemon-reload
  systemctl enable eth0-static-ip.service
  echo \"Static IP service enabled\"
'"

echo ""
echo "=== Setup complete. Reboot the Orange Pi for changes to take effect ==="
echo "  Run: ./scripts/opi-deploy.sh"
