---
name: orangepi-ssh
description: >-
  Interface with the Orange Pi payload computer over SSH for development and debugging.
  Use when connecting to the Orange Pi, running commands remotely, deploying code,
  checking system status, or troubleshooting the payload computer.
license: MIT
compatibility: Claude Code, Codex CLI, VS Code Copilot, Cursor
metadata:
  author: ncssm-robotics
  version: "1.0.0"
  category: hardware
---

# Orange Pi SSH Interface

This skill helps you interface with the Orange Pi payload computer over SSH for the NCSSM HPR 2025 Payload project.

## Connection Details

| Parameter | Value |
|-----------|-------|
| **Host** | 192.168.137.70 |
| **User** | orangepi |
| **Password** | orangepi |
| **Protocol** | SSH |

## Quick Connect

### Basic SSH Connection

```bash
ssh orangepi@192.168.137.70
```

When prompted for password, enter: `orangepi`

### Using sshpass for Non-Interactive Sessions

```bash
sshpass -p 'orangepi' ssh orangepi@192.168.137.70
```

### Running a Single Command

```bash
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'command_here'
```

Example:
```bash
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'uname -a'
```

## Common Operations

### Check System Status

```bash
# System info
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'uname -a && cat /etc/os-release'

# CPU and memory
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'top -bn1 | head -20'

# Disk usage
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'df -h'

# Running processes
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'ps aux'
```

### File Transfer with SCP

```bash
# Copy file TO Orange Pi
sshpass -p 'orangepi' scp local_file.txt orangepi@192.168.137.70:/home/orangepi/

# Copy file FROM Orange Pi
sshpass -p 'orangepi' scp orangepi@192.168.137.70:/home/orangepi/remote_file.txt ./

# Copy directory TO Orange Pi
sshpass -p 'orangepi' scp -r local_dir/ orangepi@192.168.137.70:/home/orangepi/

# Copy directory FROM Orange Pi
sshpass -p 'orangepi' scp -r orangepi@192.168.137.70:/home/orangepi/remote_dir/ ./
```

### File Transfer with rsync

```bash
# Sync directory TO Orange Pi (more efficient for large transfers)
sshpass -p 'orangepi' rsync -avz --progress local_dir/ orangepi@192.168.137.70:/home/orangepi/remote_dir/

# Sync directory FROM Orange Pi
sshpass -p 'orangepi' rsync -avz --progress orangepi@192.168.137.70:/home/orangepi/remote_dir/ ./local_dir/
```

## Patterns

- Always check connectivity before running complex commands: `ping -c 1 192.168.137.70`
- Use `sshpass` for automated/scripted operations
- Use `-o StrictHostKeyChecking=no` to skip host key verification in automated scripts
- Prefer `rsync` over `scp` for large file transfers or directory syncs
- Use `nohup` or `screen`/`tmux` for long-running processes that should survive disconnection

## Anti-Patterns

- Never store the password in version-controlled files
- Avoid running commands without checking if the device is reachable first
- Don't use `-o StrictHostKeyChecking=no` in production/security-sensitive contexts
- Don't run destructive commands (`rm -rf`, `dd`, etc.) without double-checking the target

## Examples

### Good: Check if Orange Pi is reachable before connecting

```bash
ping -c 1 192.168.137.70 && sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'echo "Connected!"'
```

### Good: Deploy and run a script

```bash
# Copy script to Orange Pi
sshpass -p 'orangepi' scp my_script.py orangepi@192.168.137.70:/home/orangepi/

# Make it executable and run
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'chmod +x /home/orangepi/my_script.py && python3 /home/orangepi/my_script.py'
```

### Good: Run a long process in background

```bash
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'nohup python3 /home/orangepi/data_collector.py > /home/orangepi/output.log 2>&1 &'
```

### Bad: Running without connectivity check

```bash
# May hang indefinitely if Orange Pi is not reachable
sshpass -p 'orangepi' ssh orangepi@192.168.137.70 'some_command'
```

## Troubleshooting

### Connection Refused

If you get "Connection refused":
1. Verify the Orange Pi is powered on
2. Check that SSH service is running: the Orange Pi should have `sshd` enabled by default
3. Verify network connectivity: `ping 192.168.137.70`
4. Check firewall settings on the Orange Pi

### Permission Denied

If you get "Permission denied":
1. Verify the password is correct: `orangepi`
2. Check that password authentication is enabled in `/etc/ssh/sshd_config`

### Host Key Changed

If you get a host key warning:
```bash
# Remove old key (only if you trust the device)
ssh-keygen -R 192.168.137.70

# Then reconnect
ssh orangepi@192.168.137.70
```

## Network Configuration

The Orange Pi is configured on the network `192.168.137.x`. Ensure your development machine is on the same network or has a route to it.

If using a direct connection, you may need to configure a static IP on your machine in the `192.168.137.x` range (e.g., `192.168.137.1`).
