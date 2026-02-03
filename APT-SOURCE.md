# APT sources (Ubuntu + ROS)

This document provides a minimal, safe way to switch APT sources on Ubuntu.
It supports both ARM (Jetson / aarch64) and x86_64.

This guide assumes Ubuntu 20.04 (codename: `focal`). If you are on a different
release (e.g., `jammy`), replace `focal` everywhere accordingly.

## 1) Back up existing sources

Always back up before modifying sources.

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo mkdir -p /etc/apt/sources.list.d/backup
sudo cp /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/backup/ 2>/dev/null || true
```

Tip: confirm your Ubuntu codename with:

```bash
. /etc/os-release
echo "$VERSION_CODENAME"
```

## 2) Switch Ubuntu official sources

### ARM (aarch64 / Jetson / SBC)

```bash
sudo tee /etc/apt/sources.list << 'EOF'
deb http://ports.ubuntu.com/ubuntu-ports focal main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports focal-updates main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports focal-backports main restricted universe multiverse
deb http://ports.ubuntu.com/ubuntu-ports focal-security main restricted universe multiverse
EOF
```

### x86_64 (Desktop / Workstation)

```bash
sudo tee /etc/apt/sources.list << 'EOF'
deb http://archive.ubuntu.com/ubuntu focal main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu focal-updates main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu focal-backports main restricted universe multiverse
deb http://archive.ubuntu.com/ubuntu focal-security main restricted universe multiverse
EOF
```

## 3) Switch ROS / ROS 2 sources

### Remove old ROS sources (if any)

```bash
sudo rm -f /etc/apt/sources.list.d/ros*.list
```

### Add ROS official GPG key (modern method)

```bash
sudo mkdir -p /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add ROS / ROS 2 repositories

If you only use ROS 1 or only ROS 2, you can keep just the relevant line.

```bash
sudo tee /etc/apt/sources.list.d/ros-official.list << 'EOF'
deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main
deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main
EOF
```

## 4) Update and verify

```bash
sudo apt update
```

Expected:
- No GPG errors
- No "repository is no longer signed"
- No malformed entry errors

Optional quick checks:

```bash
apt-cache policy | head -n 40
grep -R "^deb " /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null || true
```

## Notes

- This setup does NOT upgrade the system.
- Avoid running:
      sudo apt full-upgrade
  on Jetson / robot systems.
- NVIDIA/L4T repositories are intentionally not included here; keep the vendor
    sources your platform requires.
