# Network connection

This document describes common ways to connect to Unitree robots.

## Ethernet (robot LAN)

Typical static IPs:

| Robot | MCU IP | Jetson IP | Jetson username | Jetson password |
| --- | --- | --- | --- | --- |
| Go2 | `192.168.123.161` | `192.168.123.18` | `unitree` | `123` |
| G1  | `192.168.123.161` | `192.168.123.164` | `unitree` | `123` |

Quick checks:

```bash
ping -c 1 192.168.123.161
ping -c 1 192.168.123.18  # (Go2 Jetson)
ping -c 1 192.168.123.164 # (G1 Jetson)
```

SSH (X11 forwarding optional via `-Y`):

```bash
ssh -Y unitree@192.168.123.18
ssh -Y unitree@192.168.123.164
```

## Wi‑Fi (USB adapter on Jetson)

For the full driver + routing workflow (including rfkill fixes), see:

- [WIFI.md](WIFI.md)

After the Jetson is connected to Wi‑Fi, connect over SSH using the Wi‑Fi IP:

```bash
ssh -Y unitree@<wifi-ip>
```

## Tailscale (recommended for remote access)

Wi‑Fi typically uses DHCP, so the Jetson IP address may change. Tailscale
provides a stable tailnet IP and hostname for SSH/NoMachine access.

Install and bring it up:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

Show your Tailscale IP and connect:

```bash
tailscale ip -4
ssh -Y unitree@<tailscale-ip>
```
