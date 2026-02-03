# Wi-Fi (USB adapter on Jetson)

This guide shows how to enable and use a USB Wi‑Fi adapter on the onboard
Jetson computer of a Unitree robot.

The process has two steps:

1. Install a compatible USB Wi‑Fi driver for your Jetson kernel.
2. Prefer Wi‑Fi for internet routing while keeping ethernet for internal robot communication.

The examples below use a Realtek RTL8852BU adapter, but the same workflow
applies to other USB Wi‑Fi adapters.

Note: interface names vary by system. This guide uses `wlan0` (Wi‑Fi) and `eth1`
(robot/internal). Confirm your actual names with `ip link`.

## Step 1 — Install USB Wi‑Fi driver (Jetson)

### 1. Identify the USB Wi‑Fi device

```bash
lsusb
```

Example:

```text
Bus 001 Device 004: ID 0bda:a85b Realtek Semiconductor Corp.
```

Always use the USB device ID as the primary identifier when selecting a driver.

---

### 2. Check kernel version

```bash
uname -r
```

Example:

```text
5.10.104-tegra
```

Driver compatibility strictly depends on the kernel version.

---

### 3. Select a compatible driver

When selecting a driver, match BOTH:
- USB device ID
- Kernel version

Example (RTL8852BU on Jetson 5.10):

https://github.com/morrownr/rtl8852bu-20240418

Notes:
- Different adapters require different drivers
- Newer driver versions may require newer kernels
- Always verify `supported-device-IDs` and `README.md` in the driver repository

---

### 4. Install the driver

Follow the instructions provided by the selected driver repository.

Typical example:

```bash
sudo ./install-driver.sh
```

---

### 5. Verify driver installation

```bash
lsmod | grep -iE 'wifi|wlan|882|8852|rtw|rtl'
ip link
```

Expected:
    wlan0 / wlp* / wlx*

---

### 6. Fix rfkill soft-block (common on Jetson)

On Jetson platforms, some USB Wi-Fi drivers initialize the interface in a
soft-blocked rfkill state during probe().

If Wi-Fi is soft-blocked by rfkill, NetworkManager will report the device as
unavailable, even though the driver is correctly loaded.

Typical symptoms:

```bash
nmcli dev
```

```text
wlan0    wifi    unavailable    --
```

```bash
rfkill list
```

```text
Soft blocked: yes
Hard blocked: no
```

### Permanent fix (recommended)

Disable rfkill state persistence:

```bash
sudo systemctl mask systemd-rfkill.service
sudo systemctl mask systemd-rfkill.socket
```

Add a udev rule to unblock Wi-Fi at device appearance:

```bash
sudo tee /etc/udev/rules.d/99-fix-wifi-rfkill.rules << 'EOF'
ACTION=="add|change", SUBSYSTEM=="rfkill", ATTR{type}=="wlan", RUN+="/usr/sbin/rfkill unblock wifi"
EOF
```

Reload rules and reboot:

```bash
sudo udevadm control --reload-rules
sudo reboot
```

Verify after reboot:

```bash
rfkill list
```

Expected:
    Soft blocked: no
    Hard blocked: no

To undo this change later:

```bash
sudo systemctl unmask systemd-rfkill.service
sudo systemctl unmask systemd-rfkill.socket
sudo rm -f /etc/udev/rules.d/99-fix-wifi-rfkill.rules
sudo udevadm control --reload-rules
```

## Step 2 — Prefer Wi‑Fi as the default internet route

Goal:
- wlan0 → internet access
- eth1  → internal Unitree robot network

Linux has no concept of a “default interface”.
The default network is determined by route priority (metric).

---

### 1. Identify NetworkManager connections

```bash
nmcli connection show
```

Example:
    NAME        DEVICE
    wifi-net    wlan0
    unitree1    eth1

---

### 2. Set route priority (Wi‑Fi preferred)

```bash
nmcli connection modify wifi-net ipv4.route-metric 100
nmcli connection modify unitree1 ipv4.route-metric 600
```

Rule:
- Lower metric = higher priority

---

### 3. Reactivate connections

```bash
nmcli connection down wifi-net && nmcli connection up wifi-net
nmcli connection down unitree1 && nmcli connection up unitree1
```

---

### 4. Verify routing

```bash
ip route
ip route get 8.8.8.8
```

Expected:
    dev wlan0

---

## Optional (recommended for robots)

Prevent ethernet from ever becoming the default route:

```bash
nmcli connection modify unitree1 ipv4.never-default yes
nmcli connection down unitree1 && nmcli connection up unitree1
```

## Notes

- Ethernet is reserved for internal robot communication.
- Wi-Fi is used for external internet access.
- If the Wi-Fi network requires captive-portal authentication
  (e.g. campus networks), use NoMachine.

## Tailscale 
wifi ofen use DHCP so you will have a dynamic IP, to get a fixed one, use tailscale

curl -fsSL https://tailscale.com/install.sh | sh