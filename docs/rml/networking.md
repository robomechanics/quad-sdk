---
title: Network & IP Map
password: R0b0mech
tags:
  - rml
  - networking
---

# :material-lan: Network & IP Map

The RML mocap network is a TP-Link router (`192.168.8.0/24`) sitting between the lab laptop and the Unitree fleet. The Unitree internal Ethernet bridge to the low-level MCU is on `192.168.123.0/24` — these two subnets must not collide.

## Static IP map

| Device | IP | Purpose |
|---|---|---|
| **Mocap Router (TP-Link)** | `192.168.8.1` | Settings: <https://192.168.8.1> |
| **Theodore** (Go2) | `192.168.8.18` | Robot |
| **Alvin** (Go2) | `192.168.8.19` | Robot |
| **Simon** (Go2-W) | `192.168.8.20` | Robot |
| **Mocap PC** | `192.162.8.104` | Optitrack/Motive host |
| **Legged Controls laptop** | `192.168.8.103` | Quad-SDK remote driver |
| Unitree low-level MCU | `192.168.123.161` | Reserved — **never reassign** |

!!! warning "DNS field is required on the TP-Link"
    When you assign a static IP on the TP-Link DHCP page, **also set a DNS address** (use the router itself: `192.168.8.1`). If you leave DNS blank the assignment silently fails.

!!! danger "192.168.123.161 is reserved"
    The Unitree low-level board uses `192.168.123.161` for its internal Ethernet bridge. Confirm DHCP never offers this address to anything else on the network.

## Required IP assignments

Two subnets are in play, and **each side needs its own address on the right one**. If either is missing, `robot_driver` will fail to come up.

| Where | Interface | Subnet | Talks to |
|---|---|---|---|
| **Robot** | `eth0` | `192.168.123.0/24` | Unitree low-level MCU (`192.168.123.161`) |
| **Workstation / laptop** | Ethernet cable **or** Wi-Fi adapter on the mocap router | `192.168.8.0/24` | Robot (and the rest of the lab network) |

In short:

- The robot's `eth0` must hold a static address in `192.168.123.X` (any host except `.161`) so it can reach the low-level board. Apply the [netplan template below](#connecting-the-internet-directly-to-the-robot) to set this.
- The machine running the remote driver — whether tethered via Ethernet through the TP-Link or joined over Wi-Fi — must hold an address in `192.168.8.X`. If the laptop falls back to a CMU-DHCP address (`172.26.x.x`), it cannot reach the robot.

!!! tip "Verify before launching"
    `launch_robot_env.sh` prints the addresses it sees on both interfaces and flags whether they match the expected subnets. **Source it before `ros2 launch quad_utils robot_driver.py`** — a mis-assigned NIC is the most common cause of driver startup failures. If the script reports an interface on the wrong subnet, fix the network config first (netplan on the robot, TP-Link DHCP page for the laptop) and re-source it.

```bash
cd /root/ros2_ws/src/quad-sdk/quad_utils/scripts
source launch_robot_env.sh
```

## Connecting the internet directly to the robot

The robot has two NICs:

- `eth0` — wired connection to the mocap router (static)
- `eth2` — uplink for general internet (DHCP)

Apply the netplan template below on the robot to fix `eth0` while letting `eth2` take a DHCP lease:

```bash
sudo tee /etc/netplan/*.yaml << 'EOF'
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.123.222/24
    eth2:
      dhcp4: yes
EOF

sudo systemctl enable NetworkManager
sudo systemctl restart NetworkManager
sudo netplan apply
```

The `192.168.123.222` value is the robot's **own** address on the internal-MCU subnet — must be in `123.0/24` but **not** `123.161` (reserved).

## Resetting default-route priority

If the robot ends up routing through CMU internet when you want it on the local mocap network (or vice versa), inspect and prune the routing table:

```bash
ip route
sudo ip route del default via <gateway> dev <iface>
```

Conventions for telling them apart:

- **Local mocap addresses** start with `192.168.8.x`
- **CMU campus addresses** start with `172.26.x.x`

Example — drop a stale CMU default that's stealing traffic:

```bash
sudo ip route del default via 172.26.0.1 dev eth2
```

## SSH

Through the mocap router:

```bash
ssh unitree@192.168.8.18   # Theodore — change last octet for Alvin/Simon
# Password: 123
```

!!! note "Default credential"
    `unitree:123` is the manufacturer default. Change it for any robot leaving the lab — and obviously don't expose port 22 to the internet.

[:octicons-arrow-right-24: Time sync (chrony)](time-sync.md)
