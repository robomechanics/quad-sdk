---
title: RML Lab Hub
password: R0b0mech
tags:
  - rml
  - internal
---

# :material-shield-lock: RML Lab Hub

!!! danger "Internal Robomechanics Lab content"
    This section contains lab-specific information for RML members. Some details (network IPs, default credentials, lab procedures) are not relevant to external users.

    For general Quad-SDK documentation, return to the [public docs](../index.md).

This hub collects everything an RML member needs to bring up Quad-SDK on the lab's Unitree fleet — from the network map to the 5-terminal launch procedure.

## What's in here

<div class="grid cards" markdown>

-   :material-lan:{ .lg .middle } **Network & IP map**

    ---

    Robot IPs, mocap router, static netplan, and the reserved Unitree MCU address.

    [:octicons-arrow-right-24: Networking](networking.md)

-   :material-clock-time-eight:{ .lg .middle } **Time sync (chrony)**

    ---

    Robot ↔ remote chrony.conf templates and restart commands.

    [:octicons-arrow-right-24: Time sync](time-sync.md)

-   :material-camera-control:{ .lg .middle } **Motion capture**

    ---

    Optitrack/Motive streaming, NatNet config, eSync 2 settings, UDP buffer sizing.

    [:octicons-arrow-right-24: Mocap setup](mocap.md)

-   :material-robot-industrial:{ .lg .middle } **Robot bringup**

    ---

    SSH, compile-on-robot vs compile-on-remote, and the canonical 5-terminal launch procedure.

    [:octicons-arrow-right-24: Bringup](robot-bringup.md)

-   :material-tools:{ .lg .middle } **Hardware design**

    ---

    Mounted rail dimensions, attachment design notes, Jetpack flashing.

    [:octicons-arrow-right-24: Hardware design](hardware-design.md)

</div>

## Lab fleet

| Robot | IP | Platform |
|---|---|---|
| **Theodore** | `192.168.8.18` | Unitree Go2 |
| **Alvin** | `192.168.8.19` | Unitree Go2 |
| **Simon** | `192.168.8.20` | Unitree Go2-W (wheeled) |

Mocap router: `192.168.8.1` &middot; Mocap PC: `192.162.8.104` &middot; Lab laptop: `192.168.8.103`

(Unitree low-level MCU reserves `192.168.123.161` — never let DHCP assign that.)

## Maintainer

For questions about lab infrastructure, contact **David Ologan** (`dologan@andrew.cmu.edu`).

---

*Last revised from the internal hardware notes PDF. Updates welcome — edit pages here on a feature branch and PR them back.*
