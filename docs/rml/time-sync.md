---
title: Time Sync (chrony)
password: minions
tags:
  - rml
  - networking
  - chrony
---

# :material-clock-time-eight: Time Sync (chrony)

The remote laptop, robot, and mocap PC must share a clock — otherwise mocap-vs-IMU fusion drifts and bag timestamps lose meaning. We use **chrony** with the lab laptop (`192.168.8.103`) as the time server.

## On the robot — `/etc/chrony/chrony.conf`

Add the laptop as a preferred server and comment out any internet pools so the robot doesn't drift to NTP servers it can't reach when offline:

```ini
server 192.168.8.103 iburst prefer

# Comment out the default Ubuntu pools:
#pool ntp.ubuntu.com         iburst maxsources 4
#pool 0.ubuntu.pool.ntp.org  iburst maxsources 1
#pool 1.ubuntu.pool.ntp.org  iburst maxsources 1
#pool 2.ubuntu.pool.ntp.org  iburst maxsources 2
```

## On the remote laptop — `/etc/chrony/chrony.conf`

The remote can either serve time itself (most common — no internet required) or sync to CMU NTP. The minimum config that lets the robot lock onto the laptop:

```ini
# Fast initial step + periodic resync to RTC
makestep 0.1 3
rtcsync

# Allow the robot subnet to query us
allow 192.168.8.0/24
```

If the laptop has internet, also keep one upstream:

```ini
pool ntp.ubuntu.com iburst maxsources 4
```

## Apply changes

After editing `chrony.conf`, restart and verify:

```bash
sudo systemctl restart chrony
sudo systemctl status chrony.service
```

You should see `chrony` `active (running)`. To confirm the robot is actually locked onto the laptop, on the **robot**:

```bash
chronyc sources -v
```

The line for `192.168.8.103` should show `^*` (currently selected). If it shows `^?` it's unreachable; check the laptop is up on `.103` and `chrony` is allowing the subnet.

## A minimal chrony.conf for the robot

If you want to overwrite the file wholesale (cleaner than editing in place):

```bash
sudo tee /etc/chrony/chrony.conf << 'EOF'
server 192.168.8.103 iburst prefer
makestep 0.1 3
rtcsync
EOF

sudo systemctl restart chrony
```

## Common failure modes

??? warning "Robot clock is jumping ±seconds during a run"
    `makestep` is allowed to do a hard step for the first 3 corrections only. After that chrony slews. If the robot was offline for hours its drift may exceed the slew rate — `sudo chronyc -a makestep` forces a step.

??? warning "All robots show `^?` against the laptop"
    Laptop firewall is blocking UDP/123. Either disable the firewall or add an explicit allow rule for `192.168.8.0/24` on UDP/123.

??? warning "Mocap timestamps disagree with the robot bag by ~minutes"
    The Mocap PC isn't on chrony — it's a Windows box. Either run a Windows NTP client pointing at the laptop, or accept the offset and strip mocap timestamps in post.

[:octicons-arrow-right-24: Motion capture setup](mocap.md)
