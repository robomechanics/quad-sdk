# Go2 stock-controller bag recorder

Records **every** ROS 2 topic the Unitree Go2 exposes while it runs its stock
controller, on the onboard Jetson, then pushes the finished bag to the remote
computer over Wi-Fi with rsync and verifies it there.

```
Go2 stock controller ──native DDS, domain 0──▶ Jetson enP8p1s0 (192.168.123.18)
                                                  │  ros2 bag record -a --include-hidden-topics
                                                  ▼  (own container, subscribe-only)
                                         ~/unitree_bags/unitree_stock_<stamp>[_label]/
                                                  │  stop → finalize → SHA256SUMS
                                                  ▼  rsync over Wi-Fi, retried
                     rml_admin@<remote>:~/ros2_ws/rosbags/unitree_stock/<bag>/  (checksums verified)
```

| file | runs on | what |
|---|---|---|
| `../go2_bag_recorder.sh` | Jetson host | start / stop / finalize / transfer / recover |
| `../go2_bag_remote.sh` | remote computer | drives the Jetson script over SSH, verifies received bags |
| `Dockerfile` | Jetson | ROS 2 Jazzy + CycloneDDS + `unitree_go`, `unitree_api`, `unitree_hg` msgs |
| `cyclone_dds_unitree.xml` | (mounted into the recorder) | binds DDS to the Go2 Ethernet only |
| `extra_msgs/` | | two firmware types missing from unitree_ros2 (from unitree_sdk2 IDL) |

It is independent of Quad-SDK: no robot driver, planner, MPC, or 5-terminal
launch is needed, and it never publishes anything the stock controller reads.
Run it in a **plain shell on the Jetson host**, not inside the Quad-SDK dev
container (that container has no Docker access; the recorder brings its own).

## Where the Go2's topics actually are

Checked on alvin (`unitree-jetson-payload`) on 2026-10-16, not assumed:

| DDS domain / interface | Go2 topics discovered |
|---|---|
| **domain 0 on `enP8p1s0` (192.168.123.0/24)** | **119** (`/lowstate`, `/lowcmd`, `/sportmodestate`, `/wirelesscontroller`, `/utlidar/*`, `/frontvideostream`, `/api/*`, ...) |
| domain 42 (Quad-SDK) on `enP8p1s0` | 0 |
| domain 0 on Wi-Fi `wlx00c0cab02fa5` | 0 |

So the stock-controller graph is **domain 0, internal Ethernet, multicast
discovery**, the same as `unitree_ros2/setup.sh`. Quad-SDK's
`ROS_DOMAIN_ID=42` sees none of it. The recorder uses domain 0 and pins
CycloneDDS to the 192.168.123.0/24 interface (auto-detected). As a result, Wi-Fi drops can't
affect recording, and the recorder stays invisible to the Quad-SDK domain-42 graph.
`--include-hidden-topics` added nothing at the time of the check, but stays on.

Coverage of a typical recording (`recorder/coverage.txt` in every bag):

* **93 topics recorded** = every topic that has a publisher, except the four below.
  Topics that show up after `start` are picked up automatically (rosbag2
  polls discovery).
* **4 published but not recordable:** `/arm_Command` (`unitree_arm/msg/ArmString`),
  `/qt_add_edge`, `/qt_add_node`, `/qt_command` (`unitree_interfaces/msg/*`).
  No public `.msg` exists for them, and rosbag2 can only record types it can load.
  They were checked with a type-name stub, and the same method received
  `/lowstate` at 500 Hz in a control test. All four carried **0 messages in 30 s** of stock
  operation. They are arm and app-SLAM command channels.
* **23 subscriber-only** (e.g. `/api/sport_lease/request`, `/uslam/*`). Nobody
  publishes on them, so there is nothing to record. If a publisher appears, they
  get recorded.
* **Not accessible at all:** anything the stock controller keeps internal
  (gait phase, estimator internals, the controller's internal references) is never put on DDS, so
  no recorder can capture it. What *is* published includes `/lowcmd` at
  500 Hz, which is the stock controller's actual motor commands.

## One-time setup

**1. Recorder image (Jetson).** Already built on alvin. On another robot:
```bash
~/ros2_ws/src/quad-sdk/quad_utils/scripts/go2_bag_recorder.sh build
```

**2. DDS socket buffers (Jetson, needs sudo).** The kernel default (208 KiB) caps
CycloneDDS's receive buffer. No lidar clouds were lost at that size in testing,
but raise it for headroom (same 128 MiB `init_robot.sh` uses) and make it persist:
```bash
printf 'net.core.rmem_max=134217728\nnet.core.wmem_max=134217728\n' \
  | sudo tee /etc/sysctl.d/60-cyclonedds.conf && sudo sysctl --system >/dev/null
```

**3. SSH keys, both directions.**
* Jetson → remote (unattended transfer): already working on alvin. Elsewhere:
  `go2_bag_recorder.sh setup-ssh -r <remote-ip>` on the Jetson.
* Remote → Jetson (start/stop from the remote): on the **remote** computer:
  ```bash
  ssh-copy-id unitree@192.168.8.50        # enter the Jetson password once
  ```

**4. Get the remote script onto the remote computer.** It ships in this repo,
so either pull the branch there, or copy it once:
```bash
scp unitree@192.168.8.50:ros2_ws/src/quad-sdk/quad_utils/scripts/go2_bag_remote.sh ~/ros2_ws/src/quad-sdk/quad_utils/scripts/
```

**5. Check (from the remote):**
```bash
~/ros2_ws/src/quad-sdk/quad_utils/scripts/go2_bag_remote.sh -r alvin check
```
This checks the image, the Go2 Ethernet, MCU ping, rmem, free disk, Jetson → remote SSH and remote rsync.

## Usage

From the **remote computer** (robot: `alvin` | `theodore` | `simon` | an IP, default alvin):
```bash
cd ~/ros2_ws/src/quad-sdk/quad_utils/scripts
./go2_bag_remote.sh -r alvin start trot_test   # label optional
./go2_bag_remote.sh -r alvin status            # size, duration, topics subscribed
#   ... drive the Go2 with the stock controller ...
./go2_bag_remote.sh -r alvin stop              # finalize, transfer, wait, verify here
```
`stop` returns once the bag has arrived and passed `verify`. Ctrl-C while it
waits is safe: the transfer keeps running on the Jetson. Other commands:
`list`, `log [NAME]`, `topics [SECS]`, `retry` (re-send every untransferred
bag), `verify [NAME|DIR]`, `stop --no-wait`.

`start` tells the Jetson to send the bag back to **whichever address the
remote has on the robot's subnet right now** (`ip route get`), so a changed
remote IP needs no config. Bag names use the remote's timezone (the Jetson is on UTC).

Directly **on the Jetson** (e.g. over a plain `ssh unitree@192.168.8.50`):
```bash
S=~/ros2_ws/src/quad-sdk/quad_utils/scripts/go2_bag_recorder.sh
$S start -l trot_test -r 192.168.8.103
$S stop                 # transfer runs in the background
$S transfer-status      # pending | running | done | failed
$S list                 # every local bag + its transfer state
$S transfer --pending   # retry everything not yet transferred
$S recover              # finalize a bag left by a crash / power loss
$S prune                # delete local copies already verified on the remote
```

### Where bags end up

* Jetson: `/home/unitree/unitree_bags/unitree_stock_YYYYMMDD_HHMMSS[_label]/`
  (kept after transfer; `prune` removes the ones already verified remotely, or set
  `GO2_DELETE_AFTER_TRANSFER=1`)
* Remote: `/home/rml_admin/ros2_ws/rosbags/unitree_stock/<same name>/`
  (in-flight copies sit in `.incoming/` and are moved in only after checksums match)

Each bag holds `metadata.yaml`, `*.mcap` (zstd, a new file every 5 min),
`SHA256SUMS`, and `recorder/` (coverage report, `ros2 bag info`, the full DDS
graph at stop, recorder log, the DDS config used).

### Settings

Environment variables, or put them in `~/.config/go2_bag_recorder.conf` on the
Jetson (it is `source`d):

| variable | default | |
|---|---|---|
| `GO2_BAG_ROOT` | `~/unitree_bags` | |
| `GO2_DOMAIN_ID` | `0` | Unitree DDS domain |
| `GO2_DDS_IFACE` | auto (192.168.123.x) | |
| `GO2_REMOTE_USER` / `GO2_REMOTE_HOST` / `GO2_REMOTE_DIR` | `rml_admin` / SSH client or `192.168.8.103` / `ros2_ws/rosbags/unitree_stock` | |
| `GO2_STORAGE_PRESET` | `zstd_fast` | `none` = uncompressed |
| `GO2_SPLIT_SECS` | `300` | 0 = one file |
| `GO2_TZ` | Jetson's (UTC) | bag-name timezone |
| `GO2_MIN_FREE_GB` | `5` | refuse to start below this |
| `GO2_TRANSFER_TRIES` | `30` | 0 = retry forever (backoff to 5 min) |
| `GO2_DELETE_AFTER_TRANSFER` | `0` | |

## Robustness: what happens when...

| event | result |
|---|---|
| Wi-Fi drops while recording | nothing; DDS never touches Wi-Fi and the recorder is a detached container |
| Wi-Fi drops during `stop` | `stop` ignores the hangup and still finalizes and starts the transfer (tested with SIGHUP) |
| Wi-Fi drops during transfer | rsync `--partial` resumes; retries with backoff; local bag is never deleted |
| remote computer off / wrong IP | transfer state `failed` after 30 tries; bag stays local; `retry` later |
| recorder crash / Jetson power loss | `recover` (also run automatically by the next `start`) rebuilds `metadata.yaml` with `ros2 bag reindex`. Tested with SIGKILL: 17.3 s of 18 s kept, `/lowstate` gapless at 499 Hz |
| disk filling | `start` refuses below 5 GB free; 153 GB free ≈ 10 h at 3.9 MiB/s |

**Data rate** (stock controller standing, measured): 10.6 MiB/s uncompressed,
**3.9 MiB/s with zstd_fast (~14 GiB/h)** at the same CPU (about 55% of one core).
Most of it is `/utlidar/cloud_deskewed`, `/utlidar/cloud` and `/utlidar/height_map_array`.
The transfer ran at ~8 MB/s over Wi-Fi, so a 10-minute recording
takes about 5 minutes to arrive.

**Timestamps:** rosbag2 stores the receive time on the Jetson clock (the host runs
chronyd and reports synchronized); message `header.stamp`s keep the Go2's own clock. Message types and
CDR payloads are stored exactly as received.

## Verification

While recording (Jetson):
```bash
$S status                                    # topics subscribed so far
$S topics 5                                  # everything discoverable right now, with types
docker logs go2_bag_recorder 2>&1 | grep -c 'Subscribed to topic'
docker logs go2_bag_recorder 2>&1 | grep -i 'unknown type'   # should list only the 4 above
```

After transfer (remote):
```bash
./go2_bag_remote.sh verify                   # newest bag; or verify <name>
cd ~/ros2_ws/rosbags/unitree_stock/<bag>
sha256sum -c SHA256SUMS                      # identical to what the Jetson wrote
cat recorder/coverage.txt                    # discovered vs recorded
source /opt/ros/jazzy/setup.bash && ros2 bag info -v .   # per-topic counts and sizes
```
Rate sanity: `/lowstate` and `/lowcmd` count ≈ 500 × duration, and `/utlidar/cloud` ≈ 15 × duration.

Reading the data: MCAP embeds every message definition. Foxglove Studio and Python
(`pip install mcap-ros2-support`, or `rosbags`) decode the `unitree_go` types
without any Unitree packages. `ros2 bag play` / `ros2 topic echo` of
`unitree_go/*` needs the `unitree_go` msg package sourced on the remote.

## Troubleshooting

* `no interface on 192.168.123.0/24`: the Go2 internal Ethernet is down; the
  Jetson should have 192.168.123.18 on `enP8p1s0`.
* `image ... missing`: run `go2_bag_recorder.sh build` (needs internet).
* `key-based SSH ... failed`: redo step 3 for that direction.
* Very few topics: is the Go2 powered and booted? `ping 192.168.123.161`.
* Logs: `$S log [NAME]` on the Jetson; transfer logs are in `~/unitree_bags/.logs/`.
