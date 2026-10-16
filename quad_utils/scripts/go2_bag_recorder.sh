#!/bin/bash
# go2_bag_recorder.sh -- record every ROS 2 topic the Go2's stock controller
# exposes (native Unitree DDS, domain 0 on the internal Ethernet) on the
# onboard Jetson, then push the finished bag to the remote computer.
#
# Runs on the Jetson HOST (not inside the Quad-SDK dev container). Recording
# happens in its own throwaway container (image built from
# go2_bag_recorder/Dockerfile), so it is independent of the robot driver,
# planner and MPC, and of the 5-terminal launch. It only subscribes; it never
# publishes anything the stock controller listens to.
#
#   go2_bag_recorder.sh build                 build the recorder image (once)
#   go2_bag_recorder.sh check                 environment / DDS / SSH sanity checks
#   go2_bag_recorder.sh topics [SECS]         list discoverable topics + types
#   go2_bag_recorder.sh start [-l LABEL] [-r REMOTE_HOST]
#   go2_bag_recorder.sh stop [--no-transfer]  finalize bag, then transfer in background
#   go2_bag_recorder.sh status
#   go2_bag_recorder.sh list                  local bags + transfer state
#   go2_bag_recorder.sh transfer [NAME|--pending] [--foreground] [-r REMOTE_HOST]
#   go2_bag_recorder.sh transfer-status NAME  prints pending|running|done|failed
#   go2_bag_recorder.sh recover               finalize bags left by a crash/power loss
#   go2_bag_recorder.sh prune                 delete LOCAL copies already transferred
#   go2_bag_recorder.sh log [NAME]            recorder / transfer logs
#   go2_bag_recorder.sh setup-ssh [-r REMOTE_HOST]
#
# Settings (env vars, or put them in ~/.config/go2_bag_recorder.conf):
#   GO2_BAG_ROOT      local bag directory          (~/unitree_bags)
#   GO2_DOMAIN_ID     Unitree DDS domain           (0)
#   GO2_DDS_IFACE     interface on 192.168.123/24  (auto-detected)
#   GO2_REMOTE_USER   remote user                  (rml_admin)
#   GO2_REMOTE_HOST   remote IP; default is the SSH client that ran `start`,
#                     else 192.168.8.103
#   GO2_REMOTE_DIR    remote dir, relative to remote $HOME
#                                                  (ros2_ws/rosbags/unitree_stock)
#   GO2_SPLIT_SECS    start a new .mcap file every N s, 0 = never (300)
#   GO2_STORAGE_PRESET  MCAP preset: zstd_fast | zstd_small | fastwrite | none (zstd_fast)
#                     zstd_fast measured 2.7x smaller (10.6 -> 3.9 MiB/s) at
#                     the same CPU as none, which cuts Wi-Fi transfer time 2.7x
#   GO2_TZ            timezone for bag names, e.g. America/New_York (Jetson's);
#                     go2_bag_remote.sh passes the remote computer's timezone
#   GO2_MIN_FREE_GB   refuse to start below this much free disk (5)
#   GO2_TRANSFER_TRIES  rsync attempts before giving up, 0 = forever (30)
#   GO2_DELETE_AFTER_TRANSFER  1 = remove the local bag after a verified transfer (0)

set -o pipefail

SCRIPT_PATH=$(readlink -f "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(dirname "$SCRIPT_PATH")
CONF_FILE=${GO2_CONF:-$HOME/.config/go2_bag_recorder.conf}
# shellcheck disable=SC1090
[[ -f $CONF_FILE ]] && source "$CONF_FILE"

BAG_ROOT=${GO2_BAG_ROOT:-$HOME/unitree_bags}
DOMAIN_ID=${GO2_DOMAIN_ID:-0}
REMOTE_USER=${GO2_REMOTE_USER:-rml_admin}
DEFAULT_REMOTE_HOST=192.168.8.103
REMOTE_DIR=${GO2_REMOTE_DIR:-ros2_ws/rosbags/unitree_stock}
SPLIT_SECS=${GO2_SPLIT_SECS:-300}
STORAGE_PRESET=${GO2_STORAGE_PRESET:-zstd_fast}
MIN_FREE_GB=${GO2_MIN_FREE_GB:-5}
TRANSFER_TRIES=${GO2_TRANSFER_TRIES:-30}
DELETE_AFTER_TRANSFER=${GO2_DELETE_AFTER_TRANSFER:-0}
IMAGE=${GO2_RECORDER_IMAGE:-quad-sdk/go2-bag-recorder:jazzy}
CONTAINER=go2_bag_recorder
DDS_XML=$SCRIPT_DIR/go2_bag_recorder/cyclone_dds_unitree.xml
BAG_PREFIX=unitree_stock

STATE_DIR=$BAG_ROOT/.state      # never transferred
LOG_DIR=$BAG_ROOT/.logs
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=10
          -o ServerAliveCountMax=3 -o StrictHostKeyChecking=accept-new)

log()  { echo "[go2_bag $(date +%H:%M:%S)] $*"; }
warn() { echo "[go2_bag $(date +%H:%M:%S)] WARNING: $*" >&2; }
die()  { echo "[go2_bag $(date +%H:%M:%S)] ERROR: $*" >&2; exit 1; }

mkdir -p "$BAG_ROOT" "$STATE_DIR" "$LOG_DIR"

# ---------------------------------------------------------------- helpers ---

detect_iface() {
  if [[ -n $GO2_DDS_IFACE ]]; then echo "$GO2_DDS_IFACE"; return; fi
  ip -4 -o addr show | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}'
}

# Remote host priority: -r flag > GO2_REMOTE_HOST > SSH client that invoked us
# (when it's on the lab subnet) > documented default.
resolve_remote_host() {
  local cli=$1
  if [[ -n $cli ]]; then echo "$cli"; return; fi
  if [[ -n $GO2_REMOTE_HOST ]]; then echo "$GO2_REMOTE_HOST"; return; fi
  local client=${SSH_CLIENT%% *}
  if [[ $client =~ ^192\.168\.8\. ]]; then echo "$client"; return; fi
  echo "$DEFAULT_REMOTE_HOST"
}

container_running() {
  [[ $(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null) == true ]]
}
container_exists() { docker inspect "$CONTAINER" &>/dev/null; }

current_name() { cat "$STATE_DIR/current" 2>/dev/null; }

transfer_state() { cat "$STATE_DIR/$1.transfer" 2>/dev/null || echo none; }
set_transfer_state() { echo "$2" > "$STATE_DIR/$1.transfer"; }

# Run a one-off command in the recorder image on the Unitree DDS domain.
in_image() {
  local iface
  iface=$(detect_iface)
  docker run --rm --network host --user "$(id -u):$(id -g)" \
    -v "$BAG_ROOT:/bags" -v "$DDS_XML:/etc/go2_cyclone.xml:ro" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" -e GO2_DDS_IFACE="$iface" \
    -e CYCLONEDDS_URI=file:///etc/go2_cyclone.xml \
    "$IMAGE" "$@"
}

require_image() {
  docker image inspect "$IMAGE" &>/dev/null \
    || die "image $IMAGE missing -- run: $SCRIPT_PATH build"
}

check_rmem() {
  local rmem
  rmem=$(cat /proc/sys/net/core/rmem_max)
  if (( rmem < 8388608 )); then
    # Same 128 MiB init_robot.sh uses. Large samples (lidar clouds) lose
    # fragments with the 208 KiB kernel default.
    if sudo -n sysctl -q -w net.core.rmem_max=134217728 net.core.wmem_max=134217728 2>/dev/null; then
      log "raised net.core.rmem_max/wmem_max to 128 MiB"
    else
      warn "net.core.rmem_max is only $rmem bytes; large messages (e.g. /utlidar/cloud) may be dropped."
      warn "  fix: sudo sysctl -w net.core.rmem_max=134217728 net.core.wmem_max=134217728"
    fi
  fi
}

# Topics rosbag2 actually subscribed to, parsed from the recorder log.
subscribed_topics() {
  grep -oE "Subscribed to topic '[^']+'" "$1" 2>/dev/null \
    | sed -E "s/Subscribed to topic '([^']+)'/\1/" | sort -u
}

# ------------------------------------------------------------------ build ---

cmd_build() {
  log "building $IMAGE (pulls ros:jazzy-ros-base, builds unitree_go/api/hg msgs)"
  docker build -t "$IMAGE" "$SCRIPT_DIR/go2_bag_recorder"
}

# ------------------------------------------------------------------ check ---

cmd_check() {
  local ok=1 iface ip remote
  remote=$(resolve_remote_host "$1")
  command -v docker >/dev/null || { warn "docker not found"; ok=0; }
  docker image inspect "$IMAGE" &>/dev/null && log "image: $IMAGE ok" \
    || { warn "image $IMAGE missing (run: build)"; ok=0; }
  iface=$(detect_iface)
  if [[ -z $iface ]]; then
    warn "no interface on 192.168.123.0/24 -- Go2 internal Ethernet is down"; ok=0
  else
    ip=$(ip -4 -o addr show dev "$iface" | awk '{split($4,a,"/"); print a[1]; exit}')
    log "Unitree DDS: domain $DOMAIN_ID on $iface ($ip)"
    ping -c1 -W1 192.168.123.161 &>/dev/null && log "Go2 MCU 192.168.123.161 reachable" \
      || { warn "Go2 MCU 192.168.123.161 not answering ping (robot off?)"; ok=0; }
  fi
  log "net.core.rmem_max = $(cat /proc/sys/net/core/rmem_max)"
  log "free disk in $BAG_ROOT: $(df -BG --output=avail "$BAG_ROOT" | tail -1 | tr -d ' ')"
  log "remote: $REMOTE_USER@$remote:~/$REMOTE_DIR"
  if ! ping -c1 -W2 "$remote" &>/dev/null; then
    warn "$remote does not answer ping (wrong IP? check 'hostname -I' on the remote)"; ok=0
  elif ! ssh "${SSH_OPTS[@]}" "$REMOTE_USER@$remote" true 2>/dev/null; then
    warn "key-based SSH to $REMOTE_USER@$remote failed (run: setup-ssh -r $remote)"; ok=0
  else
    log "SSH key auth to $REMOTE_USER@$remote ok ($(ssh "${SSH_OPTS[@]}" "$REMOTE_USER@$remote" hostname))"
    ssh "${SSH_OPTS[@]}" "$REMOTE_USER@$remote" command -v rsync >/dev/null \
      && log "remote rsync present" || warn "remote has no rsync; will fall back to scp"
  fi
  (( ok )) && log "all checks passed" || { warn "some checks failed"; return 1; }
}

# ----------------------------------------------------------------- topics ---

cmd_topics() {
  require_image
  local secs=${1:-5}
  log "discovering topics on domain $DOMAIN_ID via $(detect_iface) for ${secs}s ..."
  in_image ros2 topic list -t --include-hidden-topics --no-daemon --spin-time "$secs"
}

# ------------------------------------------------------------------ start ---

cmd_start() {
  local label="" remote_cli=""
  while [[ $# -gt 0 ]]; do
    case $1 in
      -l|--label) label=$2; shift 2 ;;
      -r|--remote-host) remote_cli=$2; shift 2 ;;
      *) die "unknown start option: $1" ;;
    esac
  done
  require_image
  if container_running; then
    die "already recording $(current_name) -- stop it first"
  fi
  # A leftover (crashed / power-cut) recording is finalized before starting anew.
  cmd_recover

  local iface free_gb name remote
  iface=$(detect_iface)
  [[ -n $iface ]] || die "no interface on 192.168.123.0/24; is the Go2 Ethernet up?"
  free_gb=$(df -BG --output=avail "$BAG_ROOT" | tail -1 | tr -dc 0-9)
  (( free_gb >= MIN_FREE_GB )) || die "only ${free_gb}G free in $BAG_ROOT (< ${MIN_FREE_GB}G)"
  check_rmem

  # The Jetson runs on UTC; name bags in the operator's local time.
  if [[ -n $GO2_TZ ]]; then
    name=${BAG_PREFIX}_$(TZ=$GO2_TZ date +%Y%m%d_%H%M%S)
  else
    name=${BAG_PREFIX}_$(date +%Y%m%d_%H%M%S)
  fi
  [[ -n $label ]] && name+="_${label//[^A-Za-z0-9_-]/_}"
  [[ -e $BAG_ROOT/$name ]] && die "$BAG_ROOT/$name already exists"
  remote=$(resolve_remote_host "$remote_cli")

  local split_args=() storage_args=()
  (( SPLIT_SECS > 0 )) && split_args=(--max-bag-duration "$SPLIT_SECS")
  [[ $STORAGE_PRESET != none ]] && storage_args=(--storage-preset-profile "$STORAGE_PRESET")

  # --init: tini forwards SIGINT to ros2 bag so `stop` shuts it down cleanly.
  # nice: the stock controller runs on the MCU, but keep the Jetson responsive.
  # No --restart: a restarted recorder would collide with the existing bag dir.
  docker run -d --name "$CONTAINER" --init --network host \
    --user "$(id -u):$(id -g)" \
    -v "$BAG_ROOT:/bags" -v "$DDS_XML:/etc/go2_cyclone.xml:ro" \
    -e ROS_DOMAIN_ID="$DOMAIN_ID" -e GO2_DDS_IFACE="$iface" \
    -e CYCLONEDDS_URI=file:///etc/go2_cyclone.xml \
    --label "go2_bag.name=$name" \
    "$IMAGE" nice -n 5 ros2 bag record \
      -a --include-hidden-topics \
      -s mcap -o "/bags/$name" \
      "${storage_args[@]}" "${split_args[@]}" \
    >/dev/null || die "docker run failed"

  echo "$name" > "$STATE_DIR/current"
  echo "$remote" > "$STATE_DIR/$name.remote"
  set_transfer_state "$name" recording
  log "recording started: $BAG_ROOT/$name"
  log "  domain $DOMAIN_ID on $iface; transfer target $REMOTE_USER@$remote:~/$REMOTE_DIR"

  # Fail fast if ros2 bag dies on startup (bad config, missing plugin, ...).
  sleep 4
  if ! container_running; then
    docker logs "$CONTAINER" 2>&1 | tail -20 >&2
    finalize "$name"
    die "recorder exited immediately (log above)"
  fi
  local n
  n=$(docker logs "$CONTAINER" 2>&1 | grep -c "Subscribed to topic")
  log "subscribed to $n topics so far (new topics are picked up as they appear)"
  echo "BAG=$name"
}

# ------------------------------------------------------------------- stop ---

# Snapshot of the DDS graph from inside the running recorder, for the
# coverage report: everything discoverable vs. everything recorded.
# `-v` splits topics into published / subscribed sections; topics that only
# have subscribers (e.g. /api/*/request nobody is calling) carry no data.
snapshot_graph() {
  local meta=$1
  docker exec "$CONTAINER" /usr/local/bin/go2_env \
    ros2 topic list -v --include-hidden-topics --no-daemon --spin-time 3 \
    > "$meta/discovered_graph.txt" 2>/dev/null || { warn "topic snapshot failed"; return; }
  grep -E '^ \* ' "$meta/discovered_graph.txt" | awk '{print $2, $3}' | sort -u \
    > "$meta/discovered_topics.txt"
  sed -n '/^Published topics:/,/^Subscribed topics:/p' "$meta/discovered_graph.txt" \
    | grep -E '^ \* ' | awk '{print $2}' | sort -u > "$meta/published_topics.txt"
}

# Stop the container (if any) and make the bag self-describing and checksummed.
finalize() {
  local name=$1 bag=$BAG_ROOT/$1
  local meta=$bag/recorder
  mkdir -p "$meta"

  if container_exists; then
    if container_running; then
      snapshot_graph "$meta"
      log "sending SIGINT to ros2 bag record ..."
      docker kill -s INT "$CONTAINER" >/dev/null
      if ! timeout 90 docker wait "$CONTAINER" >/dev/null; then
        warn "recorder did not exit after 90 s; sending SIGTERM"
        docker kill -s TERM "$CONTAINER" >/dev/null 2>&1
        timeout 30 docker wait "$CONTAINER" >/dev/null || docker kill "$CONTAINER" >/dev/null 2>&1
      fi
    fi
    docker logs "$CONTAINER" > "$meta/recorder.log" 2>&1
    docker rm -f "$CONTAINER" >/dev/null 2>&1
  fi
  rm -f "$STATE_DIR/current"

  if [[ ! -d $bag ]]; then
    warn "no bag directory was created for $name"
    set_transfer_state "$name" failed
    return 1
  fi

  # metadata.yaml is written on clean shutdown; rebuild it if we got killed.
  if [[ ! -f $bag/metadata.yaml ]]; then
    warn "metadata.yaml missing -- reindexing $name"
    in_image ros2 bag reindex "/bags/$name" -s mcap >> "$meta/recorder.log" 2>&1 \
      || warn "reindex failed; raw .mcap files are kept and still transferred"
  fi

  in_image ros2 bag info "/bags/$name" > "$meta/bag_info.txt" 2>&1 \
    || warn "ros2 bag info failed on $name"

  # Coverage: discovered-but-not-recorded topics (usually an unknown msg type).
  {
    echo "# topics rosbag2 subscribed to: $(subscribed_topics "$meta/recorder.log" | wc -l)"
    if [[ -s $meta/published_topics.txt ]]; then
      echo "# discoverable at stop: $(wc -l < "$meta/discovered_topics.txt")" \
           "($(wc -l < "$meta/published_topics.txt") with a publisher)"
      echo "# published but NOT recorded:"
      comm -23 "$meta/published_topics.txt" <(subscribed_topics "$meta/recorder.log")
      echo "# subscriber-only (no publisher, nothing to record):"
      comm -23 <(awk '{print $1}' "$meta/discovered_topics.txt" | sort -u) \
               "$meta/published_topics.txt" \
        | comm -23 - <(subscribed_topics "$meta/recorder.log")
    fi
    echo "# recorder warnings:"
    grep -iE "warn|error|unknown|skipp" "$meta/recorder.log" | sort | uniq -c | head -50
  } > "$meta/coverage.txt"

  cp "$DDS_XML" "$meta/cyclone_dds_unitree.xml"
  {
    echo "name: $name"
    echo "host: $(hostname)"
    echo "domain_id: $DOMAIN_ID"
    echo "dds_iface: $(detect_iface)"
    echo "storage_preset: $STORAGE_PRESET"
    echo "image: $IMAGE ($(docker image inspect -f '{{.Id}}' "$IMAGE" 2>/dev/null))"
    echo "finalized: $(date -Iseconds)"
  } > "$meta/info.txt"

  # End-to-end integrity: verified on the remote after the copy.
  ( cd "$bag" && find . -type f ! -name SHA256SUMS -printf '%P\0' | sort -z \
      | xargs -0 sha256sum > SHA256SUMS ) || warn "checksum generation failed"

  set_transfer_state "$name" pending
  log "finalized $bag ($(du -sh "$bag" | cut -f1))"
  grep -E "^(Duration|Messages|Topic information)" "$meta/bag_info.txt" 2>/dev/null | sed 's/^/    /'
  if grep -q "NOT recorded:" "$meta/coverage.txt"; then
    local missing
    missing=$(sed -n '/NOT recorded:/,/^# subscriber-only/p' "$meta/coverage.txt" | grep -v '^#')
    [[ -n $missing ]] && warn "published but not recorded (no msg definition available):" \
      && echo "$missing" | sed 's/^/    /' >&2
  fi
}

cmd_stop() {
  # Finish finalizing and hand off the transfer even if the SSH session that
  # ran us drops mid-stop (Wi-Fi): ignore the hangup and the dead stdout pipe.
  trap '' HUP PIPE
  local transfer=1
  [[ $1 == --no-transfer ]] && transfer=0
  local name
  name=$(current_name)
  if [[ -z $name ]]; then
    name=$(docker inspect -f '{{index .Config.Labels "go2_bag.name"}}' "$CONTAINER" 2>/dev/null)
  fi
  [[ -n $name ]] || die "nothing is recording"
  finalize "$name" || true
  echo "BAG=$name"
  if (( transfer )); then
    spawn_transfer "$name"
  else
    log "transfer skipped; run: $SCRIPT_PATH transfer $name"
  fi
}

# Bags whose recorder died without `stop` (crash, power loss, docker restart).
cmd_recover() {
  if container_exists && ! container_running; then
    local n
    n=$(docker inspect -f '{{index .Config.Labels "go2_bag.name"}}' "$CONTAINER")
    log "recovering bag from stopped recorder: $n"
    finalize "$n"
  fi
  local f n
  for f in "$STATE_DIR"/*.transfer; do
    [[ -e $f ]] || continue
    n=$(basename "$f" .transfer)
    if [[ $(cat "$f") == recording ]] && ! container_running; then
      log "recovering interrupted bag: $n"
      finalize "$n"
    fi
  done
}

# --------------------------------------------------------------- transfer ---

# Detached so it survives the SSH session that ran `stop` dropping.
spawn_transfer() {
  local name=$1
  local logf=$LOG_DIR/$name.transfer.log
  setsid nohup "$SCRIPT_PATH" transfer "$name" --foreground >> "$logf" 2>&1 < /dev/null &
  log "transfer started in background (log: $logf)"
  log "  check with: $SCRIPT_PATH transfer-status $name"
}

transfer_once() {
  local name=$1 dest=$2
  local bag=$BAG_ROOT/$name
  local incoming=$REMOTE_DIR/.incoming
  local qdir qinc
  qdir=$(printf %q "$REMOTE_DIR"); qinc=$(printf %q "$incoming")

  # shellcheck disable=SC2029  # expand locally on purpose
  ssh "${SSH_OPTS[@]}" "$dest" "mkdir -p $qinc" || return 1

  # Copy into .incoming/ so a half-copied bag is never mistaken for a
  # finished one; --partial keeps progress across Wi-Fi drops.
  if ssh "${SSH_OPTS[@]}" "$dest" command -v rsync >/dev/null; then
    rsync -a --partial --partial-dir=.rsync-partial --timeout=60 \
      -e "ssh ${SSH_OPTS[*]}" "$bag/" "$dest:$incoming/$name/" || return 1
  else
    scp -r "${SSH_OPTS[@]}" "$bag" "$dest:$incoming/" || return 1
  fi

  # shellcheck disable=SC2029
  ssh "${SSH_OPTS[@]}" "$dest" "cd $qinc/$name && sha256sum --quiet -c SHA256SUMS" \
    || { warn "remote checksum mismatch"; return 1; }

  # shellcheck disable=SC2029
  ssh "${SSH_OPTS[@]}" "$dest" \
    "if [ -e $qdir/$name ]; then rm -rf $qdir/$name.old && mv $qdir/$name $qdir/$name.old; fi && mv $qinc/$name $qdir/$name" \
    || return 1
}

do_transfer() {
  local name=$1 remote=$2
  local bag=$BAG_ROOT/$name
  [[ -d $bag ]] || { warn "$bag does not exist"; return 1; }
  [[ $(transfer_state "$name") == done ]] && { log "$name already transferred"; return 0; }
  [[ $(transfer_state "$name") == recording ]] && { warn "$name is still recording"; return 1; }
  [[ -f $bag/SHA256SUMS ]] || { warn "$name not finalized (no SHA256SUMS); run recover"; return 1; }

  [[ -n $remote ]] || remote=$(cat "$STATE_DIR/$name.remote" 2>/dev/null)
  [[ -n $remote ]] || remote=$(resolve_remote_host "")
  local dest=$REMOTE_USER@$remote

  # One transfer per bag at a time.
  exec 9> "$STATE_DIR/$name.lock"
  flock -n 9 || { log "$name: another transfer is already running"; return 0; }

  set_transfer_state "$name" running
  local try=0 delay=10
  while :; do
    try=$((try + 1))
    log "$name -> $dest:~/$REMOTE_DIR (attempt $try)"
    if transfer_once "$name" "$dest"; then
      set_transfer_state "$name" done
      log "$name transferred and verified on $remote"
      if (( DELETE_AFTER_TRANSFER )); then
        rm -rf "$bag" && log "deleted local copy $bag"
      fi
      exec 9>&-
      return 0
    fi
    if (( TRANSFER_TRIES > 0 && try >= TRANSFER_TRIES )); then
      set_transfer_state "$name" failed
      warn "$name: giving up after $try attempts; local bag kept. Retry: $SCRIPT_PATH transfer $name"
      exec 9>&-
      return 1
    fi
    warn "attempt $try failed; retrying in ${delay}s"
    sleep "$delay"
    delay=$(( delay * 2 > 300 ? 300 : delay * 2 ))
  done
}

cmd_transfer() {
  local target="" fg=0 remote=""
  while [[ $# -gt 0 ]]; do
    case $1 in
      --foreground) fg=1; shift ;;
      -r|--remote-host) remote=$2; shift 2 ;;
      *) target=$1; shift ;;
    esac
  done
  if [[ $target == --pending || -z $target ]]; then
    local f n rc=0
    for f in "$STATE_DIR"/*.transfer; do
      [[ -e $f ]] || continue
      n=$(basename "$f" .transfer)
      case $(cat "$f") in
        pending|failed|running)
          if (( fg )); then do_transfer "$n" "$remote" || rc=1
          else spawn_transfer "$n"; fi ;;
      esac
    done
    return $rc
  fi
  if (( fg )); then
    do_transfer "$target" "$remote"
  else
    [[ -n $remote ]] && echo "$remote" > "$STATE_DIR/$target.remote"
    spawn_transfer "$target"
  fi
}

cmd_transfer_status() {
  local name=${1:-$(ls -t "$STATE_DIR"/*.transfer 2>/dev/null | head -1 | xargs -r basename -s .transfer)}
  [[ -n $name ]] || die "no bags"
  local s
  s=$(transfer_state "$name")
  # "running" with no lock holder means the transfer process died.
  if [[ $s == running ]] && flock -n "$STATE_DIR/$name.lock" true 2>/dev/null; then
    s=failed
  fi
  echo "$s"
}

# ----------------------------------------------------------- status / list ---

cmd_status() {
  if container_running; then
    local name started
    name=$(current_name)
    started=$(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER")
    log "RECORDING $name"
    log "  since $(date -d "$started" '+%F %T') ($(( $(date +%s) - $(date -d "$started" +%s) )) s)"
    log "  size $(du -sh "$BAG_ROOT/$name" 2>/dev/null | cut -f1)"
    log "  topics subscribed: $(docker logs "$CONTAINER" 2>&1 | grep -c 'Subscribed to topic')"
    local w
    w=$(docker logs "$CONTAINER" 2>&1 | grep -ciE "warn|error")
    (( w > 0 )) && log "  $w warning/error lines (see: $SCRIPT_PATH log)"
  else
    log "not recording"
  fi
  cmd_list | tail -5
}

cmd_list() {
  local d n
  printf '%-48s %8s  %s\n' BAG SIZE TRANSFER
  for d in "$BAG_ROOT"/${BAG_PREFIX}_*/; do
    [[ -d $d ]] || continue
    n=$(basename "$d")
    printf '%-48s %8s  %s\n' "$n" "$(du -sh "$d" | cut -f1)" "$(cmd_transfer_status "$n")"
  done
}

cmd_prune() {
  local f n
  for f in "$STATE_DIR"/*.transfer; do
    [[ -e $f ]] || continue
    n=$(basename "$f" .transfer)
    if [[ $(cat "$f") == done && -d $BAG_ROOT/$n ]]; then
      rm -rf "${BAG_ROOT:?}/$n" && log "deleted local $n (verified on remote)"
    fi
  done
}

cmd_log() {
  local name=${1:-$(current_name)}
  if [[ -z $1 ]] && container_running; then
    docker logs --tail 50 "$CONTAINER" 2>&1
    return
  fi
  [[ -n $name ]] || name=$(ls -t "$STATE_DIR"/*.transfer 2>/dev/null | head -1 | xargs -r basename -s .transfer)
  [[ -f $BAG_ROOT/$name/recorder/recorder.log ]] && tail -30 "$BAG_ROOT/$name/recorder/recorder.log"
  [[ -f $LOG_DIR/$name.transfer.log ]] && { echo "--- transfer"; tail -20 "$LOG_DIR/$name.transfer.log"; }
}

cmd_setup_ssh() {
  local remote
  remote=$(resolve_remote_host "$([[ $1 == -r ]] && echo "$2")")
  [[ -f $HOME/.ssh/id_ed25519 ]] || ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519" -C "$(whoami)@$(hostname) go2_bag_recorder"
  log "copying key to $REMOTE_USER@$remote (enter the remote password once)"
  ssh-copy-id -o StrictHostKeyChecking=accept-new -i "$HOME/.ssh/id_ed25519.pub" "$REMOTE_USER@$remote"
  ssh "${SSH_OPTS[@]}" "$REMOTE_USER@$remote" "mkdir -p $(printf %q "$REMOTE_DIR") && echo key auth ok"
}

# ------------------------------------------------------------------- main ---

cmd=${1:-help}; shift
case $cmd in
  build)            cmd_build ;;
  check)            cmd_check "$([[ $1 == -r ]] && echo "$2")" ;;
  topics)           cmd_topics "$@" ;;
  start)            cmd_start "$@" ;;
  stop)             cmd_stop "$@" ;;
  status)           cmd_status ;;
  list)             cmd_list ;;
  transfer)         cmd_transfer "$@" ;;
  transfer-status)  cmd_transfer_status "$@" ;;
  recover)          cmd_recover ;;
  prune)            cmd_prune ;;
  log)              cmd_log "$@" ;;
  setup-ssh)        cmd_setup_ssh "$@" ;;
  *) sed -n '2,/^set -o/p' "$SCRIPT_PATH" | sed '$d; s/^# \{0,1\}//'; exit 1 ;;
esac
