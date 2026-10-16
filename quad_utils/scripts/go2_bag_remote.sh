#!/bin/bash
# go2_bag_remote.sh -- drive the Jetson's go2_bag_recorder.sh from the remote
# computer over SSH, and verify bags once they arrive.
#
#   go2_bag_remote.sh [-r ROBOT] start [LABEL]   start recording on the robot
#   go2_bag_remote.sh [-r ROBOT] stop [--no-wait]
#                    stop, wait for the automatic transfer, verify the bag here
#   go2_bag_remote.sh [-r ROBOT] status | list | log [NAME] | topics [SECS] | check
#   go2_bag_remote.sh [-r ROBOT] retry           re-send every untransferred bag
#   go2_bag_remote.sh verify [NAME|DIR]          verify a received bag (default: newest)
#   go2_bag_remote.sh [-r ROBOT] setup-ssh       install this machine's key on the robot
#
# ROBOT is alvin | theodore | simon (Wi-Fi IPs, as in init_remote.sh) or an
# IP/hostname. Default: $GO2_ROBOT_HOST, else alvin (192.168.8.50).
#
# Env: GO2_ROBOT_USER (unitree), GO2_ROBOT_SCRIPT (path on the robot),
#      GO2_LOCAL_DIR (~/ros2_ws/rosbags/unitree_stock), GO2_WAIT_SECS (3600)

set -o pipefail

ROBOT_USER=${GO2_ROBOT_USER:-unitree}
ROBOT_SCRIPT=${GO2_ROBOT_SCRIPT:-'~/ros2_ws/src/quad-sdk/quad_utils/scripts/go2_bag_recorder.sh'}
LOCAL_DIR=${GO2_LOCAL_DIR:-$HOME/ros2_ws/rosbags/unitree_stock}
WAIT_SECS=${GO2_WAIT_SECS:-3600}
ROBOT=${GO2_ROBOT_HOST:-alvin}
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=10
          -o ServerAliveCountMax=3 -o StrictHostKeyChecking=accept-new)

log()  { echo "[go2_remote $(date +%H:%M:%S)] $*"; }
warn() { echo "[go2_remote $(date +%H:%M:%S)] WARNING: $*" >&2; }
die()  { echo "[go2_remote $(date +%H:%M:%S)] ERROR: $*" >&2; exit 1; }

if [[ $1 == -r || $1 == --robot ]]; then ROBOT=$2; shift 2; fi
case ${ROBOT,,} in
  t|theodore) ROBOT_HOST=192.168.8.51 ;;
  a|alvin)    ROBOT_HOST=192.168.8.50 ;;
  s|simon)    ROBOT_HOST=192.168.8.52 ;;
  *)          ROBOT_HOST=$ROBOT ;;
esac

# Our own address as the robot sees it -> where the robot sends the bag.
my_ip() {
  ip -4 route get "$ROBOT_HOST" 2>/dev/null | sed -nE 's/.* src ([0-9.]+).*/\1/p'
}

my_tz() {
  timedatectl show -p Timezone --value 2>/dev/null || cat /etc/timezone 2>/dev/null
}

robot() {
  # The Jetson runs on UTC; pass our timezone so bag names match local time.
  local tz
  tz=$(my_tz)
  # shellcheck disable=SC2029
  ssh "${SSH_OPTS[@]}" "$ROBOT_USER@$ROBOT_HOST" "${tz:+GO2_TZ=$tz }bash $ROBOT_SCRIPT $*"
}

cmd_verify() {
  local target=$1 bag
  if [[ -z $target ]]; then
    bag=$(ls -dt "$LOCAL_DIR"/unitree_stock_*/ 2>/dev/null | head -1)
  elif [[ -d $target ]]; then
    bag=$target
  else
    bag=$LOCAL_DIR/$target
  fi
  bag=${bag%/}
  [[ -d $bag ]] || die "bag not found: ${target:-$LOCAL_DIR/unitree_stock_*}"
  log "verifying $bag"
  local ok=1
  [[ -f $bag/metadata.yaml ]] && log "  metadata.yaml present" || { warn "metadata.yaml missing"; ok=0; }
  if [[ -f $bag/SHA256SUMS ]]; then
    (cd "$bag" && sha256sum --quiet -c SHA256SUMS) && log "  checksums match the Jetson's" \
      || { warn "checksum mismatch"; ok=0; }
  else
    warn "no SHA256SUMS"; ok=0
  fi
  # ros2 bag info needs only the base install, not the unitree msg packages.
  if ! command -v ros2 >/dev/null && [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
  fi
  if command -v ros2 >/dev/null; then
    local info
    if info=$(ros2 bag info "$bag" 2>&1); then
      echo "$info" | grep -E "^(Files|Bag size|Storage id|Duration|Start|End|Messages)" | sed 's/^/    /'
      log "  topics with messages: $(echo "$info" | grep -cE 'Count: [1-9]')," \
          "with zero: $(echo "$info" | grep -c 'Count: 0 ')"
      # /wirelesscontroller only has data while the handheld remote is in use.
      local t n
      for t in /lowstate /lowcmd /sportmodestate /wirelesscontroller; do
        n=$(echo "$info" | sed -nE "s#.*Topic: $t \| .*Count: ([0-9]+) .*#\1#p")
        if [[ -z $n ]]; then warn "$t missing from bag"
        elif (( n == 0 )); then
          warn "  $t recorded but has 0 messages$([[ $t == /wirelesscontroller ]] && echo ' (handheld remote not in use?)')"
        else log "  $t: $n msgs"; fi
      done
    else
      echo "$info" | tail -5; warn "ros2 bag info failed"; ok=0
    fi
  else
    warn "ros2 not on PATH (source init_remote.sh); skipped ros2 bag info"
  fi
  if [[ -f $bag/recorder/coverage.txt ]]; then
    local missing
    missing=$(sed -n '/NOT recorded:/,/^# subscriber-only/p' "$bag/recorder/coverage.txt" | grep -v '^#')
    if [[ -n $missing ]]; then
      # /arm_Command and /qt_* (unitree_arm, unitree_interfaces) have no public
      # .msg definition; they carried 0 msgs in 30 s of stock operation (Oct 2026).
      log "  published on the robot but not recordable (no msg definition; see recorder/coverage.txt):"
      echo "$missing" | sed 's/^/    /'
    else
      log "  every published topic was recorded"
    fi
  fi
  (( ok )) && log "OK: $bag" || die "verification FAILED for $bag"
}

cmd_stop() {
  local out name
  out=$(robot stop) ; echo "$out"
  name=$(echo "$out" | sed -n 's/^BAG=//p' | tail -1)
  [[ -n $name ]] || die "robot did not report a bag name"
  [[ $1 == --no-wait ]] && { log "not waiting; later run: $0 verify $name"; return; }

  log "waiting for $name to arrive (Ctrl-C is safe: the transfer continues on the robot)"
  local t0=$SECONDS s
  while (( SECONDS - t0 < WAIT_SECS )); do
    s=$(robot transfer-status "$name" 2>/dev/null | tail -1)
    case $s in
      done)   log "transfer complete"; cmd_verify "$name"; return ;;
      failed) die "transfer failed; bag is safe on the robot. Retry: $0 -r $ROBOT retry" ;;
    esac
    sleep 5
  done
  die "timed out waiting; check: $0 -r $ROBOT status"
}

cmd_setup_ssh() {
  [[ -f $HOME/.ssh/id_ed25519 ]] || ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
  ssh-copy-id -o StrictHostKeyChecking=accept-new -i "$HOME/.ssh/id_ed25519.pub" "$ROBOT_USER@$ROBOT_HOST"
  mkdir -p "$LOCAL_DIR"
  log "now on the robot run once (sets up robot -> this machine):"
  log "  $ROBOT_SCRIPT setup-ssh -r $(my_ip)"
}

cmd=${1:-help}; shift
case $cmd in
  start)
    mkdir -p "$LOCAL_DIR"
    args=(start -r "$(my_ip)")
    [[ -n $1 ]] && args+=(-l "$1")
    robot "${args[@]}" ;;
  stop)      cmd_stop "$@" ;;
  status|list|check|recover) robot "$cmd" ;;
  topics)    robot topics "$@" ;;
  log)       robot log "$@" ;;
  retry)     robot transfer --pending -r "$(my_ip)" ;;
  verify)    cmd_verify "$@" ;;
  setup-ssh) cmd_setup_ssh ;;
  *) sed -n '2,/^set -o/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 1 ;;
esac
