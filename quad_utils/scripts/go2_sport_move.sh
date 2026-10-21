#!/bin/bash
# go2_sport_move.sh -- command the Go2 stock controller to walk at a constant
# velocity (default 0.5 m/s straight for 4 s) instead of using the joystick.
#
# Runs on the Jetson HOST, in the go2_bag_recorder image (which already has
# unitree_api), on the Unitree DDS domain over the Go2 Ethernet only.
#
#   go2_sport_move.sh [--vx 0.5] [--vy 0] [--wz 0] [--duration 4] [--countdown 3]
#
# Env: GO2_DOMAIN_ID (0), GO2_DDS_IFACE (auto), GO2_RECORDER_IMAGE

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
DOMAIN_ID=${GO2_DOMAIN_ID:-0}
IMAGE=${GO2_RECORDER_IMAGE:-quad-sdk/go2-bag-recorder:jazzy}
IFACE=${GO2_DDS_IFACE:-$(ip -4 -o addr show | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')}
[[ -n $IFACE ]] || { echo "ERROR: no interface on 192.168.123.0/24; is the Go2 Ethernet up?" >&2; exit 1; }

exec docker run --rm -it --network host --user "$(id -u):$(id -g)" \
  -v "$SCRIPT_DIR/go2_bag_recorder/go2_sport_move.py:/go2_sport_move.py:ro" \
  -v "$SCRIPT_DIR/go2_bag_recorder/cyclone_dds_unitree.xml:/etc/go2_cyclone.xml:ro" \
  -e ROS_DOMAIN_ID="$DOMAIN_ID" -e GO2_DDS_IFACE="$IFACE" \
  -e CYCLONEDDS_URI=file:///etc/go2_cyclone.xml \
  "$IMAGE" python3 /go2_sport_move.py "$@"
