#!/usr/bin/env bash
# Exercise the built image's packaged simulation without rebuilding it.
set -euo pipefail
conga_repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
conga_results=$(mktemp -d /tmp/conga-container-check.XXXXXX)
echo "Container simulation results: $conga_results"
docker run --rm --init --network host --shm-size=2g \
  -e ROS_DOMAIN_ID=88 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e GZ_PARTITION=conga_container_verify \
  -v "$conga_repo/scripts/verify_conga_sim.py:/tmp/verify_conga_sim.py:ro" \
  -v "$conga_results:/tmp/conga-container-check" \
  quad-sdk:conga python3 /tmp/verify_conga_sim.py \
  2>&1 | tee /tmp/conga-container-run.log
