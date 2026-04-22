#!/bin/bash
# Export a Pre-built Image
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE="${1:-arm-mpc}"
OUTPUT_DIR="${2:-.}"

# Map service names to image tags
IMAGE="quad-sdk:${SERVICE}"

echo "Building service: $SERVICE"
docker compose -f "$SCRIPT_DIR/docker-compose.yml" build "$SERVICE"

OUTPUT_FILE="${OUTPUT_DIR}/quad-sdk-${SERVICE}.tar.gz"
echo "Exporting $IMAGE to $OUTPUT_FILE ..."
docker save "$IMAGE" | gzip > "$OUTPUT_FILE"

echo "Done. Load on another machine with:"
echo "  docker load < $OUTPUT_FILE"
