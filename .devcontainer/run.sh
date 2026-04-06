#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE="${1:-arm-mpc}"

echo "Starting service: $SERVICE"
docker compose -f "$SCRIPT_DIR/docker-compose.yml" up --build "$SERVICE"
