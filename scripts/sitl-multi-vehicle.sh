#!/usr/bin/env bash
# Launch multiple gazebo_bridge processes (one per vehicle) with mavp2p aggregation.
#
# Usage:
#   ./scripts/sitl-multi-vehicle.sh [COUNT]
#
# Environment variables:
#   GAZEBO_PORT_BASE   - Gazebo fdm_port_in for vehicle 1 (default: 9002)
#   GAZEBO_PORT_STRIDE - Port offset between vehicles (default: 10)
#   MAVLINK_PORT_BASE  - MAVLink TCP port for vehicle 1 (default: 5760)
#   MAVLINK_PORT_STRIDE- Port offset between MAVLink ports (default: 2)
#   MAVP2P_PORT        - mavp2p GCS listen port (default: 5770)

set -euo pipefail

COUNT=${1:-3}
GAZEBO_PORT_BASE=${GAZEBO_PORT_BASE:-9002}
GAZEBO_PORT_STRIDE=${GAZEBO_PORT_STRIDE:-10}
MAVLINK_PORT_BASE=${MAVLINK_PORT_BASE:-5760}
MAVLINK_PORT_STRIDE=${MAVLINK_PORT_STRIDE:-2}
MAVP2P_PORT=${MAVP2P_PORT:-5770}

# Check for mavp2p
if ! command -v mavp2p &>/dev/null; then
    echo "Error: mavp2p not found in PATH." >&2
    echo "" >&2
    echo "Install mavp2p:" >&2
    echo "  go install github.com/bluenviron/mavp2p@latest" >&2
    echo "" >&2
    echo "Or download a binary from:" >&2
    echo "  https://github.com/bluenviron/mavp2p/releases" >&2
    exit 1
fi

PIDS=()
cleanup() {
    echo ""
    echo "Stopping all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "All processes stopped."
}
trap cleanup EXIT INT TERM

echo "=== pico_trail Multi-Vehicle SITL ==="
echo "Vehicles: $COUNT"
echo ""

# Start one gazebo_bridge per vehicle
for i in $(seq 1 "$COUNT"); do
    SYS_ID=$i
    GAZEBO_PORT=$((GAZEBO_PORT_BASE + (i - 1) * GAZEBO_PORT_STRIDE))
    MAVLINK_PORT=$((MAVLINK_PORT_BASE + (i - 1) * MAVLINK_PORT_STRIDE))

    cargo run -p pico_trail_sitl --bin gazebo_bridge -- \
        --system-id "$SYS_ID" \
        --gazebo-port "$GAZEBO_PORT" \
        --mavlink-port "$MAVLINK_PORT" &
    PIDS+=($!)
    echo "Vehicle $i: system-id=$SYS_ID, gazebo=$GAZEBO_PORT, mavlink=TCP:$MAVLINK_PORT (PID ${PIDS[-1]})"
done

# Wait for bridges to start listening
sleep 2

# Build mavp2p endpoint list
MAVP2P_ARGS=()
for i in $(seq 1 "$COUNT"); do
    PORT=$((MAVLINK_PORT_BASE + (i - 1) * MAVLINK_PORT_STRIDE))
    MAVP2P_ARGS+=("tcpc:127.0.0.1:$PORT")
done
MAVP2P_ARGS+=("tcps:0.0.0.0:$MAVP2P_PORT")

echo ""
echo "Starting mavp2p on TCP:$MAVP2P_PORT..."
mavp2p "${MAVP2P_ARGS[@]}" &
PIDS+=($!)

echo ""
echo "All vehicles running. Connect GCS to TCP:$MAVP2P_PORT"
echo "Press Ctrl+C to stop all processes."
echo ""

wait
