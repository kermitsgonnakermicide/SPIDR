#!/usr/bin/env bash
# record_experiment.sh - ROS 2 bag recording script for OAK-D Lite experiments
# Usage: ./record_experiment.sh [options]
#
# Options:
#   -o, --output DIR    Output directory (default: ~/bags/exp_<timestamp>)
#   -t, --topics FILE   File containing list of topics to record (one per line)
#   -d, --duration SEC  Recording duration in seconds (0 = infinite)
#   -s, --storage TYPE  Storage type: mcap (default), sqlite3
#   --all-topics        Record all topics (not recommended, large bags)
#
# Examples:
#   ./record_experiment.sh                           # Record default topics
#   ./record_experiment.sh -d 60                    # Record for 60 seconds
#   ./record_experiment.sh -o ~/my_experiment -d 0 # Custom output, infinite recording

set -euo pipefail

# Default values
OUTPUT_DIR=""
TOPICS_FILE=""
DURATION="0"
STORAGE="mcap"
ALL_TOPICS=false

# Default topics for OAK-D Lite experiments
DEFAULT_TOPICS=(
    "/oak_d/points"
    "/oak_d/points/optimized"
    "/tf"
    "/tf_static"
    "/odom"
    "/projected_map"
    "/cmd_vel"
    "/joint_states"
    "/terrain_cost_map"
    "/foothold_plan"
    "/octomap_binary"
)

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -t|--topics)
            TOPICS_FILE="$2"
            shift 2
            ;;
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -s|--storage)
            STORAGE="$2"
            shift 2
            ;;
        --all-topics)
            ALL_TOPICS=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -o, --output DIR    Output directory (default: ~/bags/exp_<timestamp>)"
            echo "  -t, --topics FILE   File containing list of topics to record"
            echo "  -d, --duration SEC  Recording duration in seconds (0 = infinite)"
            echo "  -s, --storage TYPE  Storage type: mcap (default), sqlite3"
            echo "  --all-topics        Record all topics"
            echo "  -h, --help          Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Generate output directory if not specified
if [[ -z "$OUTPUT_DIR" ]]; then
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    OUTPUT_DIR="$HOME/bags/exp_$TIMESTAMP"
fi

mkdir -p "$OUTPUT_DIR"

# Build ros2 bag record command
ROS2_BAG_CMD=(ros2 bag record -o "$OUTPUT_DIR" -s "$STORAGE")

if [[ "$ALL_TOPICS" == "true" ]]; then
    echo "Recording ALL topics to $OUTPUT_DIR..."
    "$ROS2_BAG_CMD" &
else
    # Use custom topics file or default topics
    if [[ -n "$TOPICS_FILE" && -f "$TOPICS_FILE" ]]; then
        echo "Recording topics from $TOPICS_FILE to $OUTPUT_DIR..."
        "$ROS2_BAG_CMD" "$(cat "$TOPICS_FILE")" &
    else
        echo "Recording default OAK-D Lite experiment topics to $OUTPUT_DIR..."
        echo "Topics:"
        for topic in "${DEFAULT_TOPICS[@]}"; do
            echo "  - $topic"
        done
        echo ""
        "$ROS2_BAG_CMD" "${DEFAULT_TOPICS[@]}" &
    fi
fi

RECORD_PID=$!

# Handle duration
if [[ "$DURATION" != "0" ]]; then
    echo "Recording for $DURATION seconds..."
    sleep "$DURATION"
    kill "$RECORD_PID" 2>/dev/null || true
    wait "$RECORD_PID" 2>/dev/null || true
    echo "Recording complete: $OUTPUT_DIR"
else
    echo "Recording started (PID: $RECORD_PID)"
    echo "Press Ctrl+C to stop..."
    wait "$RECORD_PID"
fi
