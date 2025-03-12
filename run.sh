#!/bin/bash

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts"
source "$SCRIPT_DIR/common.sh" || { echo "Error: Failed to source common.sh" >&2; exit 1; }

# Show help if requested
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Runs the specified binary for the target board."
    show_usage
    exit 0
fi

# Get arguments
BIN=$1
BOARD=$2
BUILD_TYPE=$3

# Check if we're running in release mode
RELEASE_FLAG=""
BUILD_DIR="debug"
if [[ "$BUILD_TYPE" == "release" ]]; then
    RELEASE_FLAG="--release"
    BUILD_DIR="release"
fi

# Validate arguments
validate_args "$BIN" "$BOARD"

# Validate build type
validate_build_type "$BUILD_TYPE" || exit 1

# Get the appropriate target for the board
TARGET=$(get_target "$BOARD")

# Print run information
echo "Running $BIN for $BOARD (target: $TARGET)"

# Execute cargo run with the appropriate parameters
set -x
cargo run --bin "$BIN" --features "$BIN,$BOARD" --target "$TARGET" $RELEASE_FLAG
RUN_EXIT_CODE=$?
set +x

if [ $RUN_EXIT_CODE -ne 0 ]; then
    echo "Run failed with exit code $RUN_EXIT_CODE" >&2
    exit $RUN_EXIT_CODE
fi
