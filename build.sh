#!/bin/bash

# Source common functions
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/scripts"
source "$SCRIPT_DIR/common.sh" || { echo "Error: Failed to source common.sh" >&2; exit 1; }

# Show help if requested
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Builds the specified binary for the target board."
    show_usage
    exit 0
fi

# Get arguments
MODE=$1
BOARD=$2
BUILD_TYPE=$3

# Check if we're building in release mode
RELEASE_FLAG=""
BUILD_DIR="debug"
if [[ "$BUILD_TYPE" == "release" ]]; then
    RELEASE_FLAG="--release"
    BUILD_DIR="release"
fi

# Validate arguments
validate_args "$MODE" "$BOARD"

# Validate build type
validate_build_type "$BUILD_TYPE" || exit 1

# Function to build a single binary for a specific board
build_single() {
    local mode=$1
    local board=$2
    
    # Get the appropriate target for the board
    local target=$(get_target "$board")
    
    # Print build information
    echo "--------------------------------------------"
    echo "Building $mode for $board (target: $target)"
    echo "--------------------------------------------"
    
    # Execute cargo build with the appropriate parameters
    set -x
    cargo build --bin pico1541 --features "$mode,$board" --target "$target" $RELEASE_FLAG
    local build_exit_code=$?
    set +x
    
    if [ $build_exit_code -eq 0 ]; then
        echo "Build successful!"
        # Show the path to the built binary
        echo "Binary located at: target/$target/$BUILD_DIR/$mode"
        echo ""
        return 0
    else
        echo "Build failed with exit code $build_exit_code" >&2
        return $build_exit_code
    fi
}

# Handle "all" option for MODE and BOARD
if [[ "$MODE" == "all" && "$BOARD" == "all" ]]; then
    # Build all combinations
    echo "Building all mode and board combinations..."
    
    exit_code=0
    for mode in "pico1541"; do
        for board in "pico" "pico2"; do
            build_single "$mode" "$board"
            if [ $? -ne 0 ]; then
                exit_code=1
            fi
        done
    done
    
    if [ $exit_code -eq 0 ]; then
        echo "All builds completed successfully!"
    else
        echo "Some builds failed. Check the output above for details." >&2
    fi
    exit $exit_code
    
elif [[ "$MODE" == "all" ]]; then
    # Build all binaries for a specific board
    echo "Building all binaries for $BOARD..."
    
    exit_code=0
    for mode in "pico1541"; do
        build_single "$mode" "$BOARD"
        if [ $? -ne 0 ]; then
            exit_code=1
        fi
    done
    
    if [ $exit_code -eq 0 ]; then
        echo "All builds for $BOARD completed successfully!"
    else
        echo "Some builds for $BOARD failed. Check the output above for details." >&2
    fi
    exit $exit_code
    
elif [[ "$BOARD" == "all" ]]; then
    # Build a specific binary for all boards
    echo "Building $MODE for all boards..."
    
    exit_code=0
    for board in "pico" "pico2"; do
        build_single "$MODE" "$board"
        if [ $? -ne 0 ]; then
            exit_code=1
        fi
    done
    
    if [ $exit_code -eq 0 ]; then
        echo "All builds of $MODE completed successfully!"
    else
        echo "Some builds of $MODE failed. Check the output above for details." >&2
    fi
    exit $exit_code
    
else
    # Build a single binary for a specific board
    build_single "$MODE" "$BOARD"
    exit $?
fi