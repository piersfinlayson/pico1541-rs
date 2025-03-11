#!/bin/bash

# Common functions for build.sh and run.sh

# Valid argument values
VALID_BINS=("xum1541" "pico1541" "all")
VALID_BOARDS=("pico" "pico2" "all")

# Target mappings
declare -A TARGETS
TARGETS["pico"]="thumbv6m-none-eabi"
TARGETS["pico2"]="thumbv8m.main-none-eabihf"

# Function to show usage
show_usage() {
    local script_name=$(basename "$0")
    echo "Usage: $script_name <binary> <board> [release]"
    echo ""
    echo "Arguments:"
    echo "  <binary>   Binary to build/run (xum1541, pico1541, all)"
    echo "  <board>    Target board (pico, pico2, all)"
    echo "  [release]  Optional: Add this to build/run in release mode"
    echo ""
    echo "Example: $script_name xum1541 pico        # Debug build/run"
    echo "         $script_name xum1541 pico release # Release build/run"
    echo "         $script_name all pico"
    echo "         $script_name xum1541 all"
    echo "         $script_name all all release"
}

# Function to validate arguments
validate_args() {
    local bin=$1
    local board=$2

    # Check if we have both arguments
    if [[ -z "$bin" || -z "$board" ]]; then
        echo "Error: Missing required arguments" >&2
        show_usage
        exit 1
    fi

    # Validate binary
    if [[ ! " ${VALID_BINS[*]} " =~ " ${bin} " ]]; then
        echo "Error: Invalid binary '$bin'" >&2
        echo "Valid binaries are: xum1541, pico1541, all" >&2
        exit 1
    fi

    # Validate board
    if [[ ! " ${VALID_BOARDS[*]} " =~ " ${board} " ]]; then
        echo "Error: Invalid board '$board'" >&2
        echo "Valid boards are: pico, pico2, all" >&2
        exit 1
    fi
}

# Function to validate build type
validate_build_type() {
    local build_type=$1
    
    # If empty, it's valid (debug build)
    if [[ -z "$build_type" ]]; then
        return 0
    fi
    
    # Otherwise, must be "release"
    if [[ "$build_type" != "release" ]]; then
        echo "Error: Invalid build type '$build_type'" >&2
        echo "Build type must be either empty (for debug) or 'release'" >&2
        return 1
    fi
    
    return 0
}

# Function to get target for board
get_target() {
    local board=$1
    echo "${TARGETS[$board]}"
}