#!/bin/bash
set -e
set -x

# Install Rust and requried targets
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
export PATH="$HOME/.cargo/bin:$PATH"
rustup target add thumbv6-none-eabi
rustup target add thumbv8m.main-none-eabihf

# Build debug and release binaries for all boards
./build.sh all all
./build.sh all all release

# Clean up
rm -rf "$HOME/.cargo/registry"