#!/usr/bin/env bash
# Build examples for RP2350 (Pico 2 W) and convert to UF2 format
#
# Usage:
#   ./scripts/build-rp2350.sh [--release] [example_name...]
#
# Examples:
#   ./scripts/build-rp2350.sh                    # Build all examples (debug)
#   ./scripts/build-rp2350.sh --release          # Build all examples (release)
#   ./scripts/build-rp2350.sh scheduler_demo     # Build specific example (debug)
#   ./scripts/build-rp2350.sh --release scheduler_demo scheduler_demo_usb
#
# Network-enabled examples (mavlink_demo_network):
#   WiFi credentials are configured via MAVLink parameters after flashing.
#   See docs/wifi-configuration.md for detailed setup instructions.
#
#   Quick start:
#   1. Build and flash: ./scripts/build-rp2350.sh --release mavlink_demo_network
#   2. Connect via UART (115200 baud)
#   3. Set parameters: NET_SSID, NET_PASS via QGroundControl
#   4. Reboot device to connect to WiFi

set -euo pipefail

# Load .env file if present (for WiFi configuration)
if [[ -f .env ]]; then
  echo "Loading .env file..."
  set -a  # Automatically export all variables
  # shellcheck source=/dev/null
  source .env
  set +a
  echo "Environment variables loaded from .env"
else
  echo "No .env file found (WiFi will use runtime configuration)"
fi

# RP2350-ARM-S family ID
FAMILY_ID="0xe48bff59"

# Parse arguments
BUILD_MODE="debug"
EXAMPLES=()

for arg in "$@"; do
  if [[ "$arg" == "--release" ]]; then
    BUILD_MODE="release"
  else
    EXAMPLES+=("$arg")
  fi
done

# If no examples specified, find all examples
if [[ ${#EXAMPLES[@]} -eq 0 ]]; then
  if [[ -d "examples" ]]; then
    mapfile -t EXAMPLES < <(find examples -name "*.rs" -type f | sed 's|examples/||; s|\.rs$||')
  else
    echo "Error: examples directory not found"
    exit 1
  fi
fi

# Build configuration
TARGET="thumbv8m.main-none-eabihf"
FEATURES="pico2_w"
TARGET_DIR="target/${TARGET}/${BUILD_MODE}/examples"
OUTPUT_DIR="target"

# Build flag
BUILD_FLAG=""
if [[ "$BUILD_MODE" == "release" ]]; then
  BUILD_FLAG="--release"
fi

echo "Building examples for RP2350 (${BUILD_MODE} mode)..."
echo "Target: ${TARGET}"
echo "Features: ${FEATURES}"
echo ""

# Build and convert each example
for example in "${EXAMPLES[@]}"; do
  echo "Building example: ${example}"

  # Build the example
  cargo build --target "$TARGET" --example "$example" --features "$FEATURES" $BUILD_FLAG

  # Check if ELF file exists
  ELF_PATH="${TARGET_DIR}/${example}"
  if [[ ! -f "$ELF_PATH" ]]; then
    echo "Error: ELF file not found at ${ELF_PATH}"
    exit 1
  fi

  # Convert to UF2
  UF2_PATH="${OUTPUT_DIR}/${example}.uf2"
  echo "Converting to UF2: ${UF2_PATH}"
  elf2flash convert --family "$FAMILY_ID" "$ELF_PATH" "$UF2_PATH"

  # Show file info
  if command -v file &> /dev/null; then
    file "$UF2_PATH"
  fi

  echo "Created: ${UF2_PATH}"
  echo ""
done

echo "Build complete!"
echo ""
echo "To flash to Pico 2 W:"
echo "1. Hold BOOTSEL button while connecting USB"
echo "2. Copy UF2 file to the mounted drive"
echo "3. Pico will automatically reboot and run"
