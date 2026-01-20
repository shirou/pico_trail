#!/bin/bash
# check-core-no-cfg.sh - CI lint script to enforce zero cfg directives in core crate
#
# This script ensures the pico_trail_core crate remains pure no_std without
# any feature-conditional compilation. This is a critical architectural invariant.
#
# Usage:
#   ./scripts/check-core-no-cfg.sh
#
# Exit codes:
#   0 - No cfg(feature directives found (pass)
#   1 - cfg(feature directives found (fail)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CORE_SRC="$PROJECT_ROOT/crates/core/src"

# Check if core crate exists
if [ ! -d "$CORE_SRC" ]; then
    echo "Error: Core crate not found at $CORE_SRC"
    exit 1
fi

echo "Checking for #[cfg(feature directives in $CORE_SRC..."

# Search for cfg(feature patterns
# We look for:
#   - #[cfg(feature = "...")]
#   - #[cfg(not(feature = "..."))]
#   - #[cfg_attr(feature = "...", ...)]
#   - cfg!(feature = "...")
PATTERN='#\[cfg\(.*feature|#\[cfg_attr\(.*feature|cfg!\(.*feature'

# Use grep to find matches
MATCHES=$(grep -rn "$PATTERN" "$CORE_SRC" 2>/dev/null || true)

if [ -n "$MATCHES" ]; then
    echo ""
    echo "ERROR: Found cfg(feature directives in core crate!"
    echo "========================================================"
    echo ""
    echo "$MATCHES"
    echo ""
    echo "========================================================"
    echo "The core crate must be pure no_std without feature flags."
    echo "Move platform-specific code to crates/firmware instead."
    echo ""
    exit 1
fi

# Count Rust files checked
FILE_COUNT=$(find "$CORE_SRC" -name "*.rs" | wc -l)

echo "OK: No cfg(feature directives found"
echo "Checked $FILE_COUNT Rust file(s) in $CORE_SRC"
exit 0
