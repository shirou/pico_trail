#!/bin/bash
# Check for HAL imports outside src/platform/
# This enforces NFR-nmmu0 (Platform Code Isolation)

set -e

echo "Checking for HAL imports outside src/platform/..."

# Search for HAL imports in src/, excluding src/platform/
violations=$(grep -r "use.*rp235x_hal\|use.*rp2040_hal\|use.*embassy_rp" src/ 2>/dev/null | grep -v "src/platform/" || true)

if [ -n "$violations" ]; then
    echo "❌ ERROR: HAL imports found outside src/platform/"
    echo ""
    echo "The following files violate NFR-nmmu0 (Platform Code Isolation):"
    echo "$violations"
    echo ""
    echo "All HAL-specific code must be isolated to src/platform/"
    echo "Please use platform abstraction traits instead."
    exit 1
fi

echo "✅ No HAL imports found outside src/platform/"
echo "Platform isolation check passed!"
exit 0
