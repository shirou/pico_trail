#!/usr/bin/env bash
#
# Generate a new sequential TDL document ID (5-digit zero-padded).
#
# Scans the docs directory for existing IDs and returns the next
# available number in the sequence.
#
# Usage:
#   tdl-new-id.sh [docs_dir]
#
# Arguments:
#   docs_dir  Root directory to scan (default: "docs")
#
# Environment Variables:
#   DOCS_DIR  Root directory to scan (overridden by positional argument)
#
# Output:
#   A 5-digit zero-padded number (e.g., 00001, 00042)

set -euo pipefail

docs_dir="${1:-${DOCS_DIR:-docs}}"

max_id=0

if [ -d "$docs_dir" ]; then
  while IFS= read -r -d '' entry; do
    basename_entry="$(basename "$entry")"
    # Match patterns like AN-00001-, FR-00042-, NFR-00003-, ADR-00010-, T-00005-
    if [[ "$basename_entry" =~ ^(AN|FR|NFR|ADR|T)-([0-9]+)- ]]; then
      num="${BASH_REMATCH[2]}"
      # Remove leading zeros for arithmetic comparison
      num=$((10#$num))
      if (( num > max_id )); then
        max_id=$num
      fi
    fi
  done < <(find "$docs_dir" -mindepth 1 -print0 2>/dev/null)
fi

next_id=$((max_id + 1))

if (( next_id > 99999 )); then
  echo "Error: ID overflow (max 99999)" >&2
  exit 1
fi

printf '%05d\n' "$next_id"
