# NFR-00088 CI Core Purity Lint

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [NFR-00089-zero-cfg-core-crate](../requirements/NFR-00089-zero-cfg-core-crate.md)
- Related Analyses:
  - [AN-00039-crate-workspace-separation](../analysis/AN-00039-crate-workspace-separation.md)
- Related Tasks:
  - [T-00035-workspace-separation](../tasks/T-00035-workspace-separation/README.md)

## Requirement Statement

The CI pipeline shall include a lint script that verifies the core crate contains no `#[cfg(feature = ...)]` directives, no `#[cfg_attr(feature = ...)]` directives, and no forbidden imports (embassy-\*, defmt), failing the build if violations are detected.

## Rationale

Automated enforcement prevents regression:

1. **Prevent drift**: Without CI enforcement, cfg may gradually leak back into core
2. **Immediate feedback**: Developers know immediately if they violate the constraint
3. **Documentation**: Script serves as executable specification of the rule
4. **Consistency**: Every PR is checked, no manual review required

## User Story (if applicable)

The system shall automatically enforce core crate purity rules in CI to ensure the architectural boundary is maintained without relying on manual code review.

## Acceptance Criteria

- [ ] Script `scripts/check-core-no-cfg.sh` exists and is executable
- [ ] Script checks for `#[cfg(feature` patterns in `crates/core/src/`
- [ ] Script checks for `#[cfg_attr(feature` patterns in `crates/core/src/`
- [ ] Script checks for `use embassy` and `use defmt` imports
- [ ] Script exits with code 0 on success, non-zero on failure
- [ ] CI workflow includes the script in the check pipeline
- [ ] Script provides clear error messages showing violating lines
- [ ] Script gracefully handles missing `crates/core/` directory (skip with success)

## Technical Details

### Non-Functional Requirement Details

#### Script Implementation

```bash
#!/bin/bash
# scripts/check-core-no-cfg.sh
# Ensures crates/core has no #[cfg(...)] attributes or forbidden imports

set -e

CORE_PATH="crates/core/src"

# Skip if workspace not yet created
if [ ! -d "$CORE_PATH" ]; then
    echo "Core crate not found at $CORE_PATH - skipping check"
    exit 0
fi

ERRORS=0

# Check for cfg(feature patterns
CFG_FEATURE=$(grep -rn '#\[cfg(feature' "$CORE_PATH" --include="*.rs" || true)
if [ -n "$CFG_FEATURE" ]; then
    echo "ERROR: Found #[cfg(feature ...)] in core crate:"
    echo "$CFG_FEATURE"
    ERRORS=$((ERRORS + 1))
fi

# Check for cfg_attr(feature patterns
CFG_ATTR=$(grep -rn '#\[cfg_attr(feature' "$CORE_PATH" --include="*.rs" || true)
if [ -n "$CFG_ATTR" ]; then
    echo "ERROR: Found #[cfg_attr(feature ...)] in core crate:"
    echo "$CFG_ATTR"
    ERRORS=$((ERRORS + 1))
fi

# Check for forbidden imports
EMBASSY_IMPORT=$(grep -rn 'use embassy' "$CORE_PATH" --include="*.rs" || true)
if [ -n "$EMBASSY_IMPORT" ]; then
    echo "ERROR: Found 'use embassy' import in core crate:"
    echo "$EMBASSY_IMPORT"
    ERRORS=$((ERRORS + 1))
fi

DEFMT_IMPORT=$(grep -rn 'use defmt' "$CORE_PATH" --include="*.rs" || true)
if [ -n "$DEFMT_IMPORT" ]; then
    echo "ERROR: Found 'use defmt' import in core crate:"
    echo "$DEFMT_IMPORT"
    ERRORS=$((ERRORS + 1))
fi

# Summary
if [ "$ERRORS" -gt 0 ]; then
    echo ""
    echo "Core crate purity check FAILED with $ERRORS violation(s)."
    echo "Move conditional code to crates/firmware instead."
    exit 1
fi

echo "Core crate is clean: no cfg, cfg_attr, or forbidden imports found."
exit 0
```

#### CI Integration

```yaml
# .github/workflows/ci.yml
jobs:
  check:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Check core crate purity
        run: ./scripts/check-core-no-cfg.sh

      - name: Format check
        run: cargo fmt --check

      - name: Clippy
        run: cargo clippy --all-targets -- -D warnings

      - name: Test core crate
        run: cargo test -p pico_trail_core --lib
```

#### Error Message Format

When violations are found:

```
ERROR: Found #[cfg(feature ...)] in core crate:
crates/core/src/navigation/state.rs:15:#[cfg(feature = "embassy")]
crates/core/src/control/pid.rs:42:#[cfg(feature = "pico2_w")]

Core crate purity check FAILED with 1 violation(s).
Move conditional code to crates/firmware instead.
```

- Performance: Script executes in < 1 second
- Reliability: Deterministic results on every run

## Platform Considerations

N/A - CI runs on Linux, script is POSIX-compatible.

## Risks & Mitigation

| Risk                    | Impact | Likelihood | Mitigation                          | Validation                   |
| ----------------------- | ------ | ---------- | ----------------------------------- | ---------------------------- |
| False positives         | Low    | Low        | Exclude cfg(test) from check        | Test with cfg(test) module   |
| Script not run          | Medium | Low        | Add to required CI checks           | PR cannot merge without pass |
| Regex misses edge cases | Low    | Low        | Use exact patterns, test thoroughly | Test with known violations   |

## Implementation Notes

- Add script before workspace migration begins
- Initially skip if `crates/core/` doesn't exist
- Consider adding to pre-commit hooks for faster feedback
- Script should be part of the standard development workflow in AGENTS.md

## External References

- [GitHub Actions](https://docs.github.com/en/actions) - CI workflow reference
- [Shell Scripting Guide](https://www.gnu.org/software/bash/manual/) - Bash scripting
