# NFR-00096 SITL Lightweight Adapter No External Dependencies

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Category: Testability

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00152-sitl-lightweight-adapter](FR-00152-sitl-lightweight-adapter.md)
- Related Tasks:
  - [T-00158-sitl-platform-and-lightweight-adapter](../tasks/T-00158-sitl-platform-and-lightweight-adapter/README.md)

## Requirement Statement

`LightweightAdapter` shall run without any external process dependencies, enabling CI testing with only `cargo test` — no Gazebo, Docker, or other external tools required.

## Rationale

CI environments vary widely. Requiring external simulators:

1. Increases CI setup complexity
2. Adds flaky test failure modes (simulator startup, networking)
3. Slows down feedback loops
4. Limits where tests can run (local dev, CI, embedded)

A pure-Rust adapter ensures tests work anywhere Rust compiles.

## Category-Specific Details

### Testability Requirements

- `cargo test --features sitl` sufficient to run SITL tests
- No Docker, virtual machines, or container dependencies
- No network services required
- No X11/display server required
- Works on Linux, macOS, and Windows

## Acceptance Criteria

- [ ] SITL tests pass with only `cargo test --features sitl`
- [ ] No Gazebo installation required for CI
- [ ] No Docker/container runtime required
- [ ] No network ports opened by LightweightAdapter
- [ ] No files written outside of test output directory
- [ ] Test startup time <1 second per test
- [ ] Works on GitHub Actions ubuntu-latest without additional setup
- [ ] Works on local development machines without special configuration

## Technical Details (if applicable)

### CI Configuration

```yaml
# .github/workflows/test.yml
jobs:
  sitl-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: dtolnay/rust-toolchain@stable

      # No additional setup needed!
      - name: Run SITL tests
        run: cargo test --features sitl

      # Optional: Run with specific scenarios
      - name: Run navigation scenarios
        run: cargo test --features sitl -- navigation_
```

### Local Development

```bash
# Just works - no setup needed
cargo test --features sitl

# Run specific SITL test
cargo test --features sitl test_guided_mode_navigation

# Run with logging
RUST_LOG=debug cargo test --features sitl -- --nocapture
```

### Dependency Audit

```toml
# crates/sitl/Cargo.toml
[dependencies]
# All pure Rust, no system dependencies
rand = "0.8"           # RNG for noise
nalgebra = "0.32"      # Math (optional, could use simpler)

# No FFI, no system libraries, no network by default
```

## Measurement / Validation

| Metric                | Target              | Measurement Method   |
| --------------------- | ------------------- | -------------------- |
| External process deps | 0                   | Dependency audit     |
| Test startup time     | <1s                 | `time cargo test`    |
| CI setup steps        | 2 (checkout + test) | Workflow file review |

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                                    |
| ------------------------------------- | ------ | ---------- | --------------------------------------------- |
| Physics too simplified for some tests | Medium | Medium     | Document limitations, use Gazebo for fidelity |
| Random seed management complexity     | Low    | Low        | Deterministic mode with explicit seeds        |

## Implementation Notes

- LightweightAdapter implements all physics internally
- Uses `rand` crate with optional seeding for determinism
- Consider `#[cfg(test)]` for test-only features

## External References

- [GitHub Actions Rust](https://github.com/actions-rs/toolchain)
