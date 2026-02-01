# NFR-00052 Pre-Disarm Validation Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00017-pre-disarm-validation](../analysis/AN-00017-pre-disarm-validation.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Pre-disarm validation shall add no more than 20 bytes of RAM overhead for configuration structures to maintain strict memory budget on embedded platforms.

## Rationale

Pre-disarm validation requires storing configuration thresholds (max throttle values, max velocity, validation flags) in RAM. RP2040 and RP2350 have limited RAM (264 KB), requiring strict memory management:

- **Memory Budget**: Total vehicle system must fit in 264 KB RAM (RP2040/RP2350)
- **Validation Config**: Thresholds and flags must be stored in RAM for runtime access
- **No Dynamic Allocation**: Embedded systems avoid heap allocation, use static/stack only
- **Minimal Overhead**: Validation is utility feature, should not consume significant memory
- **Industry Standard**: ArduPilot stores similar validation configuration in \~16-20 bytes

The 20 byte limit ensures validation adds minimal overhead while providing configurable thresholds for safe disarm. Combined with post-disarm cleanup (\~70 B), total arming system overhead is \~90 B.

## User Story (if applicable)

The system shall store pre-disarm validation configuration (throttle thresholds, velocity thresholds, validation flags) in no more than 20 bytes of RAM to maintain strict memory budget on embedded platforms.

## Acceptance Criteria

- [ ] DisarmValidationConfig structure size verified using `core::mem::size_of::<DisarmValidationConfig>()` <= 20 bytes
- [ ] Memory usage measured on both Pico W (RP2040) and Pico 2 W (RP2350) platforms
- [ ] Configuration structure uses packed layout to minimize padding
- [ ] No dynamic memory allocation (heap) used for validation logic
- [ ] Validation logic uses stack-only data structures (no heap)
- [ ] Memory overhead documented in code comments with size breakdown
- [ ] Static memory analysis added to CI to detect size regressions

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Resource Constraints:**

- **Memory Budget**: <= 20 bytes RAM for DisarmValidationConfig
  - `max_throttle_manual: f32` - 4 bytes (0.0-1.0 normalized throttle)
  - `max_throttle_gcs: f32` - 4 bytes (0.0-1.0 normalized throttle)
  - `max_velocity_mps: f32` - 4 bytes (velocity in m/s)
  - `check_velocity_manual: bool` - 1 byte (enable velocity check for RC)
  - `check_velocity_gcs: bool` - 1 byte (enable velocity check for GCS)
  - Padding - 2 bytes (struct alignment)
  - **Total**: 16 bytes (within 20 byte budget)

**Memory Layout:**

```rust
/// Disarm validation configuration
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DisarmValidationConfig {
    /// Maximum throttle value allowed for manual disarm (0.0-1.0)
    pub max_throttle_manual: f32,  // 4 bytes

    /// Maximum throttle value allowed for GCS disarm (0.0-1.0)
    pub max_throttle_gcs: f32,     // 4 bytes

    /// Maximum velocity allowed for disarm (m/s)
    pub max_velocity_mps: f32,     // 4 bytes

    /// Require velocity check for manual disarm
    pub check_velocity_manual: bool, // 1 byte

    /// Require velocity check for GCS disarm
    pub check_velocity_gcs: bool,    // 1 byte

    // Padding: 2 bytes (alignment to 4-byte boundary)
}

// Verify size at compile time
const _: () = assert!(core::mem::size_of::<DisarmValidationConfig>() <= 20);

impl Default for DisarmValidationConfig {
    fn default() -> Self {
        Self {
            max_throttle_manual: 0.05,   // 5% throttle
            max_throttle_gcs: 0.15,      // 15% throttle
            max_velocity_mps: 0.5,       // 0.5 m/s
            check_velocity_manual: true,  // RC requires velocity check
            check_velocity_gcs: false,   // GCS skips velocity check
        }
    }
}
```

**Memory Verification:**

```rust
#[test]
fn test_disarm_validation_config_size() {
    use core::mem::size_of;

    let config_size = size_of::<DisarmValidationConfig>();
    println!("DisarmValidationConfig size: {} bytes", config_size);

    // Assert size within budget
    assert!(config_size <= 20, "Config size exceeds 20 byte budget: {} bytes", config_size);

    // Print breakdown for documentation
    println!("Field sizes:");
    println!("  max_throttle_manual: {} bytes", size_of::<f32>());
    println!("  max_throttle_gcs: {} bytes", size_of::<f32>());
    println!("  max_velocity_mps: {} bytes", size_of::<f32>());
    println!("  check_velocity_manual: {} bytes", size_of::<bool>());
    println!("  check_velocity_gcs: {} bytes", size_of::<bool>());
    println!("  Total (with padding): {} bytes", config_size);
}
```

**Optimization Strategies:**

**Option A: Float32 Thresholds (Recommended)**

```rust
pub struct DisarmValidationConfig {
    pub max_throttle_manual: f32,  // 4 bytes, 0.0-1.0
    pub max_throttle_gcs: f32,     // 4 bytes, 0.0-1.0
    pub max_velocity_mps: f32,     // 4 bytes, m/s
    pub check_velocity_manual: bool, // 1 byte
    pub check_velocity_gcs: bool,    // 1 byte
    // Total: 14 bytes + 2 bytes padding = 16 bytes
}
```

**Pros**: Simple, readable, sufficient precision for thresholds
**Cons**: None significant

**Option B: Fixed-Point Thresholds (If Needed)**

```rust
pub struct DisarmValidationConfig {
    pub max_throttle_manual: u8,  // 1 byte, 0-100 (percent)
    pub max_throttle_gcs: u8,     // 1 byte, 0-100 (percent)
    pub max_velocity_cmps: u16,   // 2 bytes, cm/s (0-655 m/s range)
    pub flags: u8,                // 1 byte (bit flags)
    // Total: 5 bytes + 1 byte padding = 6 bytes
}

// Bit flags for validation options
const CHECK_VELOCITY_MANUAL: u8 = 1 << 0;
const CHECK_VELOCITY_GCS: u8 = 1 << 1;
```

**Pros**: Minimal memory (6 bytes), leaves 14 bytes budget for future features
**Cons**: More complex, fixed-point conversions add overhead, premature optimization

**Recommendation**: Use Option A (float32 thresholds) for Phase 1. 16 bytes well within 20 byte budget, leaves 4 bytes margin for future fields.

**Stack Memory:**

Validation logic uses only stack memory (no heap allocation):

```rust
fn validate_disarm(&self, method: DisarmMethod) -> DisarmValidationResult {
    // Stack only: method (1 byte), config reference (borrowed), result (1 byte)
    let config = &self.disarm_validation_config; // Reference only

    // All validation logic uses stack variables
    match method {
        DisarmMethod::GcsCommand => {
            let throttle = self.get_throttle_normalized(); // Stack: 4 bytes
            if throttle > config.max_throttle_gcs {
                return DisarmValidationResult::DeniedThrottleActive;
            }

            if config.check_velocity_gcs {
                let velocity = self.get_velocity_mps(); // Stack: 4 bytes
                if velocity > config.max_velocity_mps {
                    return DisarmValidationResult::DeniedVelocityTooHigh;
                }
            }
        }
        // Other methods...
    }

    DisarmValidationResult::Allowed
}

// Maximum stack usage: ~20 bytes (method, throttle, velocity, result)
```

**Total Memory Overhead:**

| Component                  | Size     | Notes                            |
| -------------------------- | -------- | -------------------------------- |
| DisarmValidationConfig     | 16 B     | Configuration thresholds         |
| DisarmValidationResult     | 1 B      | Enum (1 byte)                    |
| Validation logic (code)    | 0 B      | Code in flash, not RAM           |
| Stack usage (during call)  | \~20 B   | Temporary, released after return |
| **Total RAM (persistent)** | **17 B** | Within 20 B budget               |

**Combined with Post-Disarm Cleanup:**

| Component                 | Size     | Notes                           |
| ------------------------- | -------- | ------------------------------- |
| Pre-disarm validation     | 17 B     | Validation config + result      |
| Post-disarm cleanup       | 70 B     | Cleanup state + logging         |
| **Total (arming system)** | **87 B** | Total arming/disarming overhead |

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz, 264 KB RAM:

- DisarmValidationConfig stored in .bss section (16 bytes)
- Validation logic compiled to Thumb-2 instructions (flash only)
- Stack usage minimal (\~20 bytes during validation)
- Total overhead: 16 bytes RAM, negligible for 264 KB budget

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz with FPU, 520 KB RAM:

- Same memory layout as RP2040
- More RAM available (520 KB), overhead even less significant
- FPU accelerates float32 operations (no performance penalty)

### Cross-Platform

Pre-disarm validation memory overhead consistent across both platforms. 16-17 byte overhead negligible on both RP2040 and RP2350.

## Risks & Mitigation

| Risk                                       | Impact | Likelihood | Mitigation                                                        | Validation                                           |
| ------------------------------------------ | ------ | ---------- | ----------------------------------------------------------------- | ---------------------------------------------------- |
| Struct size exceeds 20 byte budget         | High   | Low        | Use compile-time assertion, verify size in CI                     | `assert!(size_of::<Config>() <= 20)` at compile time |
| Future fields increase size beyond budget  | Medium | Medium     | Reserve 4 bytes margin (16 B current, 20 B budget), document size | Track size in CI, fail if exceeds 20 B               |
| Struct padding wastes memory               | Low    | Low        | Use `#[repr(C)]` or `#[repr(packed)]` to control layout           | Measure actual size, optimize if needed              |
| Stack overflow during validation           | High   | Low        | Validation uses minimal stack (\~20 B), profile stack usage       | Stack profiling on embedded target                   |
| Dynamic allocation introduced accidentally | High   | Low        | Code review, no `Box`, `Vec`, `String` in validation logic        | Manual review, embedded linting rules                |
| Memory regression in future updates        | Medium | Medium     | Add size test to CI, fail if DisarmValidationConfig exceeds 20 B  | Automated size check in test suite                   |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Pre-disarm validation configuration (memory-optimized)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct DisarmValidationConfig {
    pub max_throttle_manual: f32,  // 4 bytes
    pub max_throttle_gcs: f32,     // 4 bytes
    pub max_velocity_mps: f32,     // 4 bytes
    pub check_velocity_manual: bool, // 1 byte
    pub check_velocity_gcs: bool,    // 1 byte
    // Padding: 2 bytes
}

// Compile-time size verification
const _: () = assert!(core::mem::size_of::<DisarmValidationConfig>() <= 20);

impl DisarmValidationConfig {
    /// Create default configuration (embedded-friendly, no allocation)
    pub const fn default_config() -> Self {
        Self {
            max_throttle_manual: 0.05,
            max_throttle_gcs: 0.15,
            max_velocity_mps: 0.5,
            check_velocity_manual: true,
            check_velocity_gcs: false,
        }
    }
}

/// SystemState with validation config
pub struct SystemState {
    // ... other fields ...

    /// Pre-disarm validation configuration (16 bytes RAM)
    disarm_validation_config: DisarmValidationConfig,
}

impl SystemState {
    pub fn new() -> Self {
        Self {
            // ... other initializations ...
            disarm_validation_config: DisarmValidationConfig::default_config(),
        }
    }
}
```

**Memory Testing:**

```rust
#[test]
fn test_memory_budget_compliance() {
    use core::mem::size_of;

    // Verify DisarmValidationConfig size
    let config_size = size_of::<DisarmValidationConfig>();
    assert!(config_size <= 20, "DisarmValidationConfig exceeds budget: {} bytes", config_size);

    // Verify DisarmValidationResult size
    let result_size = size_of::<DisarmValidationResult>();
    assert!(result_size <= 1, "DisarmValidationResult exceeds 1 byte: {} bytes", result_size);

    // Document total overhead
    let total_overhead = config_size + result_size;
    println!("Pre-disarm validation memory overhead: {} bytes", total_overhead);
    println!("  Config: {} bytes", config_size);
    println!("  Result: {} bytes", result_size);
    println!("Budget: 20 bytes, Remaining: {} bytes", 20 - total_overhead);
}

#[test]
fn test_no_dynamic_allocation() {
    // Verify validation logic doesn't use heap
    // This is a manual review check - no dynamic types in validation code:
    // - No Box<T>
    // - No Vec<T>
    // - No String
    // - No Rc<T>, Arc<T>
    // - Stack only: primitives, references, enums
}
```

**CI Memory Check:**

Add to CI pipeline:

```bash
# Build and check size
cargo build --target thumbv8m.main-none-eabihf --release

# Extract DisarmValidationConfig size from debug info
# Fail if exceeds 20 bytes
./scripts/check_struct_size.sh DisarmValidationConfig 20
```

Related code areas:

- `src/vehicle/arming/types.rs` - DisarmValidationConfig structure
- `src/communication/mavlink/state.rs` - SystemState with embedded config
- `src/platform/*/memory.rs` - Memory profiling utilities

## External References

- Analysis: [AN-00017-pre-disarm-validation](../analysis/AN-00017-pre-disarm-validation.md)
  N/A - No external references
