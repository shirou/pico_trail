# NFR-35otr EKF Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-3f2cn-quaternion-ekf-ahrs](FR-3f2cn-quaternion-ekf-ahrs.md)
- Dependent Requirements: None
- Related ADRs:
  - [ADR-ymkzt-ekf-ahrs-implementation](../adr/ADR-ymkzt-ekf-ahrs-implementation.md)
- Related Tasks:
  - [T-p8w8f-ekf-ahrs-implementation](../tasks/T-p8w8f-ekf-ahrs-implementation/README.md)

## Requirement Statement

The EKF AHRS implementation shall use less than 10KB of RAM for state storage, covariance matrices, and working buffers, ensuring adequate memory remains for other subsystems.

## Rationale

Memory is constrained on embedded platforms:

- **RP2040**: 264KB RAM total
- **RP2350**: 520KB RAM total
- **Other Subsystems**: GPS, MAVLink, navigation, control all need memory
- **No Heap**: Static allocation only, memory usage must be predictable

A 7-state EKF with 7x7 covariance requires \~400 bytes for matrices alone. With working buffers, 10KB provides generous headroom while remaining practical.

## User Story (if applicable)

The system shall implement EKF AHRS within 10KB RAM budget to coexist with other subsystems without memory exhaustion.

## Acceptance Criteria

- [ ] `AhrsEkf` struct size < 1KB (verified via `core::mem::size_of`)
- [ ] Working buffers for matrix operations < 2KB
- [ ] Total AHRS module static memory < 5KB
- [ ] `AttitudeState` global < 100 bytes
- [ ] Total EKF-related RAM usage < 10KB
- [ ] Memory usage verified on both RP2040 and RP2350
- [ ] No heap allocation in AHRS code

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Memory Breakdown:**

| Component                 | Size (bytes) | Notes             |
| ------------------------- | ------------ | ----------------- |
| State vector x (7 floats) | 28           | f32 = 4 bytes     |
| Covariance P (7x7)        | 196          | 49 × 4 bytes      |
| Process noise Q (7x7)     | 196          | 49 × 4 bytes      |
| Accel noise R (3x3)       | 36           | 9 × 4 bytes       |
| Mag noise R (3x3)         | 36           | 9 × 4 bytes       |
| Mag reference (3 floats)  | 12           |                   |
| Config/misc               | \~100        | Timestamps, flags |
| **AhrsEkf total**         | **\~600**    |                   |

| Working Buffers     | Size (bytes) | Notes               |
| ------------------- | ------------ | ------------------- |
| Jacobian F (7x7)    | 196          | Temp for prediction |
| Measurement H (3x7) | 84           | Temp for correction |
| Innovation S (3x3)  | 36           | Temp for correction |
| Kalman gain K (7x3) | 84           | Temp for correction |
| Temp matrices       | \~400        | Various operations  |
| **Working total**   | **\~800**    |                     |

| Global State  | Size (bytes) | Notes                    |
| ------------- | ------------ | ------------------------ |
| AttitudeState | \~80         | Quaternion, Euler, rates |
| **Total**     | **\~80**     |                          |

**Total Estimated: \~1.5KB** (well under 10KB budget)

**Verification Method:**

```rust
// Compile-time size verification
const _: () = {
    assert!(core::mem::size_of::<AhrsEkf>() < 1024);
    assert!(core::mem::size_of::<AttitudeState>() < 128);
};

// Runtime verification
fn log_memory_usage() {
    log_info!("AhrsEkf size: {} bytes", core::mem::size_of::<AhrsEkf>());
    log_info!("AttitudeState size: {} bytes", core::mem::size_of::<AttitudeState>());
}
```

**Memory Optimization Strategies:**

1. **In-place Operations**: Reuse matrices instead of allocating new ones
2. **Symmetric Matrix Storage**: Store only lower/upper triangle (saves 50%)
3. **Reduced Precision**: Use f16 for covariance if accuracy allows
4. **Lazy Computation**: Compute Jacobians on demand, not stored

**No-Heap Verification:**

```rust
// In Cargo.toml
[profile.release]
panic = "abort"  # No unwinding, smaller code

// In code
#![no_std]
#![deny(unsafe_code)]

// Use static allocation only
static EKF: StaticCell<AhrsEkf> = StaticCell::new();
```

## Platform Considerations

### Pico W (RP2040)

- 264KB RAM total
- AHRS budget: 10KB (3.8% of total)
- Leaves \~250KB for other subsystems

### Pico 2 W (RP2350)

- 520KB RAM total
- AHRS budget: 10KB (1.9% of total)
- Generous headroom

### Cross-Platform

- Same memory layout on both platforms
- No platform-specific memory optimizations needed
- Static allocation works identically

## Risks & Mitigation

| Risk                                   | Impact | Likelihood | Mitigation                            | Validation               |
| -------------------------------------- | ------ | ---------- | ------------------------------------- | ------------------------ |
| Matrix operations need more temp space | Medium | Low        | Pre-allocate largest needed buffer    | Compile-time size check  |
| Stack overflow from large locals       | High   | Medium     | Move large matrices to static storage | Test with stack painting |
| Hidden allocations from dependencies   | High   | Low        | Audit dependencies for heap usage     | Use `no_std` crates only |
| Memory fragmentation                   | N/A    | N/A        | No heap = no fragmentation            | N/A                      |

## Implementation Notes

**Static Allocation Pattern:**

```rust
use static_cell::StaticCell;

// Static storage for EKF
static EKF_CELL: StaticCell<AhrsEkf> = StaticCell::new();

pub fn init_ahrs(config: EkfConfig) -> &'static mut AhrsEkf {
    EKF_CELL.init(AhrsEkf::new(config))
}
```

**Working Buffer Management:**

```rust
impl AhrsEkf {
    // Pre-allocated working buffers as struct members
    // (included in struct size above)
    work_f: Matrix7<f32>,    // State Jacobian
    work_h: Matrix3x7<f32>,  // Measurement Jacobian
    work_s: Matrix3<f32>,    // Innovation covariance
    work_k: Matrix7x3<f32>,  // Kalman gain
}
```

**Memory Map Inspection:**

```bash
# Check section sizes after build
arm-none-eabi-size target/thumbv8m.main-none-eabihf/release/pico_trail

# Check symbol sizes
arm-none-eabi-nm --size-sort target/thumbv8m.main-none-eabihf/release/pico_trail | grep -i ekf
```

## External References

- [Embedded Rust Memory Management](https://docs.rust-embedded.org/book/static-guarantees/index.html)
- [static_cell Crate](https://crates.io/crates/static_cell)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
