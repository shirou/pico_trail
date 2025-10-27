# NFR-pj11s Memory Safety - No Unsafe Rust

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall not use `unsafe` Rust code blocks outside the Hardware Abstraction Layer (HAL), prioritizing memory safety and correctness over micro-optimizations.

## Rationale

Rust's ownership model provides strong memory safety guarantees, but `unsafe` blocks bypass these protections. Restricting `unsafe` usage:

- **Prevents Memory Bugs**: No use-after-free, double-free, null pointer dereferences
- **Simplifies Auditing**: Reviewers only need to audit HAL layer, not entire codebase
- **Aligns with Project Principles**: "Memory Safety Over Micro-optimization" (see AGENTS.md)
- **Reduces Debugging Time**: Memory safety bugs are notoriously difficult to debug

ArduPilot and PX4 (written in C/C++) have encountered memory safety issues. Rust eliminates this entire class of bugs when `unsafe` is avoided.

## User Story (if applicable)

The system shall avoid `unsafe` Rust code outside the platform HAL to ensure memory safety guarantees are maintained, preventing undefined behavior and making the codebase easier to audit and maintain.

## Acceptance Criteria

- [ ] Zero `unsafe` blocks in application layer (`src/vehicle/`, `src/communication/`)
- [ ] Zero `unsafe` blocks in subsystem layer (`src/subsystems/`, `src/core/`)
- [ ] Zero `unsafe` blocks in device layer (`src/devices/`)
- [ ] `unsafe` allowed only in platform layer (`src/platform/`) for hardware register access
- [ ] All `unsafe` blocks in HAL must have safety comments explaining invariants
- [ ] Automated check in CI pipeline: `grep -r "unsafe" src/ | grep -v "src/platform"` returns no matches
- [ ] Code review checklist includes "No new unsafe blocks outside HAL"

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Allowed `unsafe` Usage (Platform Layer Only):**

```rust
// src/platform/rp2350/uart.rs
pub fn init_uart() {
    unsafe {
        // SAFETY: UART0 peripheral register is valid and accessed from single thread
        let uart_ptr = 0x4003_4000 as *mut u32;
        uart_ptr.write_volatile(0x01); // Enable UART
    }
}
```

**Forbidden `unsafe` Usage (Application/Subsystem/Device Layers):**

```rust
// src/devices/gps/ubx.rs - FORBIDDEN
pub fn parse_message(data: &[u8]) -> GpsPosition {
    unsafe {
        // FORBIDDEN: Avoid unsafe even for performance optimization
        let ptr = data.as_ptr() as *const GpsPosition;
        *ptr // Violates alignment, endianness, validity assumptions
    }
}

// Correct approach: Use safe deserialization
pub fn parse_message(data: &[u8]) -> Result<GpsPosition> {
    GpsPosition::from_bytes(data) // Safe parsing, error handling
}
```

**Alternative Techniques:**

Instead of `unsafe`, use:

1. **Type System**: Enforce invariants at compile time
2. **Safe Abstractions**: Use crates like `embedded-hal`, `nalgebra`, `heapless`
3. **Accept Performance Trade-offs**: Clone small strings instead of using `Box::leak()`
4. **Use Verified Crates**: Prefer well-audited crates over custom `unsafe` code

**Auditing `unsafe` in HAL:**

All `unsafe` blocks must include SAFETY comments:

```rust
unsafe {
    // SAFETY: This register access is safe because:
    // 1. TIMER0 peripheral is initialized and owned by this struct
    // 2. No other code accesses this register concurrently
    // 3. Write operation is atomic and does not violate hardware invariants
    (*TIMER0::ptr()).ctrl.write(|w| w.enable().set_bit());
}
```

## Platform Considerations

### Pico W (RP2040)

HAL layer (`src/platform/rp2040/`) may use `unsafe` for register access. All other layers must be safe.

### Pico 2 W (RP2350)

HAL layer (`src/platform/rp2350/`) may use `unsafe` for register access. All other layers must be safe.

### Cross-Platform

Platform-independent code (`src/vehicle/`, `src/subsystems/`, `src/devices/`, `src/core/`) must be 100% safe Rust.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                               | Validation                                       |
| ----------------------------------------- | ------ | ---------- | -------------------------------------------------------- | ------------------------------------------------ |
| Performance insufficient without `unsafe` | Medium | Low        | Profile first, optimize algorithm, accept trade-off      | Benchmark critical paths, verify meets NFR-ukjvr |
| HAL crate uses `unsafe` internally        | Low    | High       | Accept (audited by HAL maintainers), isolate to platform | Review HAL source, pin versions                  |
| Developer adds `unsafe` for convenience   | Medium | Medium     | Enforce via CI, code review, reject PRs with unsafe      | Automated grep check in CI                       |
| Lifetime issues force use of `unsafe`     | High   | Low        | Refactor to use references, accept cloning, consult docs | Pair programming for complex lifetime issues     |

## Implementation Notes

**CI Enforcement:**

```yaml
# .github/workflows/ci.yml
- name: Check for unsafe code outside platform layer
  run: |
    if grep -r "unsafe" src/ | grep -v "src/platform" | grep -v "//.*unsafe"; then
      echo "ERROR: unsafe code found outside platform layer"
      exit 1
    fi
```

**Alternatives to `unsafe`:**

**Problem**: Need static string from runtime value

```rust
// Bad: Use Box::leak() to create static reference (memory leak)
let s: &'static str = Box::leak(value.to_string().into_boxed_str());

// Good: Clone string or use Cow<'static, str>
let s: String = value.to_string(); // Accept allocation
```

**Problem**: Need to access hardware register

```rust
// Bad: Raw pointer access outside HAL
let gpio_ptr = 0x4001_4000 as *mut u32;
unsafe { gpio_ptr.write_volatile(0x01); }

// Good: Use platform abstraction trait
let gpio = platform.gpio(GPIO_PIN_25);
gpio.set_high();
```

Related code areas:

- `src/platform/` - Only layer where `unsafe` is allowed
- All other `src/` directories - Must be 100% safe Rust
- `.github/workflows/` - CI pipeline with unsafe check

## External References

- Rust Unsafe Code Guidelines: <https://rust-lang.github.io/unsafe-code-guidelines/>
- Embedded Rust Book - Unsafe: <https://docs.rust-embedded.org/book/c-tips/index.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
