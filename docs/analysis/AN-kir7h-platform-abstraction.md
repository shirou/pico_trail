# AN-kir7h Platform Abstraction Strategy for Multi-Platform Support

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-cp76d-ardupilot-analysis](AN-cp76d-ardupilot-analysis.md)
  - [AN-5nucb-core-systems](AN-5nucb-core-systems.md)
- Related Requirements: N/A - Requirements will be created based on this analysis
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis examines strategies for abstracting hardware differences between Raspberry Pi Pico W (RP2040, Cortex-M0+) and Pico 2 W (RP2350, Cortex-M33) while maintaining the flexibility to support additional platforms in the future (ESP32, STM32F4). The goal is to design an abstraction layer that isolates platform-specific code to a minimal surface area (`src/platform/`), enabling the majority of the codebase to be platform-independent.

Key findings: A 3-tier abstraction approach (embedded-hal → Platform Traits → Device/Subsystem Traits) provides the best balance of flexibility and simplicity. The embedded-hal crate provides foundational traits for peripherals (UART, I2C, SPI, PWM), but project-specific traits are needed for higher-level abstractions (e.g., `GpsSensor`, `ImuSensor`). This approach supports future expansion to ESP32 and STM32 platforms with minimal code changes.

## Problem Space

### Current State

The project currently uses:

- `rp235x-hal` for RP2350 (Pico 2 W) support
- Thumbv8m.main-none-eabihf target (Cortex-M33 with FPU)
- `cargo` build aliases for ARM and RISC-V targets (though RISC-V not currently used)
- No abstraction layer - HAL is directly imported in `src/lib.rs`

### Desired State

A platform abstraction system that:

- Supports Pico W (RP2040) and Pico 2 W (RP2350) from a single codebase
- Isolates all platform-specific code to `src/platform/` directory
- Enables adding new platforms (ESP32, STM32) by implementing a trait interface
- Allows compile-time platform selection via Cargo features
- Maintains zero-cost abstractions (no runtime overhead)

### Gap Analysis

**Missing Infrastructure**:

- No platform abstraction traits defined
- No feature flags for platform selection
- No build system for multi-platform compilation
- No testing strategy for platform-independent code

**Key Challenges**:

1. **CPU Differences**: Cortex-M0+ (Pico W) has no FPU, Cortex-M33 (Pico 2 W) has FPU - affects math-heavy code (AHRS, control)
2. **Memory Differences**: Pico W has 264 KB RAM vs Pico 2 W's 520 KB - affects buffer sizes, EKF complexity
3. **Peripheral Differences**: RP2040 and RP2350 have different peripheral counts and capabilities
4. **HAL Crate Differences**: `rp2040-hal` vs `rp235x-hal` have similar but not identical APIs

## Stakeholder Analysis

| Stakeholder              | Interest/Need                                    | Impact | Priority |
| ------------------------ | ------------------------------------------------ | ------ | -------- |
| Pico W Users             | Support for lower-cost hardware                  | High   | P0       |
| Pico 2 W Users           | Full feature set with more capable hardware      | High   | P0       |
| Future Platform Adopters | Easy path to port to ESP32, STM32, etc.          | Medium | P1       |
| Core Developers          | Clean separation between platform and logic code | High   | P0       |
| CI/CD Systems            | Ability to build and test for multiple platforms | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - New project without user base.

### Competitive Analysis

**ArduPilot's Approach** (`AP_HAL`):

- Single `AP_HAL` interface for all platforms
- Platform-specific backends: `AP_HAL_ChibiOS`, `AP_HAL_Linux`, `AP_HAL_ESP32`
- Abstracted interfaces: UART, I2C, SPI, GPIO, Storage, RCInput, RCOutput, Scheduler
- Pros: Very portable (runs on 20+ hardware platforms), well-tested
- Cons: C++ complexity, runtime polymorphism overhead, requires significant HAL implementation effort per platform

**Embassy's Approach**:

- Hardware-specific async drivers per platform
- Shared async runtime across platforms
- `embassy-rp` for RP2040/RP2350, `embassy-stm32` for STM32, etc.
- Pros: Modern async/await model, zero-cost abstractions, active development
- Cons: Still requires platform-specific driver code, async model may be overkill for some applications

**embedded-hal Ecosystem**:

- Standard traits: `embedded_hal::spi::SpiDevice`, `embedded_hal::i2c::I2c`, etc.
- Platform HALs implement these traits
- Pros: Widely adopted, zero-cost abstractions, compile-time dispatch
- Cons: Low-level abstractions (byte-level I2C, not "sensor-level"), requires wrapper traits for higher-level abstractions

### Technical Investigation

**RP2040 vs RP2350 Comparison**:

| Feature | RP2040 (Pico W)          | RP2350 (Pico 2 W)        | Impact on Abstraction                       |
| ------- | ------------------------ | ------------------------ | ------------------------------------------- |
| CPU     | Dual Cortex-M0+ @ 133MHz | Dual Cortex-M33 @ 150MHz | FPU availability affects math-heavy code    |
| FPU     | None                     | Single-precision FPU     | Must provide soft-float fallback for M0+    |
| RAM     | 264 KB                   | 520 KB                   | Affects buffer sizes, EKF state count       |
| Flash   | 2 MB (external)          | 4 MB (external)          | More space for logs, parameters on Pico 2 W |
| UART    | 2x                       | 2x                       | Identical                                   |
| I2C     | 2x                       | 2x                       | Identical                                   |
| SPI     | 2x                       | 2x                       | Identical                                   |
| PWM     | 8 channels               | 12 channels              | More motor/servo support on Pico 2 W        |
| Timers  | Standard ARM timers      | Standard ARM timers      | Identical                                   |

**Conclusion**: Peripherals are largely identical; main differences are CPU (FPU) and memory capacity.

**HAL Crate API Comparison**:

Both `rp2040-hal` and `rp235x-hal` implement `embedded-hal` traits, with similar structure:

```rust
// Similar in both HALs
let mut i2c = hal::I2c::new(
    peripherals.I2C0,
    sda_pin,
    scl_pin,
    400.kHz(),
    &mut resets,
);
```

Differences are mostly in peripheral counts and boot process, not core API.

### Data Analysis

N/A - No operational data available yet.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-9**: System shall support compilation for Pico W and Pico 2 W via Cargo feature flags → Will become FR-009
  - Rationale: Enable single codebase for both platforms
  - Acceptance Criteria: `cargo build --features pico_w` and `cargo build --features pico2_w` produce valid binaries

- [ ] **FR-DRAFT-10**: Platform-specific code shall be isolated to `src/platform/` directory → Will become FR-010
  - Rationale: Enforce clean separation, enable code audits
  - Acceptance Criteria: Zero direct HAL imports outside `src/platform/`, verified via `grep` or linter

- [ ] **FR-DRAFT-11**: Device drivers shall use platform-independent traits, not HAL types directly → Will become FR-011
  - Rationale: Enable driver reuse across platforms
  - Acceptance Criteria: GPS, IMU drivers compile for both Pico W and Pico 2 W without changes

- [ ] **FR-DRAFT-12**: System shall provide soft-float fallback for math operations on Pico W (no FPU) → Will become FR-012
  - Rationale: Cortex-M0+ lacks hardware FPU
  - Acceptance Criteria: AHRS and control algorithms function correctly on Pico W, validated via unit tests

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-6**: Platform abstraction shall introduce zero runtime overhead (compile-time dispatch only) → Will become NFR-006
  - Category: Performance
  - Rationale: Avoid vtable/dyn overhead in control loops
  - Target: Disassembly shows direct function calls, not indirect calls

- [ ] **NFR-DRAFT-7**: Adding a new platform shall require only implementing platform traits, no changes to device/subsystem layers → Will become NFR-007
  - Category: Portability
  - Rationale: Reduce porting effort
  - Target: ESP32 port requires only `src/platform/esp32/` implementation, measured in lines of code changed

## Design Considerations

### Technical Constraints

1. **No Dynamic Dispatch in Control Loops**: Must use compile-time monomorphization for zero-cost abstractions
2. **Feature Flags for Platform Selection**: Only one platform can be active per build
3. **FPU Availability**: Must handle Cortex-M0+ (no FPU) and Cortex-M33 (with FPU) in same codebase
4. **HAL API Variability**: Different HAL crates have similar but not identical APIs (e.g., pin configuration)

### Potential Approaches

#### 1. **Option A: Direct HAL Usage (No Abstraction)**

```rust
// In application code
#[cfg(feature = "pico_w")]
use rp2040_hal as hal;
#[cfg(feature = "pico2_w")]
use rp235x_hal as hal;

// Use hal:: throughout codebase
```

- Pros: Simplest approach, no abstraction overhead
- Cons: Platform-specific code scattered throughout codebase, difficult to port to non-RP platforms, violates portability requirement
- Effort: Low initial, high long-term maintenance

#### 2. **Option B: embedded-hal Only**

```rust
// Device drivers use embedded-hal traits
pub struct Gps<I: embedded_hal::serial::Read> {
    uart: I,
}
```

- Pros: Standard traits, wide ecosystem support, zero overhead
- Cons: Very low-level (byte-oriented), doesn't abstract higher-level concepts (e.g., "get GPS fix"), still requires platform-specific initialization code
- Effort: Medium

#### 3. **Option C: Two-Tier Abstraction (embedded-hal + Platform Traits)**

```rust
// Platform trait
pub trait Platform {
    type Uart: embedded_hal::serial::Read + embedded_hal::serial::Write;
    type I2c: embedded_hal::i2c::I2c;
    // ...
    fn init() -> Self;
}

// Platform implementation
#[cfg(feature = "pico2_w")]
pub struct Pico2W;

#[cfg(feature = "pico2_w")]
impl Platform for Pico2W {
    type Uart = rp235x_hal::uart::UartPeripheral</* ... */>;
    // ...
}
```

- Pros: Clean separation, platform initialization encapsulated, device drivers use embedded-hal traits
- Cons: Still low-level for devices, requires wrapper types
- Effort: Medium

#### 4. **Option D: Three-Tier Abstraction (HAL + Platform + Device)** ⭐ **Recommended**

```rust
// Tier 1: Platform traits (low-level peripherals)
pub trait Platform {
    type Uart: UartInterface;
    type I2c: I2cInterface;
    type Pwm: PwmInterface;
    // ...
}

// Tier 2: Device traits (high-level abstractions)
pub trait GpsSensor {
    fn read_position(&mut self) -> Result<GpsPosition>;
    fn has_fix(&self) -> bool;
}

pub trait ImuSensor {
    fn read_accel(&mut self) -> Result<Vector3>;
    fn read_gyro(&mut self) -> Result<Vector3>;
}

// Tier 3: Implementations use device traits
pub struct WaypointNavigator<G: GpsSensor> {
    gps: G,
    // ...
}
```

- Pros: Maximum flexibility, clean separation at all levels, device/subsystem layers completely platform-independent, easy to add new platforms and new devices
- Cons: More initial boilerplate, three layers of abstraction to understand
- Effort: Medium-High (upfront), Low (long-term)

**Recommendation**: **Option D** (Three-Tier Abstraction) provides the best balance of portability, testability, and long-term maintainability. The upfront effort is justified by the goal of supporting multiple platforms.

### Architecture Impact

This analysis will drive the following ADRs:

- **ADR-006**: Platform abstraction layer design (three-tier approach)
- **ADR-007**: Feature flag strategy for platform selection
- **ADR-008**: FPU usage strategy (hardware FPU on Pico 2 W, soft-float on Pico W)

## Risk Assessment

| Risk                                                  | Probability | Impact | Mitigation Strategy                                                                    |
| ----------------------------------------------------- | ----------- | ------ | -------------------------------------------------------------------------------------- |
| Abstraction overhead affects control loop performance | Low         | Medium | Use compile-time monomorphization, benchmark critical paths, profile on both platforms |
| HAL API changes break platform implementations        | Medium      | Low    | Pin HAL crate versions, test on both platforms in CI                                   |
| FPU performance difference too large for Pico W       | Medium      | Medium | Optimize soft-float paths, consider dropping Pico W support if performance inadequate  |
| Future platform (ESP32) doesn't fit abstraction model | Low         | Medium | Design traits based on common embedded patterns, review ESP32 HAL early                |

## Open Questions

- [ ] Should we support RISC-V Hazard3 core on Pico 2 W, or focus on ARM Cortex-M33 only? → Next step: Prototype build for RISC-V target, assess toolchain maturity
- [ ] How do we handle platform-specific feature differences (e.g., Pico 2 W has more PWM channels)? → Method: Use const generics or associated constants in Platform trait
- [ ] Should platform selection be via feature flags or build profiles? → Next step: Draft ADR-007 to evaluate feature flags vs profiles
- [ ] Can we share a single memory layout (`memory.x`) or do we need platform-specific ones? → Method: Review linker scripts for RP2040 vs RP2350

## Recommendations

### Immediate Actions

1. Define core platform traits in `src/platform/traits/` (UART, I2C, SPI, PWM, Timer, Storage, GPIO)
2. Implement Pico 2 W platform (`src/platform/pico2_w/`) as the primary development target
3. Add Cargo feature flags for `pico_w` and `pico2_w` (mutually exclusive)

### Next Steps

1. [ ] Create ADR-006 for platform abstraction layer design
2. [ ] Create ADR-007 for feature flag strategy
3. [ ] Create ADR-008 for FPU usage strategy
4. [ ] Create NFR-006 and NFR-007 for performance and portability requirements
5. [ ] Create task T-002 for platform abstraction implementation

### Out of Scope

The following are explicitly excluded from the initial platform abstraction:

- **RISC-V Support**: Focus on ARM cores initially; RISC-V can be added later if demand exists
- **USB Device Support**: Not required for initial autopilot functionality (use UART for MAVLink)
- **Bluetooth/WiFi Abstraction**: Platform-specific wireless features are out of scope (use external modules if needed)
- **Multi-Core Support**: Run on single core initially; dual-core can be added later for performance

## Appendix

### References

- embedded-hal Documentation: <https://docs.rs/embedded-hal/>
- rp2040-hal Crate: <https://docs.rs/rp2040-hal/>
- rp235x-hal Crate: <https://docs.rs/rp235x-hal/>
- RP2040 Datasheet: <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf>
- RP2350 Datasheet: <https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf>
- Embassy Framework: <https://embassy.dev/>

### Raw Data

**Feature Flag Strategy Example**:

```toml
# Cargo.toml
[features]
default = ["pico2_w"]
pico_w = ["rp2040-hal"]
pico2_w = ["rp235x-hal"]

# Ensure mutual exclusivity
[package.metadata.cargo-all-features]
skip_feature_sets = [
    ["pico_w", "pico2_w"],
]
```

**Platform Trait Example**:

```rust
// src/platform/traits/mod.rs
pub trait Platform: Sized {
    type Uart: UartInterface;
    type I2c: I2cInterface;
    type Spi: SpiInterface;
    type Pwm: PwmInterface;
    type Timer: TimerInterface;
    type Storage: StorageInterface;
    type Gpio: GpioInterface;

    fn init() -> Self;
    fn system_clock_hz(&self) -> u32;
    fn has_fpu(&self) -> bool;
}
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
