# ADR-8icsq Vehicle Type Separation: Compile-Time Feature Flags

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-aw3h3-rover-loiter-mode](../requirements/FR-aw3h3-rover-loiter-mode.md)
  - [FR-sp3at-control-modes](../requirements/FR-sp3at-control-modes.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-n24yy-rover-loiter-mode](../tasks/T-n24yy-rover-loiter-mode/README.md)

## Context

This project supports multiple vehicle types (Rover, Boat) that share core infrastructure but require different behavioral implementations for certain modes. Specifically:

- **Loiter Mode**: Rover typically stops in place (Type 0 sufficient); Boat requires continuous station keeping against currents
- **Circle Mode**: Different radius constraints and path following behavior
- **RTL Mode**: Different approach patterns (direct vs. considering waterway constraints)

### Problem

We need an architecture that:

- Provides clean separation between vehicle-type-specific implementations
- Avoids runtime conditional logic that increases code complexity
- Optimizes binary size by including only relevant code
- Allows independent evolution of Rover and Boat implementations
- Maintains shared code for common functionality (GPS, navigation math, MAVLink)

### Constraints

- **Binary Size**: Embedded target (RP2350) has limited flash
- **Maintainability**: Avoid tangled conditionals mixing Rover/Boat logic
- **Testing**: Each vehicle type should be testable independently
- **ArduPilot Compatibility**: Follow similar patterns where possible

### Forces in Tension

1. **Code Reuse vs. Specialization**: Maximizing shared code vs. optimized behavior per vehicle
2. **Runtime Flexibility vs. Build Simplicity**: Single binary for all vehicles vs. vehicle-specific builds
3. **Development Speed vs. Code Organization**: Quick iteration vs. clean architecture

## Decision

**We will separate Rover and Boat implementations at compile-time using Cargo feature flags.**

### Architecture

```
src/
├── core/                    # Shared infrastructure (no feature gates)
│   ├── parameters/
│   └── navigation/
│       └── geo.rs          # Distance/bearing calculations (shared)
│
├── rover/                   # Rover-specific implementation
│   ├── mod.rs              # #[cfg(feature = "rover")]
│   └── mode/
│       ├── mod.rs
│       ├── manual.rs
│       ├── hold.rs
│       └── loiter.rs       # Rover-specific Loiter (Type 0/1)
│
└── boat/                    # Boat-specific implementation (future)
    ├── mod.rs              # #[cfg(feature = "boat")]
    └── mode/
        ├── mod.rs
        └── loiter.rs       # Boat-specific Loiter (active station keeping)
```

### Feature Flags

```toml
[features]
rover = []      # Ground vehicle implementation
boat = []       # Watercraft implementation (future)
```

### Decision Drivers

1. **Clean Separation**: Each vehicle type has focused, readable code
2. **Optimized Binary**: Only relevant code included in build
3. **Independent Evolution**: Rover and Boat can evolve separately
4. **Simplified Testing**: Test each vehicle type in isolation
5. **ArduPilot Pattern**: ArduPilot uses separate vehicle directories

### Considered Options

- **Option A: Compile-Time Feature Flags** ⭐ Selected
- **Option B: Runtime Configuration**
- **Option C: Trait-Based Vehicle Abstraction**

### Option Analysis

**Option A: Compile-Time Feature Flags**

- **Pros**:
  - Zero runtime overhead
  - Smaller binary size (only relevant code)
  - Clean code organization
  - Easy to understand which code applies to which vehicle
  - Matches ArduPilot's vehicle directory structure
- **Cons**:
  - Requires separate build per vehicle type
  - Potential code duplication between vehicles
  - Cannot switch vehicle type at runtime

**Option B: Runtime Configuration**

- **Pros**:
  - Single binary for all vehicles
  - Can switch vehicle type via parameter
- **Cons**:
  - Larger binary (includes all vehicle code)
  - Runtime conditionals increase complexity
  - Mixed logic harder to maintain
  - Runtime overhead for vehicle type checks

**Option C: Trait-Based Vehicle Abstraction**

- **Pros**:
  - Maximum code reuse via shared traits
  - Clean abstraction
- **Cons**:
  - Over-engineering for two vehicle types
  - Complex trait hierarchies
  - Dynamic dispatch overhead
  - Harder to optimize per vehicle

## Rationale

**Compile-Time Feature Flags** was selected because:

1. **Clarity**: Developers know exactly which code applies to their vehicle
2. **Performance**: No runtime overhead for vehicle type checks
3. **Size**: Binary includes only relevant code (important for RP2350)
4. **Maintainability**: Easier to modify Rover without affecting Boat (and vice versa)
5. **ArduPilot Precedent**: Similar architecture proven at scale

### Trade-offs Accepted

- **Separate Builds**: Must build separately for Rover vs. Boat (acceptable, different hardware configs anyway)
- **Potential Duplication**: Some logic may be duplicated; extract to shared modules when pattern emerges

## Consequences

### Positive

- **Clean Organization**: Vehicle-specific code in dedicated directories
- **Optimized Binaries**: Smaller, faster builds per vehicle type
- **Independent Development**: Rover and Boat teams can work independently
- **Clear Ownership**: Each vehicle type has clear code boundaries

### Negative

- **Build Complexity**: CI must build multiple vehicle configurations
- **Duplication Risk**: Similar logic may be duplicated across vehicles
- **No Runtime Switching**: Cannot change vehicle type without reflashing

### Neutral

- **Pattern Familiarity**: Follows ArduPilot's established architecture
- **Future Vehicles**: Pattern extends to Copter, Plane if needed

## Implementation Notes

### Feature Gate Usage

```rust
// src/rover/mode/loiter.rs
#[cfg(feature = "rover")]
pub struct RoverLoiter {
    loiter_point: GpsPosition,
    loiter_type: u8,
    radius: f32,
}

#[cfg(feature = "rover")]
impl Mode for RoverLoiter {
    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        // Rover-specific Loiter logic
        // Type 0: simple stop
        // Type 1: occasional correction
    }
}
```

```rust
// src/boat/mode/loiter.rs (future)
#[cfg(feature = "boat")]
pub struct BoatLoiter {
    loiter_point: GpsPosition,
    radius: f32,
    // No loiter_type - boats always use active station keeping
}

#[cfg(feature = "boat")]
impl Mode for BoatLoiter {
    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        // Boat-specific Loiter logic
        // Always active station keeping against current
    }
}
```

### Shared Components

Shared code remains ungated in `src/subsystems/` and `src/core/`:

```rust
// src/subsystems/navigation/geo.rs - No feature gate
pub fn distance_m(a: &GpsPosition, b: &GpsPosition) -> f32 {
    // Haversine calculation - shared by all vehicles
}

pub fn bearing_deg(from: &GpsPosition, to: &GpsPosition) -> f32 {
    // Bearing calculation - shared by all vehicles
}
```

### Build Configuration

```bash
# Build for Rover (default)
./scripts/build-rp2350.sh pico_trail_rover

# Build for Boat (future)
./scripts/build-rp2350.sh pico_trail_boat --features boat
```

## Open Questions

- [x] Should parameters be shared or vehicle-specific? → Method: Shared parameter infrastructure, vehicle-specific parameter sets
- [ ] How to handle features that overlap (e.g., Circle mode)? → Method: Evaluate when implementing; likely share path calculation, separate execution
- [ ] Should we enforce mutually exclusive features? → Method: Add compile-time check to prevent `rover` + `boat` in same build

## External References

- [ArduPilot Vehicle Types](https://ardupilot.org/dev/docs/apmcopter-code-overview.html) - ArduPilot's vehicle separation architecture
- [Rust Conditional Compilation](https://doc.rust-lang.org/reference/conditional-compilation.html) - Feature flag documentation
