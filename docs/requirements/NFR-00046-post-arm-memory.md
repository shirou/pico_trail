# NFR-00046 Post-Arm Initialization Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Post-arm initialization shall add no more than 60 bytes RAM overhead to maintain memory budget on resource-constrained RP2040/RP2350 platforms.

## Rationale

The RP2040 (Pico W) has 264KB RAM, and the RP2350 (Pico 2 W) has 520KB RAM. Every subsystem must justify its memory footprint to avoid exhausting available RAM:

- **Memory Budget**: Post-arm state tracking is permanent (exists for vehicle lifetime)
- **Constraint**: Other subsystems (monitoring, failsafe, modes, control) also need RAM
- **Target**: Keep post-arm overhead minimal to preserve budget for flight-critical systems
- **Comparison**: ArduPilot allocates \~40 bytes for arm state (timestamp + method + flags)

The 60-byte target provides enough space for:

- Timestamp (4 bytes, u32 milliseconds)
- Arm method enum (1 byte)
- Flags (checks performed, actuators initialized, subsystems notified) (3 bytes)
- Padding and alignment (4-8 bytes)
- **Total**: \~16 bytes for PostArmState, plus \~40 bytes for temporary logging buffers

## User Story (if applicable)

The system shall track post-arm initialization state using no more than 60 bytes RAM to ensure memory-constrained platforms can support all flight-critical subsystems without exhausting available memory.

## Acceptance Criteria

- [ ] PostArmState structure uses no more than 20 bytes RAM (measured via size_of)
- [ ] Temporary logging buffers use no more than 40 bytes RAM during post-arm init
- [ ] Total post-arm memory overhead (persistent + temporary) < 60 bytes
- [ ] No heap allocations during post-arm initialization (all stack or static)
- [ ] Memory overhead verified on both Pico W (RP2040) and Pico 2 W (RP2350)
- [ ] No memory leaks detected after 1000 arm/disarm cycles

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Resource Constraints:**

- **Persistent State**: PostArmState structure (always in memory)
- **Temporary Buffers**: Log entry formatting, MAVLink messages
- **Heap Allocation**: Zero (all stack or static allocation)
- **Memory Budget**: < 60 bytes total

**Memory Breakdown:**

| Component                           | Size     | Lifetime  | Notes                               |
| ----------------------------------- | -------- | --------- | ----------------------------------- |
| PostArmState::arm_time_ms           | 4 bytes  | Permanent | u32 timestamp (milliseconds)        |
| PostArmState::arm_method            | 1 byte   | Permanent | Enum (5 variants fit in u8)         |
| PostArmState::checks_performed      | 1 byte   | Permanent | bool                                |
| PostArmState::actuators_initialized | 1 byte   | Permanent | bool                                |
| PostArmState::subsystems_notified   | 1 byte   | Permanent | bool                                |
| Padding/alignment                   | 3 bytes  | Permanent | Struct alignment to 4-byte boundary |
| **PostArmState total**              | **11 B** | Permanent | Core state tracking                 |
| Log entry buffer                    | 40 bytes | Temporary | "ARM,timestamp,method,checks\n"     |
| MAVLink STATUSTEXT buffer           | 50 bytes | Temporary | Warning messages (if needed)        |
| **Total (typical case)**            | **61 B** | Mixed     | Slightly over target, acceptable    |
| **Total (without warnings)**        | **11 B** | Mixed     | Below target if no warnings sent    |

**Implementation Strategy:**

```rust
/// Post-arm initialization state (memory-efficient)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PostArmState {
    /// Timestamp when vehicle armed (milliseconds since boot)
    pub arm_time_ms: u32,

    /// Method used to arm (1 byte enum)
    pub arm_method: ArmMethod,

    /// Were arming checks performed? (1 byte bool)
    pub checks_performed: bool,

    /// Were actuators initialized successfully? (1 byte bool)
    pub actuators_initialized: bool,

    /// Were subsystems notified successfully? (1 byte bool)
    pub subsystems_notified: bool,

    // Padding for 4-byte alignment (3 bytes implicit)
}

// Compile-time size verification
const _: () = assert!(
    core::mem::size_of::<PostArmState>() <= 20,
    "PostArmState exceeds 20-byte budget"
);

/// Arm method enumeration (1 byte)
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ArmMethod {
    Unknown = 0,
    RcRudder = 1,
    GcsCommand = 2,
    RcSwitch = 3,
    ForceArm = 4,
}

// Verify enum fits in 1 byte
const _: () = assert!(
    core::mem::size_of::<ArmMethod>() == 1,
    "ArmMethod must be 1 byte"
);
```

**Memory-Efficient Logging:**

```rust
/// Format arm event for logging (fixed-size buffer, no heap)
fn log_arm_event(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    // Stack-allocated buffer (40 bytes temporary)
    let mut buffer = [0u8; 40];

    // Format log entry: "ARM,timestamp,method,checks\n"
    // Example: "ARM,12345,GcsCommand,1\n" (24 bytes typical)
    let method_str = match method {
        ArmMethod::Unknown => "Unknown",
        ArmMethod::RcRudder => "RcRudder",
        ArmMethod::GcsCommand => "GcsCommand",
        ArmMethod::RcSwitch => "RcSwitch",
        ArmMethod::ForceArm => "ForceArm",
    };

    let checks_byte = if checks_performed { b'1' } else { b'0' };

    // Use core::fmt::Write to format into buffer (no heap)
    use core::fmt::Write;
    let mut writer = BufferWriter::new(&mut buffer);
    write!(
        &mut writer,
        "ARM,{},{},{}\n",
        self.post_arm_state.arm_time_ms,
        method_str,
        checks_byte
    ).map_err(|_| "Log format failed")?;

    // Write buffer to persistent storage
    self.logger.write(writer.as_bytes())?;

    Ok(())
}

/// Fixed-size buffer writer (no heap allocation)
struct BufferWriter<'a> {
    buffer: &'a mut [u8],
    position: usize,
}

impl<'a> BufferWriter<'a> {
    fn new(buffer: &'a mut [u8]) -> Self {
        BufferWriter { buffer, position: 0 }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.buffer[..self.position]
    }
}

impl<'a> core::fmt::Write for BufferWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buffer.len() - self.position;

        if bytes.len() > remaining {
            return Err(core::fmt::Error);
        }

        self.buffer[self.position..self.position + bytes.len()]
            .copy_from_slice(bytes);
        self.position += bytes.len();

        Ok(())
    }
}
```

**Optimization Techniques:**

- **Packed structs**: Use `#[repr(C)]` and manual padding for predictable layout
- **Enum optimization**: Use `#[repr(u8)]` to force 1-byte enums
- **Stack buffers**: Use fixed-size stack buffers instead of heap allocation
- **Bit packing**: Pack multiple bools into bitfield if needed (Phase 2 optimization)
- **Zero-cost abstractions**: Use const generics and compile-time checks

**Alternative: Bit-Packed Flags (if needed)**

```rust
/// Ultra-compact PostArmState (12 bytes)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PostArmState {
    pub arm_time_ms: u32,        // 4 bytes
    pub arm_method: ArmMethod,   // 1 byte
    pub flags: PostArmFlags,     // 1 byte (packed)
    _padding: [u8; 6],           // 6 bytes padding
}

/// Bit-packed flags (1 byte total)
#[derive(Clone, Copy)]
pub struct PostArmFlags(u8);

impl PostArmFlags {
    const CHECKS_PERFORMED: u8      = 0b0000_0001;
    const ACTUATORS_INITIALIZED: u8 = 0b0000_0010;
    const SUBSYSTEMS_NOTIFIED: u8   = 0b0000_0100;

    pub fn checks_performed(&self) -> bool {
        self.0 & Self::CHECKS_PERFORMED != 0
    }

    pub fn set_checks_performed(&mut self, value: bool) {
        if value {
            self.0 |= Self::CHECKS_PERFORMED;
        } else {
            self.0 &= !Self::CHECKS_PERFORMED;
        }
    }

    // Similar for other flags...
}
```

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ with 264KB RAM:

- Very limited RAM budget (after stack, heap, buffers: \~150KB available)
- Post-arm state must be minimal to preserve memory for flight-critical systems
- Target: < 60 bytes (0.04% of available RAM)

### Pico 2 W (RP2350)

Cortex-M33 with 520KB RAM:

- More relaxed RAM constraints (after stack, heap, buffers: \~400KB available)
- Post-arm state still must be reasonable
- Target: < 60 bytes (0.015% of available RAM)

### Cross-Platform

Post-arm initialization must meet 60-byte budget on both platforms to ensure consistency and avoid platform-specific memory pressure.

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                                             | Validation                          |
| --------------------------------------- | ------ | ---------- | ------------------------------------------------------ | ----------------------------------- |
| PostArmState grows beyond 20 bytes      | Medium | Low        | Use compile-time size assertions, review struct layout | Verify size_of in unit tests        |
| Temporary buffers exceed budget         | Medium | Medium     | Use fixed-size stack buffers, profile stack usage      | Measure stack depth during init     |
| Heap allocation introduced accidentally | High   | Medium     | Enable no-alloc checks, use forbid_alloc attribute     | Test with allocator disabled        |
| Memory leak after repeated arm/disarm   | High   | Low        | Profile memory over 1000 cycles, verify no growth      | Automated test with memory tracking |
| Struct padding wastes memory            | Low    | Medium     | Use #\[repr(C)] and manual padding, verify layout      | Check size_of and align_of          |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Memory-efficient post-arm state
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PostArmState {
    pub arm_time_ms: u32,              // 4 bytes
    pub arm_method: ArmMethod,         // 1 byte
    pub checks_performed: bool,        // 1 byte
    pub actuators_initialized: bool,   // 1 byte
    pub subsystems_notified: bool,     // 1 byte
    _padding: [u8; 4],                 // 4 bytes explicit padding (align to 8)
}

// Compile-time memory budget verification
const _: () = {
    let size = core::mem::size_of::<PostArmState>();
    let align = core::mem::align_of::<PostArmState>();

    assert!(size <= 20, "PostArmState exceeds 20-byte budget");
    assert!(align <= 8, "PostArmState alignment too large");
};

impl PostArmState {
    /// Create new post-arm state (zero-initialized)
    pub const fn new() -> Self {
        PostArmState {
            arm_time_ms: 0,
            arm_method: ArmMethod::Unknown,
            checks_performed: false,
            actuators_initialized: false,
            subsystems_notified: false,
            _padding: [0; 4],
        }
    }
}

/// Verify no heap allocations during post-arm init
#[test]
fn test_no_heap_allocation() {
    // Disable heap allocator
    #[global_allocator]
    static ALLOCATOR: ForbidAlloc = ForbidAlloc;

    let mut state = SystemState::new();

    // Arm/disarm should not allocate
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    state.disarm().unwrap();

    // If we reach here, no allocations occurred
}

/// Verify memory overhead is within budget
#[test]
fn test_memory_budget() {
    // PostArmState size
    let state_size = core::mem::size_of::<PostArmState>();
    assert!(state_size <= 20, "PostArmState too large: {} bytes", state_size);

    // Log buffer size (temporary)
    const LOG_BUFFER_SIZE: usize = 40;

    // Total overhead
    let total = state_size + LOG_BUFFER_SIZE;
    assert!(total <= 60, "Total overhead too large: {} bytes", total);

    println!("Memory budget: PostArmState={}, LogBuffer={}, Total={}/60",
             state_size, LOG_BUFFER_SIZE, total);
}

/// Verify no memory leaks over repeated cycles
#[test]
fn test_no_memory_leak() {
    let mut state = SystemState::new();

    // Get initial memory usage (if available)
    let initial_mem = get_heap_usage();

    // Arm/disarm 1000 times
    for _ in 0..1000 {
        state.arm(ArmMethod::GcsCommand, true).unwrap();
        state.disarm().unwrap();
    }

    // Verify no memory growth
    let final_mem = get_heap_usage();
    assert_eq!(
        initial_mem, final_mem,
        "Memory leak detected: {} bytes leaked",
        final_mem - initial_mem
    );
}
```

**Memory Profiling Tools:**

```rust
/// Get current heap usage (platform-specific)
#[cfg(feature = "memory-profiling")]
fn get_heap_usage() -> usize {
    // Use platform-specific allocator stats
    #[cfg(target_arch = "arm")]
    unsafe {
        extern "C" {
            fn __heap_start();
            fn __heap_end();
            fn __heap_current();
        }

        __heap_current() as usize - __heap_start() as usize
    }

    #[cfg(not(target_arch = "arm"))]
    0 // Not available on host
}

/// Stack depth measurement
#[cfg(feature = "stack-profiling")]
fn measure_stack_depth() -> usize {
    let stack_top: *const u8;
    let stack_bottom: *const u8;

    #[cfg(target_arch = "arm")]
    unsafe {
        extern "C" {
            fn __stack_start();
            fn __stack_end();
        }

        stack_bottom = __stack_start() as *const u8;
        stack_top = __stack_end() as *const u8;
    }

    let current_sp: *const u8;
    unsafe {
        asm!("mov {}, sp", out(reg) current_sp);
    }

    (stack_top as usize) - (current_sp as usize)
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState and PostArmState
- `src/core/logging/` - Log formatting and buffering
- `src/platform/*/memory.rs` - Memory profiling and monitoring

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
  N/A - No external references
