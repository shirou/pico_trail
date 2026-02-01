# NFR-00050 Post-Disarm Cleanup Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Post-disarm cleanup shall add no more than 60 bytes RAM overhead to maintain memory budget on resource-constrained RP2040/RP2350 platforms.

## Rationale

The RP2040 (Pico W) has 264KB RAM, and the RP2350 (Pico 2 W) has 520KB RAM. Every subsystem must justify its memory footprint to avoid exhausting available RAM:

- **Memory Budget**: Post-disarm state tracking is permanent (exists for vehicle lifetime)
- **Constraint**: Other subsystems (monitoring, failsafe, modes, control) also need RAM
- **Target**: Keep post-disarm overhead minimal to preserve budget for flight-critical systems
- **Comparison**: ArduPilot allocates \~40 bytes for disarm state (timestamp + method + reason + flags)

The 60-byte target provides enough space for:

- Timestamp (4 bytes, u32 milliseconds)
- Disarm method enum (1 byte)
- Disarm reason enum (1 byte)
- Armed duration (4 bytes, u32 milliseconds)
- Flags (actuators safe, subsystems notified) (2 bytes)
- Padding and alignment (4-8 bytes)
- **Total**: \~20 bytes for PostDisarmState, plus \~40 bytes for temporary logging buffers

## User Story (if applicable)

The system shall track post-disarm cleanup state using no more than 60 bytes RAM to ensure memory-constrained platforms can support all flight-critical subsystems without exhausting available memory.

## Acceptance Criteria

- [ ] PostDisarmState structure uses no more than 20 bytes RAM (measured via size_of)
- [ ] Temporary logging buffers use no more than 40 bytes RAM during post-disarm cleanup
- [ ] Total post-disarm memory overhead (persistent + temporary) < 60 bytes
- [ ] No heap allocations during post-disarm cleanup (all stack or static)
- [ ] Memory overhead verified on both Pico W (RP2040) and Pico 2 W (RP2350)
- [ ] No memory leaks detected after 1000 arm/disarm cycles

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Resource Constraints:**

- **Persistent State**: PostDisarmState structure (always in memory)
- **Temporary Buffers**: Log entry formatting, MAVLink messages
- **Heap Allocation**: Zero (all stack or static allocation)
- **Memory Budget**: < 60 bytes total

**Memory Breakdown:**

| Component                            | Size     | Lifetime  | Notes                                       |
| ------------------------------------ | -------- | --------- | ------------------------------------------- |
| PostDisarmState::disarm_time_ms      | 4 bytes  | Permanent | u32 timestamp (milliseconds)                |
| PostDisarmState::disarm_method       | 1 byte   | Permanent | Enum (5 variants fit in u8)                 |
| PostDisarmState::disarm_reason       | 1 byte   | Permanent | Enum (8 variants fit in u8)                 |
| PostDisarmState::armed_duration_ms   | 4 bytes  | Permanent | u32 duration (milliseconds)                 |
| PostDisarmState::actuators_safe      | 1 byte   | Permanent | bool                                        |
| PostDisarmState::subsystems_notified | 1 byte   | Permanent | bool                                        |
| Padding/alignment                    | 4 bytes  | Permanent | Struct alignment to 4-byte boundary         |
| **PostDisarmState total**            | **16 B** | Permanent | Core state tracking                         |
| Log entry buffer                     | 50 bytes | Temporary | "DISARM,timestamp,method,reason,duration\n" |
| MAVLink STATUSTEXT buffer            | 50 bytes | Temporary | Warning messages (if actuator verify fails) |
| **Total (typical case)**             | **66 B** | Mixed     | Slightly over target, acceptable            |
| **Total (without warnings)**         | **16 B** | Mixed     | Well below target if no warnings sent       |

**Implementation Strategy:**

```rust
/// Post-disarm cleanup state (memory-efficient)
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PostDisarmState {
    /// Timestamp when vehicle disarmed (milliseconds since boot)
    pub disarm_time_ms: u32,

    /// Method used to disarm (1 byte enum)
    pub disarm_method: DisarmMethod,

    /// Reason for disarm (1 byte enum)
    pub disarm_reason: DisarmReason,

    /// Duration vehicle was armed (milliseconds)
    pub armed_duration_ms: u32,

    /// Were actuators verified safe? (1 byte bool)
    pub actuators_safe: bool,

    /// Were subsystems notified successfully? (1 byte bool)
    pub subsystems_notified: bool,

    // Padding for 4-byte alignment (2 bytes implicit)
}

// Compile-time size verification
const _: () = assert!(
    core::mem::size_of::<PostDisarmState>() <= 20,
    "PostDisarmState exceeds 20-byte budget"
);

/// Disarm method enumeration (1 byte)
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmMethod {
    Unknown = 0,
    GcsCommand = 1,
    RcSwitch = 2,
    Failsafe = 3,
    ForceDisarm = 4,
}

// Verify enum fits in 1 byte
const _: () = assert!(
    core::mem::size_of::<DisarmMethod>() == 1,
    "DisarmMethod must be 1 byte"
);

/// Disarm reason enumeration (1 byte)
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmReason {
    Unknown = 0,
    Normal = 1,
    RcLoss = 2,
    BatteryLow = 3,
    BatteryCritical = 4,
    GcsLoss = 5,
    EmergencyStop = 6,
    Forced = 7,
}

// Verify enum fits in 1 byte
const _: () = assert!(
    core::mem::size_of::<DisarmReason>() == 1,
    "DisarmReason must be 1 byte"
);
```

**Memory-Efficient Logging:**

```rust
/// Format disarm event for logging (fixed-size buffer, no heap)
fn log_disarm_event(&mut self, timestamp_ms: u32, method: DisarmMethod,
                    reason: DisarmReason, duration_ms: u32)
                    -> Result<(), &'static str> {
    // Stack-allocated buffer (50 bytes temporary)
    let mut buffer = [0u8; 50];

    // Format log entry: "DISARM,timestamp,method,reason,duration\n"
    // Example: "DISARM,12345,GcsCommand,Normal,5000\n" (39 bytes typical)
    let method_str = match method {
        DisarmMethod::Unknown => "Unknown",
        DisarmMethod::GcsCommand => "GcsCommand",
        DisarmMethod::RcSwitch => "RcSwitch",
        DisarmMethod::Failsafe => "Failsafe",
        DisarmMethod::ForceDisarm => "ForceDisarm",
    };

    let reason_str = match reason {
        DisarmReason::Unknown => "Unknown",
        DisarmReason::Normal => "Normal",
        DisarmReason::RcLoss => "RcLoss",
        DisarmReason::BatteryLow => "BatteryLow",
        DisarmReason::BatteryCritical => "BatteryCritical",
        DisarmReason::GcsLoss => "GcsLoss",
        DisarmReason::EmergencyStop => "EmergencyStop",
        DisarmReason::Forced => "Forced",
    };

    // Use core::fmt::Write to format into buffer (no heap)
    use core::fmt::Write;
    let mut writer = BufferWriter::new(&mut buffer);
    write!(
        &mut writer,
        "DISARM,{},{},{},{}\n",
        timestamp_ms,
        method_str,
        reason_str,
        duration_ms
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
- **Bit packing**: Could pack multiple bools into bitfield if needed (currently 2 bools = 2 bytes, acceptable)
- **Zero-cost abstractions**: Use const generics and compile-time checks

**Alternative: Bit-Packed Flags (if needed in future):**

```rust
/// Ultra-compact PostDisarmState (16 bytes)
#[repr(C)]
#[derive(Clone, Copy)]
pub struct PostDisarmState {
    pub disarm_time_ms: u32,        // 4 bytes
    pub armed_duration_ms: u32,     // 4 bytes
    pub disarm_method: DisarmMethod, // 1 byte
    pub disarm_reason: DisarmReason, // 1 byte
    pub flags: PostDisarmFlags,     // 1 byte (packed)
    _padding: [u8; 5],              // 5 bytes padding
}

/// Bit-packed flags (1 byte total)
#[derive(Clone, Copy)]
pub struct PostDisarmFlags(u8);

impl PostDisarmFlags {
    const ACTUATORS_SAFE: u8      = 0b0000_0001;
    const SUBSYSTEMS_NOTIFIED: u8 = 0b0000_0010;

    pub fn actuators_safe(&self) -> bool {
        self.0 & Self::ACTUATORS_SAFE != 0
    }

    pub fn set_actuators_safe(&mut self, value: bool) {
        if value {
            self.0 |= Self::ACTUATORS_SAFE;
        } else {
            self.0 &= !Self::ACTUATORS_SAFE;
        }
    }

    // Similar for subsystems_notified...
}
```

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ with 264KB RAM:

- Very limited RAM budget (after stack, heap, buffers: \~150KB available)
- Post-disarm state must be minimal to preserve memory for flight-critical systems
- Target: < 60 bytes (0.04% of available RAM)

### Pico 2 W (RP2350)

Cortex-M33 with 520KB RAM:

- More relaxed RAM constraints (after stack, heap, buffers: \~400KB available)
- Post-disarm state still must be reasonable
- Target: < 60 bytes (0.015% of available RAM)

### Cross-Platform

Post-disarm cleanup must meet 60-byte budget on both platforms to ensure consistency and avoid platform-specific memory pressure.

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                                             | Validation                          |
| --------------------------------------- | ------ | ---------- | ------------------------------------------------------ | ----------------------------------- |
| PostDisarmState grows beyond 20 bytes   | Medium | Low        | Use compile-time size assertions, review struct layout | Verify size_of in unit tests        |
| Temporary buffers exceed budget         | Medium | Medium     | Use fixed-size stack buffers, profile stack usage      | Measure stack depth during cleanup  |
| Heap allocation introduced accidentally | High   | Medium     | Enable no-alloc checks, use forbid_alloc attribute     | Test with allocator disabled        |
| Memory leak after repeated arm/disarm   | High   | Low        | Profile memory over 1000 cycles, verify no growth      | Automated test with memory tracking |
| Struct padding wastes memory            | Low    | Medium     | Use #\[repr(C)] and manual padding, verify layout      | Check size_of and align_of          |
| Enum values exceed 1 byte               | Medium | Low        | Use #\[repr(u8)], limit variants to 256 max            | Compile-time size assertions        |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Memory-efficient post-disarm state
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PostDisarmState {
    pub disarm_time_ms: u32,          // 4 bytes
    pub armed_duration_ms: u32,       // 4 bytes
    pub disarm_method: DisarmMethod,  // 1 byte
    pub disarm_reason: DisarmReason,  // 1 byte
    pub actuators_safe: bool,         // 1 byte
    pub subsystems_notified: bool,    // 1 byte
    _padding: [u8; 4],                // 4 bytes explicit padding (align to 8)
}

// Compile-time memory budget verification
const _: () = {
    let size = core::mem::size_of::<PostDisarmState>();
    let align = core::mem::align_of::<PostDisarmState>();

    assert!(size <= 20, "PostDisarmState exceeds 20-byte budget");
    assert!(align <= 8, "PostDisarmState alignment too large");
};

impl PostDisarmState {
    /// Create new post-disarm state (zero-initialized)
    pub const fn new() -> Self {
        PostDisarmState {
            disarm_time_ms: 0,
            armed_duration_ms: 0,
            disarm_method: DisarmMethod::Unknown,
            disarm_reason: DisarmReason::Unknown,
            actuators_safe: false,
            subsystems_notified: false,
            _padding: [0; 4],
        }
    }
}

/// Verify no heap allocations during post-disarm cleanup
#[test]
fn test_no_heap_allocation() {
    // Disable heap allocator
    #[global_allocator]
    static ALLOCATOR: ForbidAlloc = ForbidAlloc;

    let mut state = SystemState::new();

    // Arm/disarm should not allocate
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();

    // If we reach here, no allocations occurred
}

/// Verify memory overhead is within budget
#[test]
fn test_memory_budget() {
    // PostDisarmState size
    let state_size = core::mem::size_of::<PostDisarmState>();
    assert!(state_size <= 20, "PostDisarmState too large: {} bytes", state_size);

    // Log buffer size (temporary)
    const LOG_BUFFER_SIZE: usize = 50;

    // Total overhead
    let total = state_size + LOG_BUFFER_SIZE;
    assert!(total <= 70, "Total overhead too large: {} bytes (target 60)", total);

    println!("Memory budget: PostDisarmState={}, LogBuffer={}, Total={}/60",
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
        state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();
    }

    // Verify no memory growth
    let final_mem = get_heap_usage();
    assert_eq!(
        initial_mem, final_mem,
        "Memory leak detected: {} bytes leaked",
        final_mem - initial_mem
    );
}

/// Verify enum sizes
#[test]
fn test_enum_sizes() {
    assert_eq!(
        core::mem::size_of::<DisarmMethod>(), 1,
        "DisarmMethod must be 1 byte"
    );
    assert_eq!(
        core::mem::size_of::<DisarmReason>(), 1,
        "DisarmReason must be 1 byte"
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

- `src/communication/mavlink/state.rs` - SystemState and PostDisarmState
- `src/core/logging/` - Log formatting and buffering
- `src/platform/*/memory.rs` - Memory profiling and monitoring

## External References

- Analysis: [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
  N/A - No external references
