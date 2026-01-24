# AN-wf43c CYW43 WiFi Stability Analysis

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses: None
- Related Requirements: None
- Related ADRs: None
- Related Tasks: None

## Executive Summary

Investigation into Pico 2 W (RP2350 + CYW43439) WiFi stability issues revealed two critical findings:

1. **Control Memory Location**: The CYW43 driver's `Control` struct must NOT be moved after creation. Moving it to static memory (StaticCell) or passing it to another task causes ping/ICMP to stop working, even though WiFi appears connected.

2. **Join Timeout Pattern**: The first `control.join()` attempt always times out due to CYW43439 internal state. Calling `control.leave()` after timeout resets the state and allows subsequent joins to succeed.

The solution restructures `initialize_wifi()` to never return, keeping Control as a local variable on the task's stack, and provides a separate `wait_wifi_ready()` API for callers to obtain the network stack.

## Problem Space

### Current State

WiFi initialization on Pico 2 W exhibited two issues:

1. **Ping Not Responding**: After WiFi connection, ping requests were not answered even though:
   - WiFi appeared connected
   - DHCP obtained valid IP address
   - Link state showed UP
   - Keep-alive loop ran normally

2. **Join Timeout**: First `control.join()` call consistently timed out (30+ seconds), requiring multiple retry attempts.

### Desired State

- WiFi connects reliably on first or second attempt
- Ping responds correctly after connection
- Network stack functions normally for UDP/TCP communication

### Gap Analysis

| Issue               | Root Cause                        | Solution                       |
| ------------------- | --------------------------------- | ------------------------------ |
| Ping not responding | Control moved to StaticCell       | Keep Control as local variable |
| Join timeout        | CYW43439 internal state not reset | Call leave() after timeout     |
| PowerSave issues    | PowerSave mode causes instability | Use PowerManagementMode::None  |

## Research & Discovery

### Technical Investigation

#### Isolation Testing

Created `wifi_test.rs` standalone binary to isolate WiFi functionality:

| Configuration                       | Ping Response |
| ----------------------------------- | ------------- |
| wifi_test (standalone)              | OK            |
| pico_trail_rover + direct WiFi init | OK            |
| pico_trail_rover + network.rs       | NG            |

#### Control Movement Testing

Systematic testing of Control placement:

| Control Location                        | Ping Response |
| --------------------------------------- | ------------- |
| Local variable (never moved)            | OK            |
| StaticCell (at function end)            | NG            |
| StaticCell (immediately after creation) | NG            |
| Moved to separate task                  | NG            |
| Kept in initialize_wifi() loop          | OK            |

**Conclusion**: Control must remain in its original memory location (on the stack of the function that created it).

#### Join Timeout Investigation

```
[Step 8/8] Joining WiFi network...
  Attempt 1: joining 'SSID'...
  Attempt 1: TIMEOUT after 30 seconds!
  Calling leave()...
  Attempt 2: joining 'SSID'...
  Attempt 2: SUCCESS!
```

Calling `control.leave()` after timeout resets internal driver state.

### Data Analysis

#### Working vs Non-Working Code Comparison

**Working (Control stays local)**:

```rust
let (net_device, mut control, runner) = cyw43::new(...).await;
// ... use control for init, join, etc ...
loop {
    // Control stays in this function forever
    Timer::after(Duration::from_secs(3600)).await;
}
```

**Not Working (Control moved)**:

```rust
let (net_device, mut control, runner) = cyw43::new(...).await;
// ... use control for init, join, etc ...
static CONTROL: StaticCell<Control<'static>> = StaticCell::new();
let control_ref = CONTROL.init(control);  // MOVED - breaks ping!
Ok((stack, control_ref))
```

## Design Considerations

### Technical Constraints

1. **CYW43 Driver Limitation**: Control cannot be moved after creation. This is likely due to internal pointers or references that assume Control's memory address is stable.

2. **Embassy Task Model**: Embassy tasks store their state in static memory. Passing Control to a task effectively moves it.

3. **API Design**: The original `initialize_wifi()` returned `(Stack, Control)`, but Control cannot be safely returned.

### Potential Approaches

1. **Option A**: Never-returning function with Signal (Implemented)
   - Pros: Simple, guaranteed to work, no unsafe code
   - Cons: Function never returns, requires separate wait API
   - Effort: Low

2. **Option B**: Pin\<Box<Control>> to prevent moves
   - Pros: Could return Control reference
   - Cons: Complex lifetime management, may still have issues
   - Effort: High

3. **Option C**: Upstream driver fix
   - Pros: Proper solution at source
   - Cons: Not under our control, uncertain timeline
   - Effort: N/A (external)

### Architecture Impact

Modified `network.rs` API:

**Before**:

```rust
pub async fn initialize_wifi(...) -> Result<(&'static Stack, &'static mut Control), WifiError>
```

**After**:

```rust
pub async fn initialize_wifi(...) -> !  // Never returns
pub async fn wait_wifi_ready() -> &'static Stack  // Separate API to get Stack
```

New task wrapper for spawning:

```rust
#[embassy_executor::task]
pub async fn wifi_init_task(...) -> !
```

## Risk Assessment

| Risk                     | Probability | Impact | Mitigation                              |
| ------------------------ | ----------- | ------ | --------------------------------------- |
| Control API needed later | Medium      | Medium | Can add channel-based command interface |
| Upstream driver changes  | Low         | Low    | Our solution is compatible with fixes   |
| Memory usage increase    | Low         | Low    | Minimal - one task stack                |

## Open Questions

- [x] Why does moving Control break ping? - Likely internal pointer/reference issue in CYW43 driver
- [x] Is this RP2350-specific? - Unknown, but behavior is consistent on RP2350
- [ ] Should this be reported upstream? - Yes, to embassy-rs/cyw43

## Recommendations

### Immediate Actions

1. ✅ Implement never-returning `initialize_wifi()` with `wait_wifi_ready()` API
2. ✅ Add `wifi_init_task()` for easy spawning
3. ✅ Document the Control movement restriction

### Next Steps

1. [ ] Report issue to embassy-rs/cyw43 repository
2. [ ] Monitor upstream for potential fixes
3. [ ] Consider adding Control command interface via channels if needed

### Out of Scope

- Fixing the CYW43 driver itself (upstream responsibility)
- Supporting Control access after initialization (not needed currently)

## Appendix

### Solution Implementation

**File**: `crates/firmware/src/platform/rp2350/network.rs`

```rust
/// Signal for WiFi ready notification
static WIFI_STACK_PTR: AtomicPtr<Stack<'static>> = AtomicPtr::new(core::ptr::null_mut());
static WIFI_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Wait for WiFi to be ready
pub async fn wait_wifi_ready() -> &'static Stack<'static> {
    WIFI_READY.wait().await;
    unsafe { &*WIFI_STACK_PTR.load(Ordering::Acquire) }
}

/// Initialize WiFi - never returns
pub async fn initialize_wifi(...) -> ! {
    // ... initialization code ...

    // Store stack pointer and signal ready
    WIFI_STACK_PTR.store(stack as *const _ as *mut _, Ordering::Release);
    WIFI_READY.signal(());

    // CRITICAL: Keep Control alive as local variable forever
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

/// Spawnable task wrapper
#[embassy_executor::task]
pub async fn wifi_init_task(...) -> ! {
    initialize_wifi(...).await
}
```

**Usage**:

```rust
// Spawn WiFi init task
spawner.spawn(wifi_init_task(spawner, config, pins...).unwrap());

// Wait for WiFi and get stack
let stack = wait_wifi_ready().await;
```

### Key Configuration

```rust
// Required for stable ping response
control.set_power_management(cyw43::PowerManagementMode::None).await;

// Join with timeout and leave() on failure
let join_result = embassy_time::with_timeout(
    Duration::from_secs(5),
    control.join(ssid, options),
).await;

match join_result {
    Ok(Ok(_)) => break,  // Success
    _ => {
        control.leave().await;  // Reset driver state
        // Retry...
    }
}
```

### References

- [pico-sdk #915](https://github.com/raspberrypi/pico-sdk/issues/915) - Similar TX/RX issues
- [pico-sdk PR #2209](https://github.com/raspberrypi/pico-sdk/pull/2209) - PIO SPI race condition fix
- [micropython #9316](https://github.com/micropython/micropython/issues/9316) - Zombie PIO state machines

### Files Modified

- `crates/firmware/src/platform/rp2350/network.rs` - Major API restructure
- `crates/firmware/examples/pico_trail_rover.rs` - Updated to use new API
- `crates/firmware/examples/wifi_test.rs` - Created for isolation testing
