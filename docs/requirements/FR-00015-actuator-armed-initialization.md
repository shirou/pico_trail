# FR-00015 Actuator Armed State Initialization

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00049-post-arm-event-recording](FR-00049-post-arm-event-recording.md)

- Dependent Requirements:
  - [FR-00017-arm-subsystem-notification](FR-00017-arm-subsystem-notification.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall initialize actuators to armed state after arming by setting throttle to armed idle PWM (typically 1100 µs for ESCs) and steering to center PWM (1500 µs), verifying actuator commands are accepted without errors, with initialization completing within 10 ms of the arm command.

## Rationale

Actuator armed state initialization establishes a known, safe starting point for vehicle control:

- **Deterministic state**: Control loops begin from predictable actuator positions
- **ESC arming**: Many ESCs require specific idle PWM before accepting throttle commands
- **Prevent jumps**: Ensures actuators don't suddenly jump from disarmed (neutral) to arbitrary positions
- **Safety baseline**: Steering centered and throttle at idle (zero thrust) is safest initial state
- **Control stability**: Controllers assume actuators start from known state when armed

ArduPilot initializes actuators during post-arm sequence, transitioning from disarmed PWM (typically 1500 µs for all channels, or no output) to armed idle (1100 µs throttle, 1500 µs steering).

## User Story (if applicable)

As a control system, I want actuators initialized to a known armed state when the vehicle arms, so that my control loops start from a predictable baseline and don't experience sudden position jumps.

## Acceptance Criteria

- [ ] Set throttle to armed idle PWM (1100 µs for ESCs, configurable via MOT_SAFE_DISARM behavior)
- [ ] Set steering to center PWM (1500 µs for servos, neutral position)
- [ ] Verify actuator commands accepted (no errors returned from actuator interface)
- [ ] Initialization completes within 10 ms of arm command (measured from arm() call to ready state)
- [ ] Initialization occurs after timestamp recording and logging, before subsystem notification
- [ ] If actuator initialization fails, disarm vehicle and report error to operator (rollback)
- [ ] Test coverage: verify PWM values set correctly, verify initialization timing, verify rollback on failure

## Technical Details (if applicable)

### Functional Requirement Details

**Actuator States:**

| State       | Throttle PWM | Steering PWM | Behavior                      |
| ----------- | ------------ | ------------ | ----------------------------- |
| Disarmed    | 1500 µs      | 1500 µs      | Neutral (no thrust, centered) |
| Armed Idle  | 1100 µs      | 1500 µs      | ESC armed, zero thrust        |
| Armed Ready | 1100+ µs     | 1000-2000 µs | Ready for control commands    |

**Implementation:**

```rust
impl SystemState {
    /// Post-arm initialization sequence
    fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                     -> Result<(), &'static str> {
        // 1. Record timestamp
        let arm_time_ms = get_time_ms();
        self.post_arm_state = PostArmState { arm_time_ms, /* ... */ };

        // 2. Log arming event
        self.log_arm_event(arm_time_ms, method, checks_performed)?;

        // 3. Initialize actuators (after timestamp and logging)
        self.initialize_actuators()?;
        self.post_arm_state.actuators_initialized = true;

        // 4. Continue with other post-arm steps...
        Ok(())
    }

    /// Initialize actuators to armed state
    fn initialize_actuators(&mut self) -> Result<(), &'static str> {
        // Set throttle to armed idle (1100 µs PWM)
        // For ESCs, this is typically the "armed but zero thrust" position
        self.actuators.set_throttle_pwm(1100)?;

        // Set steering to center (1500 µs PWM)
        // Neutral position for servos
        self.actuators.set_steering_pwm(1500)?;

        // Verify commands accepted
        // Actuator interface should return error if hardware not responding
        if !self.actuators.is_ready() {
            return Err("Actuators not ready for armed state");
        }

        info!("Actuators initialized to armed state (throttle=1100µs, steering=1500µs)");
        Ok(())
    }
}
```

**Actuator Interface (Assumed):**

```rust
pub trait Actuators {
    /// Set throttle PWM in microseconds (1000-2000 range typical)
    fn set_throttle_pwm(&mut self, pwm_us: u16) -> Result<(), &'static str>;

    /// Set steering PWM in microseconds (1000-2000 range typical)
    fn set_steering_pwm(&mut self, pwm_us: u16) -> Result<(), &'static str>;

    /// Check if actuators ready to accept commands
    fn is_ready(&self) -> bool;
}
```

**Timing:**

- Target: < 10 ms for initialization
- Typical: 2-5 ms (PWM command transmission over I2C/PWM interface)
- Worst case: 10 ms (includes retry on transient error)

If initialization exceeds 10 ms, log warning but continue (don't fail arm operation unless actuator error).

**Error Handling:**

If actuator initialization fails:

1. Attempt retry once (transient errors common with I2C, timing-sensitive interfaces)
2. If retry fails, disarm vehicle: `self.disarm()?`
3. Report error to operator: Send STATUSTEXT with severity ERROR
4. Log error: `error!("Actuator initialization failed: {}", reason)`
5. Return error from post_arm_init() to abort arm sequence

Rationale: Partial initialization worse than no initialization. If actuators fail, vehicle not safe to operate.

**MOT_SAFE_DISARM Interaction:**

ArduPilot parameter `MOT_SAFE_DISARM` controls disarmed actuator behavior:

- **MOT_SAFE_DISARM = 0** (default): Output trim values (1500 µs all channels)
- **MOT_SAFE_DISARM = 1**: No PWM output when disarmed

When arming with MOT_SAFE_DISARM = 0, actuators transition from 1500 µs to armed state (1100 µs throttle, 1500 µs steering). When MOT_SAFE_DISARM = 1, actuators transition from no output to armed state.

For pico_trail Phase 1, assume MOT_SAFE_DISARM = 0 behavior (output trim when disarmed).

**PWM Value Rationale:**

- **1100 µs throttle**: Standard ESC armed idle, zero thrust
- **1500 µs steering**: Standard servo center position, neutral
- **Configurable**: Future enhancement may support custom PWM values via parameters

## Platform Considerations

### Pico W (RP2040)

Actuator output via PWM peripheral:

- PWM frequency: 50 Hz typical for servos/ESCs
- Command latency: < 1 ms (local GPIO control)
- Interface: Direct PWM via `embassy-rp` HAL

### Pico 2 W (RP2350)

Same as RP2040:

- PWM peripheral compatible
- No platform-specific differences for actuator control

### Cross-Platform

Actuator initialization logic platform-agnostic. PWM values standard across all platforms supporting RC servos/ESCs.

## Risks & Mitigation

| Risk                                                              | Impact   | Likelihood | Mitigation                                                                                    | Validation                                                          |
| ----------------------------------------------------------------- | -------- | ---------- | --------------------------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| **Actuator initialization fails, vehicle armed but not drivable** | CRITICAL | Low        | **Disarm on failure, report error to operator, retry once before failing**                    | **Test: simulate actuator error, verify vehicle disarms**           |
| ESC not ready for throttle commands (still calibrating)           | High     | Low        | Delay actuator init until ESC ready signal, or add 100ms wait before sending throttle         | Test with actual ESC: verify accepts 1100µs immediately after arm   |
| PWM command transmission timeout (I2C, CAN)                       | High     | Low        | Set 10ms timeout for actuator commands, retry once, fail if still timeout                     | Test: disconnect actuator, verify timeout and retry logic           |
| Actuator jumps to armed state too fast (mechanical shock)         | Medium   | Low        | Consider rate-limiting transition (gradual ramp from 1500µs to 1100µs), but adds complexity   | Test with hardware: verify no mechanical shock on arm               |
| Throttle armed idle value wrong for specific ESC                  | Medium   | Low        | Document 1100µs as standard, allow future configuration via parameter (MOT_THR_MIN analogue)  | Test with multiple ESC brands: verify 1100µs accepted               |
| Steering center value wrong for specific servo                    | Medium   | Low        | Document 1500µs as standard, allow future trim adjustment via parameter (RCx_TRIM analogue)   | Test with multiple servos: verify 1500µs is neutral                 |
| Initialization latency exceeds 10ms                               | Low      | Very Low   | Profile on target hardware, optimize if needed, acceptable up to 20ms (operator won't notice) | Benchmark: measure init time, verify < 10ms typical                 |
| Actuator commands sent while still disarmed (race condition)      | Medium   | Very Low   | Actuator interface enforces armed check (defense in depth), arm() is synchronous so no race   | Unit test: verify actuator interface rejects commands when disarmed |

## Implementation Notes

Preferred approaches:

- **Simple transition**: Direct write to PWM registers, no gradual ramp (Phase 1)
- **Retry logic**: Single retry on transient errors (I2C glitch, timing)
- **Fail-safe rollback**: Disarm if initialization fails (don't leave vehicle in inconsistent state)
- **Standard PWM values**: Use industry-standard 1100 µs throttle, 1500 µs steering

Known pitfalls:

- **ESC calibration**: Some ESCs require calibration sequence (high then low PWM), assume pre-calibrated
- **Servo trim**: 1500 µs may not be exact center for all servos, future trim parameters needed
- **I2C timing**: PWM-over-I2C (e.g., PCA9685) slower than direct PWM, may need higher timeout
- **Partial init**: If throttle succeeds but steering fails, disarm (don't leave partial state)

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState post_arm_init() calls initialize_actuators()
- `src/vehicle/actuators.rs` - Actuator interface (PWM command API)
- `src/platform/rp2040/pwm.rs` - Platform-specific PWM implementation
- `src/vehicle/modes/manual.rs` - Manual mode uses actuators after initialization

Suggested libraries:

- **embassy-rp**: RP2040/RP2350 PWM HAL for direct GPIO PWM
- **pwm-pca9685**: If using external PWM driver chip

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- ArduPilot MOT_SAFE_DISARM: <https://ardupilot.org/copter/docs/parameters.html#mot-safe-disarm>
- RC Servo PWM Standard: 1000-2000 µs range, 1500 µs center (industry standard)
- ESC Calibration: <https://ardupilot.org/rover/docs/common-esc-calibration.html>
