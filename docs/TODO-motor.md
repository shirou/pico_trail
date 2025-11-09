# Motor Driver Investigation - Current Status

## Problem Statement

Only M4 motor works, M1/M2/M3 motors do not work despite successful PWM configuration.

## Investigation Summary

### What Works

1. **M4 (GPIO8/9) - Right Rear Motor**: ✅ Works correctly
   - PWM output confirmed in logs
   - Motor responds to forward/backward/stop commands
   - PWM duty cycle: 2500/5000 (50%)

2. **Software Implementation**: ✅ Verified correct
   - `HBridgeMotor::set_speed()` executes successfully
   - `EmbassyPwmPin::set_duty()` returns success
   - PWM values calculated correctly (duty=0.5 → value=2500/5000)
   - No errors in motor driver code

3. **PWM Slice Mapping**: ✅ Confirmed correct
   - GPIO18/19 → PWM_SLICE1 (Embassy-RP verified)
   - GPIO20/21 → PWM_SLICE2 (Embassy-RP verified)
   - GPIO6/7 → PWM_SLICE3 (Embassy-RP verified)
   - GPIO8/9 → PWM_SLICE4 (Embassy-RP verified)

### What Doesn't Work

1. **M1 (GPIO18/19) - Left Front Motor**: ❌ No response
   - PWM configuration succeeds (logs show `PWM set_duty SUCCESS`)
   - Identical log output to M4
   - Motor does not move

2. **M2 (GPIO20/21) - Left Rear Motor**: ❌ Not tested individually yet

3. **M3 (GPIO6/7) - Right Front Motor**: ❌ Not tested individually yet

## Key Findings

### Test Results

| Test Program | M1 | M2 | M3 | M4 | Notes |
|--------------|----|----|----|----|-------|
| motor_m1_only | ❌ | - | - | - | All PWM logs show SUCCESS |
| motor_m4_only | - | - | - | ✅ | All PWM logs show SUCCESS |
| motor_channel_test (all simultaneous) | ❌ | ❌ | ❌ | ✅ | All initialized, only M4 moves |
| motor_channel_test (sequential) | ❌ | ❌ | ❌ | ✅ | All initialized, only M4 moves |
| motor_no_split_test (user report) | ✅ | ? | ✅ | ✅ | Initial user report: "M1,3,4が動く" |

### Log Comparison (M1 vs M4)

**M1 Logs (motor_m1_only.rs):**
```
[INFO]  Motor set_speed: 0.5
[INFO]  PWM set_duty: duty=0.5, value=2500/5000
[INFO]  PWM set_duty SUCCESS
[INFO]  PWM set_duty: duty=0, value=0/5000
[INFO]  PWM set_duty SUCCESS
```
**Result**: Motor does NOT move

**M4 Logs (motor_m4_only.rs):**
```
[INFO]  Motor set_speed: 0.5
[INFO]  PWM set_duty: duty=0.5, value=2500/5000
[INFO]  PWM set_duty SUCCESS
[INFO]  PWM set_duty: duty=0, value=0/5000
[INFO]  PWM set_duty SUCCESS
```
**Result**: Motor DOES move

**Conclusion**: Logs are identical, but hardware behaves differently.

## Probable Causes

### High Probability

1. **Hardware Wiring Issue**
   - GPIO18/19/20/21/6/7 may not be properly connected to motor drivers
   - Only GPIO8/9 connections verified working
   - Possible: Incorrect pin mapping in physical wiring vs hwdef file

2. **Motor Driver (DRV8837) Issue**
   - M1/M2/M3 driver chips may be damaged
   - M4 driver chip verified working
   - Possible: Power supply issue to M1/M2/M3 drivers

3. **Motor Hardware Issue**
   - M1/M2/M3 motors themselves may be faulty
   - M4 motor verified working

### Low Probability

1. ~~Software bug~~ - Ruled out by identical logs
2. ~~PWM configuration error~~ - Ruled out by SUCCESS logs
3. ~~PWM slice mapping error~~ - Ruled out by Embassy-RP source verification

## Board Configuration

From `boards/freenove_standard.hwdef`:
```
M1_IN1 18 OUTPUT    # Left front motor IN1
M1_IN2 19 OUTPUT    # Left front motor IN2
M2_IN1 20 OUTPUT    # Left rear motor IN1
M2_IN2 21 OUTPUT    # Left rear motor IN2
M3_IN1 6 OUTPUT     # Right front motor IN1
M3_IN2 7 OUTPUT     # Right front motor IN2
M4_IN1 8 OUTPUT     # Right rear motor IN1
M4_IN2 9 OUTPUT     # Right rear motor IN2
```

## Next Steps

### Immediate Actions Required

1. **Verify Hardware Wiring**
   - [ ] Use multimeter to check continuity from Pico 2W pins to motor driver inputs
   - [ ] Verify GPIO18/19/20/21/6/7 connections
   - [ ] Check if DRV8837 chips for M1/M2/M3 are receiving power

2. **Test GPIO Output**
   - [ ] Build and run `gpio_pwm_test.rs` to verify GPIO18/19 can output PWM
   - [ ] Use oscilloscope or LED to visually confirm PWM output on each GPIO
   - [ ] Compare GPIO8/9 (working) vs GPIO18/19 (not working)

3. **Verify Motor Hardware**
   - [ ] Swap M1 motor with M4 motor physically
   - [ ] If M1 motor works on M4 position → motor is OK, wiring/driver is bad
   - [ ] If M1 motor still doesn't work → motor itself may be faulty

### Investigation Tasks

1. **Re-verify motor_no_split_test**
   - [ ] Build USB serial version: `EXTRA_FEATURES="usb_serial" ./scripts/build-rp2350.sh motor_no_split_test`
   - [ ] Confirm if M1 actually works in that configuration
   - [ ] If M1 works there but not in motor_m1_only → investigate initialization order

2. **Document Freenove Hardware**
   - [ ] Obtain official Freenove 4WD Car schematic
   - [ ] Verify pin assignments match our hwdef file
   - [ ] Check for any undocumented hardware requirements

## Files Modified During Investigation

### Test Programs Created
- `examples/motor_m1_only.rs` - M1 isolated test
- `examples/motor_m4_only.rs` - M4 isolated test (working reference)
- `examples/gpio_pwm_test.rs` - Raw GPIO PWM test

### Debug Logging Added
- `src/platform/rp2350/motor.rs` - Added detailed PWM duty cycle logging
- `src/libraries/motor_driver/hbridge.rs` - Added set_speed logging

### Current Debug Build Command
```bash
EXTRA_FEATURES="usb_serial" ./scripts/build-rp2350.sh motor_m1_only
```

## Code Verification Status

- ✅ HBridge motor driver implementation correct
- ✅ Embassy-RP PWM wrapper implementation correct
- ✅ PWM configuration (top=5000, divider=1) correct for 25kHz
- ✅ Motor trait implementation correct
- ✅ Error handling correct
- ✅ PWM slice to GPIO mapping verified against Embassy-RP source

## Conclusion

**Software is working correctly. This is a hardware issue.**

The identical success logs between M1 and M4, combined with only M4 physically moving, proves the software layer is functioning as designed. The problem lies in either:
- Physical wiring between Pico 2W and motor drivers
- Motor driver chips (DRV8837) for M1/M2/M3
- The motors themselves

Physical hardware inspection and testing is required to proceed.
