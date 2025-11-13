# T-q3psc Battery Telemetry Implementation

## Metadata

- Type: Task
- Status: Completed

## Links

- Related Requirements:
  - [FR-015k2-adc-battery-voltage-reading](../../requirements/FR-015k2-adc-battery-voltage-reading.md)
  - [FR-uq6as-voltage-conversion-calculation](../../requirements/FR-uq6as-voltage-conversion-calculation.md)
  - [FR-zxxlp-battery-status-telemetry](../../requirements/FR-zxxlp-battery-status-telemetry.md)
  - [FR-ygjkj-battery-state-scheduler-update](../../requirements/FR-ygjkj-battery-state-scheduler-update.md)
  - [NFR-r1h41-adc-performance-constraint](../../requirements/NFR-r1h41-adc-performance-constraint.md)
  - [NFR-tximn-host-test-support](../../requirements/NFR-tximn-host-test-support.md)
- Related ADRs:
  - [ADR-97ebh-battery-telemetry-architecture](../../adr/ADR-97ebh-battery-telemetry-architecture.md)
- Associated Design Document:
  - [design.md](./design.md)
- Associated Plan Document:
  - [plan.md](./plan.md)

## Summary

Implement battery voltage monitoring for the pico_trail rover platform using the Freenove 4WD car kit's GPIO 26 voltage sensing hardware. Replace placeholder battery voltage values (12.0V) with real ADC readings converted via ArduPilot-standard `BATT_VOLT_MULT` parameter. Stream MAVLink BATTERY_STATUS (#147) messages at 2 Hz for ground control station compatibility, enabling safety-critical pre-arm checks and voltage-based failsafes.

## Implementation Phases

1. **Phase 1**: Foundation (Board trait extension, Battery parameter)
2. **Phase 2**: Core Implementation (ADC driver integration, Voltage conversion)
3. **Phase 3**: Telemetry & Scheduler (BATTERY_STATUS message, Scheduler integration)
4. **Phase 4**: Testing & Verification (Host tests, Hardware validation)

## Success Criteria

- ✅ Battery voltage accuracy within ±0.2V compared to multimeter reading
- ✅ ADC read completes within 2ms worst-case (NFR-r1h41) - 5-sample averaging \~10μs
- ✅ Ground control station displays battery status correctly (Mission Planner verification)
- ✅ Host unit tests pass without hardware (`cargo test --lib`) - 361 tests passing
- ✅ No timing disruption to 400 Hz control loop
- ✅ 2S/3S LiPo battery support with auto-detection

## Key Design Decisions

- **embassy-rp ADC**: Official HAL for RP2350/RP2040, async API integrates with embassy executor
- **BATT_VOLT_MULT parameter**: ArduPilot-standard parameter for voltage divider coefficient (default 3.95)
- **10 Hz battery update rate**: Matches ArduPilot medium-frequency monitoring, 100ms response time for failsafes
- **2 Hz BATTERY_STATUS streaming**: Full ArduPilot compatibility with ground control stations
- **Board trait abstraction**: MockBoard enables host unit tests without hardware access
- **5-sample ADC averaging**: Per Freenove reference implementation for noise reduction
- **Graceful degradation**: ADC read failures use last known value, log error without crashing

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- Follows ADR-97ebh: Full BATTERY_STATUS implementation selected for ArduPilot compatibility
- Freenove voltage divider coefficient \~3.95 (user-calibratable via BATT_VOLT_MULT)
- No current sensor or per-cell voltage monitoring (hardware limitations)
- SYS_STATUS message continues to include battery voltage (backward compatibility)
- Compatible with RP2350 (Pico 2 W)
- **Implemented Features:**
  - Embassy-rp ADC driver integration with GPIO 26
  - 5-sample averaging for noise reduction
  - 10 Hz battery state updates
  - 2 Hz BATTERY_STATUS streaming
  - Automatic 2S/3S LiPo detection based on voltage range
  - Battery remaining percentage calculation (linear interpolation)
  - MockPlatform for host unit testing
  - USB serial and probe-rs debug support
