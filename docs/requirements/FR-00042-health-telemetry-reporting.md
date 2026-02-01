# FR-00042 Periodic Health Telemetry to GCS

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00057-system-health-status-tracking](FR-00057-system-health-status-tracking.md)
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall send periodic health telemetry to the GCS at 1 Hz via MAVLink SYS_STATUS messages, including sensor health bits, CPU load, battery voltage, and packet loss statistics.

## Rationale

Operators need real-time visibility into vehicle health status. 1 Hz rate is standard GCS telemetry rate, balancing update frequency with bandwidth usage.

## Acceptance Criteria

- [ ] Generate MAVLink SYS_STATUS message every 1 second
- [ ] Include sensor health bits (RC, IMU, GPS, compass, battery) per MAVLink specification
- [ ] Include CPU load percentage (0-1000, where 1000 = 100%)
- [ ] Include battery voltage (mV) and remaining capacity estimate (%)
- [ ] Include packet loss statistics (if available)
- [ ] Send via all active MAVLink transports

## Technical Details (if applicable)

**MAVLink SYS_STATUS**:

- `onboard_control_sensors_present`: Sensors installed (bitmask)
- `onboard_control_sensors_enabled`: Sensors enabled (bitmask)
- `onboard_control_sensors_health`: Sensors healthy (bitmask)
- `load`: CPU load (0-1000)
- `voltage_battery`: Battery voltage (mV)
- `battery_remaining`: Battery remaining (0-100%)

**Sensor Health Bits**:

- Bit 0: RC_RECEIVER
- Bit 3-4: GYRO, ACCEL (IMU)
- Bit 5: MAG (Compass)
- Bit 6: GPS
- Bit 11: BATTERY

## External References

- MAVLink SYS_STATUS: <https://mavlink.io/en/messages/common.html#SYS_STATUS>
- Analysis: [AN-00009-armed-state-monitoring](../analysis/AN-00009-armed-state-monitoring.md)
