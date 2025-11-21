# T-vxtxn UART0 GPS Integration for Waypoint Navigation

## Metadata

- Type: Task
- Status: Completed

**Completion Date:** 2025-11-20

## Links

- Related Analyses:
  - [AN-xfiyr-gps-hardware-integration](../../analysis/AN-xfiyr-gps-hardware-integration.md)
- Related Requirements:
  - [FR-93b5v-gps-uart-driver](../../requirements/FR-93b5v-gps-uart-driver.md)
  - [FR-333ym-gps-waypoint-navigation](../../requirements/FR-333ym-gps-waypoint-navigation.md)
  - [FR-3ik7l-gps-operation-data-management](../../requirements/FR-3ik7l-gps-operation-data-management.md)
- Related ADRs:
  - [ADR-8tp69-uart0-gps-allocation](../../adr/ADR-8tp69-uart0-gps-allocation.md)
- Associated Design Document:
  - [T-vxtxn-uart0-gps-integration-design](./design.md)
- Associated Plan Document:
  - [T-vxtxn-uart0-gps-integration-plan](./plan.md)

## Summary

Integrate UART-based GPS driver with GPS operation manager to provide continuous position data for waypoint navigation. Replace I2C GPS implementation with UART0 GPS connection on GPIO 0 (RX), GPIO 1 (TX) at 9600 baud.

## Scope

- In scope:
  - GPS operation manager implementation with async polling, error recovery, and state management
  - Integration of existing UART GPS driver (`src/devices/gps.rs`)
  - Board configuration update for UART0 GPIO allocation
  - Removal of I2C GPS implementation (`gps_i2c.rs`)
  - Unit tests for GPS operation manager with MockUart
  - Documentation updates (hwdef, architecture)
- Out of scope:
  - New GPS driver implementation (existing UART driver sufficient)
  - UBX protocol support for advanced GPS configuration (defer to future enhancement)
  - IMU integration (separate future task; I2C0 remains available)
  - GPS hardware testing (validation deferred to hardware availability)

## Success Metrics

- GPS position data received at 1-10Hz: NMEA sentences parsed successfully with <300ms latency
- Error recovery functional: Automatic retry within 5 seconds (3 retries, exponential backoff)
- Failsafe triggering: GPS failsafe activated after 3 consecutive NoFix readings (\~3s)
- Code quality: All unit tests pass, no clippy warnings, embedded build succeeds
- Documentation: Board hwdef updated, traceability regenerated, all docs lint clean

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
