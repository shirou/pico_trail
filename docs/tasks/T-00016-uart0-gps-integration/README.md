# T-00016 UART0 GPS Integration for Waypoint Navigation

## Metadata

- Type: Task
- Status: Completed

**Completion Date:** 2025-11-20

## Links

- Related Analyses:
  - [AN-00021-gps-hardware-integration](../../analysis/AN-00021-gps-hardware-integration.md)
- Related Requirements:
  - [FR-00077-gps-uart-driver](../../requirements/FR-00077-gps-uart-driver.md)
  - [FR-00004-gps-waypoint-navigation](../../requirements/FR-00004-gps-waypoint-navigation.md)
  - [FR-00076-gps-operation-data-management](../../requirements/FR-00076-gps-operation-data-management.md)
- Related ADRs:
  - [ADR-00020-uart0-gps-allocation](../../adr/ADR-00020-uart0-gps-allocation.md)
- Associated Design Document:
  - [T-00016-uart0-gps-integration-design](./design.md)
- Associated Plan Document:
  - [T-00016-uart0-gps-integration-plan](./plan.md)

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
