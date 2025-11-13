# NFR-j17oa | Calibration Persistence

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-n8yb7-battery-telemetry](../analysis/AN-n8yb7-battery-telemetry.md)
- Prerequisite Requirements:
  - [FR-uq6as-voltage-conversion-calculation](../requirements/FR-uq6as-voltage-conversion-calculation.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-q3psc-battery-telemetry](../tasks/T-q3psc-battery-telemetry/README.md)

## Requirement Statement

Battery voltage calibration coefficient (`BATT_VOLT_MULT` parameter) shall persist across reboots by storing it in non-volatile flash memory, eliminating the need for users to recalibrate after power cycles.

## Rationale

Users calibrate the voltage divider coefficient (`BATT_VOLT_MULT`) by comparing system voltage readings to multimeter measurements. Without persistent storage, this calibration would be lost on every reboot, forcing users to recalibrate before each flight. Storing parameters in flash memory provides the expected behavior: configure once, persist indefinitely. This follows ArduPilot conventions where all parameters persist in EEPROM/flash.

## User Story (if applicable)

The system shall store the `BATT_VOLT_MULT` parameter in non-volatile flash memory to ensure calibration values persist across reboots and power cycles.

## Acceptance Criteria

- [ ] `BATT_VOLT_MULT` parameter stored in flash memory using ParameterStore
- [ ] Parameter value survives reboot (power cycle test confirms retention)
- [ ] Parameter can be modified via MAVLink PARAM_SET command
- [ ] Modified parameter value persists after save and reboot
- [ ] Flash write operation does not block scheduler for more than 10ms
- [ ] Parameter storage integrated with existing `ParameterStore` at `src/parameters/storage.rs`
- [ ] Default value (3.95) used on first boot or after parameter reset
- [ ] Parameter loading completes during system initialization before scheduler starts

## Technical Details

### Non-Functional Requirement Details

- **Reliability**: Data persistence
  - Parameters stored in RP2040/RP2350 internal flash (last sector reserved)
  - Flash endurance: 100,000 erase/write cycles minimum (datasheet spec)
  - Expected usage: <1000 writes per year (parameter changes are rare)
  - Lifespan: >100 years of normal use
  - Data integrity: CRC or checksum validation on read (implementation-dependent)

- **Performance**: Flash write latency
  - Flash write must not block scheduler for extended periods
  - Target: <10ms for parameter save operation
  - RP2040/RP2350 flash write: \~1-5ms for 4KB sector erase + write
  - Acceptable: brief scheduler pause during save (user-initiated via PARAM_SET)
  - Not acceptable: frequent background writes causing jitter

- **Usability**: Configuration workflow
  - User sets `BATT_VOLT_MULT` via Mission Planner or QGroundControl
  - System sends MAVLink PARAM_VALUE acknowledgment
  - Parameter persists immediately (no explicit "save" command required)
  - Reboot: system loads parameter from flash automatically

**ParameterStore Integration:**

The project already has parameter storage infrastructure at `src/parameters/storage.rs`. `BATT_VOLT_MULT` should integrate with this existing system:

1. **Parameter Definition**: Add to `BatteryParams` in `src/parameters/battery.rs`

   ```rust
   pub struct BatteryParams {
       pub batt_arm_volt: f32,
       pub batt_crt_volt: f32,
       pub batt_fs_crt_act: u8,
       pub batt_volt_mult: f32,  // New parameter
   }
   ```

2. **Default Value**: 3.95 (per Freenove reference)

3. **Storage**: ParameterStore serializes all parameters to flash

4. **Loading**: On boot, ParameterStore reads from flash and populates BatteryParams

**Flash Layout (RP2040/RP2350):**

Typical parameter storage uses the last flash sector:

- Flash size: 2MB (RP2040), 4MB (RP2350)
- Sector size: 4KB
- Parameter storage: Last 4KB sector (0x1FF000 for 2MB flash)
- Format: Serialized parameter struct + CRC

**MAVLink Parameter Protocol:**

ArduPilot-compatible parameter handling:

1. GCS sends `PARAM_SET(BATT_VOLT_MULT, 3.85)`
2. System updates value in memory
3. System writes to flash asynchronously
4. System sends `PARAM_VALUE(BATT_VOLT_MULT, 3.85)` acknowledgment
5. On next boot, system loads 3.85 from flash

## Platform Considerations

### Unix

N/A – Platform agnostic (embedded-only feature)

### Windows

N/A – Platform agnostic (embedded-only feature)

### Cross-Platform

Flash storage applies only to embedded targets (RP2040/RP2350). Host tests shall use in-memory parameter storage.

## Risks & Mitigation

| Risk                                                | Impact | Likelihood | Mitigation                                           | Validation                                      |
| --------------------------------------------------- | ------ | ---------- | ---------------------------------------------------- | ----------------------------------------------- |
| Flash corruption causes parameter loss              | High   | Low        | CRC validation; fallback to defaults on read failure | Corrupt flash intentionally and verify recovery |
| Flash write blocks scheduler causing motor jitter   | High   | Low        | Measure flash write timing; ensure <10ms             | Hardware test with defmt timing measurement     |
| Flash endurance limit reached                       | Low    | Very Low   | 100k cycles = 100 years at 1000 writes/year          | Document expected lifespan in user manual       |
| ParameterStore incompatible with new parameter type | Medium | Low        | Verify ParameterStore supports f32 parameters        | Code review and unit test                       |
| Parameter not loaded before battery update starts   | High   | Low        | Load parameters during init, before scheduler starts | Unit test initialization order                  |

## Implementation Notes

**Preferred Approaches:**

- Use existing `ParameterStore` infrastructure if available
- If ParameterStore does not exist, consider using:
  - `embassy-rp` flash driver for low-level access
  - `sequential-storage` crate for wear-leveling and crash-safe writes
- Store entire parameter struct (all battery parameters) atomically
- Validate data integrity with CRC32 on read
- Example pattern:

  ```rust
  pub struct ParameterStore {
      flash: Flash,
  }

  impl ParameterStore {
      pub async fn save_parameters(&mut self, params: &BatteryParams) -> Result<(), FlashError> {
          let data = serialize_with_crc(params);
          self.flash.write(PARAM_SECTOR_ADDR, &data).await?;
          Ok(())
      }

      pub async fn load_parameters(&mut self) -> Result<BatteryParams, FlashError> {
          let data = self.flash.read(PARAM_SECTOR_ADDR, PARAM_SIZE).await?;
          validate_crc_and_deserialize(&data)
      }
  }
  ```

**Known Pitfalls:**

- Flash erase/write requires disabling interrupts or runs in critical section (embassy-rp handles this)
- Writing to flash from interrupt context is not allowed
- Flash operations must run from RAM (code cannot execute from flash during write)
- Parameter struct must have stable serialization format (use versioning if schema changes)
- Do not write to flash on every parameter change in rapid succession (debounce/throttle writes)

**Related Code Areas:**

- `src/parameters/storage.rs` - ParameterStore implementation (if exists)
- `src/parameters/battery.rs` - BatteryParams struct
- `src/communication/mavlink/handlers/params.rs` - MAVLink parameter protocol handlers
- `examples/mavlink_rc_control.rs` - System initialization (parameter loading)

**Validation Strategy:**

- Unit test: Mock flash; verify save and load operations
- Integration test: Write parameter, reboot (simulated), verify value retained
- Hardware test: Set parameter via MAVLink, power cycle Pico 2 W, verify persistence
- Corruption test: Intentionally corrupt flash, verify fallback to defaults

## External References

- [RP2040 Datasheet - Flash Memory](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) - Flash specifications and endurance
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - Flash memory (similar to RP2040)
- [ArduPilot Parameter Protocol](https://ardupilot.org/dev/docs/mavlink-parameters.html) - MAVLink parameter handling
- [Embassy-RP Flash Driver](https://docs.embassy.dev/embassy-rp/) - Flash API documentation
- [sequential-storage crate](https://crates.io/crates/sequential-storage) - Wear-leveling for embedded flash

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
