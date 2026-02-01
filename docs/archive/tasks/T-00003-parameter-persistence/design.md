# T-00003 Parameter Persistence to Flash Storage

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-00003-parameter-persistence-plan](plan.md)
- Related ADRs:
  - [ADR-00004-storage-strategy](../../../adr/ADR-00004-storage-strategy.md)
- Related Requirements:
  - [FR-00006-runtime-parameters](../../../requirements/FR-00006-runtime-parameters.md)
  - [NFR-00003-memory-limits](../../../requirements/NFR-00003-memory-limits.md)

## Overview

Implement Flash-backed parameter persistence with wear leveling using redundant block rotation. Parameters survive reboots, load in < 100ms, and support at least 200 configurable parameters with async Flash writes that don't block control loops.

## Success Metrics

- [ ] Support 200+ parameters persisted to Flash
- [ ] Parameter load time < 100ms during system initialization
- [ ] Parameter save is async (non-blocking, < 5ms blocking time)
- [ ] Support 10,000+ parameter save cycles (wear leveling)
- [ ] < 0.1% data corruption rate during power-loss testing
- [ ] RAM usage for parameter cache < 2 KB

## Background and Current State

- Context: T-00002 (MAVLink Communication) implemented RAM-only parameter registry accessible via MAVLink protocol. Parameters reset to defaults on reboot.
- Current behavior: Parameters stored in RAM, modified via PARAM_SET, but don't persist across reboots.
- Pain points: Every reboot requires re-tuning via GCS. Field operators cannot save calibration or tuning changes permanently.
- Constraints:
  - Flash endurance: 10,000-100,000 erase cycles per block
  - Flash write takes 100ms+ (must be async)
  - Parameter load must be < 100ms (startup time requirement)
  - RAM budget < 2 KB for parameter cache
- Related ADRs: ADR-00004 selected redundant block rotation with 4-block wear leveling

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────┐
│         Application Layer                   │
│  (MAVLink, Control Loops, Telemetry)        │
└───────────────┬─────────────────────────────┘
                │ get_param() / set_param()
┌───────────────▼─────────────────────────────┐
│      Parameter System (RAM Cache)           │
│  - Parameter Registry (name, value, bounds) │
│  - Modified flag tracking                   │
│  - Async save scheduling                    │
└───────────────┬─────────────────────────────┘
                │ save_to_flash() / load_from_flash()
┌───────────────▼─────────────────────────────┐
│      Flash Storage (Persistence)            │
│  - Block rotation (wear leveling)           │
│  - CRC validation                            │
│  - Corruption recovery                       │
└───────────────┬─────────────────────────────┘
                │ erase_block() / write_block()
┌───────────────▼─────────────────────────────┐
│       Platform Flash Abstraction            │
│  (RP2040/RP2350 Flash Driver)               │
└─────────────────────────────────────────────┘
```

### Components

**Parameter Registry** (`src/core/parameters/registry.rs`)

- Static array of `ParamMetadata` structs (name, type, value, bounds)
- Hash map for fast lookup by name (using `heapless::FnvIndexMap`)
- Modified flag tracking for each parameter (know when to save)

**Flash Storage** (`src/core/parameters/storage.rs`)

- `FlashParamStorage` struct managing 4 Flash blocks
- Block rotation algorithm (round-robin with sequence numbers)
- CRC32 validation on read/write
- Corruption recovery (fall back to backup blocks)

**Platform Flash Abstraction** (`src/platform/traits/flash.rs`)

- `FlashInterface` trait (read, write, erase)
- RP2350 implementation (`src/platform/rp2350/flash.rs`)
- Mock implementation for testing (`src/platform/mock/flash.rs`)

### Data Flow

**Parameter Load (Startup):**

1. System initializes, calls `load_params_from_flash()`
2. FlashParamStorage reads blocks 0-3, finds block with highest valid sequence
3. Validate CRC32 checksum
4. If corruption detected, try next block (fallback to backup)
5. Deserialize parameter values into RAM cache
6. Update parameter registry with loaded values
7. Return success (< 100ms total)

**Parameter Save (User Request):**

1. User sets parameter via MAVLink PARAM_SET
2. Parameter registry validates bounds, updates value in RAM
3. Set modified flag for parameter
4. Schedule async Flash save (debounced 5 seconds)
5. After debounce delay, trigger `save_to_flash()`
6. FlashParamStorage finds active block (highest sequence)
7. Choose next block (round-robin: (active + 1) % 4)
8. Erase next block (async, \~100ms)
9. Serialize parameters, calculate CRC32
10. Write to next block with incremented sequence
11. Verify write (read back, check CRC)
12. Clear modified flags

### Data Models and Types

**Parameter Block Format (4 KB):**

```rust
pub struct ParameterBlock {
    magic: u32,              // 0x50415241 ("PARA")
    version: u16,            // Format version (1)
    sequence: u16,           // Increments on each write
    param_count: u16,        // Number of parameters
    reserved: u16,           // Padding for alignment
    params: [Parameter; 200],// Parameter entries (16 bytes each)
    crc32: u32,              // CRC32 checksum
}
```

**Parameter Entry (16 bytes):**

```rust
pub struct Parameter {
    name_hash: u32,          // FNV-1a hash of parameter name
    value: u32,              // f32 or u32 value (union)
    param_type: u8,          // ParamType enum (Float=0, Uint32=1)
    modified: u8,            // Modified flag (1 = changed since load)
    reserved: u16,           // Padding
}
```

**Flash Layout:**

```
Pico W (2 MB) / Pico 2 W (4 MB):
[Firmware]           0x000000 - 0x040000 (256 KB)
[Parameter Block 0]  0x040000 - 0x041000 (4 KB)   <- Active or backup
[Parameter Block 1]  0x041000 - 0x042000 (4 KB)   <- Rotation
[Parameter Block 2]  0x042000 - 0x043000 (4 KB)   <- Rotation
[Parameter Block 3]  0x043000 - 0x044000 (4 KB)   <- Rotation
[Mission Storage]    0x044000 - 0x046000 (8 KB)   <- Future use
[Log Storage]        0x046000 - ...      (Rest)   <- Future use
```

### Error Handling

- **CRC Failure**: Try next block (blocks 0-3), fall back to defaults if all invalid
- **Flash Write Failure**: Keep old block, log error via defmt, return error to user
- **Parameter Not Found**: Return error, GCS displays "Unknown parameter"
- **Out of Bounds**: Reject value, send PARAM_VALUE with unchanged value
- **Flash Full**: Should never occur (4 blocks always available), log critical error if detected

All error messages in English. Use defmt for embedded logging.

### Security Considerations

- **Parameter Validation**: Enforce min/max bounds to prevent unsafe configurations
- **CRC Validation**: Prevent corrupted parameters from loading (safety critical)
- **Default Values**: If all blocks corrupted, fall back to safe defaults
- **No Authentication**: Parameters can be modified by anyone with MAVLink access (deferred to future task)

### Performance Considerations

- **Fast Load**: Direct Flash read (< 50ms), single CRC check per block
- **Async Save**: Flash erase/write runs in background task, doesn't block control loops
- **Debounced Writes**: Batch multiple PARAM_SET into single Flash write (reduce wear)
- **Wear Leveling**: 4-block rotation → 40,000 effective write cycles (4x endurance)
- **RAM Usage**:
  - Parameter registry: \~3.2 KB (200 params × 16 bytes)
  - Active working set: \~1 KB (frequently accessed params)
  - Total: < 5 KB (within budget)

### Platform Considerations

#### RP2040 (Pico W)

- Flash: 2 MB, 4 KB erase blocks
- Flash erase: \~100ms (blocking)
- Flash write: \~500 us per 256-byte page
- 264 KB RAM total, < 5 KB for parameters (< 2% of RAM)

#### RP2350 (Pico 2 W)

- Flash: 4 MB, 4 KB erase blocks
- Flash erase: \~100ms (blocking)
- Flash write: \~500 us per 256-byte page
- 520 KB RAM total, < 5 KB for parameters (< 1% of RAM)

#### Cross-Platform

- Use platform abstraction Flash trait from T-00004 (or create if not available)
- Flash configuration (base address, block size) passed at initialization
- No platform-specific code outside `src/platform/`

## Alternatives Considered

1. **littlefs (Journaling Filesystem)**
   - Pros: Built-in wear leveling, power-loss protection, standard API
   - Cons: High RAM usage (\~16 KB), slower performance, integration complexity
   - Decision: Rejected per ADR-00004, manual implementation is simpler and more efficient

2. **EEPROM Emulation**
   - Pros: Standard approach for embedded systems, proven in STM32
   - Cons: No EEPROM in RP2040/RP2350, similar complexity to manual approach
   - Decision: Rejected, redundant block rotation is more flexible

3. **Single Block with CRC**
   - Pros: Simplest implementation, minimal code
   - Cons: No wear leveling (10,000 writes exhausts Flash), no backup on corruption
   - Decision: Rejected, endurance requirement is 10,000+ cycles

## Decision Rationale

- **Redundant Block Rotation**: Chosen for simplicity, performance, and low RAM usage (per ADR-00004)
- **Async Writes**: Required to avoid blocking control loops during 100ms Flash erase
- **Debounced Saves**: Reduce Flash wear by batching multiple PARAM_SET into single write
- **CRC32 Validation**: Industry standard for data integrity, fast on ARM Cortex-M

Trade-offs accepted:

- **Manual Implementation**: We implement block rotation ourselves vs using littlefs (\~500 lines of code vs 16 KB RAM savings)
- **Fixed Layout**: Storage layout is compile-time fixed (acceptable for embedded autopilot)

## Migration and Compatibility

- Backward compatibility: New feature, no migration required from T-00002 (RAM-only parameters)
- Forward compatibility: Version field in block header allows future format changes
- Rollout plan: Single-phase enablement, parameters auto-load from Flash if present
- Deprecation plan: N/A

## Testing Strategy

### Unit Tests

- **Block Serialization**: Test writing/reading ParameterBlock to buffer
- **CRC Calculation**: Test CRC32 checksum matches expected values
- **Block Rotation**: Test sequence number increments, round-robin selection
- **Corruption Recovery**: Test fallback to backup blocks when CRC fails
- **Bounds Validation**: Test parameter set rejects out-of-bounds values

Place tests in `#[cfg(test)]` modules within each file.

### Integration Tests

- **Save/Load Cycle**: Write parameters, reboot (simulated), verify loaded values match
- **Wear Leveling**: Write parameters 100 times, verify all 4 blocks used (round-robin)
- **Power Loss**: Interrupt write mid-operation (mock Flash), verify recovery from backup
- **Concurrent Access**: Set parameters from MAVLink while Flash save in progress, verify no data races

Note: Hardware integration tests require RP2350 target, use `#[cfg(feature = "pico2_w")]`.

### Hardware Testing

- **Endurance Test**: Loop parameter saves 1000 times, verify Flash blocks still valid
- **Power-Loss Test**: Cut power during Flash write, reboot, verify parameters recovered
- **Performance Test**: Measure parameter load time on real hardware (target < 100ms)
- **Stress Test**: Set parameters at 10Hz for 60 seconds, verify no missed saves

## Documentation Impact

- Add `docs/parameters.md` with usage guide (defining parameters, save/load, Flash layout)
- Update `docs/architecture.md` with parameter persistence diagram
- Add code examples in parameter storage files
- Update `README.md` with parameter persistence section

## External References

- RP2040/RP2350 Flash Programming: <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf>
- ArduPilot Parameter System: <https://ardupilot.org/dev/docs/code-overview-adding-a-new-parameter.html>
- CRC32 Algorithm: <https://en.wikipedia.org/wiki/Cyclic_redundancy_check>
- embedded-storage Trait: <https://docs.rs/embedded-storage/latest/embedded_storage/>

## Open Questions

- [ ] Should we implement write-through or write-back caching? → Decision: Write-back with debounced saves (reduce Flash wear)
- [ ] How to handle parameter migration when format version changes? → Method: Keep version field, implement migration function if needed in future
- [ ] Should we compress parameter data to reduce Flash usage? → Next step: Measure actual usage, compression likely unnecessary for 200 params
- [ ] What debounce delay is optimal for Flash saves? → Decision: Start with 5 seconds, make configurable parameter if needed

## Appendix

### Wear Leveling Example

```
Initial state (all blocks empty):
Block 0: seq=0, invalid CRC
Block 1: seq=0, invalid CRC
Block 2: seq=0, invalid CRC
Block 3: seq=0, invalid CRC

After 1st save:
Block 0: seq=1, valid CRC  <- Active
Block 1: seq=0, invalid CRC
Block 2: seq=0, invalid CRC
Block 3: seq=0, invalid CRC

After 2nd save:
Block 0: seq=1, valid CRC
Block 1: seq=2, valid CRC  <- Active
Block 2: seq=0, invalid CRC
Block 3: seq=0, invalid CRC

After 5th save:
Block 0: seq=5, valid CRC  <- Active (wrapped)
Block 1: seq=2, valid CRC
Block 2: seq=4, valid CRC
Block 3: seq=3, valid CRC
```

### Parameter Block Size Calculation

```
Header: 12 bytes (magic, version, sequence, count, reserved)
Params: 200 × 16 bytes = 3200 bytes
CRC32: 4 bytes
Total: 3216 bytes (fits in 4 KB block)
```

### Estimated Flash Endurance

```
Assumption: 10,000 erase cycles per block (conservative)
4 blocks × 10,000 cycles = 40,000 parameter saves
Daily saves: 10 parameter saves/day (typical usage)
Lifespan: 40,000 / 10 = 4,000 days = 11 years
```

### Glossary

- **Wear Leveling**: Distributing Flash writes across multiple blocks to extend endurance
- **CRC32**: 32-bit Cyclic Redundancy Check for data integrity validation
- **Sequence Number**: Monotonically increasing counter to identify newest block
- **Debouncing**: Delaying Flash writes to batch multiple changes into single write
- **Magic Header**: Fixed value (`PARA`) to identify valid parameter block
