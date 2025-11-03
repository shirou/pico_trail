# ADR-hj79f Flash Storage Strategy: Wear Leveling with Redundant Blocks

## Metadata

- Type: ADR
- Status: Approved

## Links

- Related Analyses:
  - [AN-5nucb-core-systems](../analysis/AN-5nucb-core-systems.md)
- Impacted Requirements:
  - [FR-a1cuu-runtime-parameters](../requirements/FR-a1cuu-runtime-parameters.md)
  - [FR-4e922-data-logging](../requirements/FR-4e922-data-logging.md)
  - [NFR-z2iuk-memory-limits](../requirements/NFR-z2iuk-memory-limits.md)
- Supersedes ADRs: N/A
- Related Tasks: Will be created after approval

## Context

The autopilot requires persistent storage for:

- **Parameters**: \~200 parameters (tuning values, configuration)
- **Missions**: Up to 200 waypoints
- **Logs**: High-frequency sensor/control data (\~6 KB/s)
- **Calibration Data**: IMU offsets, magnetometer calibration

### Problem

RP2040/RP2350 Flash has limited write endurance:

- **Erase cycles**: 10,000-100,000 per block (manufacturer dependent)
- **Block size**: 4 KB (smallest erasable unit)
- **Flash size**: Pico W: 2 MB, Pico 2 W: 4 MB

Without wear leveling, parameter saves or log writes can exhaust Flash endurance prematurely, bricking the device.

### Constraints

- Flash write takes 100ms+ (blocks control loops if synchronous)
- Must not lose data during power loss (mid-write)
- Parameters must load quickly (< 100ms on startup)
- Logs must not block control loops during writes

### Prior Art

- **ArduPilot**: Uses microSD card (unlimited writes, no wear leveling needed)
- **PX4**: Uses EEPROM emulation with wear leveling on STM32 Flash
- **Embedded Systems**: Circular buffers, multiple block rotation, journaling filesystems

## Success Metrics

- **Endurance**: Support 10,000 parameter save cycles minimum
- **Reliability**: < 0.1% data corruption rate (power loss during write)
- **Performance**: Parameter load < 100ms, saves non-blocking (async)
- **Capacity**: Store 200 parameters + 200 waypoints + logs

## Decision

**We will implement wear leveling for parameters and missions using redundant block rotation, and circular buffering for logs.**

### Storage Layout

```
Pico W (2 MB Flash):
[Firmware]           0x000000 - 0x040000 (256 KB)
[Parameter Block 0]  0x040000 - 0x041000 (4 KB)   <- Active
[Parameter Block 1]  0x041000 - 0x042000 (4 KB)   <- Backup
[Parameter Block 2]  0x042000 - 0x043000 (4 KB)   <- Rotation
[Parameter Block 3]  0x043000 - 0x044000 (4 KB)   <- Rotation
[Mission Storage]    0x044000 - 0x046000 (8 KB)   <- 200 waypoints
[Log Storage]        0x046000 - 0x200000 (~1.7 MB) <- Circular buffer

Pico 2 W (4 MB Flash):
[Firmware]           0x000000 - 0x040000 (256 KB)
[Parameter Block 0]  0x040000 - 0x041000 (4 KB)
[Parameter Block 1]  0x041000 - 0x042000 (4 KB)
[Parameter Block 2]  0x042000 - 0x043000 (4 KB)
[Parameter Block 3]  0x043000 - 0x044000 (4 KB)
[Mission Storage]    0x044000 - 0x046000 (8 KB)
[Log Storage]        0x046000 - 0x400000 (~3.7 MB) <- Circular buffer
```

### Decision Drivers

1. **Wear Leveling**: Rotate parameter writes across 4 blocks → 40,000 effective write cycles
2. **Redundancy**: Keep backup block for corruption recovery
3. **Non-blocking Writes**: Use async writes with RAM cache
4. **Circular Logs**: Prevent Flash exhaustion, overwrite oldest logs when full

### Considered Options

- **Option A: Redundant Block Rotation with Circular Logs** ⭐ Selected
- **Option B: Journaling Filesystem (littlefs)**
- **Option C: EEPROM Emulation**

### Option Analysis

**Option A: Redundant Block Rotation + Circular Logs**

- **Pros**:
  - Simple to implement (no filesystem overhead)
  - Explicit wear leveling control
  - Fast parameter load (read one block)
  - Circular logs prevent Flash exhaustion
- **Cons**:
  - Manual implementation (no existing library)
  - Must handle corruption detection (CRC checksums)
- **Estimated Performance**: < 50ms param load, async writes

**Option B: littlefs (Journaling Filesystem)**

- **Pros**:
  - Built-in wear leveling and power-loss protection
  - Standard filesystem API (open/read/write/close)
  - Well-tested library
- **Cons**:
  - Higher RAM usage (\~8-16 KB for filesystem state)
  - Slower performance (filesystem overhead)
  - More complex to integrate
- **Estimated Performance**: \~100ms param load, \~200 KB RAM overhead

**Option C: EEPROM Emulation**

- **Pros**:
  - Standard approach for embedded systems
  - Proven in STM32 applications
- **Cons**:
  - No EEPROM in RP2040/RP2350 (must emulate in Flash)
  - Manual wear leveling required
  - Similar complexity to Option A but less flexible
- **Estimated Performance**: Similar to Option A

## Rationale

Redundant block rotation was chosen over littlefs and EEPROM emulation for:

1. **Simplicity**: No filesystem overhead, minimal RAM usage
2. **Performance**: Direct Flash access faster than filesystem API
3. **Control**: Explicit wear leveling policy (rotate across 4 blocks)
4. **Memory Efficiency**: \~2 KB RAM vs littlefs \~16 KB

### Trade-offs Accepted

- **Manual Implementation**: We implement corruption detection and block rotation ourselves (vs using littlefs)
- **Fixed Layout**: Storage layout is fixed at compile-time (vs dynamic filesystem)

**Decision**: We accept the implementation effort for better performance and lower RAM usage.

## Consequences

### Positive

- **Extended Endurance**: 4-block rotation → 40,000 effective write cycles (vs 10,000 single-block)
- **Fast Parameter Load**: < 50ms (direct block read)
- **Low RAM Usage**: \~2 KB for parameter cache (vs littlefs \~16 KB)
- **Non-blocking Writes**: Async writes don't stall control loops
- **Circular Logs**: Prevent Flash exhaustion, automatic old log overwrite

### Negative

- **Manual Implementation**: Must implement block rotation, CRC checking, corruption recovery
- **Fixed Layout**: Storage layout is compile-time fixed (less flexible than filesystem)
- **Corruption Risk**: Power loss during write can corrupt active block (mitigated by backup block)

### Neutral

- **No Filesystem**: Explicit storage management (intentional for embedded systems)

## Implementation Notes

### Parameter Block Format

```rust
pub struct ParameterBlock {
    magic: u32,              // 0x50415241 ("PARA")
    version: u16,            // Parameter format version
    sequence: u16,           // Increments on each write (for block freshness)
    param_count: u16,        // Number of parameters
    reserved: u16,           // Padding
    params: [Parameter; 200],// Parameter data
    crc32: u32,              // CRC32 checksum
}

// Parameter entry (16 bytes)
pub struct Parameter {
    name_hash: u32,          // Hash of parameter name (for fast lookup)
    value: u32,              // f32 or u32 value (union)
    flags: u16,              // Type, modified, etc.
    reserved: u16,           // Padding
}
```

### Wear Leveling Algorithm

```rust
pub async fn save_parameters(params: &Parameters) {
    // 1. Find active block (highest sequence number)
    let active_block = find_active_block().await;

    // 2. Choose next block (round-robin rotation)
    let next_block = (active_block + 1) % 4;

    // 3. Erase next block
    erase_block(next_block).await;

    // 4. Write parameters with incremented sequence
    let sequence = read_sequence(active_block) + 1;
    write_block(next_block, params, sequence).await;

    // 5. Verify write (read back and check CRC)
    if !verify_block(next_block).await {
        // Fallback to backup block
        warn!("Parameter write failed, keeping old block");
    }
}

pub async fn load_parameters() -> Result<Parameters> {
    // Find block with highest sequence and valid CRC
    for block_id in 0..4 {
        if let Ok(params) = read_and_verify_block(block_id).await {
            return Ok(params);
        }
    }
    Err(Error::NoValidParameterBlock)
}
```

### Circular Log Buffer

```rust
pub struct CircularLogBuffer {
    start_addr: u32,         // Log region start address
    end_addr: u32,           // Log region end address
    write_ptr: u32,          // Current write position
    ram_buffer: [u8; 8192],  // 8 KB RAM buffer
    buffer_used: usize,      // Bytes in RAM buffer
}

impl CircularLogBuffer {
    pub fn append(&mut self, data: &[u8]) {
        // Buffer in RAM
        self.ram_buffer[self.buffer_used..].copy_from_slice(data);
        self.buffer_used += data.len();

        // Flush to Flash when buffer 75% full
        if self.buffer_used > 6144 {
            self.flush_async();
        }
    }

    async fn flush_async(&mut self) {
        // Write RAM buffer to Flash
        write_flash(self.write_ptr, &self.ram_buffer[..self.buffer_used]).await;

        // Advance write pointer (wrap to start if at end)
        self.write_ptr += self.buffer_used as u32;
        if self.write_ptr >= self.end_addr {
            self.write_ptr = self.start_addr; // Circular wrap
        }

        self.buffer_used = 0;
    }
}
```

## Platform Considerations

- **Pico W**: 2 MB Flash, \~1.7 MB for logs (\~5 hours @ 6 KB/s)
- **Pico 2 W**: 4 MB Flash, \~3.7 MB for logs (\~10 hours @ 6 KB/s)
- **Cross-Platform**: Storage abstraction works on both platforms

## Open Questions

- [ ] Should we implement filesystem (littlefs) if manual approach proves too complex? → Decision: Implement manual first, migrate to littlefs if needed
- [ ] How do we test wear leveling without waiting for 10,000 write cycles? → Method: Accelerated test (tight write loop, monitor block erase counts)
- [ ] Should we compress logs to extend storage capacity? → Next step: Measure log size, implement compression if < 50% savings

## External References

- RP2040/RP2350 Flash Programming: <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf>
- littlefs: <https://github.com/littlefs-project/littlefs>
- STM32 EEPROM Emulation: <https://www.st.com/resource/en/application_note/an4061-eeprom-emulation-in-stm32-microcontrollers-stmicroelectronics.pdf>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
