# T-x8mq2 BNO086 Driver Implementation - Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Parent Task:
  - [T-x8mq2-bno086-driver-implementation](./README.md)
- Associated Plan:
  - [T-x8mq2-bno086-driver-implementation-plan](./plan.md)
- Related Analysis:
  - [AN-srhcj-bno086-imu-integration](../../analysis/AN-srhcj-bno086-imu-integration.md)

## Overview

This document describes the design for implementing the BNO086 9-axis IMU driver with on-chip sensor fusion. Unlike raw IMU sensors (ICM-20948, MPU-9250), the BNO086 provides pre-computed quaternion output via its integrated ARM Cortex-M0+ processor, eliminating the need for external EKF implementation.

Key design goals:

- New `QuaternionSensor` trait for quaternion-native sensors
- SHTP (Sensor Hub Transport Protocol) implementation over I2C
- Interrupt-driven (INT pin) async data acquisition
- Hardware reset (RST pin) based error recovery
- 100Hz quaternion output for attitude estimation

## Success Metrics

- [ ] BNO086 WHO_AM_I verification passes
- [ ] Quaternion output at 100Hz (±5Hz) measured over 10 seconds
- [ ] I2C read latency < 2ms per SHTP packet
- [ ] Automatic recovery from errors within 200ms
- [ ] Memory footprint < 2KB for driver state
- [ ] Works on RP2350 (Pico 2 W)

## Background and Current State

- Context: pico_trail requires attitude estimation for autonomous navigation
- Current behavior:
  - `ImuSensor` trait exists for raw sensor data (`src/devices/traits/imu.rs`)
  - ICM-20948 and MPU-9250 drivers exist but sensors are unavailable (counterfeit/EOL)
  - DCM AHRS implementation exists (`src/subsystems/ahrs/`)
- Pain points:
  - External EKF/DCM complexity for raw sensor fusion
  - Counterfeit hardware issues with ICM-20948
  - CPU overhead for sensor fusion calculations
- Constraints:
  - Embassy async runtime
  - No heap allocation (static buffers only)
  - Must handle SHTP protocol complexity
- Related ADRs:
  - ADR-t5cq4: I2C driver architecture (general reference)

## Proposed Design

### High-Level Architecture

```text
┌──────────────────────────────────────────────────────────────┐
│                    Navigation/Control                        │
│              (consumes QuaternionSensor)                     │
└──────────────────────────┬───────────────────────────────────┘
                           │ QuaternionSensor trait
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                      Bno086Driver                            │
│  ┌──────────────┬───────────────┬──────────────────────────┐ │
│  │ SHTP Handler │ Report Parser │ Error Recovery (RST)     │ │
│  └──────┬───────┴───────┬───────┴──────────┬───────────────┘ │
└─────────┼───────────────┼──────────────────┼─────────────────┘
          │               │                  │
          ▼               ▼                  ▼
┌──────────────────────────────────────────────────────────────┐
│        I2C Bus                     GPIO (INT, RST)           │
└──────────────────────────┬───────────────────────────────────┘
                           │
                           ▼
                    ┌──────────┐
                    │ BNO086   │
                    │  (0x4A)  │
                    └──────────┘
```

### Components

**New Trait: `QuaternionSensor`**

```rust
/// Trait for sensors that provide direct quaternion output
#[allow(async_fn_in_trait)]
pub trait QuaternionSensor {
    /// Read quaternion from sensor (w, x, y, z)
    ///
    /// Returns unit quaternion representing orientation.
    async fn read_quaternion(&mut self) -> Result<Quaternion<f32>, QuaternionError>;

    /// Read quaternion with accuracy estimate
    ///
    /// Returns quaternion and accuracy in radians.
    async fn read_quaternion_with_accuracy(&mut self)
        -> Result<(Quaternion<f32>, f32), QuaternionError>;

    /// Get sensor health status
    fn is_healthy(&self) -> bool;

    /// Get last update timestamp (microseconds since boot)
    fn last_update_us(&self) -> u64;
}
```

**SHTP Protocol (Generic, Sensor-Independent)**

SHTP (Sensor Hub Transport Protocol) is used by multiple Hillcrest/CEVA sensors (BNO080, BNO085, BNO086, FSM300, etc.). We implement it as a reusable protocol module independent of any specific sensor.

```rust
// Location: src/communication/shtp/mod.rs

/// SHTP packet structure (sensor-independent)
pub struct ShtpPacket<const N: usize = 128> {
    /// Payload length (excluding header)
    pub length: u16,
    /// Channel number
    pub channel: u8,
    /// Sequence number
    pub sequence: u8,
    /// Payload data
    pub payload: [u8; N],
}

/// SHTP channel definitions (common across all SHTP devices)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ShtpChannel {
    Command = 0,
    Executable = 1,
    Control = 2,
    InputReport = 3,
    WakeInputReport = 4,
    Gyro = 5,
}

/// SHTP error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShtpError {
    /// Transport layer error (I2C, SPI, etc.)
    TransportError,
    /// Invalid packet header
    InvalidHeader,
    /// Payload too large for buffer
    PayloadTooLarge,
    /// Sequence number mismatch
    SequenceMismatch,
}

/// SHTP transport trait - implement for I2C, SPI, or UART
#[allow(async_fn_in_trait)]
pub trait ShtpTransport {
    /// Read a complete SHTP packet
    async fn read_packet<const N: usize>(
        &mut self,
        packet: &mut ShtpPacket<N>,
    ) -> Result<(), ShtpError>;

    /// Write a complete SHTP packet
    async fn write_packet<const N: usize>(
        &mut self,
        packet: &ShtpPacket<N>,
    ) -> Result<(), ShtpError>;
}

/// SHTP over I2C implementation
pub struct ShtpI2c<I2C> {
    i2c: I2C,
    address: u8,
    sequence: [u8; 6],  // Per-channel sequence numbers
}

impl<I2C: embedded_hal_async::i2c::I2c> ShtpTransport for ShtpI2c<I2C> {
    async fn read_packet<const N: usize>(
        &mut self,
        packet: &mut ShtpPacket<N>,
    ) -> Result<(), ShtpError> {
        // Read 4-byte header first
        let mut header = [0u8; 4];
        self.i2c.read(self.address, &mut header).await
            .map_err(|_| ShtpError::TransportError)?;

        // Parse header
        packet.length = u16::from_le_bytes([header[0], header[1]]) & 0x7FFF;
        packet.channel = header[2];
        packet.sequence = header[3];

        // Read payload if any
        if packet.length > 4 {
            let payload_len = (packet.length - 4) as usize;
            if payload_len > N {
                return Err(ShtpError::PayloadTooLarge);
            }
            self.i2c.read(self.address, &mut packet.payload[..payload_len]).await
                .map_err(|_| ShtpError::TransportError)?;
        }

        Ok(())
    }

    async fn write_packet<const N: usize>(
        &mut self,
        packet: &ShtpPacket<N>,
    ) -> Result<(), ShtpError> {
        // Build header + payload and write
        // ...
        Ok(())
    }
}
```

**BNO086 Driver (Uses SHTP)**

```rust
// Location: src/devices/imu/bno086/driver.rs

/// BNO086 driver using generic SHTP transport
pub struct Bno086Driver<T: ShtpTransport, INT, RST> {
    transport: T,              // SHTP transport (I2C, SPI, etc.)
    int_pin: INT,
    rst_pin: RST,
    last_quaternion: Quaternion<f32>,
    last_accuracy: f32,
    last_update_us: u64,
    healthy: bool,
    error_count: u32,
}

impl<T: ShtpTransport, INT, RST> Bno086Driver<T, INT, RST> {
    /// Create driver with SHTP transport
    pub fn new(transport: T, int_pin: INT, rst_pin: RST) -> Self {
        Self {
            transport,
            int_pin,
            rst_pin,
            last_quaternion: Quaternion::identity(),
            last_accuracy: 0.0,
            last_update_us: 0,
            healthy: false,
            error_count: 0,
        }
    }
}
```

### Data Flow

1. **Initialization**:
   - Assert RST low for 20ms, then high
   - Wait for INT falling edge (boot complete)
   - Read and discard initial reports
   - Send "Set Feature Command" to enable Rotation Vector at 100Hz

2. **Normal Operation**:
   - Await INT falling edge (data ready)
   - Read SHTP header (4 bytes) to get payload length
   - Read SHTP payload (N bytes)
   - Parse Rotation Vector report (ID 0x05)
   - Update driver state with new quaternion

3. **Error Recovery**:
   - If INT timeout (500ms), trigger hardware reset
   - Assert RST low for 20ms
   - Re-initialize sensor
   - Resume normal operation

### Data Models and Types

**Quaternion Error Types**

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuaternionError {
    /// I2C communication failed
    I2cError,
    /// SHTP protocol error
    ProtocolError,
    /// Sensor not initialized
    NotInitialized,
    /// Sensor reset in progress
    Resetting,
    /// Invalid data received
    InvalidData,
    /// Timeout waiting for data
    Timeout,
}
```

**Rotation Vector Report**

```rust
/// BNO086 Rotation Vector report (ID 0x05)
pub struct RotationVectorReport {
    /// Quaternion i component (Q14 fixed-point)
    pub q_i: i16,
    /// Quaternion j component (Q14 fixed-point)
    pub q_j: i16,
    /// Quaternion k component (Q14 fixed-point)
    pub q_k: i16,
    /// Quaternion real component (Q14 fixed-point)
    pub q_real: i16,
    /// Accuracy estimate (Q12 fixed-point, radians)
    pub accuracy: i16,
}

impl RotationVectorReport {
    /// Convert to nalgebra Quaternion
    pub fn to_quaternion(&self) -> Quaternion<f32> {
        let scale = 1.0 / 16384.0; // Q14
        Quaternion::new(
            self.q_real as f32 * scale,  // w
            self.q_i as f32 * scale,     // x
            self.q_j as f32 * scale,     // y
            self.q_k as f32 * scale,     // z
        )
    }

    /// Convert accuracy to radians
    pub fn accuracy_radians(&self) -> f32 {
        self.accuracy as f32 / 4096.0 // Q12
    }
}
```

### Error Handling

- **I2C Errors**: Retry up to 3 times, then trigger RST reset
- **SHTP Parse Errors**: Log warning, discard packet, wait for next INT
- **Timeout**: If no INT within 500ms, trigger RST reset
- **Reset Loop**: If 3 consecutive resets fail, mark sensor unhealthy

### Performance Considerations

**Timing Budget (100Hz = 10ms period)**

| Operation        | Time      | Notes                      |
| ---------------- | --------- | -------------------------- |
| INT wait         | \~0ms     | Async await, no CPU usage  |
| I2C header read  | \~0.2ms   | 4 bytes @ 400kHz           |
| I2C payload read | \~0.5ms   | \~20 bytes typical         |
| Parse quaternion | \~0.1ms   | Fixed-point conversion     |
| State update     | \~0.1ms   | Mutex lock, copy           |
| **Total**        | **\~1ms** | 9ms margin for other tasks |

**Memory Usage**

| Component        | Size       | Notes                 |
| ---------------- | ---------- | --------------------- |
| Driver state     | \~200B     | Quaternion, sequences |
| SHTP buffer      | 132B       | Header + max payload  |
| Sequence numbers | 6B         | Per-channel           |
| **Total**        | **\~350B** | Well under 2KB target |

### Platform Considerations

#### RP2350 (Pico 2 W)

- I2C0: GPIO4 (SDA), GPIO5 (SCL) - 400kHz Fast Mode
- INT: GPIO6 (input with pull-up)
- RST: GPIO7 (output)
- FPU available for quaternion math

#### GPIO Requirements

| Pin | Function  | Direction | Notes                    |
| --- | --------- | --------- | ------------------------ |
| SDA | I2C Data  | Bidir     | I2C0 default             |
| SCL | I2C Clock | Output    | I2C0 default             |
| INT | Interrupt | Input     | Active low, falling edge |
| RST | Reset     | Output    | Active low               |

## Alternatives Considered

1. **Use existing bno080 Rust crate**
   - Pros: Less development time
   - Cons: Blocking (not async), may not fit Embassy model

2. **Extend ImuSensor trait**
   - Pros: Single trait for all IMUs
   - Cons: Raw data methods don't apply to BNO086

3. **Full SHTP implementation**
   - Pros: Access to all BNO086 outputs
   - Cons: Complexity not needed for quaternion-only use case

Decision Rationale:

- New `QuaternionSensor` trait cleanly separates quaternion-native sensors
- Minimal SHTP implementation covers our needs (Rotation Vector only)
- Async design fits Embassy ecosystem

## Testing Strategy

### Unit Tests

- SHTP packet parsing with known byte sequences
- Rotation Vector report conversion to quaternion
- Q14/Q12 fixed-point to float conversion
- Error handling for malformed packets

### Integration Tests

- Driver initialization sequence
- Quaternion reading at 100Hz
- Error recovery (simulated timeout)
- RST pin toggle verification

### Hardware Tests

- WHO_AM_I verification on real sensor
- Quaternion accuracy vs known orientations
- Long-term stability (1 hour continuous operation)
- Recovery from power glitch

## Module Structure

```
src/
├── communication/
│   ├── mod.rs              # Existing (add shtp export)
│   ├── mavlink/            # Existing MAVLink protocol
│   └── shtp/
│       ├── mod.rs          # SHTP public API (ShtpPacket, ShtpChannel, ShtpError)
│       ├── transport.rs    # ShtpTransport trait
│       └── i2c.rs          # ShtpI2c implementation
│
├── devices/
│   ├── traits/
│   │   ├── imu.rs          # Existing ImuSensor trait
│   │   └── quaternion.rs   # NEW: QuaternionSensor trait
│   └── imu/
│       ├── mod.rs          # Add bno086 export
│       ├── bno086/
│       │   ├── mod.rs      # Driver public API
│       │   ├── reports.rs  # Rotation Vector parsing (BNO086-specific)
│       │   └── driver.rs   # Core driver (uses ShtpTransport)
│       ├── icm20948/       # Retained
│       ├── mpu9250/        # Retained
│       └── mock.rs         # Existing mock
```

**Separation of Concerns:**

- `src/communication/shtp/` - Generic SHTP protocol, reusable for any SHTP device
- `src/devices/imu/bno086/reports.rs` - BNO086-specific report parsing (Rotation Vector ID 0x05)
- `src/devices/imu/bno086/driver.rs` - BNO086 driver logic, composes `ShtpTransport`

## External References

- [BNO086 Datasheet](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- [SHTP Protocol Specification](https://cdn-learn.adafruit.com/assets/assets/000/076/762/original/1000-3925-Sensor-Hub-Transport-Protocol-v1.7.pdf)
- [Adafruit BNO085 Guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085)

## Open Questions

- [ ] Which specific BNO086 breakout board is being used? (Adafruit/SparkFun) → Method: Check pin compatibility
- [ ] Should we also expose Game Rotation Vector (no magnetometer)? → Decision: Start with Rotation Vector only, add later if needed

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
