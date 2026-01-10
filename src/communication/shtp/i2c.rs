//! SHTP over I2C Transport
//!
//! I2C implementation of the SHTP transport layer for BNO08x sensors.
//!
//! # I2C Protocol Notes
//!
//! BNO08x sensors use I2C with the following characteristics:
//! - Default address: 0x4A (SA0 high) or 0x4B (SA0 low)
//! - Speed: Up to 400kHz (Fast Mode)
//! - Clock stretching: Required (sensor may hold SCL low)
//!
//! # Read Operation
//!
//! 1. Send START + address + READ
//! 2. Read 4-byte header
//! 3. Parse length from header
//! 4. Continue reading payload bytes
//! 5. Send STOP
//!
//! Note: Unlike typical I2C devices, BNO08x does NOT use register addresses.
//! All reads start from the beginning of the output buffer.
//!
//! # Write Operation
//!
//! 1. Send START + address + WRITE
//! 2. Write 4-byte header
//! 3. Write payload bytes
//! 4. Send STOP

#[cfg(feature = "embassy")]
use super::HEADER_SIZE;
use super::{ShtpError, ShtpPacket, ShtpTransport};

/// Number of SHTP channels to track sequence numbers for
const NUM_CHANNELS: usize = 6;

/// SHTP over I2C transport
///
/// Implements `ShtpTransport` for I2C communication with SHTP devices.
///
/// # Type Parameters
///
/// * `I2C` - I2C bus type implementing `embedded_hal_async::i2c::I2c`
///
/// # Example
///
/// ```ignore
/// use pico_trail::communication::shtp::ShtpI2c;
///
/// let transport = ShtpI2c::new(i2c, 0x4A);
/// ```
pub struct ShtpI2c<I2C> {
    /// I2C bus instance
    i2c: I2C,
    /// Device I2C address (7-bit)
    address: u8,
    /// Per-channel sequence numbers (for tracking)
    sequence: [u8; NUM_CHANNELS],
}

impl<I2C> ShtpI2c<I2C> {
    /// Default BNO08x I2C address (SA0 = high)
    pub const DEFAULT_ADDRESS: u8 = 0x4A;

    /// Alternate BNO08x I2C address (SA0 = low)
    pub const ALTERNATE_ADDRESS: u8 = 0x4B;

    /// Create a new SHTP I2C transport
    ///
    /// # Arguments
    ///
    /// * `i2c` - I2C bus instance
    /// * `address` - Device I2C address (typically 0x4A or 0x4B)
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            sequence: [0u8; NUM_CHANNELS],
        }
    }

    /// Get the I2C address
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Get the expected sequence number for a channel
    pub fn expected_sequence(&self, channel: u8) -> Option<u8> {
        if (channel as usize) < NUM_CHANNELS {
            Some(self.sequence[channel as usize])
        } else {
            None
        }
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[cfg(feature = "embassy")]
impl<I2C> ShtpTransport for ShtpI2c<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    async fn read_packet<const N: usize>(
        &mut self,
        packet: &mut ShtpPacket<N>,
    ) -> Result<(), ShtpError> {
        // BNO08x I2C protocol requires reading the ENTIRE packet in ONE transaction
        // Each I2C read starts from the beginning of the sensor's output buffer
        // So we cannot read header first, then payload - we must read everything at once

        // Read header + max payload in one transaction
        // The packet buffer is [header (4 bytes)][payload (N bytes)]
        let mut buf = [0u8; 132]; // 4 + 128 max payload
        let read_len = buf.len().min(HEADER_SIZE + N);

        self.i2c
            .read(self.address, &mut buf[..read_len])
            .await
            .map_err(|_| ShtpError::TransportError)?;

        // Check for "no data" response (length = 0 or all 0xFF)
        // When INT is not connected, sensor may return empty data
        let raw_length = u16::from_le_bytes([buf[0], buf[1]]) & 0x7FFF;
        if raw_length == 0 || buf[0] == 0xFF {
            // No data available - return empty packet indicator
            return Err(ShtpError::NoData);
        }

        // Parse header from first 4 bytes
        let header: [u8; HEADER_SIZE] = [buf[0], buf[1], buf[2], buf[3]];
        let payload_len = packet.parse_header(&header)?;

        // Copy payload from buffer to packet
        if payload_len > 0 && payload_len <= N {
            packet.payload[..payload_len]
                .copy_from_slice(&buf[HEADER_SIZE..HEADER_SIZE + payload_len]);
        }

        // Update expected sequence number for this channel
        if (packet.channel as usize) < NUM_CHANNELS {
            self.sequence[packet.channel as usize] = packet.sequence.wrapping_add(1);
        }

        Ok(())
    }

    async fn write_packet<const N: usize>(
        &mut self,
        packet: &ShtpPacket<N>,
    ) -> Result<(), ShtpError> {
        // Build header
        let header = packet.build_header();

        // Combine header and payload into single buffer for single I2C transaction
        let mut buf = [0u8; 132]; // 4 + 128 max payload
        buf[..HEADER_SIZE].copy_from_slice(&header);

        let total_len = HEADER_SIZE + packet.payload_len;
        if packet.payload_len > 0 {
            buf[HEADER_SIZE..total_len].copy_from_slice(&packet.payload[..packet.payload_len]);
        }

        self.i2c
            .write(self.address, &buf[..total_len])
            .await
            .map_err(|_| ShtpError::TransportError)?;

        Ok(())
    }

    fn reset(&mut self) {
        self.sequence = [0u8; NUM_CHANNELS];
    }
}

// Non-embassy implementation for host tests
#[cfg(not(feature = "embassy"))]
impl<I2C> ShtpTransport for ShtpI2c<I2C> {
    async fn read_packet<const N: usize>(
        &mut self,
        _packet: &mut ShtpPacket<N>,
    ) -> Result<(), ShtpError> {
        // Stub for host tests - actual I2C not available
        Err(ShtpError::TransportError)
    }

    async fn write_packet<const N: usize>(
        &mut self,
        _packet: &ShtpPacket<N>,
    ) -> Result<(), ShtpError> {
        // Stub for host tests - actual I2C not available
        Err(ShtpError::TransportError)
    }

    fn reset(&mut self) {
        self.sequence = [0u8; NUM_CHANNELS];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock I2C for testing
    struct MockI2c;

    #[test]
    fn test_shtp_i2c_new() {
        let transport = ShtpI2c::new(MockI2c, 0x4A);
        assert_eq!(transport.address(), 0x4A);
        assert_eq!(transport.expected_sequence(0), Some(0));
        assert_eq!(transport.expected_sequence(5), Some(0));
        assert_eq!(transport.expected_sequence(6), None);
    }

    #[test]
    fn test_shtp_i2c_addresses() {
        assert_eq!(ShtpI2c::<MockI2c>::DEFAULT_ADDRESS, 0x4A);
        assert_eq!(ShtpI2c::<MockI2c>::ALTERNATE_ADDRESS, 0x4B);
    }

    #[test]
    fn test_shtp_i2c_reset() {
        let mut transport = ShtpI2c::new(MockI2c, 0x4A);
        // Manually set some sequence numbers
        transport.sequence[0] = 5;
        transport.sequence[3] = 10;

        transport.reset();

        assert_eq!(transport.expected_sequence(0), Some(0));
        assert_eq!(transport.expected_sequence(3), Some(0));
    }

    #[test]
    fn test_shtp_i2c_release() {
        let transport = ShtpI2c::new(MockI2c, 0x4A);
        let _i2c = transport.release();
        // MockI2c doesn't implement anything, just verify it compiles
    }
}
