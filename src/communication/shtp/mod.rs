//! SHTP (Sensor Hub Transport Protocol) Implementation
//!
//! Generic SHTP protocol module for Hillcrest/CEVA sensors including BNO080,
//! BNO085, BNO086, and FSM300 series. This module is sensor-independent and
//! can be reused for any SHTP-compatible device.
//!
//! # Protocol Overview
//!
//! SHTP is a packet-based protocol with the following structure:
//! - 4-byte header: length (2 bytes, LSB), channel (1 byte), sequence (1 byte)
//! - Variable payload (up to ~32KB, typically < 128 bytes)
//!
//! # Channels
//!
//! - Channel 0: Command - Host to sensor commands
//! - Channel 1: Executable - Firmware download
//! - Channel 2: Control - Sensor configuration
//! - Channel 3: Input Report - Sensor data to host
//! - Channel 4: Wake Input Report - Wake-on-motion data
//! - Channel 5: Gyro - High-rate gyroscope data
//!
//! # Transport Layers
//!
//! SHTP can operate over:
//! - I2C (default for BNO08x breakout boards)
//! - SPI (higher throughput, more pins)
//! - UART (rare)
//!
//! # References
//!
//! - [SHTP Protocol Specification v1.7](https://cdn-learn.adafruit.com/assets/assets/000/076/762/original/1000-3925-Sensor-Hub-Transport-Protocol-v1.7.pdf)
//!
//! # Usage
//!
//! ```ignore
//! use pico_trail::communication::shtp::{ShtpI2c, ShtpTransport, ShtpPacket};
//!
//! let mut transport = ShtpI2c::new(i2c, 0x4A);
//! let mut packet = ShtpPacket::<128>::new();
//! transport.read_packet(&mut packet).await?;
//! ```

mod i2c;
mod transport;

pub use i2c::ShtpI2c;
pub use transport::ShtpTransport;

/// Maximum payload size for SHTP packets
///
/// Most sensor reports are well under 128 bytes. The protocol supports
/// up to 32KB but larger buffers waste memory in embedded systems.
pub const MAX_PAYLOAD_SIZE: usize = 128;

/// SHTP header size in bytes
pub const HEADER_SIZE: usize = 4;

/// SHTP channel definitions
///
/// Channels define the type of data being transmitted. Each channel
/// maintains its own sequence number for ordering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
#[repr(u8)]
pub enum ShtpChannel {
    /// Command channel - Host to sensor commands
    Command = 0,
    /// Executable channel - Firmware download
    Executable = 1,
    /// Control channel - Sensor configuration (Set Feature, etc.)
    Control = 2,
    /// Input Report channel - Sensor data to host
    InputReport = 3,
    /// Wake Input Report channel - Wake-on-motion data
    WakeInputReport = 4,
    /// Gyro channel - High-rate gyroscope data
    Gyro = 5,
}

impl ShtpChannel {
    /// Convert from raw channel number
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Command),
            1 => Some(Self::Executable),
            2 => Some(Self::Control),
            3 => Some(Self::InputReport),
            4 => Some(Self::WakeInputReport),
            5 => Some(Self::Gyro),
            _ => None,
        }
    }
}

/// SHTP error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
pub enum ShtpError {
    /// Transport layer error (I2C, SPI, UART)
    TransportError,

    /// Invalid packet header (wrong magic, invalid length)
    InvalidHeader,

    /// Payload too large for buffer
    PayloadTooLarge,

    /// Sequence number mismatch (packet loss detected)
    SequenceMismatch,

    /// Timeout waiting for response
    Timeout,

    /// Channel not recognized
    InvalidChannel,

    /// No data available (polling mode without INT)
    NoData,
}

/// SHTP packet structure
///
/// Generic packet container with configurable buffer size.
/// Default size of 128 bytes is sufficient for most sensor reports.
///
/// # Header Format
///
/// ```text
/// Byte 0-1: Length (little-endian, includes header)
///           Bit 15 (continuation): 0 = first/only packet, 1 = continuation
/// Byte 2:   Channel number (0-5)
/// Byte 3:   Sequence number (per-channel)
/// ```
#[derive(Clone)]
pub struct ShtpPacket<const N: usize = MAX_PAYLOAD_SIZE> {
    /// Total packet length including header
    pub length: u16,
    /// Channel number
    pub channel: u8,
    /// Sequence number for this channel
    pub sequence: u8,
    /// Payload data (excluding header)
    pub payload: [u8; N],
    /// Actual payload length (may be less than N)
    pub payload_len: usize,
}

impl<const N: usize> Default for ShtpPacket<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ShtpPacket<N> {
    /// Create an empty packet
    pub const fn new() -> Self {
        Self {
            length: 0,
            channel: 0,
            sequence: 0,
            payload: [0u8; N],
            payload_len: 0,
        }
    }

    /// Create a packet with header values
    pub fn with_header(channel: ShtpChannel, sequence: u8) -> Self {
        Self {
            length: HEADER_SIZE as u16,
            channel: channel as u8,
            sequence,
            payload: [0u8; N],
            payload_len: 0,
        }
    }

    /// Get the channel as enum
    pub fn channel_type(&self) -> Option<ShtpChannel> {
        ShtpChannel::from_u8(self.channel)
    }

    /// Check if this is a continuation packet
    pub fn is_continuation(&self) -> bool {
        self.length & 0x8000 != 0
    }

    /// Get payload length (excluding header)
    pub fn payload_length(&self) -> usize {
        self.payload_len
    }

    /// Get payload slice
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }

    /// Get mutable payload slice
    pub fn payload_mut(&mut self) -> &mut [u8] {
        &mut self.payload[..self.payload_len]
    }

    /// Set payload data
    pub fn set_payload(&mut self, data: &[u8]) -> Result<(), ShtpError> {
        if data.len() > N {
            return Err(ShtpError::PayloadTooLarge);
        }
        self.payload[..data.len()].copy_from_slice(data);
        self.payload_len = data.len();
        self.length = (HEADER_SIZE + data.len()) as u16;
        Ok(())
    }

    /// Parse header from 4-byte buffer
    pub fn parse_header(&mut self, header: &[u8; 4]) -> Result<usize, ShtpError> {
        // Length is little-endian, bits 0-14 are length, bit 15 is continuation
        let raw_length = u16::from_le_bytes([header[0], header[1]]);
        self.length = raw_length;
        self.channel = header[2];
        self.sequence = header[3];

        // Payload length is total length minus header
        let total_len = (raw_length & 0x7FFF) as usize;
        if total_len < HEADER_SIZE {
            return Err(ShtpError::InvalidHeader);
        }

        let payload_len = total_len - HEADER_SIZE;
        if payload_len > N {
            return Err(ShtpError::PayloadTooLarge);
        }

        self.payload_len = payload_len;
        Ok(payload_len)
    }

    /// Build header into 4-byte buffer
    pub fn build_header(&self) -> [u8; 4] {
        let len_bytes = self.length.to_le_bytes();
        [len_bytes[0], len_bytes[1], self.channel, self.sequence]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shtp_channel_from_u8() {
        assert_eq!(ShtpChannel::from_u8(0), Some(ShtpChannel::Command));
        assert_eq!(ShtpChannel::from_u8(1), Some(ShtpChannel::Executable));
        assert_eq!(ShtpChannel::from_u8(2), Some(ShtpChannel::Control));
        assert_eq!(ShtpChannel::from_u8(3), Some(ShtpChannel::InputReport));
        assert_eq!(ShtpChannel::from_u8(4), Some(ShtpChannel::WakeInputReport));
        assert_eq!(ShtpChannel::from_u8(5), Some(ShtpChannel::Gyro));
        assert_eq!(ShtpChannel::from_u8(6), None);
        assert_eq!(ShtpChannel::from_u8(255), None);
    }

    #[test]
    fn test_shtp_packet_new() {
        let packet = ShtpPacket::<128>::new();
        assert_eq!(packet.length, 0);
        assert_eq!(packet.channel, 0);
        assert_eq!(packet.sequence, 0);
        assert_eq!(packet.payload_len, 0);
    }

    #[test]
    fn test_shtp_packet_with_header() {
        let packet = ShtpPacket::<128>::with_header(ShtpChannel::InputReport, 5);
        assert_eq!(packet.length, 4); // Header size
        assert_eq!(packet.channel, 3); // InputReport = 3
        assert_eq!(packet.sequence, 5);
        assert_eq!(packet.channel_type(), Some(ShtpChannel::InputReport));
    }

    #[test]
    fn test_shtp_packet_parse_header() {
        let mut packet = ShtpPacket::<128>::new();

        // Header: length=24 (0x0018), channel=3, sequence=7
        let header = [0x18, 0x00, 0x03, 0x07];
        let payload_len = packet.parse_header(&header).unwrap();

        assert_eq!(packet.length, 24);
        assert_eq!(packet.channel, 3);
        assert_eq!(packet.sequence, 7);
        assert_eq!(payload_len, 20); // 24 - 4 header bytes
    }

    #[test]
    fn test_shtp_packet_parse_header_continuation() {
        let mut packet = ShtpPacket::<128>::new();

        // Header with continuation bit set: length=0x8018 (bit 15 set)
        let header = [0x18, 0x80, 0x03, 0x07];
        packet.parse_header(&header).unwrap();

        assert!(packet.is_continuation());
    }

    #[test]
    fn test_shtp_packet_parse_header_invalid() {
        let mut packet = ShtpPacket::<128>::new();

        // Length less than header size (invalid)
        let header = [0x02, 0x00, 0x03, 0x07];
        let result = packet.parse_header(&header);
        assert_eq!(result, Err(ShtpError::InvalidHeader));
    }

    #[test]
    fn test_shtp_packet_payload_too_large() {
        let mut packet = ShtpPacket::<16>::new();

        // Length=100 bytes, but buffer is only 16
        let header = [0x64, 0x00, 0x03, 0x07];
        let result = packet.parse_header(&header);
        assert_eq!(result, Err(ShtpError::PayloadTooLarge));
    }

    #[test]
    fn test_shtp_packet_set_payload() {
        let mut packet = ShtpPacket::<128>::with_header(ShtpChannel::Control, 0);

        let data = [0x01, 0x02, 0x03, 0x04];
        packet.set_payload(&data).unwrap();

        assert_eq!(packet.payload_len, 4);
        assert_eq!(packet.length, 8); // 4 header + 4 payload
        assert_eq!(packet.payload(), &data);
    }

    #[test]
    fn test_shtp_packet_build_header() {
        let mut packet = ShtpPacket::<128>::with_header(ShtpChannel::Control, 3);
        packet.set_payload(&[0x01, 0x02]).unwrap();

        let header = packet.build_header();
        assert_eq!(header, [0x06, 0x00, 0x02, 0x03]); // length=6, channel=2, seq=3
    }

    #[test]
    fn test_shtp_error_variants() {
        let errors = [
            ShtpError::TransportError,
            ShtpError::InvalidHeader,
            ShtpError::PayloadTooLarge,
            ShtpError::SequenceMismatch,
            ShtpError::Timeout,
            ShtpError::InvalidChannel,
        ];

        for (i, e1) in errors.iter().enumerate() {
            for (j, e2) in errors.iter().enumerate() {
                if i == j {
                    assert_eq!(e1, e2);
                } else {
                    assert_ne!(e1, e2);
                }
            }
        }
    }
}
