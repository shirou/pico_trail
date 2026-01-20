//! SHTP Transport Layer Abstraction
//!
//! This module defines the `ShtpTransport` trait for reading and writing
//! SHTP packets over various physical layers (I2C, SPI, UART).
//!
//! # Implementation Notes
//!
//! Each transport implementation is responsible for:
//! - Managing physical layer communication
//! - Handling packet framing (header + payload)
//! - Tracking per-channel sequence numbers
//! - Detecting and reporting transport errors

use super::{ShtpError, ShtpPacket};

/// SHTP transport layer trait
///
/// Implement this trait for each physical transport (I2C, SPI, UART).
/// The trait provides async packet-level read/write operations.
///
/// # Sequence Number Management
///
/// Each channel maintains an independent sequence number. Implementations
/// should track sequence numbers internally and detect mismatches.
///
/// # Error Handling
///
/// All errors are mapped to `ShtpError` variants:
/// - Physical layer errors → `ShtpError::TransportError`
/// - Invalid headers → `ShtpError::InvalidHeader`
/// - Buffer overflow → `ShtpError::PayloadTooLarge`
#[allow(async_fn_in_trait)]
pub trait ShtpTransport {
    /// Read a complete SHTP packet
    ///
    /// This method:
    /// 1. Reads the 4-byte header
    /// 2. Parses length and channel
    /// 3. Reads the payload bytes
    /// 4. Updates the packet structure
    ///
    /// # Arguments
    ///
    /// * `packet` - Mutable reference to packet buffer
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Packet read successfully
    /// * `Err(ShtpError::TransportError)` - Physical layer error
    /// * `Err(ShtpError::InvalidHeader)` - Malformed header
    /// * `Err(ShtpError::PayloadTooLarge)` - Payload exceeds buffer
    async fn read_packet<const N: usize>(
        &mut self,
        packet: &mut ShtpPacket<N>,
    ) -> Result<(), ShtpError>;

    /// Write a complete SHTP packet
    ///
    /// This method:
    /// 1. Builds the 4-byte header from packet fields
    /// 2. Writes header followed by payload
    ///
    /// # Arguments
    ///
    /// * `packet` - Reference to packet to write
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Packet written successfully
    /// * `Err(ShtpError::TransportError)` - Physical layer error
    async fn write_packet<const N: usize>(
        &mut self,
        packet: &ShtpPacket<N>,
    ) -> Result<(), ShtpError>;

    /// Reset the transport layer
    ///
    /// Called to reset sequence numbers and clear any pending state.
    /// Useful after sensor hardware reset.
    fn reset(&mut self);
}
