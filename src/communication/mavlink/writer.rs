//! MAVLink Message Writer
//!
//! Async message serialization and writing to UART using rust-mavlink library.
//!
//! # Architecture
//!
//! - Serializes MAVLink messages to bytes using rust-mavlink
//! - Writes bytes to UART asynchronously
//! - Tracks statistics (messages sent, buffer overflows)
//! - Handles buffer full conditions gracefully (drop message with warning)
//!
//! # Buffer Management
//!
//! - TX buffer: 1024 bytes (heapless::Vec for no_std)
//! - Sufficient for queuing multiple messages before UART transmission

use heapless::Vec;
use mavlink;

/// Maximum size of transmit buffer (bytes)
pub const TX_BUFFER_SIZE: usize = 1024;

/// Writer statistics for monitoring and diagnostics
#[derive(Debug, Clone, Copy, Default)]
pub struct WriterStats {
    /// Total messages successfully sent
    pub messages_sent: u32,
    /// Buffer overflow events (message dropped due to full buffer)
    pub buffer_overflows: u32,
    /// UART write errors
    pub write_errors: u32,
}

/// MAVLink message writer
///
/// Wraps rust-mavlink's serialization functionality with async UART writing
/// and statistics tracking.
pub struct MavlinkWriter {
    /// TX buffer for outgoing bytes
    tx_buffer: Vec<u8, TX_BUFFER_SIZE>,
    /// System ID (our autopilot ID)
    system_id: u8,
    /// Component ID (autopilot component)
    component_id: u8,
    /// Message sequence counter
    sequence: u8,
    /// Writer statistics
    stats: WriterStats,
}

impl MavlinkWriter {
    /// Create a new MAVLink writer
    ///
    /// # Arguments
    ///
    /// * `system_id` - MAVLink system ID (default: 1)
    /// * `component_id` - MAVLink component ID (default: 1 for autopilot)
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            tx_buffer: Vec::new(),
            system_id,
            component_id,
            sequence: 0,
            stats: WriterStats::default(),
        }
    }

    /// Get writer statistics
    pub fn stats(&self) -> WriterStats {
        self.stats
    }

    /// Reset writer statistics
    pub fn reset_stats(&mut self) {
        self.stats = WriterStats::default();
    }

    /// Get current sequence number
    pub fn sequence(&self) -> u8 {
        self.sequence
    }

    /// Write a MAVLink 2.0 message to UART
    ///
    /// This function serializes a MAVLink message and writes it to the UART
    /// interface asynchronously.
    ///
    /// # Arguments
    ///
    /// * `writer` - UART interface implementing `embedded_io_async::Write`
    /// * `message` - MAVLink message to send
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success, or `Err` on:
    /// - Buffer overflow (message dropped)
    /// - UART write errors
    /// - Serialization errors
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use pico_trail::communication::mavlink::writer::MavlinkWriter;
    /// use mavlink::common::MavMessage;
    ///
    /// let mut writer = MavlinkWriter::new(1, 1);
    /// let heartbeat = MavMessage::HEARTBEAT(/* ... */);
    /// match writer.write_message(&mut uart, &heartbeat).await {
    ///     Ok(()) => {
    ///         defmt::debug!("Message sent");
    ///     }
    ///     Err(e) => {
    ///         defmt::warn!("Write error: {:?}", e);
    ///     }
    /// }
    /// ```
    pub async fn write_message<W>(
        &mut self,
        writer: &mut W,
        message: &mavlink::common::MavMessage,
    ) -> Result<(), WriterError>
    where
        W: embedded_io_async::Write,
    {
        // Create MAVLink header
        let header = mavlink::MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: self.sequence,
        };

        // Increment sequence for next message
        self.sequence = self.sequence.wrapping_add(1);

        // Write message using rust-mavlink's async write function
        match mavlink::write_v2_msg_async(writer, header, message).await {
            Ok(_bytes_written) => {
                // Update statistics
                self.stats.messages_sent += 1;
                Ok(())
            }
            Err(_e) => {
                // Update error statistics
                self.stats.buffer_overflows += 1;

                // Convert rust-mavlink error to WriterError
                Err(WriterError::UartError)
            }
        }
    }

    /// Serialize a message into the TX buffer
    ///
    /// Helper function that serializes a MAVLink message into bytes.
    ///
    /// # Arguments
    ///
    /// * `header` - MAVLink header
    /// * `message` - MAVLink message
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` if serialization succeeds, or `Err` if buffer is full.
    #[allow(dead_code)]
    fn _serialize_message(
        &mut self,
        _header: &mavlink::MavHeader,
        _message: &mavlink::common::MavMessage,
    ) -> Result<(), WriterError> {
        // Check if buffer has enough space
        // Largest common MAVLink message is ~280 bytes
        if self.tx_buffer.len() + 280 > TX_BUFFER_SIZE {
            self.stats.buffer_overflows += 1;
            return Err(WriterError::BufferOverflow);
        }

        // Serialize message using rust-mavlink
        // This is a placeholder - actual implementation will use rust-mavlink's
        // write_v2_msg() with a custom Write implementation.

        Ok(())
    }
}

/// Writer error types
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum WriterError {
    /// Buffer overflow (TX buffer full)
    BufferOverflow,
    /// UART write error
    UartError,
    /// Serialization error
    SerializationError,
}

impl core::fmt::Display for WriterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WriterError::BufferOverflow => write!(f, "TX buffer overflow"),
            WriterError::UartError => write!(f, "UART write error"),
            WriterError::SerializationError => write!(f, "Message serialization failed"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_writer_creation() {
        let writer = MavlinkWriter::new(1, 1);
        assert_eq!(writer.sequence(), 0);
        assert_eq!(writer.stats().messages_sent, 0);
        assert_eq!(writer.stats().buffer_overflows, 0);
    }

    #[test]
    fn test_sequence_increment() {
        let mut writer = MavlinkWriter::new(1, 1);
        assert_eq!(writer.sequence(), 0);

        // Simulate sending messages
        writer.sequence = writer.sequence.wrapping_add(1);
        assert_eq!(writer.sequence(), 1);

        writer.sequence = writer.sequence.wrapping_add(1);
        assert_eq!(writer.sequence(), 2);
    }

    #[test]
    fn test_sequence_wrap() {
        let mut writer = MavlinkWriter::new(1, 1);
        writer.sequence = 255;

        // Next increment should wrap to 0
        writer.sequence = writer.sequence.wrapping_add(1);
        assert_eq!(writer.sequence(), 0);
    }

    #[test]
    fn test_stats_reset() {
        let mut writer = MavlinkWriter::new(1, 1);
        writer.stats.messages_sent = 100;
        writer.stats.buffer_overflows = 5;
        writer.reset_stats();
        assert_eq!(writer.stats().messages_sent, 0);
        assert_eq!(writer.stats().buffer_overflows, 0);
    }

    #[test]
    fn test_buffer_overflow_detection() {
        let mut writer = MavlinkWriter::new(1, 1);

        // Fill buffer to near capacity
        for _ in 0..(TX_BUFFER_SIZE - 100) {
            let _ = writer.tx_buffer.push(0x55);
        }

        // Try to serialize a large message (should fail due to insufficient space)
        let header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };
        let message =
            mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA::default());

        let result = writer._serialize_message(&header, &message);
        assert!(matches!(result, Err(WriterError::BufferOverflow)));
        assert_eq!(writer.stats().buffer_overflows, 1);
    }
}
