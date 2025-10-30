//! MAVLink Message Parser
//!
//! Async message parsing from UART using rust-mavlink library.
//!
//! # Architecture
//!
//! - Reads bytes from UART asynchronously
//! - Uses rust-mavlink's `read_v2_msg()` for MAVLink 2.0 parsing
//! - Tracks statistics (messages received, parse errors)
//! - Handles incomplete messages and CRC failures gracefully
//!
//! # Buffer Management
//!
//! - RX buffer: 512 bytes (heapless::Vec for no_std)
//! - Sufficient for largest common MAVLink messages (~280 bytes)

use heapless::Vec;
use mavlink;

/// Maximum size of receive buffer (bytes)
pub const RX_BUFFER_SIZE: usize = 512;

/// Parser statistics for monitoring and diagnostics
#[derive(Debug, Clone, Copy, Default)]
pub struct ParserStats {
    /// Total messages successfully parsed
    pub messages_received: u32,
    /// Parse errors (CRC failures, malformed messages)
    pub parse_errors: u32,
    /// Buffer overflow events (message too large)
    pub buffer_overflows: u32,
}

/// MAVLink message parser
///
/// Wraps rust-mavlink's parsing functionality with async UART reading
/// and statistics tracking.
pub struct MavlinkParser {
    /// RX buffer for incoming bytes
    rx_buffer: Vec<u8, RX_BUFFER_SIZE>,
    /// Parser statistics
    stats: ParserStats,
}

impl MavlinkParser {
    /// Create a new MAVLink parser
    pub fn new() -> Self {
        Self {
            rx_buffer: Vec::new(),
            stats: ParserStats::default(),
        }
    }

    /// Get parser statistics
    pub fn stats(&self) -> ParserStats {
        self.stats
    }

    /// Reset parser statistics
    pub fn reset_stats(&mut self) {
        self.stats = ParserStats::default();
    }

    /// Read a single MAVLink 2.0 message from UART
    ///
    /// This function reads bytes from the UART interface asynchronously and
    /// parses them into a MAVLink message using rust-mavlink's parser.
    ///
    /// # Arguments
    ///
    /// * `uart` - UART interface implementing `embedded_io_async::Read`
    ///
    /// # Returns
    ///
    /// Returns `Ok((header, message))` on successful parse, or `Err` on:
    /// - Parse errors (CRC failure, malformed message)
    /// - UART read errors
    /// - Buffer overflow (message too large)
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use pico_trail::communication::mavlink::parser::MavlinkParser;
    ///
    /// let mut parser = MavlinkParser::new();
    /// match parser.read_message(&mut uart).await {
    ///     Ok((header, msg)) => {
    ///         // Process message
    ///     }
    ///     Err(e) => {
    ///         defmt::warn!("Parse error: {:?}", e);
    ///     }
    /// }
    /// ```
    pub async fn read_message<R>(
        &mut self,
        _reader: &mut R,
    ) -> Result<(mavlink::MavHeader, mavlink::common::MavMessage), ParserError>
    where
        R: embedded_io_async::Read,
    {
        // Clear buffer for new message
        self.rx_buffer.clear();

        // Read message using rust-mavlink parser
        // Note: This is a simplified implementation. The actual implementation
        // will need to integrate with rust-mavlink's async reading or implement
        // a byte-by-byte reading loop.
        //
        // For now, we'll implement a placeholder that demonstrates the structure.
        // The actual implementation will be completed when integrating with UART.

        // Placeholder: Increment error count for unimplemented functionality
        self.stats.parse_errors += 1;
        Err(ParserError::NotImplemented)
    }

    /// Process a single byte from UART
    ///
    /// Helper function for byte-by-byte parsing. Accumulates bytes in buffer
    /// until a complete message is received.
    ///
    /// # Arguments
    ///
    /// * `byte` - Single byte from UART
    ///
    /// # Returns
    ///
    /// Returns `Some((header, message))` when a complete message is parsed,
    /// or `None` if more bytes are needed.
    #[allow(dead_code)]
    fn _process_byte(
        &mut self,
        byte: u8,
    ) -> Option<Result<(mavlink::MavHeader, mavlink::common::MavMessage), ParserError>> {
        // Add byte to buffer
        if self.rx_buffer.push(byte).is_err() {
            self.stats.buffer_overflows += 1;
            return Some(Err(ParserError::BufferOverflow));
        }

        // Try to parse message from buffer
        // This is a placeholder - actual implementation will use rust-mavlink's
        // read_v2_msg() with a custom Read implementation.
        None
    }
}

impl Default for MavlinkParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Parser error types
#[derive(Debug, Clone, Copy)]
pub enum ParserError {
    /// CRC validation failed
    CrcError,
    /// Message format invalid
    MalformedMessage,
    /// Buffer overflow (message too large)
    BufferOverflow,
    /// UART read error
    UartError,
    /// Not implemented yet (placeholder)
    NotImplemented,
}

impl core::fmt::Display for ParserError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ParserError::CrcError => write!(f, "CRC validation failed"),
            ParserError::MalformedMessage => write!(f, "Malformed MAVLink message"),
            ParserError::BufferOverflow => write!(f, "RX buffer overflow"),
            ParserError::UartError => write!(f, "UART read error"),
            ParserError::NotImplemented => write!(f, "Parser not fully implemented"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parser_creation() {
        let parser = MavlinkParser::new();
        assert_eq!(parser.stats().messages_received, 0);
        assert_eq!(parser.stats().parse_errors, 0);
    }

    #[test]
    fn test_stats_reset() {
        let mut parser = MavlinkParser::new();
        parser.stats.messages_received = 10;
        parser.stats.parse_errors = 2;
        parser.reset_stats();
        assert_eq!(parser.stats().messages_received, 0);
        assert_eq!(parser.stats().parse_errors, 0);
    }

    #[test]
    fn test_buffer_overflow() {
        let mut parser = MavlinkParser::new();
        // Fill buffer to capacity
        for _ in 0..RX_BUFFER_SIZE {
            let _ = parser._process_byte(0x55);
        }
        // Next byte should trigger overflow
        let result = parser._process_byte(0x55);
        assert!(matches!(result, Some(Err(ParserError::BufferOverflow))));
        assert_eq!(parser.stats().buffer_overflows, 1);
    }
}
