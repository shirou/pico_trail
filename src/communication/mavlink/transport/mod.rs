//! MAVLink Transport Abstraction
//!
//! This module provides a trait-based abstraction for MAVLink communication transports,
//! enabling support for multiple transport types (UART, UDP, TCP) with a unified interface.
//!
//! # Architecture
//!
//! The transport layer follows a trait-based design inspired by ArduPilot's `AP_HAL::UARTDriver`:
//!
//! - **Trait-based abstraction**: `MavlinkTransport` trait defines the interface
//! - **Zero-cost**: Monomorphization eliminates runtime overhead
//! - **Async-first**: All operations use Embassy async primitives
//! - **Transport-agnostic**: Protocol layer doesn't know about specific transports
//!
//! # Design Pattern
//!
//! ```text
//! ┌──────────────────────────────────────┐
//! │     MAVLink Protocol Layer           │
//! │  (parser, writer, message handlers)  │
//! └──────────┬───────────────────────────┘
//!            │
//!            │ Uses MavlinkTransport trait
//!            │
//!            ▼
//! ┌──────────────────────────────────────┐
//! │      Transport Router                 │
//! │  (manages multiple transports)        │
//! └──────────┬───────────────────────────┘
//!            │
//!            │ Holds Vec<dyn MavlinkTransport>
//!            │
//!            ▼
//! ┌─────────────────┬────────────────┬───────────────┐
//! │  UartTransport  │  UdpTransport  │ TcpTransport  │
//! │   (UART I/O)    │  (UDP socket)  │ (TCP socket)  │
//! └─────────────────┴────────────────┴───────────────┘
//! ```
//!
//! # Example Usage
//!
//! ```ignore
//! use pico_trail::communication::mavlink::transport::{MavlinkTransport, uart::UartTransport};
//!
//! async fn send_heartbeat(transport: &mut impl MavlinkTransport) {
//!     let data = [/* MAVLink HEARTBEAT message */];
//!     transport.write(&data).await.unwrap();
//!     transport.flush().await.unwrap();
//! }
//! ```
//!
//! # Modules
//!
//! - `uart` - UART transport implementation (Phase 1)
//! - `udp` - UDP transport implementation (Phase 2)
//! - `tcp` - TCP transport implementation (future)

pub mod uart;
pub mod udp;

use core::fmt;

/// Transport abstraction for MAVLink communication
///
/// This trait defines the interface for all MAVLink transports (UART, UDP, TCP).
/// Implementations must provide async read/write operations compatible with
/// Embassy's async executor.
///
/// # Design Rationale
///
/// - **Async operations**: Non-blocking I/O essential for concurrent transports
/// - **Byte-oriented**: MAVLink is a byte-stream protocol
/// - **Error handling**: Transports can fail, timeout, or disconnect
/// - **Buffering**: Implementations may buffer internally (e.g., UART DMA)
///
/// # Trait Methods
///
/// - `available()`: Check if data is ready to read (non-blocking check)
/// - `read()`: Read bytes into buffer (blocking until data available)
/// - `write()`: Write bytes from buffer (may buffer internally)
/// - `flush()`: Ensure all buffered writes are transmitted
///
/// # Implementation Guidelines
///
/// - `read()` should block until at least 1 byte is available or error occurs
/// - `write()` may buffer data; call `flush()` to ensure transmission
/// - `available()` is optional optimization; return 0 if unsupported
/// - Errors should be specific: IoError (generic), Timeout, Disconnected
#[allow(async_fn_in_trait)]
pub trait MavlinkTransport {
    /// Returns number of bytes available to read without blocking
    ///
    /// This is an optional optimization hint. Implementations that don't track
    /// available bytes can return 0.
    ///
    /// # Returns
    ///
    /// Number of bytes available in receive buffer, or 0 if unknown/unsupported
    async fn available(&self) -> usize;

    /// Read bytes from transport into buffer
    ///
    /// Blocks until at least 1 byte is read or an error occurs.
    /// May read fewer bytes than buffer size if less data is available.
    ///
    /// # Arguments
    ///
    /// * `buf` - Buffer to read into
    ///
    /// # Returns
    ///
    /// - `Ok(n)` - Number of bytes read (1 ≤ n ≤ buf.len())
    /// - `Err(TransportError)` - I/O error, timeout, or disconnection
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let mut buf = [0u8; 256];
    /// let n = transport.read(&mut buf).await?;
    /// // Process buf[..n]
    /// ```
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Write bytes from buffer to transport
    ///
    /// May buffer data internally. Call `flush()` to ensure transmission.
    ///
    /// # Arguments
    ///
    /// * `buf` - Data to write
    ///
    /// # Returns
    ///
    /// - `Ok(n)` - Number of bytes written (typically buf.len())
    /// - `Err(TransportError)` - I/O error or disconnection
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let data = [0xFE, 0x09, 0x00, /* ... */];
    /// transport.write(&data).await?;
    /// transport.flush().await?;
    /// ```
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError>;

    /// Flush any buffered writes to ensure transmission
    ///
    /// Blocks until all buffered data has been transmitted or an error occurs.
    /// Some transports (e.g., UDP) may not need explicit flushing.
    ///
    /// # Returns
    ///
    /// - `Ok(())` - All data transmitted
    /// - `Err(TransportError)` - Flush failed
    async fn flush(&mut self) -> Result<(), TransportError>;
}

/// Transport error types
///
/// Categorizes transport failures for appropriate error handling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransportError {
    /// Generic I/O error
    ///
    /// Use this for unclassified errors that don't fit other categories.
    /// Examples: hardware failure, driver error, buffer overflow
    IoError,

    /// Operation timed out
    ///
    /// The operation didn't complete within the expected time.
    /// Examples: WiFi connection timeout, UART read timeout
    Timeout,

    /// Transport disconnected
    ///
    /// The transport is no longer available for communication.
    /// Examples: UART cable unplugged, WiFi disconnected, TCP connection closed
    Disconnected,
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransportError::IoError => write!(f, "I/O error"),
            TransportError::Timeout => write!(f, "Operation timed out"),
            TransportError::Disconnected => write!(f, "Transport disconnected"),
        }
    }
}

// Note: TransportError implements Display, sufficient for embedded use

#[cfg(test)]
pub mod mock {
    //! Mock transport implementation for testing
    //!
    //! Provides a configurable mock transport that can simulate various
    //! transport behaviors for testing router logic, error handling, etc.

    use super::{MavlinkTransport, TransportError};
    use heapless::Vec;

    /// Mock transport for testing
    ///
    /// Can be configured to:
    /// - Return specific data on reads
    /// - Simulate read/write errors
    /// - Track write operations
    pub struct MockTransport {
        /// Data to return from read() operations
        pub read_data: Vec<u8, 256>,
        /// Current read position
        read_pos: usize,
        /// Data written via write() operations
        pub write_buffer: Vec<u8, 256>,
        /// Error to return from read()
        pub read_error: Option<TransportError>,
        /// Error to return from write()
        pub write_error: Option<TransportError>,
        /// Error to return from flush()
        pub flush_error: Option<TransportError>,
        /// Number of bytes available (for available() method)
        pub available_bytes: usize,
    }

    impl MockTransport {
        /// Create new mock transport
        pub fn new() -> Self {
            Self {
                read_data: Vec::new(),
                read_pos: 0,
                write_buffer: Vec::new(),
                read_error: None,
                write_error: None,
                flush_error: None,
                available_bytes: 0,
            }
        }

        /// Set data to return from read()
        pub fn set_read_data(&mut self, data: &[u8]) {
            self.read_data.clear();
            self.read_data.extend_from_slice(data).unwrap();
            self.read_pos = 0;
            self.available_bytes = data.len();
        }

        /// Set error to return from read()
        pub fn set_read_error(&mut self, error: TransportError) {
            self.read_error = Some(error);
        }

        /// Set error to return from write()
        pub fn set_write_error(&mut self, error: TransportError) {
            self.write_error = Some(error);
        }

        /// Set error to return from flush()
        pub fn set_flush_error(&mut self, error: TransportError) {
            self.flush_error = Some(error);
        }

        /// Get data written to transport
        pub fn written_data(&self) -> &[u8] {
            &self.write_buffer
        }

        /// Clear write buffer
        pub fn clear_write_buffer(&mut self) {
            self.write_buffer.clear();
        }

        /// Reset mock to initial state
        pub fn reset(&mut self) {
            self.read_data.clear();
            self.read_pos = 0;
            self.write_buffer.clear();
            self.read_error = None;
            self.write_error = None;
            self.flush_error = None;
            self.available_bytes = 0;
        }
    }

    impl Default for MockTransport {
        fn default() -> Self {
            Self::new()
        }
    }

    impl MavlinkTransport for MockTransport {
        async fn available(&self) -> usize {
            self.available_bytes
        }

        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
            if let Some(error) = self.read_error {
                return Err(error);
            }

            if self.read_pos >= self.read_data.len() {
                return Err(TransportError::Timeout);
            }

            let remaining = &self.read_data[self.read_pos..];
            let to_read = core::cmp::min(buf.len(), remaining.len());
            buf[..to_read].copy_from_slice(&remaining[..to_read]);
            self.read_pos += to_read;
            self.available_bytes = self.available_bytes.saturating_sub(to_read);
            Ok(to_read)
        }

        async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
            if let Some(error) = self.write_error {
                return Err(error);
            }

            self.write_buffer
                .extend_from_slice(buf)
                .map_err(|_| TransportError::IoError)?;
            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), TransportError> {
            if let Some(error) = self.flush_error {
                return Err(error);
            }

            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mock::MockTransport;

    #[tokio::test]
    async fn test_mock_transport_read() {
        let mut transport = MockTransport::new();
        transport.set_read_data(b"Hello");

        let mut buf = [0u8; 32];
        let n = transport.read(&mut buf).await.unwrap();
        assert_eq!(n, 5);
        assert_eq!(&buf[..n], b"Hello");
    }

    #[tokio::test]
    async fn test_mock_transport_write() {
        let mut transport = MockTransport::new();
        let data = b"Test message";

        transport.write(data).await.unwrap();
        assert_eq!(transport.written_data(), data);
    }

    #[tokio::test]
    async fn test_mock_transport_read_error() {
        let mut transport = MockTransport::new();
        transport.set_read_error(TransportError::Timeout);

        let mut buf = [0u8; 32];
        let result = transport.read(&mut buf).await;
        assert_eq!(result, Err(TransportError::Timeout));
    }

    #[tokio::test]
    async fn test_mock_transport_write_error() {
        let mut transport = MockTransport::new();
        transport.set_write_error(TransportError::Disconnected);

        let result = transport.write(b"test").await;
        assert_eq!(result, Err(TransportError::Disconnected));
    }

    #[tokio::test]
    async fn test_mock_transport_flush_error() {
        let mut transport = MockTransport::new();
        transport.set_flush_error(TransportError::IoError);

        let result = transport.flush().await;
        assert_eq!(result, Err(TransportError::IoError));
    }

    #[tokio::test]
    async fn test_mock_transport_available() {
        let mut transport = MockTransport::new();
        transport.set_read_data(b"Data");
        assert_eq!(transport.available().await, 4);

        // Read 2 bytes
        let mut buf = [0u8; 2];
        transport.read(&mut buf).await.unwrap();
        assert_eq!(transport.available().await, 2);
    }

    #[tokio::test]
    async fn test_mock_transport_reset() {
        let mut transport = MockTransport::new();
        transport.set_read_data(b"test");
        transport.write(b"data").await.unwrap();
        transport.set_read_error(TransportError::Timeout);

        transport.reset();

        assert_eq!(transport.available().await, 0);
        assert_eq!(transport.written_data().len(), 0);
        let mut buf = [0u8; 32];
        assert!(transport.read(&mut buf).await.is_err()); // No data to read
    }
}
